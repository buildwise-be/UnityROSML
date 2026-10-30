using System;
using System.Collections.Generic;
using NUnit.Framework;

/// <summary>Repeat permission requires a retained server plan and a matching successful execution attempt.</summary>
public sealed class RobotPlanRepeatReviewTests
{
    private const string Plan = "reviewed-plan-A";
    private static readonly string[] Names = {
        "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
        "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"
    };
    private static readonly double[] Positions = { 0.2d, -1.3d, 1.7d, -1.9d, -1.1d, 0.5d };

    [Test]
    public void AReviewedStartAloneCannotGrantAnotherExecution()
    {
        var review = Captured();
        Assert.That(review.HasRetainedPlan, Is.False);
        Assert.That(review.TryMatchStart(Current(), 0.01d, out string reason), Is.False);
        Assert.That(reason, Does.Contain("backend"));
        Assert.That(review.TryBeginAttempt(out string plan, out string attempt), Is.False);
        Assert.That(plan, Is.Null);
        Assert.That(attempt, Is.Null);
    }

    [TestCase(false, true)]
    [TestCase(true, false)]
    [TestCase(false, false)]
    public void OldOrUnsuccessfulServerResponsesNeverGrantRepeatPermission(bool succeeded, bool repeatable)
    {
        var review = Captured();
        review.Complete(succeeded, repeatable, Plan, "");
        Assert.That(review.HasRetainedPlan, Is.False);
        Assert.That(review.TryBeginAttempt(out _, out _), Is.False);
    }

    [TestCase(null)]
    [TestCase("")]
    [TestCase("  ")]
    public void AServerResponseWithoutARetainedPlanTokenCannotRearm(string planId)
    {
        var review = Captured();
        review.Complete(true, true, planId, "");
        Assert.That(review.HasRetainedPlan, Is.False);
        Assert.That(review.TryBeginAttempt(out _, out _), Is.False);
    }

    [Test]
    public void CaptureClonesTheJointNamesAndPositionsAndMatchesFeedbackByName()
    {
        string[] names = (string[])Names.Clone();
        double[] positions = (double[])Positions.Clone();
        var review = new RobotPlanRepeatReview();
        review.CaptureStart(names, positions);
        names[0] = "changed-after-capture";
        positions[0] += 2d;
        review.Complete(true, true, Plan, "");
        Dictionary<string, double> current = Current();
        current.Add("mobile_base_joint", 40d);
        Assert.That(review.TryMatchStart(current, 0.01d, out string reason), Is.True, reason);
    }

    [Test]
    public void MissingWrongAndOutOfToleranceJointsCannotMatchTheReviewedStart()
    {
        var review = Retained();
        Dictionary<string, double> current = Current();
        current.Remove(Names[3]);
        Assert.That(review.TryMatchStart(current, 0.01d, out _), Is.False);
        current.Add("wrong_wrist_name", Positions[3]);
        Assert.That(review.TryMatchStart(current, 0.01d, out _), Is.False);
        current = Current();
        current[Names[3]] += 0.02d;
        Assert.That(review.TryMatchStart(current, 0.01d, out string reason), Is.False);
        Assert.That(reason, Does.Contain("initial joint positions"));
        Assert.That(review.HasRetainedPlan, Is.True, "Moving away must not consume the retained plan.");
        Assert.That(review.TryMatchStart(Current(), 0.01d, out reason), Is.True, reason);
    }

    [TestCase(double.NaN)]
    [TestCase(double.PositiveInfinity)]
    [TestCase(double.NegativeInfinity)]
    public void NonfiniteFeedbackCannotMatchTheReviewedStart(double value)
    {
        var review = Retained();
        Dictionary<string, double> current = Current();
        current[Names[2]] = value;
        Assert.That(review.TryMatchStart(current, 0.01d, out _), Is.False);
    }

    [TestCase(0d)]
    [TestCase(-0.01d)]
    [TestCase(double.NaN)]
    [TestCase(double.PositiveInfinity)]
    public void InvalidToleranceCannotMatchTheReviewedStart(double tolerance)
    {
        Assert.That(Retained().TryMatchStart(Current(), tolerance, out _), Is.False);
    }

    [Test]
    public void AFullJointTurnIsDifferentFromTheReviewedWinding()
    {
        var review = Retained();
        foreach (string name in Names)
        {
            Dictionary<string, double> current = Current();
            current[name] += 2d * Math.PI;
            Assert.That(review.TryMatchStart(current, 0.01d, out _), Is.False, name);
        }
    }

    [Test]
    public void RepeatAttemptConsumesPermissionUntilItsOwnSuccessfulAcknowledgement()
    {
        var review = Retained();
        Assert.That(review.TryBeginAttempt(out string plan, out string attempt), Is.True);
        Assert.That(plan, Is.EqualTo(Plan));
        Assert.That(Guid.TryParseExact(attempt, "N", out _), Is.True);
        Assert.That(review.HasRetainedPlan, Is.False);
        Assert.That(review.TryBeginAttempt(out _, out _), Is.False, "A second click must not dispatch a duplicate attempt.");
        review.Complete(true, true, Plan, attempt);
        Assert.That(review.HasRetainedPlan, Is.True);
        Assert.That(review.TryMatchStart(Current(), 0.01d, out string reason), Is.True, reason);
    }

    [Test]
    public void AStalePlanOrAttemptAcknowledgementCannotRearmTheCurrentAttempt()
    {
        var review = Retained();
        Assert.That(review.TryBeginAttempt(out _, out string attempt), Is.True);
        review.Complete(true, true, "different-plan", attempt);
        Assert.That(review.HasRetainedPlan, Is.False);
        review.Complete(true, true, Plan, "");
        Assert.That(review.HasRetainedPlan, Is.False);
        review.Complete(true, true, Plan, "old-attempt");
        Assert.That(review.HasRetainedPlan, Is.False);
        review.Complete(true, true, Plan, attempt);
        Assert.That(review.HasRetainedPlan, Is.True);
    }

    [Test]
    public void EachRepeatUsesANewAttemptAndLateDuplicateSuccessDoesNotRearmTheNextOne()
    {
        var review = Retained();
        Assert.That(review.TryBeginAttempt(out _, out string first), Is.True);
        review.Complete(true, true, Plan, first);
        Assert.That(review.TryBeginAttempt(out _, out string second), Is.True);
        Assert.That(second, Is.Not.EqualTo(first));
        Assert.That(Guid.TryParseExact(second, "N", out _), Is.True);
        review.Complete(true, true, Plan, first);
        Assert.That(review.HasRetainedPlan, Is.False);
        review.Complete(false, true, Plan, second);
        Assert.That(review.HasRetainedPlan, Is.False);
    }

    [Test]
    public void InvalidOrReplacedStartDefinitionsRevokeTheOldReview()
    {
        foreach (string invalid in new[] { "null", "short", "duplicate", "empty", "nonfinite" })
        {
            var review = Retained();
            string[] names = (string[])Names.Clone();
            double[] positions = (double[])Positions.Clone();
            switch (invalid)
            {
                case "null": names = null; break;
                case "short": positions = new[] { 0d }; break;
                case "duplicate": names[1] = names[0]; break;
                case "empty": names[2] = " "; break;
                case "nonfinite": positions[4] = double.NaN; break;
            }
            review.CaptureStart(names, positions);
            review.Complete(true, true, Plan, "");
            Assert.That(review.HasRetainedPlan, Is.False, invalid);
        }
    }

    [Test]
    public void InvalidateClearsThePlanAttemptAndCapturedStart()
    {
        var review = Retained();
        Assert.That(review.TryBeginAttempt(out _, out string attempt), Is.True);
        review.Invalidate();
        Assert.That(review.PlanId, Is.Null);
        Assert.That(review.AttemptId, Is.Empty);
        Assert.That(review.HasRetainedPlan, Is.False);
        review.Complete(true, true, Plan, attempt);
        review.Complete(true, true, Plan, "");
        Assert.That(review.HasRetainedPlan, Is.False);
    }

    private static RobotPlanRepeatReview Captured()
    {
        var review = new RobotPlanRepeatReview();
        review.CaptureStart(Names, Positions);
        return review;
    }
    private static RobotPlanRepeatReview Retained()
    {
        var review = Captured();
        review.Complete(true, true, Plan, "");
        Assert.That(review.HasRetainedPlan, Is.True);
        return review;
    }
    private static Dictionary<string, double> Current()
    {
        var current = new Dictionary<string, double>();
        for (int i = 0; i < Names.Length; i++) current.Add(Names[i], Positions[i]);
        return current;
    }
}
