using System;
using System.Collections.Generic;

/// <summary>Repeat permission exists only after a successful server-retained plan and an explicit new attempt.</summary>
public sealed class RobotPlanRepeatReview
{
    private string[] _names;
    private double[] _positions;
    public string PlanId { get; private set; }
    public string AttemptId { get; private set; } = "";
    public bool HasRetainedPlan { get; private set; }

    public void CaptureStart(string[] names, double[] positions)
    {
        Invalidate();
        if (names == null || positions == null || names.Length != 6 || positions.Length != names.Length) return;
        HashSet<string> seen = new HashSet<string>();
        for (int i = 0; i < names.Length; i++)
            if (string.IsNullOrWhiteSpace(names[i]) || !seen.Add(names[i]) || !Finite(positions[i])) return;
        _names = (string[])names.Clone();
        _positions = (double[])positions.Clone();
    }

    public bool MatchesAttempt(string planId, string attemptId) =>
        AttemptId == (attemptId ?? "") && (string.IsNullOrEmpty(PlanId) || PlanId == planId);

    public void Complete(bool success, bool repeatable, string planId, string attemptId)
    {
        if (!MatchesAttempt(planId, attemptId)) return;
        HasRetainedPlan = success && repeatable && _names != null && !string.IsNullOrWhiteSpace(planId);
        if (HasRetainedPlan) PlanId = planId;
    }

    public bool TryMatchStart(IDictionary<string, double> current, double toleranceRadians, out string reason)
    {
        reason = null;
        if (!HasRetainedPlan || _names == null) reason = "The backend has not retained a repeatable successful plan. Plan again.";
        else if (current == null || !(toleranceRadians > 0d) || !Finite(toleranceRadians)) reason = "Fresh complete joint feedback is required before repeating.";
        else
            for (int i = 0; i < _names.Length; i++)
                if (!current.TryGetValue(_names[i], out double value) || !Finite(value) ||
                    Math.Abs(value - _positions[i]) > toleranceRadians)
                {
                    // UR arm joints are bounded: a full turn is not the same reviewed winding.
                    reason = "Return the robot to the plan's initial joint positions before executing again.";
                    break;
                }
        return reason == null;
    }

    public bool TryBeginAttempt(out string planId, out string attemptId)
    {
        planId = attemptId = null;
        if (!HasRetainedPlan) return false;
        HasRetainedPlan = false;
        AttemptId = Guid.NewGuid().ToString("N");
        planId = PlanId;
        attemptId = AttemptId;
        return true;
    }

    public void Invalidate()
    {
        _names = null;
        _positions = null;
        PlanId = null;
        AttemptId = "";
        HasRetainedPlan = false;
    }
    private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
}
