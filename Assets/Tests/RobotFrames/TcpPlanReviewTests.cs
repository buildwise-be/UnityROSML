using NUnit.Framework;

public sealed class TcpPlanReviewTests
{
    [TestCase(true)]
    [TestCase(false)]
    public void MatchingPreviewAndSuccessAreBothRequiredInEitherArrivalOrder(bool previewFirst)
    {
        var review = new TcpPlanReview();
        review.Begin("1792660000:101");
        Assert.That(review.TryConsume(out _), Is.False);
        if (previewFirst) review.AcceptPreview(review.RequestId);
        else review.AcceptResult(review.RequestId, true);
        Assert.That(review.CanExecute, Is.False);
        if (previewFirst) review.AcceptResult(review.RequestId, true);
        else review.AcceptPreview(review.RequestId);
        Assert.That(review.TryConsume(out string token), Is.True);
        Assert.That(token, Is.EqualTo("1792660000:101"));
        Assert.That(review.TryConsume(out _), Is.False, "A reviewed solution is single-use.");
    }

    [Test]
    public void TargetChangeMakesLateOldResultAndPreviewIneligible()
    {
        var review = new TcpPlanReview();
        review.Begin("old");
        review.AcceptResult("old", true);
        review.Invalidate(); // capture, rotation, QR rescan, model change or disconnect
        Assert.That(review.AcceptPreview("old"), Is.False);
        Assert.That(review.AcceptResult("old", true), Is.False);
        review.Begin("new");
        Assert.That(review.AcceptPreview("old"), Is.False);
        Assert.That(review.AcceptResult("old", true), Is.False);
        review.AcceptResult("new", true);
        Assert.That(review.CanExecute, Is.False);
        review.AcceptPreview("new");
        Assert.That(review.CanExecute, Is.True);
    }

    [Test]
    public void FailureCannotBeRearmedByDelayedPreviewOrSuccess()
    {
        var review = new TcpPlanReview();
        review.Begin("failed");
        review.AcceptPreview("failed");
        Assert.That(review.AcceptResult("failed", false), Is.True);
        Assert.That(review.AcceptPreview("failed"), Is.False);
        Assert.That(review.AcceptResult("failed", true), Is.False);
        Assert.That(review.CanExecute, Is.False);
    }
}
