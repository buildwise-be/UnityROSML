using System;

/// <summary>Only the matching successful result and preview can arm a single execution.</summary>
public sealed class TcpPlanReview
{
    public string RequestId { get; private set; }
    public bool IsPending { get; private set; }
    public bool HasPreview { get; private set; }
    public bool HasSuccessfulResult { get; private set; }
    public bool CanExecute => IsPending && HasPreview && HasSuccessfulResult;

    public void Begin(string requestId)
    {
        if (string.IsNullOrWhiteSpace(requestId)) throw new ArgumentException("A request token is required.", nameof(requestId));
        Invalidate();
        RequestId = requestId;
        IsPending = true;
    }

    public bool AcceptResult(string requestId, bool success)
    {
        if (!Matches(requestId)) return false;
        if (!success) { Invalidate(); return true; }
        HasSuccessfulResult = true;
        return true;
    }

    public bool AcceptPreview(string requestId)
    {
        if (!Matches(requestId)) return false;
        HasPreview = true;
        return true;
    }

    public bool TryConsume(out string requestId)
    {
        requestId = null;
        if (!CanExecute) return false;
        requestId = RequestId;
        Invalidate();
        return true;
    }

    public void Invalidate()
    {
        RequestId = null;
        IsPending = HasPreview = HasSuccessfulResult = false;
    }

    private bool Matches(string requestId) => IsPending && RequestId == requestId;
}
