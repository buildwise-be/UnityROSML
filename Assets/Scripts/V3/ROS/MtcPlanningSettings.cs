using UnityEngine;

/// <summary>The operator's planning time limit, shared by the task workflows on this headset.</summary>
public static class MtcPlanningSettings
{
    public const int DefaultTimeoutSeconds = 60;
    public const int MinTimeoutSeconds = 1;
    public const int MaxTimeoutSeconds = 600;
    private const string TimeoutPreference = "UnityROSML.MtcPlanningTimeoutSeconds";

    public static int TimeoutSeconds
    {
        get => Mathf.Clamp(PlayerPrefs.GetInt(TimeoutPreference, DefaultTimeoutSeconds), MinTimeoutSeconds, MaxTimeoutSeconds);
        set
        {
            int seconds = Mathf.Clamp(value, MinTimeoutSeconds, MaxTimeoutSeconds);
            if (PlayerPrefs.GetInt(TimeoutPreference, DefaultTimeoutSeconds) == seconds) return;
            PlayerPrefs.SetInt(TimeoutPreference, seconds);
            PlayerPrefs.Save();
        }
    }
}
