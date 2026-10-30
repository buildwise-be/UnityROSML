using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>Local commands awaiting a terminal ROS outcome, independent of plan-review lifetime.</summary>
public static class RobotExecutionState
{
    private static readonly HashSet<UnityEngine.Object> Owners = new HashSet<UnityEngine.Object>();
    public static event Action Changed;
    public static bool IsBusy
    {
        get
        {
            foreach (UnityEngine.Object owner in Owners) if (owner != null) return true;
            return false;
        }
    }

    public static void SetBusy(UnityEngine.Object owner, bool busy)
    {
        if (owner == null) return;
        Owners.RemoveWhere(value => value == null);
        bool changed = busy ? Owners.Add(owner) : Owners.Remove(owner);
        if (changed) Changed?.Invoke();
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Reset() { Owners.Clear(); Changed = null; }
}
