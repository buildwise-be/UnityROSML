using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>Local commands awaiting a terminal ROS outcome, independent of plan-review lifetime.</summary>
public static class RobotExecutionState
{
    private static readonly HashSet<UnityEngine.Object> Owners = new HashSet<UnityEngine.Object>();
    public static event Action Changed;
    /// <summary>A deliberate new Plan trigger; telemetry peaks reset even if later validation rejects it.</summary>
    public static event Action PlanningStarted;
    /// <summary>Revision of deliberate Plan/Execute starts, including starts missed by a disabled observer.</summary>
    public static long CommandRevision { get; private set; }
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
        if (changed && busy) unchecked { CommandRevision++; }
        if (changed) Changed?.Invoke();
    }

    /// <summary>Notify once after busy/pending guards, before validating a new planning request.</summary>
    public static void NotifyPlanningStarted()
    {
        unchecked { CommandRevision++; }
        PlanningStarted?.Invoke();
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Reset() { Owners.Clear(); Changed = null; PlanningStarted = null; CommandRevision = 0; }
}
