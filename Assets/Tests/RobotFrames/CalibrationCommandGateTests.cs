using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>Calibration must revoke coordinate-dependent actions even without a tool-menu instance.</summary>
public sealed class CalibrationCommandGateTests
{
    private readonly List<GameObject> _objects = new List<GameObject>();
    private Component _coordinates;

    [SetUp]
    public void SetUp()
    {
        _coordinates = NewObject("Offline alignment coordinates").AddComponent(RuntimeType("RobotCoordinateService"));
    }

    [TearDown]
    public void TearDown()
    {
        for (int index = _objects.Count - 1; index >= 0; index--)
            if (_objects[index] != null) UnityEngine.Object.DestroyImmediate(_objects[index]);
        _objects.Clear();
    }

    [Test]
    public void EndingEditRequiresExplicitFinalPoseInitialization()
    {
        GameObject root = NewObject("Offline alignment robot base");
        root.transform.SetPositionAndRotation(new Vector3(1f, 0.4f, -2f), Quaternion.Euler(0f, 35f, 0f));
        ArticulationBody body = root.AddComponent<ArticulationBody>();
        Transform tracking = NewObject("Offline alignment tracking").transform;
        Call(_coordinates, "Initialize", body, tracking);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
        Call(_coordinates, "BeginCalibrationEdit");
        Assert.That(Get<bool>(_coordinates, "IsCalibrationInProgress"), Is.True);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        // A concurrent model/placement initialization must not rearm an active edit.
        Call(_coordinates, "Initialize", body, tracking);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        Call(_coordinates, "EndCalibrationEdit");
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        Call(_coordinates, "Initialize", body, tracking);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void ToolPolicyBlocksLegacyCommandFallbackThroughoutEditing()
    {
        Type tools = RuntimeType("ToolSelectionController");
        Call(_coordinates, "BeginCalibrationEdit");
        Assert.That((bool)tools.GetProperty("CanSendRobotCommands").GetValue(null), Is.False);
        Assert.That((bool)tools.GetProperty("CanPlanPickPlace").GetValue(null), Is.False);
    }

    [Test]
    public void TcpPlanAndExecuteReportAlignmentBeforeRequestingAnyConnection()
    {
        GameObject owner = NewObject("Offline alignment TCP controller");
        owner.SetActive(false); // No Start or ROS transport.
        Component planning = owner.AddComponent(RuntimeType("TcpPlanningController"));
        Call(_coordinates, "BeginCalibrationEdit");
        Call(planning, "Plan");
        Assert.That(Get<string>(planning, "StatusText"), Does.Contain("alignment"));
        Assert.That(Get<bool>(planning, "CanExecute"), Is.False);
        Call(planning, "Execute");
        Assert.That(Get<string>(planning, "StatusText"), Does.Contain("alignment"));
    }

    [Test]
    public void TargetRotationCannotBeginDuringBaseAlignment()
    {
        Component handles = NewObject("Offline captured target").AddComponent(RuntimeType("TargetRotationHandles"));
        Type kind = RuntimeType("TargetRotationHandles+RotationHandleKind");
        object free = Enum.Parse(kind, "Free");
        Call(_coordinates, "BeginCalibrationEdit");
        Assert.That(Call(handles, "BeginRotation", free, Quaternion.identity, Vector3.zero), Is.EqualTo(false));
        Call(_coordinates, "EndCalibrationEdit");
        Assert.That(Call(handles, "BeginRotation", free, Quaternion.identity, Vector3.zero), Is.EqualTo(true));
        Call(handles, "EndRotation");
    }

    private GameObject NewObject(string name)
    {
        var value = new GameObject(name);
        _objects.Add(value);
        return value;
    }
    private static Type RuntimeType(string name) => Type.GetType(name + ", Assembly-CSharp", true);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Call(object value, string name, params object[] args) => value.GetType().InvokeMember(name,
        BindingFlags.Public | BindingFlags.Instance | BindingFlags.InvokeMethod, null, value, args);
}
