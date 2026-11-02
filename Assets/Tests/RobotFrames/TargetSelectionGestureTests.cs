using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>Native XRI selection events distinguish a short menu click from a drag, without devices or ROS.</summary>
public sealed class TargetSelectionGestureTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private readonly List<GameObject> _previousCameras = new List<GameObject>();
    private Component _manager, _interactor, _interactable, _handles, _context, _coordinates;
    private Transform _target, _tracking;
    private int _started, _changed;

    [SetUp]
    public void SetupActualXriSelectionAndTargetWithoutHardware()
    {
        _started = _changed = 0;
        foreach(Camera camera in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include,FindObjectsSortMode.None))
            if(camera.CompareTag("MainCamera")) { _previousCameras.Add(camera.gameObject);camera.tag="Untagged"; }
        GameObject viewer = New("Gesture menu camera");viewer.tag="MainCamera";
        viewer.transform.position=new Vector3(0,1,-2);viewer.AddComponent<Camera>();
        _coordinates=New("Gesture coordinates").AddComponent(TypeOf("RobotCoordinateService"));
        _manager=New("Gesture XRI manager").AddComponent(TypeOf("UnityEngine.XR.Interaction.Toolkit.XRInteractionManager"));
        Lifecycle(_manager,"Awake");Lifecycle(_manager,"OnEnable");
        _tracking=New("Gesture tracking parent").transform;
        _tracking.SetPositionAndRotation(new Vector3(.3f,.1f,-.2f),Quaternion.Euler(0,27,0));
        _target=New("Gesture contact target").transform;_target.SetParent(_tracking,false);
        _target.SetPositionAndRotation(new Vector3(.4f,.8f,.2f),Quaternion.Euler(24,-17,13));
        _handles=(Component)TypeOf("TargetRotationHandles").GetMethod("Attach").Invoke(null,
            new object[] {_target,Resources.Load<Material>("RobotTargetMarkers/Opaque"),.035f});
        Call(_handles,"ConfigureNormalInteraction");
        _context=(Component)TypeOf("TargetPoseContextMenu").GetMethod("Attach").Invoke(null,
            new object[] {_target,"Gesture contact",true,(Action)(() => { })});
        _handles.GetType().GetEvent("RotationStarted").AddEventHandler(_handles,(Action)(() => _started++));
        _handles.GetType().GetEvent("RotationChanged").AddEventHandler(_handles,(Action)(() => _changed++));
        object free=((System.Collections.IEnumerable)Field(_handles,"_handles").GetValue(_handles)).Cast<object>()
            .Single(handle => Field(handle,"Kind").GetValue(handle).ToString()=="Free");
        _interactable=(Component)Field(free,"Interactable").GetValue(free);
        SetProperty(_interactable,"interactionManager",_manager);
        Lifecycle(_interactable,"Awake");Lifecycle(_interactable,"OnEnable");
        GameObject controller=New("Gesture direct controller");controller.SetActive(false);
        controller.transform.position=_target.position+Vector3.back*.1f;
        controller.AddComponent<SphereCollider>().isTrigger=true;
        _interactor=controller.AddComponent(TypeOf("UnityEngine.XR.Interaction.Toolkit.Interactors.XRDirectInteractor"));
        SetProperty(_interactor,"interactionManager",_manager);
        SetProperty(_interactor,"improveAccuracyWithSphereCollider",true);
        Lifecycle(_interactor,"Awake");
        controller.SetActive(true);Lifecycle(_interactor,"OnEnable");
    }

    [TearDown]
    public void Cleanup()
    {
        TypeOf("TargetPoseContextMenu").GetMethod("CloseAll").Invoke(null,null);
        TypeOf("TargetRotationHandles").GetMethod("CancelAllInteractions").Invoke(null,null);
        if(_interactor!=null)Lifecycle(_interactor,"OnDisable");
        if(_interactable!=null)Lifecycle(_interactable,"OnDisable");
        if(_manager!=null)Lifecycle(_manager,"OnDisable");
        for(int index=_owned.Count-1;index>=0;index--)if(_owned[index]!=null)UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        foreach(GameObject camera in _previousCameras)if(camera!=null)camera.tag="MainCamera";
        _previousCameras.Clear();
        Assert.That(Interacting,Is.False);
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")),Is.Empty);
    }

    [Test]
    public void ShortTriggerClickOpensTheContextWithoutStartingOrChangingRotation()
    {
        Pose original=new Pose(_target.position,_target.rotation);
        Native("SelectEnter");
        Assert.That(Interacting,Is.True,"XRI selection owns the pending click before release.");
        Assert.That(_started,Is.Zero);Assert.That(_changed,Is.Zero);
        Native("SelectExit");
        Assert.That(Open,Is.True);Assert.That(Interacting,Is.False);
        Assert.That(_started,Is.Zero);Assert.That(_changed,Is.Zero);
        AssertPose(original);
    }

    [TestCase(2f,false)]
    [TestCase(12f,true)]
    public void ControllerMotionCrossesTheDragThresholdWithoutTurningADragIntoAMenuClick(float degrees,bool drag)
    {
        Quaternion original=_target.rotation;
        Native("SelectEnter");
        _interactor.transform.rotation=Quaternion.AngleAxis(degrees,Vector3.forward);
        Lifecycle(_handles,"Update");
        Assert.That(_started,Is.EqualTo(drag?1:0));
        if(drag)
        {
            Assert.That(_changed,Is.GreaterThan(0));
            Assert.That(Quaternion.Angle(_target.rotation,Quaternion.AngleAxis(degrees,Vector3.forward)*original),Is.LessThan(.001f));
        }
        else Assert.That(_changed,Is.Zero);
        Native("SelectExit");
        Assert.That(Open,Is.EqualTo(!drag));Assert.That(Interacting,Is.False);
    }

    [Test]
    public void HoldingPastTheClickWindowStartsRotationAndReleaseDoesNotOpenTheMenu()
    {
        Native("SelectEnter");
        Field(_handles,"_selectionStartedAt").SetValue(_handles,Time.unscaledTime-.5f);
        Lifecycle(_handles,"Update");
        Assert.That(_started,Is.EqualTo(1));
        Assert.That(_changed,Is.Zero,"A hold starts an edit without inventing a changed orientation.");
        Native("SelectExit");
        Assert.That(Open,Is.False);Assert.That(Interacting,Is.False);
    }

    [TestCase("canceled")]
    [TestCase("calibration")]
    [TestCase("parent")]
    public void InterruptedSelectionNeverBecomesAContextClick(string interruption)
    {
        Native("SelectEnter");
        if(interruption=="calibration")Call(_coordinates,"BeginCalibrationEdit");
        if(interruption=="parent")_target.SetParent(New("Replacement gesture parent").transform,true);
        Native(interruption=="canceled"?"SelectCancel":"SelectExit");
        Assert.That(Open,Is.False);Assert.That(Interacting,Is.False);
        Assert.That(_started,Is.Zero);Assert.That(_changed,Is.Zero);
    }

    [Test]
    public void CancelAllAbortsThePendingNativeSelectionWithoutOpeningAContextMenu()
    {
        Native("SelectEnter");
        TypeOf("TargetRotationHandles").GetMethod("CancelAllInteractions").Invoke(null,null);
        Assert.That(Open,Is.False);Assert.That(Interacting,Is.False);
        Assert.That(Get<bool>(_interactable,"isSelected"),Is.False);
        Assert.That(_started,Is.Zero);Assert.That(_changed,Is.Zero);
    }

    private void Native(string method)
    {
        _manager.GetType().GetMethod(method,new[] {
            TypeOf("UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor"),
            TypeOf("UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable")
        }).Invoke(_manager,new object[] {_interactor,_interactable});
    }
    private bool Open=>(bool)TypeOf("TargetPoseContextMenu").GetProperty("IsOpen").GetValue(null);
    private bool Interacting=>(bool)TypeOf("TargetRotationHandles").GetProperty("IsInteracting").GetValue(null);
    private void AssertPose(Pose pose)
    {Assert.That(Vector3.Distance(_target.position,pose.position),Is.LessThan(1e-6f));Assert.That(Quaternion.Angle(_target.rotation,pose.rotation),Is.LessThan(.001f));}
    private GameObject New(string name){var value=new GameObject(name);_owned.Add(value);return value;}
    private static Type TypeOf(string name)=>AppDomain.CurrentDomain.GetAssemblies().Select(assembly=>assembly.GetType(name,false)).First(type=>type!=null);
    private static FieldInfo Field(object value,string name)=>value.GetType().GetField(name,BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic);
    private static T Get<T>(object value,string property)=>(T)value.GetType().GetProperty(property).GetValue(value);
    private static void SetProperty(object value,string property,object data)=>value.GetType().GetProperty(property).SetValue(value,data);
    private static object Call(object value,string method,params object[] arguments)=>value.GetType().GetMethod(method).Invoke(value,arguments);
    private static void Lifecycle(object value,string method)=>value.GetType().GetMethod(method,BindingFlags.Instance|BindingFlags.NonPublic).Invoke(value,null);
}
