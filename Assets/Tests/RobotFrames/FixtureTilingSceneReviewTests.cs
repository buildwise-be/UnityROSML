using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>The actual scene reviewer must preserve object lifecycle and reject undisplayable later states.</summary>
public sealed class FixtureTilingSceneReviewTests
{
    private readonly List<GameObject> _owned=new List<GameObject>();
    private Component _preview;

    [SetUp]
    public void SetupRobotCoordinatesWithoutTransport()
    {
        Component coordinates=New("Fixture scene coordinates").AddComponent(TypeOf("RobotCoordinateService"));
        ArticulationBody body=New("Fixture scene base").AddComponent<ArticulationBody>();
        body.transform.SetPositionAndRotation(new Vector3(.3f,.2f,-.4f),Quaternion.Euler(0,23,0));
        Call(coordinates,"Initialize",body,New("Fixture scene tracking").transform);
        _preview=New("Fixture scene reviewer").AddComponent(TypeOf("FixtureTilingPreview"));
    }

    [TearDown]
    public void Cleanup()
    {
        for(int i=_owned.Count-1;i>=0;i--)if(_owned[i]!=null)UnityEngine.Object.DestroyImmediate(_owned[i]);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")),Is.Empty);
    }

    [Test]
    public void AttachmentMotionAndDetachmentKeepTheTileInTheCorrectFrameAndStep()
    {
        const string id="fixture_test_tile";
        RosRobotModel model=RosRobotModelCatalog.Load().Find("kenos_sl");
        string[] names=model.links.Where(RosRobotModelMath.IsRevolute).Select(link=>link.jointName).ToArray();
        var angles=names.ToDictionary(name=>name,name=>0.0);
        object initial=Scene(false,Box(id,"ur_base_link",.4,.2,.3));
        object attachScene=Scene(true,Remove(id));
        object attached=Message("AttachedCollisionObjectMsg","Moveit");
        Set(attached,"link_name","tcp");Set(attached,"object",Box(id,"tcp",.01,.02,.03));
        SetArray(Read(attachScene,"robot_state"),"attached_collision_objects",attached);
        object attach=Step(attachScene,"attach tile");
        object move=Step(Scene(true),"carry tile");
        object joint=Read(Read(move,"trajectory"),"joint_trajectory");Set(joint,"joint_names",names);
        double[] values=new double[names.Length];values[0]=.3;angles[names[0]]=.3;
        object last=Message("JointTrajectoryPointMsg","Trajectory");Set(last,"positions",values);
        Set(Read(last,"time_from_start"),"sec",1);Set(Read(last,"time_from_start"),"nanosec",(uint)250000000);
        SetArray(joint,"points",last);
        object detachScene=Scene(true,Box(id,"ur_base_link",.7,.4,.25));
        object removed=Message("AttachedCollisionObjectMsg","Moveit");Set(removed,"object",Remove(id));
        SetArray(Read(detachScene,"robot_state"),"attached_collision_objects",removed);
        object solution=Solution(attach,move,Step(detachScene,"detach tile"));
        Review(initial,solution,model);
        Assert.That(Get<bool>(_preview,"HasReview"),Is.True);Assert.That(Get<int>(_preview,"StepCount"),Is.EqualTo(4));
        Assert.That(Vector3.Distance(Object(id).localPosition,new Vector3(-.2f,.3f,.4f)),Is.LessThan(1e-6f));
        Assert.That(Object(id).localScale,Is.EqualTo(new Vector3(.3f,.01f,.2f)),"ROS box X/Y/Z must map to Unity Z/X/Y.");
        Call(_preview,"Step",1);
        Assert.That(Get<string>(_preview,"StepText"),Does.Contain("1 attached"));
        Pose tcp0=RosRobotModelMath.ForwardKinematics(model)["tcp"];
        Pose expected=RobotFrameMath.Compose(tcp0,new Pose(new Vector3(-.02f,.03f,.01f),Quaternion.identity));
        Assert.That(Vector3.Distance(Object(id).localPosition,expected.position),Is.LessThan(2e-6f));
        Call(_preview,"Step",1);
        Pose tcp1=RosRobotModelMath.ForwardKinematics(model,angles)["tcp"];
        expected=RobotFrameMath.Compose(tcp1,new Pose(new Vector3(-.02f,.03f,.01f),Quaternion.identity));
        Assert.That(Vector3.Distance(Object(id).localPosition,expected.position),Is.LessThan(2e-6f));
        Assert.That(Get<float>(_preview,"StepMotionTime"),Is.EqualTo(1.25f));
        Call(_preview,"Step",1);
        Assert.That(Get<string>(_preview,"StepText"),Does.Contain("0 attached"));
        Assert.That(Vector3.Distance(Object(id).localPosition,new Vector3(-.4f,.25f,.7f)),Is.LessThan(1e-6f));
        Call(_preview,"Step",-3);
        Assert.That(Vector3.Distance(Object(id).localPosition,new Vector3(-.2f,.3f,.4f)),Is.LessThan(1e-6f),"Later states must not mutate the initial scene snapshot.");
    }

    [Test]
    public void StaticWorldToBaseTransformsAreResolvedInBothDirections()
    {
        object initial=Scene(false,Box("world obstacle","world",.7,0,.2));
        object transform=Message("TransformStampedMsg","Geometry");
        Set(Read(transform,"header"),"frame_id","world");Set(transform,"child_frame_id","ur_base_link");
        object value=Read(transform,"transform");Set(Read(value,"translation"),"x",.5);Set(Read(value,"rotation"),"w",1.0);
        SetArray(initial,"fixed_frame_transforms",transform);
        Review(initial,Solution(Step(Scene(true),"no scene changes")),null);
        Assert.That(Vector3.Distance(Object("world obstacle").localPosition,new Vector3(0,.2f,.2f)),Is.LessThan(1e-6f));
    }

    [Test]
    public void AFrameIntroducedOnlyInALaterSceneMustBeValidatedBeforeReviewIsReady()
    {
        object initial=Scene(false,Box("initial","ur_base_link",0,0,.2));
        object unknown=Scene(true,Box("later","unresolved_frame",0,0,.2));
        Reject(initial,Solution(Step(Scene(true),"first valid state"),Step(unknown,"bad frame")));
    }

    [Test]
    public void InvalidLaterPrimitiveDimensionsCannotHideBehindAValidInitialScene()
    {
        object invalid=Box("invalid","ur_base_link",0,0,.2);
        object primitive=((Array)Read(invalid,"primitives")).GetValue(0);Set(primitive,"dimensions",new[]{.2,double.NaN,.01});
        Reject(Scene(false,Box("initial","ur_base_link",0,0,.2)),Solution(Step(Scene(true,invalid),"bad dimensions")));
    }

    [Test]
    public void UnsupportedPlanesAndNonemptyOctomapsAreRejectedInsteadOfOmitted()
    {
        object plane=Box("plane","ur_base_link",0,0,0);
        SetArray(plane,"planes",Message("PlaneMsg","Shape"));
        Reject(Scene(false,plane),Solution(Step(Scene(true),"unchanged")));
        object scene=Scene(false,Box("initial","ur_base_link",0,0,.2));
        object octomap=Read(Read(Read(scene,"world"),"octomap"),"octomap");
        Type element=octomap.GetType().GetField("data").FieldType.GetElementType();Array bytes=Array.CreateInstance(element,1);
        bytes.SetValue(Convert.ChangeType(1,element),0);Set(octomap,"data",bytes);
        Reject(scene,Solution(Step(Scene(true),"unchanged")));
    }

    [Test]
    public void VacuumAndSettlingMarkersHaveReadableReviewLabels()
    {
        object scene=Scene(false,Box("initial","ur_base_link",0,0,.2));
        Review(scene,Solution(
            Step(Scene(true),"IO_MARKER fun=1 pin=3 state=0 post_delay=0.5"),
            Step(Scene(true),"WAIT_MARKER seconds=3"),
            Step(Scene(true),"IO_MARKER fun=1 pin=3 state=1 post_delay=0")),null);
        Call(_preview,"Step",1);
        Assert.That(Get<string>(_preview,"StepText"),Does.Contain("Vacuum off").And.Contain("release hold 0.5 s"));
        Call(_preview,"Step",1);
        Assert.That(Get<string>(_preview,"StepText"),Does.Contain("Wait for settling (3 s)"));
        Call(_preview,"Step",1);
        Assert.That(Get<string>(_preview,"StepText"),Does.Contain("Vacuum on"));
    }

    [Test]
    public void AnUnreadableLaterMarkerRevokesTheWholeReview()
    {
        object scene=Scene(false,Box("initial","ur_base_link",0,0,.2));
        Reject(scene,Solution(Step(Scene(true),"normal first step"),Step(Scene(true),"WAIT_MARKER seconds=NaN")));
        Reject(scene,Solution(Step(Scene(true),"normal first step"),Step(Scene(true),"IO_MARKER state=0 post_delay=invalid")));
    }

    private object Scene(bool diff,params object[] objects)
    {
        object scene=Message("PlanningSceneMsg","Moveit");Set(scene,"name","fixture-review:1");Set(scene,"is_diff",diff);
        Set(Read(scene,"robot_state"),"is_diff",diff);SetArray(Read(scene,"world"),"collision_objects",objects);return scene;
    }
    private object Step(object scene,string comment)
    {
        object step=Message("SubTrajectoryMsg","MoveitTaskConstructor");Set(step,"scene_diff",scene);Set(Read(step,"info"),"comment",comment);return step;
    }
    private object Solution(params object[] steps)
    { object solution=Message("SolutionMsg","MoveitTaskConstructor");Set(solution,"task_id","fixture-review:1");SetArray(solution,"sub_trajectory",steps);return solution; }
    private object Box(string id,string frame,double x,double y,double z)
    {
        object box=Message("CollisionObjectMsg","Moveit");Set(box,"id",id);Set(Read(box,"header"),"frame_id",frame);
        object pose=Read(box,"pose");Set(Read(pose,"position"),"x",x);Set(Read(pose,"position"),"y",y);Set(Read(pose,"position"),"z",z);Set(Read(pose,"orientation"),"w",1.0);
        Type shapeType=box.GetType().GetField("primitives").FieldType.GetElementType();object shape=Activator.CreateInstance(shapeType);
        Set(shape,"type",Convert.ChangeType(1,shapeType.GetField("type").FieldType));Set(shape,"dimensions",new[]{.2,.3,.01});SetArray(box,"primitives",shape);
        object local=Message("PoseMsg","Geometry");Set(Read(local,"orientation"),"w",1.0);SetArray(box,"primitive_poses",local);return box;
    }
    private object Remove(string id)
    { object item=Message("CollisionObjectMsg","Moveit");Set(item,"id",id);Set(item,"operation",(sbyte)1);return item; }
    private void Review(object initial,object solution,RosRobotModel model)=>Call(_preview,"SetReview",initial,solution,Activator.CreateInstance(TypeOf("FixtureTilingSettings")),model);
    private void Reject(object initial,object solution)
    {
        TargetInvocationException exception=Assert.Throws<TargetInvocationException>(()=>Review(initial,solution,null));
        Assert.That(exception.InnerException,Is.TypeOf<ArgumentException>());
        Call(_preview,"ClearReview");Assert.That(Get<bool>(_preview,"HasReview"),Is.False);
    }
    private Transform Object(string name)
    {
        GameObject root=(GameObject)_preview.GetType().GetField("_root",BindingFlags.Instance|BindingFlags.NonPublic).GetValue(_preview);
        return root.GetComponentsInChildren<Transform>(true).Single(t=>t.name==name);
    }
    private GameObject New(string name){var value=new GameObject(name);_owned.Add(value);return value;}
    private static Type TypeOf(string name)=>AppDomain.CurrentDomain.GetAssemblies().Select(assembly=>assembly.GetType(name,false)).First(type=>type!=null);
    private static object Message(string name,string ns)=>Activator.CreateInstance(TypeOf("RosMessageTypes."+ns+"."+name));
    private static object Read(object value,string field)=>value.GetType().GetField(field).GetValue(value);
    private static void Set(object value,string field,object data)=>value.GetType().GetField(field).SetValue(value,data);
    private static T Get<T>(object value,string property)=>(T)value.GetType().GetProperty(property).GetValue(value);
    private static object Call(object value,string method,params object[] arguments)=>value.GetType().GetMethods(BindingFlags.Public|BindingFlags.Instance)
        .Single(candidate=>candidate.Name==method && candidate.GetParameters().Length==arguments.Length &&
            candidate.GetParameters().Select((parameter,index)=>arguments[index]==null || parameter.ParameterType.IsInstanceOfType(arguments[index])).All(compatible=>compatible))
        .Invoke(value,arguments);
    private static void SetArray(object value,string field,params object[] entries)
    {
        Array array=Array.CreateInstance(value.GetType().GetField(field).FieldType.GetElementType(),entries.Length);
        for(int i=0;i<entries.Length;i++)array.SetValue(entries[i],i);Set(value,field,array);
    }
}
