using System;
using System.Reflection;
using NUnit.Framework;

public sealed class FixtureSolutionPreviewTests
{
    [Test]
    public void ReordersEverySegmentAndRebasesTimeWithoutMutatingTheSolution()
    {
        object first=Segment(new[]{"shoulder","wrist"}, Point(new[]{0.0,1.0},0),Point(new[]{0.2,1.2},1,250000000));
        object second=Segment(new[]{"wrist","shoulder"}, Point(new[]{1.2,0.2},0),Point(new[]{1.4,0.4},2,500000000));
        object solution=Solution(first,Empty(),second);
        object output=Flatten(solution);
        object joint=Read(output,"joint_trajectory");
        CollectionAssert.AreEqual(new[]{"shoulder","wrist"},(string[])Read(joint,"joint_names"));
        Array points=(Array)Read(joint,"points");
        Assert.That(points.Length,Is.EqualTo(4),"IO/wait/scene-only steps add no robot motion.");
        CollectionAssert.AreEqual(new[]{0.4,1.4},(double[])Read(points.GetValue(3),"positions"));
        object end=Read(points.GetValue(3),"time_from_start");
        Assert.That(Read(end,"sec"),Is.EqualTo(3));Assert.That(Read(end,"nanosec"),Is.EqualTo((uint)750000000));
        object sourceJoint=Read(Read(second,"trajectory"),"joint_trajectory");
        Array sourcePoints=(Array)Read(sourceJoint,"points");
        Assert.That(Read(Read(sourcePoints.GetValue(1),"time_from_start"),"sec"),Is.EqualTo(2));
        ((double[])Read(points.GetValue(3),"positions"))[0]=99;
        CollectionAssert.AreEqual(new[]{1.4,0.4},(double[])Read(sourcePoints.GetValue(1),"positions"));
        ((string[])Read(joint,"joint_names"))[0]="changed";
        CollectionAssert.AreEqual(new[]{"shoulder","wrist"},(string[])Read(Read(Read(first,"trajectory"),"joint_trajectory"),"joint_names"));
    }

    [Test]
    public void IntegerNanosecondsAvoidAccumulatedFloatingPointSeamRounding()
    {
        object a=Segment(new[]{"joint"},Point(new[]{0.0},0),Point(new[]{0.1},0,999999999));
        object b=Segment(new[]{"joint"},Point(new[]{0.1},0),Point(new[]{0.2},0,2));
        Array points=(Array)Read(Read(Flatten(Solution(a,b)),"joint_trajectory"),"points");
        object time=Read(points.GetValue(3),"time_from_start");
        Assert.That(Read(time,"sec"),Is.EqualTo(1));Assert.That(Read(time,"nanosec"),Is.EqualTo((uint)1));
    }

    [Test]
    public void RejectsMissingOrDuplicateJointsRatherThanSilentlyDroppingWaypoints()
    {
        object initial=Segment(new[]{"a","b"},Point(new[]{0.0,0.0},0));
        Reject(Solution(initial,Segment(new[]{"a"},Point(new[]{0.0},0))));
        Reject(Solution(initial,Segment(new[]{"a","a"},Point(new[]{0.0,0.0},0))));
        Reject(Solution(initial,Segment(new[]{"a","b"},Point(new[]{0.0},0))));
        Reject(Solution(initial,Segment(new string[0],Point(new[]{0.0},0))));
    }

    [Test]
    public void RejectsNonfiniteIncompleteOrBackwardMotion()
    {
        Reject(Solution(Segment(new[]{"a"},Point(new[]{double.NaN},0))));
        Reject(Solution(Segment(new[]{"a"},Point(new[]{0.0},1),Point(new[]{0.1},0))));
        Reject(Solution(Segment(new[]{"a"},Point(new[]{0.0},0,1000000000))));
        object point=Point(new[]{0.0},0);Set(point,"velocities",new[]{double.PositiveInfinity});
        Reject(Solution(Segment(new[]{"a"},point)));
        Reject(Solution(Empty()));
        object solution=Solution(Segment(new[]{"a"},Point(new[]{0.0},0)));Set(solution,"task_id","");Reject(solution);
    }

    [Test]
    public void ToleratesNumericalMtcSeamsButRejectsUntimedJumps()
    {
        object first=Segment(new[]{"a"},Point(new[]{0.0},0),Point(new[]{0.1},1));
        Assert.That(Flatten(Solution(first,Segment(new[]{"a"},Point(new[]{0.1005},0),Point(new[]{0.2},1)))),Is.Not.Null);
        Reject(Solution(first,Segment(new[]{"a"},Point(new[]{0.11},0),Point(new[]{0.2},1))));
    }

    private static Type TypeOf(string name)=>Type.GetType(name+", Assembly-CSharp",false) ?? Type.GetType(name+", Unity.Robotics.ROSTCPConnector.Messages",true);
    private static object Message(string name,string ns)=>Activator.CreateInstance(TypeOf("RosMessageTypes."+ns+"."+name));
    private static object Read(object value,string field)=>value.GetType().GetField(field).GetValue(value);
    private static void Set(object value,string field,object data)=>value.GetType().GetField(field).SetValue(value,data);
    private static object Empty()=>Message("SubTrajectoryMsg","MoveitTaskConstructor");
    private static object Segment(string[] names,params object[] points)
    {
        object segment=Empty(),joint=Read(Read(segment,"trajectory"),"joint_trajectory");
        Set(joint,"joint_names",names);SetArray(joint,"points",points);return segment;
    }
    private static object Point(double[] positions,int sec,uint nanosec=0)
    {
        object point=Message("JointTrajectoryPointMsg","Trajectory");Set(point,"positions",positions);
        object time=Read(point,"time_from_start");Set(time,"sec",sec);Set(time,"nanosec",nanosec);return point;
    }
    private static object Solution(params object[] segments)
    {
        object solution=Message("SolutionMsg","MoveitTaskConstructor");Set(solution,"task_id","fixture-plan:1");
        SetArray(solution,"sub_trajectory",segments);return solution;
    }
    private static void SetArray(object value,string field,object[] entries)
    {
        Type element=value.GetType().GetField(field).FieldType.GetElementType();Array array=Array.CreateInstance(element,entries.Length);
        for(int i=0;i<entries.Length;i++)array.SetValue(entries[i],i);Set(value,field,array);
    }
    private static object Flatten(object solution)=>TypeOf("FixtureSolutionPreview").GetMethod("Flatten").Invoke(null,new[]{solution});
    private static void Reject(object solution)
    {
        TargetInvocationException exception=Assert.Throws<TargetInvocationException>(()=>Flatten(solution));
        Assert.That(exception.InnerException,Is.TypeOf<ArgumentException>());
    }
}
