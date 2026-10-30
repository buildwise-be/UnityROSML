using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using NUnit.Framework;

/// <summary>Exercises actual ROS messages and object geometry without starting a ROS connection.</summary>
public sealed class FixtureTilingGeometryTests
{
    [Test]
    public void SourceYawAndPitchPreserveTheActualStackCollisionGeometry()
    {
        object settings = Settings();
        Set(settings, "SourceTopPose", Static("SourceTopFrame", Point(0.4, 0.1, 0.8), Math.PI/2));
        Set(settings, "StackPitch", 0.015f);
        object source = Static("SourceObjectPose", settings, 2);
        Assert.That(Position(source, "z"), Is.EqualTo(0.8-0.005-0.03).Within(1e-8));
        object q = Read(Read(source,"pose"),"orientation");
        Assert.That((double)Read(q,"z"), Is.EqualTo(Math.Sqrt(0.5)).Within(1e-12));
        Assert.That(TryRequest(settings), Is.True);
        Set(settings, "StackPitch", 0.005f);
        Assert.That(TryRequest(settings), Is.False, "Adjacent tiles cannot overlap.");
    }

    [Test]
    public void PatternUsesPitchIncludingMortarGapAndAllowsAnIncompleteFinalRow()
    {
        object settings = Settings();
        Set(settings,"ItemCount",5); Set(settings,"SourceCount",6); Set(settings,"Columns",3);
        Assert.That(TryRequest(settings), Is.True);
        object tile = Static("PlacementObjectPose",settings,3);
        Assert.That(Position(tile,"x"),Is.EqualTo(1 + 0.1 + (0.2+0.005)*0.5).Within(1e-8));
        Assert.That(Position(tile,"y"),Is.EqualTo(2 + 0.1 + 0.205).Within(1e-8));
        Assert.That(Position(tile,"z"),Is.EqualTo(0.5-0.005).Within(1e-8), "The input is the final top face, not the support plane.");
        object first = Static("PlacementObjectPose",settings,0);
        Assert.That(Position(first,"x"),Is.EqualTo(1.1).Within(1e-8));
    }

    [Test]
    public void RequestIsAnImmutableSnapshotAndRejectsUnsupportedFramesOrDimensions()
    {
        object settings = Settings();
        object[] arguments = { settings, "fixture:job_1", null, null };
        Assert.That((bool)Geometry.GetMethod("TryBuildRequest").Invoke(null,arguments),Is.True);
        object message = arguments[2];
        object original = Read(settings,"SourceTopPose");
        Set(Read(Read(original,"pose"),"position"),"x",99.0);
        Assert.That(Position(Read(message,"source_top_pose"),"x"), Is.EqualTo(0.4));
        Set(Read(original,"header"),"frame_id","map");
        Assert.That(TryRequest(settings), Is.False);
        settings = Settings(); Set(settings,"ItemThickness",0.3f);
        Assert.That(TryRequest(settings), Is.False);
        settings = Settings(); Set(settings,"ReleaseClearance",float.NaN);
        Assert.That(TryRequest(settings), Is.False);
        Assert.That((bool)Static("ValidToken","fixture job"),Is.False);
    }

    [Test]
    public void SourceRejectsTiltButDestinationRetainsItsOutwardTopFaceFrame()
    {
        object settings = Settings();
        object tilted = Static("FrameFromPoints",Point(1,2,0.5),Point(2,2,1.5),Point(1,3,0.5));
        Set(settings,"SourceTopPose",tilted);
        Assert.That(TryRequest(settings),Is.False);
        Set(settings,"SourceTopPose",Static("SourceTopFrame",Point(0.4,0.1,0.8),0.0));
        Set(settings,"PlacementCorner",tilted);
        Assert.That(TryRequest(settings),Is.True);
        Assert.Throws<TargetInvocationException>(()=> Static("FrameFromPoints",Point(0,0,0),Point(1,0,0),Point(2,0,0)));
    }

    [Test]
    public void CalibrationRequiresTheOrderedBoardAndBothDownhillStops()
    {
        object calibration = Calibration();
        Assert.That(TryCalibration(calibration), Is.True);
        Array corners = (Array)Read(calibration,"BoardTopCorners");
        object p11 = corners.GetValue(2);
        Set(p11,"z",(double)Read(p11,"z")+0.001);
        Assert.That(TryCalibration(calibration), Is.False,"A 1 mm closure error exceeds the ROS 0.1 mm tolerance.");
        calibration = Calibration();
        Set(calibration,"SupportCorner",Static("FrameFromPoints",Point(0,0,0.4),Point(1,0,0.4),Point(0,1,0.4)));
        Assert.That(TryCalibration(calibration), Is.False,"A horizontal or inconsistent alignment frame cannot establish passive capture.");
        calibration = Calibration(); Set(calibration,"Margin",float.PositiveInfinity);
        Assert.That(TryCalibration(calibration),Is.False);
    }

    [Test]
    public void ExplicitBoardFitReportsResidualAndRejectsLargeOrCrossedMeasurements()
    {
        object calibration=Calibration();Array measured=(Array)Read(calibration,"BoardTopCorners");
        object p11=measured.GetValue(2);double original=(double)Read(p11,"z")+0.002;Set(p11,"z",original);
        object[] arguments={measured,0.01f,null,0f,null};
        Assert.That((bool)Geometry.GetMethod("TryFitBoard").Invoke(null,arguments),Is.True);
        Assert.That((float)arguments[3],Is.GreaterThan(0).And.LessThan(0.002f));
        Assert.That(Read(p11,"z"),Is.EqualTo(original),"Fitting must not modify the captured measurements.");
        Set(calibration,"BoardTopCorners",arguments[2]);
        Array fitted=(Array)arguments[2];
        object[] refit={fitted,0.01f,null,0f,null};
        Assert.That((bool)Geometry.GetMethod("TryFitBoard").Invoke(null,refit),Is.True);
        Assert.That((float)refit[3],Is.LessThan(1e-6f));
        Set(p11,"z",original+0.1);
        arguments=new object[]{measured,0.01f,null,0f,null};
        Assert.That((bool)Geometry.GetMethod("TryFitBoard").Invoke(null,arguments),Is.False);
        object swap=measured.GetValue(1);measured.SetValue(measured.GetValue(3),1);measured.SetValue(swap,3);
        arguments=new object[]{measured,0.01f,null,0f,null};
        Assert.That((bool)Geometry.GetMethod("TryFitBoard").Invoke(null,arguments),Is.False);
    }

    [Test]
    public void AlignmentProjectionUsesTheExactFittedPlaneAndRejectsTheWrongAxisOrder()
    {
        object calibration=Calibration(),support=Read(calibration,"SupportCorner");
        object origin=Static("PointInFrame",support,0.0,0.0,0.003);
        object x=Static("PointInFrame",support,0.3,0.0,0.004);
        object y=Static("PointInFrame",support,0.0,0.3,0.002);
        object projected=Static("ProjectFrameToBoard",calibration,origin,x,y);
        Set(calibration,"SupportCorner",projected);
        Assert.That(TryCalibration(calibration),Is.True);
        Assert.That(Position(projected,"z"),Is.EqualTo(0.4).Within(1e-10));
        object backwardsY=Static("PointInFrame",support,0.0,-0.3,0.002);
        Assert.Throws<TargetInvocationException>(()=>Static("ProjectFrameToBoard",calibration,origin,x,backwardsY));
    }

    [Test]
    public void CalibrationFixedArrayMatchesTheRosCdrSchemaWithoutALengthPrefix()
    {
        object request = Activator.CreateInstance(MessageType("SaveFixtureCalibrationRequest"));
        Array corners = (Array)Read(request,"board_top_corners");
        for(int index=0;index<4;index++) corners.SetValue(Point(index+0.125,index+0.25,index+0.5),index);
        Set(request,"board_thickness",0.03125);
        Set(request,"support_corner",Static("SourceTopFrame",Point(4,5,6),0.0));
        Set(request,"margin",0.0078125);
        var expected = new Cdr();
        for(int index=0;index<4;index++){ expected.Double(index+0.125); expected.Double(index+0.25); expected.Double(index+0.5); }
        expected.Double(0.03125); expected.Pose(4,5,6); expected.Double(0.0078125);
        CollectionAssert.AreEqual(expected.Bytes,Serialize(request));
        object decoded=Deserialize(MessageType("SaveFixtureCalibrationRequest"),expected.Bytes);
        Assert.That(((Array)Read(decoded,"board_top_corners")).Length,Is.EqualTo(4));
        Assert.That(Read(decoded,"margin"),Is.EqualTo(0.0078125));
    }

    [Test]
    public void BatchMessageMatchesTheIndependentRosCdrFieldOrder()
    {
        object message=Activator.CreateInstance(MessageType("FixtureTilingTaskMsg"));
        Set(message,"request_id","job:1"); Set(message,"item_size",0.25); Set(message,"item_thickness",0.015625);
        Set(message,"source_top_pose",Static("SourceTopFrame",Point(1,2,3),0.0));
        Set(message,"source_count",(uint)7); Set(message,"item_count",(uint)6); Set(message,"stack_pitch",0.03125);
        Set(message,"columns",(uint)3); Set(message,"gap",0.0078125); Set(message,"row_offset_fraction",0.5);
        Set(message,"placement_corner",Static("SourceTopFrame",Point(4,5,6),0.0));
        string[] fields={"release_center_x","release_center_y","release_width","release_depth","release_clearance"};
        double[] values={0.375,0.5,0.0625,0.125,0.015625};
        for(int i=0;i<fields.Length;i++)Set(message,fields[i],values[i]);
        Set(message,"release_yaw_samples",(uint)4); Set(message,"retreat_distance",0.25); Set(message,"settling_delay",3.0); Set(message,"return_home_each_tile",true);
        var expected=new Cdr(); expected.String("job:1"); expected.Double(0.25); expected.Double(0.015625);
        expected.Pose(1,2,3); expected.UInt(7); expected.UInt(6); expected.Double(0.03125); expected.UInt(3);
        expected.Double(0.0078125); expected.Double(0.5); expected.Pose(4,5,6);
        foreach(double value in values)expected.Double(value);
        expected.UInt(4); expected.Double(0.25); expected.Double(3); expected.Bool(true);
        CollectionAssert.AreEqual(expected.Bytes,Serialize(message));
        object decoded=Deserialize(MessageType("FixtureTilingTaskMsg"),expected.Bytes);
        Assert.That(Read(decoded,"columns"),Is.EqualTo((uint)3));
        Assert.That(Read(decoded,"settling_delay"),Is.EqualTo(3.0));
        Assert.That(Read(decoded,"return_home_each_tile"),Is.EqualTo(true));
    }

    private static object Settings()
    {
        object settings=Activator.CreateInstance(Runtime("FixtureTilingSettings"));
        Set(settings,"SourceTopPose",Static("SourceTopFrame",Point(0.4,0.1,0.8),0.0));
        Set(settings,"PlacementCorner",Static("SourceTopFrame",Point(1,2,0.5),0.0));
        return settings;
    }
    private static object Calibration()
    {
        double a=1/Math.Sqrt(2), b=1/Math.Sqrt(6), c=1/Math.Sqrt(3);
        object value=Activator.CreateInstance(Runtime("FixtureCalibrationData"));
        object support=Static("FrameFromPoints",Point(0,0,0.4),Point(a,-b,0.4+c),Point(0,2*b,0.4+c));
        Set(value,"SupportCorner",support);
        Array points=Array.CreateInstance(MessageType("PointMsg","Geometry"),4);
        double[] xx={-0.01,0.79,0.79,-0.01}, yy={-0.01,-0.01,0.79,0.79};
        for(int i=0;i<4;i++)points.SetValue(Static("PointInFrame",support,xx[i],yy[i],0.0),i);
        Set(value,"BoardTopCorners",points);
        return value;
    }
    private static bool TryRequest(object settings)
    { object[] args={settings,"job:1",null,null}; return (bool)Geometry.GetMethod("TryBuildRequest").Invoke(null,args); }
    private static bool TryCalibration(object data)
    { object[] args={data,null,null}; return (bool)Geometry.GetMethod("TryBuildCalibration").Invoke(null,args); }
    private static Type Runtime(string name)=>Type.GetType(name+", Assembly-CSharp",true);
    private static Type Geometry=>Runtime("FixtureTilingGeometry");
    private static Type MessageType(string name,string ns="Ur16eInterfaces")=>Type.GetType("RosMessageTypes."+ns+"."+name+", Assembly-CSharp",false) ?? Type.GetType("RosMessageTypes."+ns+"."+name+", Unity.Robotics.ROSTCPConnector.Messages",true);
    private static object Point(double x,double y,double z)=>Activator.CreateInstance(MessageType("PointMsg","Geometry"),x,y,z);
    private static object Static(string method,params object[] arguments)=>Geometry.GetMethod(method).Invoke(null,arguments);
    private static object Read(object value,string name)=>value.GetType().GetField(name).GetValue(value);
    private static void Set(object value,string name,object field)=>value.GetType().GetField(name).SetValue(value,field);
    private static double Position(object frame,string axis)=>(double)Read(Read(Read(frame,"pose"),"position"),axis);
    private static byte[] Serialize(object message)
    {
        Type type=Type.GetType("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageSerializer, Unity.Robotics.ROSTCPConnector.MessageGeneration",true);
        object serializer=Activator.CreateInstance(type);type.GetMethod("SerializeMessage").Invoke(serializer,new[]{message});
        return (byte[])type.GetMethod("GetBytes").Invoke(serializer,null);
    }
    private static object Deserialize(Type messageType,byte[] bytes)
    {
        Type type=Type.GetType("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageDeserializer, Unity.Robotics.ROSTCPConnector.MessageGeneration",true);
        object deserializer=Activator.CreateInstance(type);type.GetMethod("InitWithBuffer").Invoke(deserializer,new object[]{bytes});
        return messageType.GetMethod("Deserialize",BindingFlags.Public|BindingFlags.Static|BindingFlags.DeclaredOnly).Invoke(null,new[]{deserializer});
    }

    // Independent little-endian CDR oracle, based on the ROS .msg/.srv definitions.
    private sealed class Cdr
    {
        private readonly MemoryStream _stream=new MemoryStream();
        private readonly BinaryWriter _writer;
        public Cdr(){_writer=new BinaryWriter(_stream);_writer.Write(new byte[]{0,1,0,0});}
        public byte[] Bytes=>_stream.ToArray();
        private void Align(int size){while((_stream.Length-4)%size!=0)_writer.Write((byte)0);}
        public void UInt(uint value){Align(4);_writer.Write(value);}
        public void Double(double value){Align(8);_writer.Write(value);}
        public void Bool(bool value)=>_writer.Write(value);
        public void String(string value){byte[] bytes=Encoding.UTF8.GetBytes(value);UInt((uint)bytes.Length+1);_writer.Write(bytes);_writer.Write((byte)0);}
        public void Pose(double x,double y,double z){UInt(0);UInt(0);String("ur_base_link");Double(x);Double(y);Double(z);Double(0);Double(0);Double(0);Double(1);}
    }
}
