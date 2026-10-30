using System;
using System.Collections.Generic;
using System.Globalization;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.MoveitTaskConstructor;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>Measured inventory ghosts and an ordered, identified scene-effect review. No physics simulation.</summary>
public sealed class FixtureTilingPreview : MonoBehaviour
{
    private GameObject _root;
    private readonly List<Material> _materials = new List<Material>();
    private readonly List<Mesh> _meshes = new List<Mesh>();
    private FixtureTilingSettings _settings;
    private FixtureCalibrationData _calibration;
    private PlanningSceneMsg _initial;
    private SolutionMsg _solution;
    private RosRobotModel _model;
    private readonly List<SceneState> _states = new List<SceneState>();
    private bool _show;
    private bool _reviewValidated;
    public bool HasReview => _reviewValidated && _initial != null && _solution != null && _root != null && _root.activeSelf;
    public int StepCount => _solution?.sub_trajectory != null ? _solution.sub_trajectory.Length + 1 : 0;
    public int StepIndex { get; private set; }
    public string StepText { get; private set; } = "Setup ghosts; dimensions and captured points are in robot coordinates.";
    public float StepMotionTime { get; private set; }
    private static readonly Color SourceColor = new Color(0.1f, 0.9f, 0.3f, 0.35f);
    private static readonly Color DestinationColor = new Color(1f, 0.6f, 0.05f, 0.35f);
    private static readonly Color BoardColor = new Color(0.15f, 0.65f, 1f, 0.3f);

    public void ShowSetup(FixtureTilingSettings settings, FixtureCalibrationData calibration, bool visible)
    {
        _settings = settings; _calibration = calibration; _show = visible;
        if (_solution == null) DrawSetupSafely();
        if (_root != null) _root.SetActive(visible && RobotCoordinateService.Instance.IsInitialized);
    }
    public void ClearReview()
    {
        _reviewValidated = false; _initial = null; _solution = null; _model = null; StepIndex = 0; StepMotionTime = 0; _states.Clear();
        DrawSetupSafely();
    }
    public void SetReview(PlanningSceneMsg initial, SolutionMsg solution, FixtureTilingSettings settings, RosRobotModel model)
    {
        _reviewValidated = false;
        if (initial == null || initial.is_diff || initial.world == null || initial.robot_state == null ||
            solution?.sub_trajectory == null || solution.sub_trajectory.Length == 0 || initial.name != solution.task_id)
            throw new ArgumentException("A matching full initial scene and ordered solution are required.");
        _initial = initial; _solution = solution; _settings = settings; _model = model; _show = true;
        StepIndex = 0;
        // Validate every state before the batch becomes executable. A malformed
        // scene later in the sequence must not hide behind a valid first frame.
        BuildStates();
        DrawReview();
        _reviewValidated = true;
    }
    public void Step(int delta)
    {
        if (_solution == null) return;
        StepIndex = Mathf.Clamp(StepIndex + delta, 0, StepCount - 1);
        DrawReview();
    }
    private void LateUpdate()
    {
        if (_root == null) return;
        bool valid = RobotCoordinateService.Instance.TryGetRobotBaseWorldPose(out Pose frame);
        _root.SetActive(_show && valid);
        if (valid) _root.transform.SetPositionAndRotation(frame.position, frame.rotation);
    }
    private void ResetRoot(string title)
    {
        ClearObjects();
        _root = new GameObject(title);
        if (RobotCoordinateService.Instance.TryGetRobotBaseWorldPose(out Pose frame))
            _root.transform.SetPositionAndRotation(frame.position, frame.rotation);
        _root.SetActive(_show && RobotCoordinateService.Instance.IsInitialized);
    }
    private void DrawSetup()
    {
        ResetRoot("Fixture tiling — measured setup ghosts");
        StepText = "Setup ghosts: green source; orange destination. Square top face, centred suction.";
        if (_settings == null) return;
        if (FixtureTilingGeometry.TryBuildRequest(_settings, "ghost", out _, out _))
        {
            for (int i = 0; i < _settings.SourceCount; i++)
                Box("Source tile " + (i+1), Local(FixtureTilingGeometry.SourceObjectPose(_settings,i).pose),
                    new Vector3(_settings.ItemSize,_settings.ItemThickness,_settings.ItemSize), SourceColor);
            for (int i = 0; i < _settings.ItemCount; i++)
                Box("Destination tile " + (i+1), Local(FixtureTilingGeometry.PlacementObjectPose(_settings,i).pose),
                    new Vector3(_settings.ItemSize,_settings.ItemThickness,_settings.ItemSize), DestinationColor);
        }
        else if (_settings.SourceTopPose != null && FinitePositive(_settings.ItemSize) && FinitePositive(_settings.ItemThickness))
        {
            // The source can be inspected before the placement frame is captured.
            for (int i = 0; i < Mathf.Clamp(_settings.SourceCount,0,64); i++)
                Box("Source tile " + (i+1), Local(FixtureTilingGeometry.SourceObjectPose(_settings,i).pose),
                    new Vector3(_settings.ItemSize,_settings.ItemThickness,_settings.ItemSize), SourceColor);
        }
        if (_settings.SourceTopPose != null) Label("Source top centre", Local(_settings.SourceTopPose.pose).position, Color.green);
        if (_settings.PlacementCorner != null) Frame("Pattern top-face corner", _settings.PlacementCorner);
        if (_calibration == null) return;
        if (_calibration.BoardTopCorners?.Length == 4)
        {
            bool all = true;
            for (int i = 0; i < 4; i++)
            {
                PointMsg p = _calibration.BoardTopCorners[i];
                if (p == null) { all = false; continue; }
                Label(new[] {"P00","P10","P11","P01"}[i], p.From<FLU>(), Color.cyan);
            }
            if (all)
            {
                Vector3[] p = Array.ConvertAll(_calibration.BoardTopCorners, value => value.From<FLU>());
                // FLU->Unity reverses handedness; use the ROS-derived outward axis.
                Vector3 normal = -Vector3.Cross(p[1]-p[0],p[3]-p[0]).normalized;
                Vector3[] vertices = new Vector3[8];
                for (int i=0;i<4;i++) { vertices[i]=p[i]; vertices[i+4]=p[i]-normal*_calibration.BoardThickness; }
                Mesh board = new Mesh { name="Measured fixture plank" };
                board.vertices=vertices;
                board.triangles=new[] {0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7};
                board.RecalculateNormals(); _meshes.Add(board);
                MeshObject("Measured plank",Pose.identity,board,BoardColor);
            }
        }
        if (_calibration.SupportCorner != null)
        {
            Frame("Alignment support corner",_calibration.SupportCorner);
            double x=_settings.ReleaseCenterX,y=_settings.ReleaseCenterY,w=_settings.ReleaseWidth/2,d=_settings.ReleaseDepth/2;
            PointMsg[] points={FixtureTilingGeometry.PointInFrame(_calibration.SupportCorner,x-w,y-d,0),
                FixtureTilingGeometry.PointInFrame(_calibration.SupportCorner,x+w,y-d,0),
                FixtureTilingGeometry.PointInFrame(_calibration.SupportCorner,x+w,y+d,0),
                FixtureTilingGeometry.PointInFrame(_calibration.SupportCorner,x-w,y+d,0)};
            Line("Release tile-centre region", Array.ConvertAll(points,p=>p.From<FLU>()), Color.magenta, true);
            PoseStampedMsg rest=FixtureTilingGeometry.OffsetPose(_calibration.SupportCorner,_settings.ItemSize/2,_settings.ItemSize/2,_settings.ItemThickness/2);
            Box("Assumed aligned rest tile",Local(rest.pose),new Vector3(_settings.ItemSize,_settings.ItemThickness,_settings.ItemSize),DestinationColor);
        }
    }
    private void DrawSetupSafely()
    {
        try { DrawSetup(); }
        catch (ArgumentException error)
        {
            ResetRoot("Fixture tiling — incomplete measured setup");
            StepText="Complete the measured setup: "+error.Message;
        }
    }

    private sealed class SceneState
    {
        public readonly Dictionary<string,CollisionObjectMsg> World=new Dictionary<string,CollisionObjectMsg>();
        public readonly Dictionary<string,AttachedCollisionObjectMsg> Attached=new Dictionary<string,AttachedCollisionObjectMsg>();
        public readonly Dictionary<string,double> Joints=new Dictionary<string,double>();
        public readonly Dictionary<string,Pose> Frames=new Dictionary<string,Pose>();
        public float MotionTime;
    }
    private void BuildStates()
    {
        _states.Clear();
        SceneState state=new SceneState();
        ApplyScene(state,_initial);
        CaptureState(state);
        foreach(var sub in _solution.sub_trajectory)
        {
            if(sub==null) throw new ArgumentException("A batch contains an empty step.");
            StepDescription(sub.info?.comment);
            var jt=sub.trajectory?.joint_trajectory;
            if(jt?.points!=null && jt.points.Length>0)
            {
                var last=jt.points[jt.points.Length-1];
                for(int j=0;j<jt.joint_names.Length;j++) state.Joints[jt.joint_names[j]]=last.positions[j];
                state.MotionTime += last.time_from_start.sec + last.time_from_start.nanosec*1e-9f;
            }
            if(sub.scene_diff!=null) ApplyScene(state,sub.scene_diff);
            CaptureState(state);
        }
    }
    private void CaptureState(SceneState source)
    {
        var state=new SceneState {MotionTime=source.MotionTime};
        foreach(var p in source.World) state.World.Add(p.Key,p.Value);
        foreach(var p in source.Attached) state.Attached.Add(p.Key,p.Value);
        foreach(var p in source.Joints) state.Joints.Add(p.Key,p.Value);
        foreach(var p in source.Frames) state.Frames.Add(p.Key,p.Value);
        if(_model!=null)
            foreach(var pair in RosRobotModelMath.ForwardKinematics(_model,state.Joints)) state.Frames[pair.Key]=pair.Value;
        state.Frames["ur_base_link"]=Pose.identity;
        foreach(var pair in state.World) ValidateObject(pair.Value,null,state.Frames);
        foreach(var pair in state.Attached) ValidateObject(pair.Value.@object,pair.Value.link_name,state.Frames);
        _states.Add(state);
    }
    private static void ApplyScene(SceneState state,PlanningSceneMsg scene)
    {
        if(scene==null) return;
        if(scene.world?.octomap?.octomap?.data!=null && scene.world.octomap.octomap.data.Length>0)
            throw new ArgumentException("This batch scene contains an octomap that Unity cannot display. A complete fixture review is required.");
        if(!scene.is_diff) { state.World.Clear(); state.Attached.Clear(); state.Joints.Clear(); state.Frames.Clear(); }
        state.Frames["ur_base_link"]=Pose.identity;
        if(scene.fixed_frame_transforms!=null)
            for(int pass=0;pass<scene.fixed_frame_transforms.Length+1;pass++)
                foreach(var t in scene.fixed_frame_transforms)
                {
                    Pose transform=new Pose(t.transform.translation.From<FLU>(),t.transform.rotation.From<FLU>());
                    if(state.Frames.TryGetValue(t.header.frame_id,out Pose parent))
                        state.Frames[t.child_frame_id]=RobotFrameMath.Compose(parent,transform);
                    else if(state.Frames.TryGetValue(t.child_frame_id,out Pose child))
                        state.Frames[t.header.frame_id]=RobotFrameMath.Compose(child,Inverse(transform));
                }
        if(scene.world?.collision_objects!=null)
            foreach(var obj in scene.world.collision_objects)
            {
                if(obj.operation==CollisionObjectMsg.REMOVE) state.World.Remove(obj.id);
                else if(obj.operation==CollisionObjectMsg.ADD) state.World[obj.id]=obj;
                else throw new ArgumentException("Unsupported world scene operation in fixture review.");
            }
        var robot=scene.robot_state;
        if(robot==null) return;
        var joints=robot.joint_state;
        if(joints?.name!=null && joints.position!=null)
            for(int i=0;i<Math.Min(joints.name.Length,joints.position.Length);i++) state.Joints[joints.name[i]]=joints.position[i];
        if(!robot.is_diff) state.Attached.Clear();
        if(robot.attached_collision_objects!=null)
            foreach(var attached in robot.attached_collision_objects)
            {
                if(attached.@object.operation==CollisionObjectMsg.REMOVE) state.Attached.Remove(attached.@object.id);
                else if(attached.@object.operation==CollisionObjectMsg.ADD)
                { state.Attached[attached.@object.id]=attached; state.World.Remove(attached.@object.id); }
                else throw new ArgumentException("Unsupported attached scene operation in fixture review.");
            }
    }
    private void DrawReview()
    {
        SceneState state=_states[StepIndex];
        StepMotionTime=state.MotionTime;
        ResetRoot("Fixture batch — identified scene review");
        foreach(var pair in state.World) DrawObject(pair.Value,null,state.Frames);
        foreach(var pair in state.Attached) DrawObject(pair.Value.@object,pair.Value.link_name,state.Frames);
        string comment=StepIndex==0 ? "Initial stack and plank" : StepDescription(_solution.sub_trajectory[StepIndex-1].info?.comment);
        StepText=(StepIndex+1)+" / "+StepCount+": "+(string.IsNullOrEmpty(comment)?"Motion / scene update":comment)+
            " · "+state.Attached.Count+" attached. Settling is assumed after the timed wait.";
    }
    private static string StepDescription(string comment)
    {
        if(string.IsNullOrEmpty(comment)) return "Motion / scene update";
        if(comment.StartsWith("WAIT_MARKER ",StringComparison.Ordinal))
            return "Wait for settling ("+MarkerNumber(comment,"seconds").ToString("0.###",CultureInfo.InvariantCulture)+" s)";
        if(comment.StartsWith("IO_MARKER ",StringComparison.Ordinal))
        {
            // Both supported Kenos definitions use 1=on, 0=off.
            double state=MarkerNumber(comment,"state"),delay=MarkerNumber(comment,"post_delay");
            string action=state==1?"Vacuum on":state==0?"Vacuum off":"Vacuum change";
            return action+(delay>0?"; release hold "+delay.ToString("0.###",CultureInfo.InvariantCulture)+" s":"");
        }
        return comment;
    }
    private static double MarkerNumber(string comment,string key)
    {
        foreach(string token in comment.Split(' '))
            if(token.StartsWith(key+"=",StringComparison.Ordinal) &&
                double.TryParse(token.Substring(key.Length+1),NumberStyles.Float,CultureInfo.InvariantCulture,out double value) && double.IsFinite(value)) return value;
        throw new ArgumentException("The batch contains an unreadable vacuum or settling step.");
    }
    private static void ValidateObject(CollisionObjectMsg obj,string attachedLink,Dictionary<string,Pose> frames)
    {
        string frame=string.IsNullOrEmpty(obj.header?.frame_id)?attachedLink:obj.header.frame_id;
        if(frame==null || !frames.TryGetValue(frame,out Pose framePose) || !RobotFrameMath.IsValidPose(framePose))
            throw new ArgumentException("Cannot display collision frame: "+frame+" ("+obj.id+").");
        Local(obj.pose);
        if(obj.primitives!=null && (obj.primitive_poses==null || obj.primitives.Length!=obj.primitive_poses.Length))
            throw new ArgumentException("Collision primitive poses are incomplete.");
        if(obj.meshes!=null && (obj.mesh_poses==null || obj.meshes.Length!=obj.mesh_poses.Length))
            throw new ArgumentException("Collision mesh poses are incomplete.");
        if(obj.planes!=null && obj.planes.Length>0)
            throw new ArgumentException("Infinite collision planes are not supported by this scene review; model the support as a finite collision box.");
        if(obj.primitives!=null)
            for(int i=0;i<obj.primitives.Length;i++)
            {
                var primitive=obj.primitives[i];Local(obj.primitive_poses[i]);
                if(primitive?.dimensions==null || primitive.type<1 || primitive.type>4 ||
                    primitive.dimensions.Length!=(primitive.type==SolidPrimitiveMsg.BOX?3:primitive.type==SolidPrimitiveMsg.SPHERE?1:2))
                    throw new ArgumentException("Unsupported collision primitive in batch preview.");
                foreach(double dimension in primitive.dimensions)
                    if(!double.IsFinite(dimension) || dimension<=0) throw new ArgumentException("Invalid collision dimensions.");
            }
        if(obj.meshes!=null)
            for(int i=0;i<obj.meshes.Length;i++)
            {
                Local(obj.mesh_poses[i]);var mesh=obj.meshes[i];
                if(mesh?.vertices==null || mesh.triangles==null) throw new ArgumentException("Incomplete collision mesh.");
                foreach(var vertex in mesh.vertices)
                    if(vertex==null || !double.IsFinite(vertex.x) || !double.IsFinite(vertex.y) || !double.IsFinite(vertex.z))
                        throw new ArgumentException("Invalid collision mesh vertex.");
                foreach(var triangle in mesh.triangles)
                {
                    if(triangle?.vertex_indices==null || triangle.vertex_indices.Length!=3) throw new ArgumentException("Invalid collision mesh triangle.");
                    foreach(uint index in triangle.vertex_indices) if(index>=mesh.vertices.Length) throw new ArgumentException("Collision mesh index exceeds its vertices.");
                }
            }
    }
    private void DrawObject(CollisionObjectMsg obj,string attachedLink,Dictionary<string,Pose> frames)
    {
        string frame=string.IsNullOrEmpty(obj.header?.frame_id)?attachedLink:obj.header.frame_id;
        if(frame==null || !frames.TryGetValue(frame,out Pose parent))
            throw new ArgumentException("Cannot display collision frame: "+frame+" ("+obj.id+").");
        Pose origin=RobotFrameMath.Compose(parent,Local(obj.pose));
        Color color=obj.id=="fixture_plank"?BoardColor:attachedLink!=null?Color.cyan:
            obj.id.StartsWith("fixture_",StringComparison.Ordinal)?DestinationColor:new Color(.6f,.65f,.7f,.25f);
        if(obj.primitives!=null)
            for(int i=0;i<obj.primitives.Length;i++)
            {
                if(obj.primitive_poses==null || i>=obj.primitive_poses.Length) throw new ArgumentException("Missing primitive pose.");
                var shape=obj.primitives[i];
                Pose pose=RobotFrameMath.Compose(origin,Local(obj.primitive_poses[i]));
                if(shape.type==SolidPrimitiveMsg.BOX && shape.dimensions.Length==3)
                    Box(obj.id,pose,new Vector3((float)shape.dimensions[1],(float)shape.dimensions[2],(float)shape.dimensions[0]),color);
                else if(shape.type==SolidPrimitiveMsg.SPHERE && shape.dimensions.Length==1)
                    Primitive(obj.id,PrimitiveType.Sphere,pose,Vector3.one*(float)shape.dimensions[0]*2,color);
                else if(shape.type==SolidPrimitiveMsg.CYLINDER && shape.dimensions.Length==2)
                    Primitive(obj.id,PrimitiveType.Cylinder,pose,new Vector3((float)shape.dimensions[1]*2,(float)shape.dimensions[0]/2,(float)shape.dimensions[1]*2),color);
                else if(shape.type==SolidPrimitiveMsg.CONE && shape.dimensions.Length==2)
                    MeshObject(obj.id,pose,Cone((float)shape.dimensions[0],(float)shape.dimensions[1]),color);
                else throw new ArgumentException("Unsupported collision primitive in batch preview.");
            }
        if(obj.meshes!=null)
            for(int i=0;i<obj.meshes.Length;i++)
            {
                var data=obj.meshes[i]; var triangles=new List<int>();
                foreach(var triangle in data.triangles)
                { triangles.Add((int)triangle.vertex_indices[0]);triangles.Add((int)triangle.vertex_indices[2]);triangles.Add((int)triangle.vertex_indices[1]); }
                Mesh mesh=new Mesh {name=obj.id,indexFormat=UnityEngine.Rendering.IndexFormat.UInt32};
                mesh.vertices=Array.ConvertAll(data.vertices,p=>p.From<FLU>());mesh.triangles=triangles.ToArray();mesh.RecalculateNormals();_meshes.Add(mesh);
                MeshObject(obj.id,RobotFrameMath.Compose(origin,Local(obj.mesh_poses[i])),mesh,color);
            }
    }
    private Mesh Cone(float height,float radius)
    {
        const int count=32;
        var vertices=new Vector3[count+2];vertices[0]=Vector3.up*height/2;vertices[1]=Vector3.down*height/2;
        var triangles=new List<int>();
        for(int i=0;i<count;i++) vertices[i+2]=new Vector3(Mathf.Cos(i*2*Mathf.PI/count)*radius,-height/2,Mathf.Sin(i*2*Mathf.PI/count)*radius);
        for(int i=0;i<count;i++)
        {
            int current=i+2,next=(i+1)%count+2;
            triangles.Add(0);triangles.Add(next);triangles.Add(current);
            triangles.Add(1);triangles.Add(current);triangles.Add(next);
        }
        var mesh=new Mesh {name="Collision cone"};mesh.vertices=vertices;mesh.triangles=triangles.ToArray();mesh.RecalculateNormals();_meshes.Add(mesh);return mesh;
    }
    private static Pose Inverse(Pose pose) { Quaternion q=Quaternion.Inverse(pose.rotation);return new Pose(q*-pose.position,q); }
    private static Pose Local(PoseMsg pose)
    {
        // MoveIt sometimes emits an all-zero quaternion for a default empty pose.
        QuaternionMsg q=pose?.orientation;
        if(pose==null) return Pose.identity;
        Quaternion rotation=q==null || q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1e-20?Quaternion.identity:q.From<FLU>();
        Pose result=new Pose(pose.position.From<FLU>(),rotation);
        if(!RobotFrameMath.IsValidPose(result)) throw new ArgumentException("Collision object pose is invalid.");
        return result;
    }
    private void Frame(string title,PoseStampedMsg pose)
    {
        Pose p=Local(pose.pose);Label(title,p.position,Color.white);
        Line("+X",new[]{p.position,FixtureTilingGeometry.PointInFrame(pose,.1,0,0).From<FLU>()},Color.red,false);
        Line("+Y",new[]{p.position,FixtureTilingGeometry.PointInFrame(pose,0,.1,0).From<FLU>()},Color.green,false);
    }
    private void Label(string title,Vector3 point,Color color)
    {
        GameObject marker=Primitive(title,PrimitiveType.Sphere,new Pose(point,Quaternion.identity),Vector3.one*.012f,color);
        TargetNameLabel.Attach(marker.transform,title,color);
    }
    private void Box(string title,Pose pose,Vector3 scale,Color color) => Primitive(title,PrimitiveType.Cube,pose,scale,color);
    private GameObject Primitive(string title,PrimitiveType type,Pose pose,Vector3 scale,Color color)
    {
        GameObject obj=GameObject.CreatePrimitive(type);obj.name=title;
        Collider collider=obj.GetComponent<Collider>();if(collider!=null) {collider.enabled=false;Dispose(collider);}
        SetTransform(obj,pose);obj.transform.localScale=scale;
        obj.GetComponent<Renderer>().sharedMaterial=Material(color);
        return obj;
    }
    private void MeshObject(string title,Pose pose,Mesh mesh,Color color)
    {
        GameObject obj=new GameObject(title,typeof(MeshFilter),typeof(MeshRenderer));SetTransform(obj,pose);
        obj.GetComponent<MeshFilter>().sharedMesh=mesh;obj.GetComponent<Renderer>().sharedMaterial=Material(color);
    }
    private void Line(string title,Vector3[] points,Color color,bool loop)
    {
        GameObject obj=new GameObject(title);obj.transform.SetParent(_root.transform,false);
        var line=obj.AddComponent<LineRenderer>();line.useWorldSpace=false;line.loop=loop;line.positionCount=points.Length;line.SetPositions(points);
        line.startWidth=line.endWidth=.004f;line.sharedMaterial=Material(color);
    }
    private Material Material(Color color) {var material=TargetMarkerMaterials.Create(color,color.a<.99f);_materials.Add(material);return material;}
    private void SetTransform(GameObject obj,Pose pose) {obj.transform.SetParent(_root.transform,false);obj.transform.SetLocalPositionAndRotation(pose.position,pose.rotation);}
    private static bool FinitePositive(float value)=>!float.IsNaN(value)&&!float.IsInfinity(value)&&value>0;
    private void ClearObjects()
    {
        if(_root!=null) {_root.SetActive(false);Dispose(_root);}
        foreach(var material in _materials) Dispose(material);_materials.Clear();
        foreach(var mesh in _meshes) Dispose(mesh);_meshes.Clear();
    }
    private static void Dispose(UnityEngine.Object value) {if(value!=null) {if(Application.isPlaying) Destroy(value);else DestroyImmediate(value);}}
    private void OnDestroy()=>ClearObjects();
}
