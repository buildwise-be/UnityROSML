using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur16eInterfaces
{
    /// <summary>Wire order matches ur16e_interfaces/msg/FixtureTilingTask.msg.</summary>
    [Serializable]
    public sealed class FixtureTilingTaskMsg : Message
    {
        public const string k_RosMessageName = "ur16e_interfaces/FixtureTilingTask";
        public override string RosMessageName => k_RosMessageName;
        public string request_id = "";
        public double item_size, item_thickness;
        public Geometry.PoseStampedMsg source_top_pose = new Geometry.PoseStampedMsg();
        public uint source_count, item_count;
        public double stack_pitch;
        public uint columns;
        public double gap, row_offset_fraction;
        public Geometry.PoseStampedMsg placement_corner = new Geometry.PoseStampedMsg();
        public double release_center_x, release_center_y, release_width, release_depth, release_clearance;
        public uint release_yaw_samples;
        public double retreat_distance, settling_delay;
        public bool return_home_each_tile;

        public FixtureTilingTaskMsg() { }
        private FixtureTilingTaskMsg(MessageDeserializer d)
        {
            d.Read(out request_id); d.Read(out item_size); d.Read(out item_thickness);
            source_top_pose = Geometry.PoseStampedMsg.Deserialize(d);
            d.Read(out source_count); d.Read(out item_count); d.Read(out stack_pitch); d.Read(out columns);
            d.Read(out gap); d.Read(out row_offset_fraction);
            placement_corner = Geometry.PoseStampedMsg.Deserialize(d);
            d.Read(out release_center_x); d.Read(out release_center_y); d.Read(out release_width);
            d.Read(out release_depth); d.Read(out release_clearance); d.Read(out release_yaw_samples);
            d.Read(out retreat_distance); d.Read(out settling_delay); d.Read(out return_home_each_tile);
        }
        public static FixtureTilingTaskMsg Deserialize(MessageDeserializer d) => new FixtureTilingTaskMsg(d);
        public override void SerializeTo(MessageSerializer s)
        {
            s.Write(request_id); s.Write(item_size); s.Write(item_thickness); s.Write(source_top_pose);
            s.Write(source_count); s.Write(item_count); s.Write(stack_pitch); s.Write(columns);
            s.Write(gap); s.Write(row_offset_fraction); s.Write(placement_corner);
            s.Write(release_center_x); s.Write(release_center_y); s.Write(release_width); s.Write(release_depth);
            s.Write(release_clearance); s.Write(release_yaw_samples); s.Write(retreat_distance);
            s.Write(settling_delay); s.Write(return_home_each_tile);
        }
#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
