using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur16eInterfaces
{
    [Serializable]
    public sealed class TileFloorTaskMsg : Message
    {
        public const string k_RosMessageName = "ur16e_interfaces/TileFloorTask";
        public const byte ROW_OFFSET_NONE = 0;
        public const byte ROW_OFFSET_THIRD = 1;
        public const byte ROW_OFFSET_HALF = 2;
        public override string RosMessageName => k_RosMessageName;

        public string request_id = string.Empty;
        public uint timeout_seconds;
        public double tile_side;
        public double tile_thickness;
        public double stack_pitch;
        public bool source_from_surface;
        public PoseStampedMsg source_top = new PoseStampedMsg();
        public PointMsg[] source_top_corners = { new PointMsg(), new PointMsg(), new PointMsg() };
        public PoseStampedMsg floor_corner = new PoseStampedMsg();
        public uint tile_count;
        public byte row_offset;
        public byte mortar_mm;

        public TileFloorTaskMsg() { }

        private TileFloorTaskMsg(MessageDeserializer reader)
        {
            reader.Read(out request_id);
            reader.Read(out timeout_seconds);
            reader.Read(out tile_side);
            reader.Read(out tile_thickness);
            reader.Read(out stack_pitch);
            reader.Read(out source_from_surface);
            source_top = PoseStampedMsg.Deserialize(reader);
            reader.Read(out source_top_corners, PointMsg.Deserialize, 3);
            floor_corner = PoseStampedMsg.Deserialize(reader);
            reader.Read(out tile_count);
            reader.Read(out row_offset);
            reader.Read(out mortar_mm);
        }

        public static TileFloorTaskMsg Deserialize(MessageDeserializer reader) => new TileFloorTaskMsg(reader);

        public override void SerializeTo(MessageSerializer writer)
        {
            if (source_top_corners == null || source_top_corners.Length != 3 ||
                Array.Exists(source_top_corners, point => point == null) || source_top == null || floor_corner == null)
                throw new InvalidOperationException("TileFloorTask requires fixed source capture fields and a destination pose.");
            writer.Write(request_id);
            writer.Write(timeout_seconds);
            writer.Write(tile_side);
            writer.Write(tile_thickness);
            writer.Write(stack_pitch);
            writer.Write(source_from_surface);
            writer.Write(source_top);
            // A fixed-size ROS array has no sequence-length prefix.
            writer.Write(source_top_corners);
            writer.Write(floor_corner);
            writer.Write(tile_count);
            writer.Write(row_offset);
            writer.Write(mortar_mm);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
