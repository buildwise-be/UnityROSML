using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur16eInterfaces
{
    /// <summary>Wire contract for PickPlaceTask.msg; keeps the legacy PickPlacePoses type unchanged.</summary>
    [Serializable]
    public sealed class PickPlaceTaskMsg : Message
    {
        public const string k_RosMessageName = "ur16e_interfaces/PickPlaceTask";
        public override string RosMessageName => k_RosMessageName;
        public PickPlacePosesMsg contacts;
        public double cartesian_descent_speed_mps;
        public string item_type;
        public bool pick_from_surface;
        public PointMsg[] top_corners;

        public PickPlaceTaskMsg()
        {
            contacts = new PickPlacePosesMsg();
            cartesian_descent_speed_mps = 0.005;
            item_type = "tile";
            top_corners = new[] { new PointMsg(), new PointMsg(), new PointMsg() };
        }

        public static PickPlaceTaskMsg Deserialize(MessageDeserializer deserializer) => new PickPlaceTaskMsg(deserializer);
        private PickPlaceTaskMsg(MessageDeserializer deserializer)
        {
            contacts = PickPlacePosesMsg.Deserialize(deserializer);
            deserializer.Read(out cartesian_descent_speed_mps);
            deserializer.Read(out item_type);
            deserializer.Read(out pick_from_surface);
            deserializer.Read(out top_corners, PointMsg.Deserialize, 3);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            if (contacts == null || top_corners == null || top_corners.Length != 3 ||
                Array.Exists(top_corners, corner => corner == null))
                throw new InvalidOperationException("PickPlaceTask requires contacts and exactly three corner slots.");
            serializer.Write(contacts);
            serializer.Write(cartesian_descent_speed_mps);
            serializer.Write(item_type);
            serializer.Write(pick_from_surface);
            // Fixed-size ROS arrays carry no sequence-length prefix.
            serializer.Write(top_corners);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
