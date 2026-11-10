using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur16eInterfaces
{
    [Serializable]
    public sealed class MtcPlanRequestMsg : Message
    {
        public const string k_RosMessageName = "ur16e_interfaces/MtcPlanRequest";
        public const byte PICK = 1, TCP = 2, FIXTURE = 3;
        public override string RosMessageName => k_RosMessageName;

        public byte mode;
        public uint timeout_seconds;
        public PickPlaceTaskMsg pick = new PickPlaceTaskMsg();
        public PoseArrayMsg tcp = new PoseArrayMsg();
        public FixtureTilingTaskMsg fixture = new FixtureTilingTaskMsg();

        public MtcPlanRequestMsg() { }

        private MtcPlanRequestMsg(MessageDeserializer reader)
        {
            reader.Read(out mode);
            reader.Read(out timeout_seconds);
            pick = PickPlaceTaskMsg.Deserialize(reader);
            tcp = PoseArrayMsg.Deserialize(reader);
            fixture = FixtureTilingTaskMsg.Deserialize(reader);
        }

        public static MtcPlanRequestMsg Deserialize(MessageDeserializer reader) => new MtcPlanRequestMsg(reader);

        public override void SerializeTo(MessageSerializer writer)
        {
            writer.Write(mode);
            writer.Write(timeout_seconds);
            writer.Write(pick);
            writer.Write(tcp);
            writer.Write(fixture);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
