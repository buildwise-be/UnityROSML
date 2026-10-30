using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur16eInterfaces
{
    [Serializable]
    public sealed class SaveFixtureCalibrationResponse : Message
    {
        public const string k_RosMessageName = "ur16e_interfaces/SaveFixtureCalibration";
        public override string RosMessageName => k_RosMessageName;
        public bool success;
        public string message = "";
        public SaveFixtureCalibrationResponse() { }
        private SaveFixtureCalibrationResponse(MessageDeserializer d) { d.Read(out success); d.Read(out message); }
        public static SaveFixtureCalibrationResponse Deserialize(MessageDeserializer d) => new SaveFixtureCalibrationResponse(d);
        public override void SerializeTo(MessageSerializer s) { s.Write(success); s.Write(message); }
#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
    }
}
