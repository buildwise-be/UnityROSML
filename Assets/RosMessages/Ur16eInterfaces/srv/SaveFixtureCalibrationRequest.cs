using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Ur16eInterfaces
{
    /// <summary>The four board corners are a fixed ROS array: no array-length prefix.</summary>
    [Serializable]
    public sealed class SaveFixtureCalibrationRequest : Message
    {
        public const string k_RosMessageName = "ur16e_interfaces/SaveFixtureCalibration";
        public override string RosMessageName => k_RosMessageName;
        public Geometry.PointMsg[] board_top_corners = {
            new Geometry.PointMsg(), new Geometry.PointMsg(), new Geometry.PointMsg(), new Geometry.PointMsg() };
        public double board_thickness;
        public Geometry.PoseStampedMsg support_corner = new Geometry.PoseStampedMsg();
        public double margin;
        public SaveFixtureCalibrationRequest() { }
        private SaveFixtureCalibrationRequest(MessageDeserializer d)
        {
            d.Read(out board_top_corners, Geometry.PointMsg.Deserialize, 4);
            d.Read(out board_thickness); support_corner = Geometry.PoseStampedMsg.Deserialize(d); d.Read(out margin);
        }
        public static SaveFixtureCalibrationRequest Deserialize(MessageDeserializer d) => new SaveFixtureCalibrationRequest(d);
        public override void SerializeTo(MessageSerializer s)
        {
            if (board_top_corners == null || board_top_corners.Length != 4 || Array.Exists(board_top_corners, p => p == null))
                throw new InvalidOperationException("Fixture calibration requires exactly four board corners.");
            s.Write(board_top_corners); s.Write(board_thickness); s.Write(support_corner); s.Write(margin);
        }
#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
