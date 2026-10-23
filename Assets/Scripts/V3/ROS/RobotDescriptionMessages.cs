using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

// rcl_interfaces/msg/ParameterValue and srv/GetParameters. Read-only model inspection.
namespace RosMessageTypes.RclInterfaces
{
    [Serializable]
    public sealed class ParameterValueMsg : Message
    {
        public const string k_RosMessageName = "rcl_interfaces/ParameterValue";
        public override string RosMessageName => k_RosMessageName;
        public byte type;
        public bool bool_value;
        public long integer_value;
        public double double_value;
        public string string_value = "";
        public byte[] byte_array_value = Array.Empty<byte>();
        public bool[] bool_array_value = Array.Empty<bool>();
        public long[] integer_array_value = Array.Empty<long>();
        public double[] double_array_value = Array.Empty<double>();
        public string[] string_array_value = Array.Empty<string>();
        public ParameterValueMsg() { }
        private ParameterValueMsg(MessageDeserializer d)
        {
            d.Read(out type); d.Read(out bool_value); d.Read(out integer_value); d.Read(out double_value); d.Read(out string_value);
            d.Read(out byte_array_value, sizeof(byte), d.ReadLength());
            d.Read(out bool_array_value, sizeof(bool), d.ReadLength());
            d.Read(out integer_array_value, sizeof(long), d.ReadLength());
            d.Read(out double_array_value, sizeof(double), d.ReadLength());
            d.Read(out string_array_value, d.ReadLength());
        }
        public static ParameterValueMsg Deserialize(MessageDeserializer d) => new ParameterValueMsg(d);
        public override void SerializeTo(MessageSerializer s)
        {
            s.Write(type); s.Write(bool_value); s.Write(integer_value); s.Write(double_value); s.Write(string_value);
            s.WriteLength(byte_array_value); s.Write(byte_array_value);
            s.WriteLength(bool_array_value); s.Write(bool_array_value);
            s.WriteLength(integer_array_value); s.Write(integer_array_value);
            s.WriteLength(double_array_value); s.Write(double_array_value);
            s.WriteLength(string_array_value); s.Write(string_array_value);
        }
    }

    [Serializable]
    public sealed class GetParametersRequest : Message
    {
        public const string k_RosMessageName = "rcl_interfaces/GetParameters";
        public override string RosMessageName => k_RosMessageName;
        public string[] names = Array.Empty<string>();
        public GetParametersRequest() { }
        public GetParametersRequest(string[] names) { this.names = names; }
        private GetParametersRequest(MessageDeserializer d) { d.Read(out names, d.ReadLength()); }
        public static GetParametersRequest Deserialize(MessageDeserializer d) => new GetParametersRequest(d);
        public override void SerializeTo(MessageSerializer s) { s.WriteLength(names); s.Write(names); }
#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }

    [Serializable]
    public sealed class GetParametersResponse : Message
    {
        public const string k_RosMessageName = "rcl_interfaces/GetParameters";
        public override string RosMessageName => k_RosMessageName;
        public ParameterValueMsg[] values = Array.Empty<ParameterValueMsg>();
        public GetParametersResponse() { }
        private GetParametersResponse(MessageDeserializer d) { d.Read(out values, ParameterValueMsg.Deserialize, d.ReadLength()); }
        public static GetParametersResponse Deserialize(MessageDeserializer d) => new GetParametersResponse(d);
        public override void SerializeTo(MessageSerializer s) { s.WriteLength(values); s.Write(values); }
#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
    }
}
