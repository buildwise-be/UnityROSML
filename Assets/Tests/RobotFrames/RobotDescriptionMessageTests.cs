using System;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Interoperability with installed ROS2 rcl_interfaces/Fast-CDR bytes, without any ROS connection.</summary>
public sealed class RobotDescriptionMessageTests
{
    [Serializable] private sealed class FixtureCatalog { public string description; public Fixture[] fixtures; }
    [Serializable] private sealed class Fixture
    {
        public string name;
        public string base64;
        public string canonicalBase64;
        public int[] paddingOffsets;
    }

    private FixtureCatalog _catalog;

    [OneTimeSetUp]
    public void LoadInstalledRosFixtures()
    {
        TextAsset asset = AssetDatabase.LoadAssetAtPath<TextAsset>("Assets/Tests/RobotFrames/Fixtures/GetParametersCdr.json");
        Assert.That(asset, Is.Not.Null);
        _catalog = JsonUtility.FromJson<FixtureCatalog>(asset.text);
    }

    [TestCase("request")]
    [TestCase("unicode_request")]
    public void RequestSerializesAndDeserializesLikeInstalledRos(string fixtureName)
    {
        Fixture fixture = _catalog.fixtures.Single(value => value.name == fixtureName);
        Type type = RuntimeType("GetParametersRequest");
        string[] names = fixtureName == "request" ? new[] { "robot_description" } : new[] { "robot_description", "odd", "π" };
        object request = Activator.CreateInstance(type);
        type.GetField("names").SetValue(request, names);
        AssertWireFieldsMatch(fixture, Serialize(request));
        object decoded = Deserialize(type, fixture);
        CollectionAssert.AreEqual(names, (string[])type.GetField("names").GetValue(decoded));
    }

    [Test]
    public void ResponsePreservesXmlAndEveryParameterValueArrayAcrossTheRealCdrLayout()
    {
        Fixture fixture = _catalog.fixtures.Single(value => value.name == "response");
        object decoded = Deserialize(RuntimeType("GetParametersResponse"), fixture);
        Array values = (Array)Read(decoded, "values");
        Assert.That(values.Length, Is.EqualTo(2));
        object description = values.GetValue(0);
        Assert.That(Read(description, "type"), Is.EqualTo((byte)4));
        Assert.That(Read(description, "string_value"), Is.EqualTo(_catalog.description));
        object allFields = values.GetValue(1);
        Assert.That(Read(allFields, "type"), Is.EqualTo((byte)9));
        Assert.That(Read(allFields, "bool_value"), Is.EqualTo(true));
        Assert.That(Read(allFields, "integer_value"), Is.EqualTo(-123456789012345L));
        Assert.That(Read(allFields, "double_value"), Is.EqualTo(-1234.125));
        Assert.That(Read(allFields, "string_value"), Is.EqualTo("alignment_π"));
        CollectionAssert.AreEqual(new byte[] { 0, 1, 127, 128, 255 }, (byte[])Read(allFields, "byte_array_value"));
        CollectionAssert.AreEqual(new[] { true, false, true }, (bool[])Read(allFields, "bool_array_value"));
        CollectionAssert.AreEqual(new[] { long.MinValue, 0L, long.MaxValue }, (long[])Read(allFields, "integer_array_value"));
        CollectionAssert.AreEqual(new[] { 0.0, -0.125, 1e100 }, (double[])Read(allFields, "double_array_value"));
        CollectionAssert.AreEqual(new[] { "", "A", "calibrated_μ", "<robot/>\n" }, (string[])Read(allFields, "string_array_value"));
        AssertWireFieldsMatch(fixture, Serialize(decoded));
    }

    private static Type RuntimeType(string name) => Type.GetType("RosMessageTypes.RclInterfaces." + name + ", Assembly-CSharp", true);
    private static object Read(object value, string name) => value.GetType().GetField(name).GetValue(value);

    private static object Deserialize(Type type, Fixture fixture)
    {
        Type deserializerType = Type.GetType("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageDeserializer, Unity.Robotics.ROSTCPConnector.MessageGeneration", true);
        object deserializer = Activator.CreateInstance(deserializerType);
        deserializerType.GetMethod("InitWithBuffer").Invoke(deserializer, new object[] { Convert.FromBase64String(fixture.base64) });
        return type.GetMethod("Deserialize", BindingFlags.Public | BindingFlags.Static).Invoke(null, new object[] { deserializer });
    }

    private static byte[] Serialize(object message)
    {
        Type serializerType = Type.GetType("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageSerializer, Unity.Robotics.ROSTCPConnector.MessageGeneration", true);
        object serializer = Activator.CreateInstance(serializerType);
        serializerType.GetMethod("SerializeMessage").Invoke(serializer, new[] { message });
        return (byte[])serializerType.GetMethod("GetBytes").Invoke(serializer, null);
    }

    private static void AssertWireFieldsMatch(Fixture fixture, byte[] actual)
    {
        byte[] expected = Convert.FromBase64String(fixture.canonicalBase64);
        Assert.That(actual.Length, Is.EqualTo(expected.Length));
        // CDR does not specify alignment padding contents. Mask exactly the
        // schema-derived gaps recorded while reading the actual ROS payload.
        foreach (int index in fixture.paddingOffsets) actual[index] = 0;
        CollectionAssert.AreEqual(expected, actual);
    }
}
