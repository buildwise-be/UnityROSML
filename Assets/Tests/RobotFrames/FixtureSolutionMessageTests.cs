using System;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>Actual ROS Jazzy CDR from the fixture build, including newer nested message fields.</summary>
public sealed class FixtureSolutionMessageTests
{
    [Serializable] private sealed class Fixture { public string base64, canonicalBase64; public int[] paddingOffsets; public int unusedTrailingBytes; }

    [Test]
    public void SolutionInteroperatesWithThePinnedRosBuildIncludingPolygonAndMarkerExtensions()
    {
        TextAsset asset = Resources.Load<TextAsset>("FixtureSolutionCdr");
        Assert.That(asset, Is.Not.Null);
        Fixture fixture = JsonUtility.FromJson<Fixture>(asset.text);
        byte[] bytes = Convert.FromBase64String(fixture.base64);
        Type readerType = TypeOf("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageDeserializer");
        object reader = Activator.CreateInstance(readerType);
        readerType.GetMethod("InitWithBuffer").Invoke(reader, new object[] { bytes });
        Type type = TypeOf("RosMessageTypes.MoveitTaskConstructor.SolutionMsg");
        object decoded = type.GetMethod("Deserialize", BindingFlags.Public | BindingFlags.Static | BindingFlags.DeclaredOnly).Invoke(null, new[] { reader });
        Assert.That(Read(decoded, "task_id"), Is.EqualTo("fixture-cdr-plan-17"));
        object scene = Read(decoded, "start_scene");
        object tile = ((Array)Read(Read(scene, "world"), "collision_objects")).GetValue(0);
        object primitive = ((Array)Read(tile, "primitives")).GetValue(0);
        Assert.That(((Array)Read(Read(primitive, "polygon"), "points")).Length, Is.EqualTo(2));
        object segment = ((Array)Read(decoded, "sub_trajectory")).GetValue(0);
        Assert.That(Read(Read(segment, "info"), "planner_id"), Is.EqualTo("fixture-cdr-planner"));
        object marker = ((Array)Read(Read(segment, "info"), "markers")).GetValue(0);
        Assert.That(Read(marker, "texture_resource"), Is.EqualTo("embedded://checker"));
        Assert.That(((Array)Read(marker, "uv_coordinates")).Length, Is.EqualTo(1));
        Assert.That(Read(Read(marker, "mesh_file"), "filename"), Is.EqualTo("triangle.stl"));
        Assert.That(Read(marker, "text"), Is.EqualTo("fixture marker after UV fields"));
        Assert.That(Read(Read(segment, "scene_diff"), "name"), Is.EqualTo("after-cdr-settling"));
        Assert.That(readerType.GetField("offset", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(reader), Is.EqualTo(bytes.Length - fixture.unusedTrailingBytes));

        Type writerType = TypeOf("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageSerializer");
        object writer = Activator.CreateInstance(writerType);
        writerType.GetMethod("SerializeMessage").Invoke(writer, new[] { decoded });
        byte[] actual = (byte[])writerType.GetMethod("GetBytes").Invoke(writer, null);
        byte[] expected = Convert.FromBase64String(fixture.canonicalBase64);
        Assert.That(actual.Length, Is.EqualTo(expected.Length));
        foreach (int offset in fixture.paddingOffsets) actual[offset] = 0;
        CollectionAssert.AreEqual(expected, actual, "Only schema-derived alignment padding may differ from ROS CDR.");
    }

    [Test]
    public void PrivateNestedSerializersDoNotReplaceConnectorRegistrations()
    {
        foreach (string name in new[] { "RosMessageTypes.FixtureShape.SolidPrimitiveMsg", "RosMessageTypes.FixtureVisualization.MarkerMsg" })
            Assert.That(TypeOf(name).GetMethod("Register", BindingFlags.Static | BindingFlags.Public), Is.Null);
    }

    private static object Read(object value, string field) => value.GetType().GetField(field).GetValue(value);
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).First(t => t != null);
}
