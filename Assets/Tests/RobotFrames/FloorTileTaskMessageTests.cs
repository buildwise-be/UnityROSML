using System;
using System.Reflection;
using NUnit.Framework;

public sealed class FloorTileTaskMessageTests
{
    // Little-endian ROS 2 CDR packed independently from TileFloorTask.msg.
    private const string FloorWire =
        "AAEAAA0AAABmbG9vcjp3aXJlLTcAAAAAJQAAAAAAAAAAAPA/AAAAAAAAAEAAAAAAAAAIQAAAAAAAABBAAAAAAAAAFEAAAAAAAAAYQAAAAAAAABxAAAAAAAAAIEAAAAAAAAAiQAAAAAAAAAAADQAAAHVyX2Jhc2VfbGluawAAAAAAAAAAAAAAAAAAJEAAAAAAAAAmQAAAAAAAAChAAAAAAAAAAAAAAAAAAAAAADMzMzMzM+M/mpmZmZmZ6T8NAAAAAg8=";

    [Test]
    public void FloorTaskCdrHasThreeUnprefixedCornersAndAByteForEachPatternSetting()
    {
        byte[] wire = Convert.FromBase64String(FloorWire);
        Type readerType = ConnectorType("MessageDeserializer");
        object reader = Activator.CreateInstance(readerType);
        readerType.GetMethod("InitWithBuffer").Invoke(reader, new object[] { wire });
        Type taskType = Type.GetType("RosMessageTypes.Ur16eInterfaces.TileFloorTaskMsg, Assembly-CSharp", true);
        object task = taskType.GetMethod("Deserialize", BindingFlags.Public | BindingFlags.Static | BindingFlags.DeclaredOnly)
            .Invoke(null, new[] { reader });
        Assert.That(Field(task, "request_id"), Is.EqualTo("floor:wire-7"));
        Assert.That(Field(task, "timeout_seconds"), Is.EqualTo(37u));
        var corners = (Array)Field(task, "source_top_corners");
        Assert.That(corners.Length, Is.EqualTo(3));
        Assert.That(Field(corners.GetValue(0), "x"), Is.EqualTo(1d));
        Assert.That(Field(corners.GetValue(2), "z"), Is.EqualTo(9d));
        Assert.That(Field(Field(Field(task, "floor_corner"), "header"), "frame_id"), Is.EqualTo("ur_base_link"));
        Assert.That(Field(task, "tile_count"), Is.EqualTo(13u));
        Assert.That(Field(task, "row_offset"), Is.EqualTo((byte)2));
        Assert.That(Field(task, "mortar_mm"), Is.EqualTo((byte)15));
        Assert.That(readerType.GetField("offset", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(reader),
            Is.EqualTo(wire.Length));

        Type writerType = ConnectorType("MessageSerializer");
        object writer = Activator.CreateInstance(writerType);
        writerType.GetMethod("SerializeMessage").Invoke(writer, new[] { task });
        byte[] encoded = (byte[])writerType.GetMethod("GetBytes").Invoke(writer, null);
        CollectionAssert.AreEqual(wire, encoded);
    }

    private static object Field(object value, string name) => value.GetType().GetField(name).GetValue(value);

    private static Type ConnectorType(string name) => Type.GetType(
        "Unity.Robotics.ROSTCPConnector.MessageGeneration." + name + ", Unity.Robotics.ROSTCPConnector.MessageGeneration", true);
}
