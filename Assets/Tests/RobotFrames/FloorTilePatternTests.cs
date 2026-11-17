using System;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public sealed class FloorTilePatternTests
{
    private static Type PatternType => Type.GetType("FloorTilePattern, Assembly-CSharp", true);

    [TestCase(1, 1, 1)]
    [TestCase(2, 1, 2)]
    [TestCase(3, 2, 2)]
    [TestCase(5, 2, 3)]
    [TestCase(7, 3, 3)]
    [TestCase(10, 3, 4)]
    [TestCase(64, 8, 8)]
    public void LayoutUsesNearlySquareDimensions(int count, int expectedRows, int expectedColumns)
    {
        object pattern = Pattern(count);
        Assert.That(Read<int>(pattern, "Rows"), Is.EqualTo(expectedRows));
        Assert.That(Read<int>(pattern, "Columns"), Is.EqualTo(expectedColumns));
        Assert.That(Read<int>(pattern, "TileCount"), Is.EqualTo(count));
    }

    [TestCase(5, .305f, .305f)]
    [TestCase(15, .315f, .315f)]
    public void MortarSetsPitchOnBothAxesAndAnchorIsAtFloorLevel(int mortar, float x, float z)
    {
        object pattern = Pattern(4, mortar: mortar);
        AssertPoint(Center(pattern, 0), .1f, .0035f, .1f);
        AssertPoint(Center(pattern, 3), x, .0035f, z);
    }

    [TestCase(0, .1f, .1f, .1f)]
    [TestCase(1, .16666667f, .23333333f, .1f)]
    [TestCase(2, .2f, .1f, .2f)]
    public void StaggerRepeatsByRowAndUsesTileWidthWithoutMortar(int offset, float rowOne, float rowTwo, float rowThree)
    {
        object pattern = Pattern(13, offset, 15);
        AssertPoint(Center(pattern, 4), rowOne, .0035f, .315f);
        AssertPoint(Center(pattern, 8), rowTwo, .0035f, .53f);
        AssertPoint(Center(pattern, 12), rowThree, .0035f, .745f);
    }

    [Test]
    public void BoundsIncludeOnlyTheTilesPresentInThePartialLastRow()
    {
        object pattern = Pattern(7, offset: 1, mortar: 15);
        Vector2 center = Read<Vector2>(pattern, "BoundsCenter");
        Assert.That(center.x, Is.EqualTo(.34833333f).Within(1e-6f));
        Assert.That(center.y, Is.EqualTo(.315f).Within(1e-6f));
        AssertPoint(Center(pattern, 6), .23333333f, .0035f, .53f);
        var error = Assert.Throws<TargetInvocationException>(() => Center(pattern, 7));
        Assert.That(error.InnerException, Is.TypeOf<ArgumentOutOfRangeException>());
    }

    [TestCase(0, .2f, .007f, 0, 5)]
    [TestCase(65, .2f, .007f, 0, 5)]
    [TestCase(1, .2f, .007f, -1, 5)]
    [TestCase(1, .2f, .007f, 3, 5)]
    [TestCase(1, .2f, .007f, 0, 4)]
    [TestCase(1, .2f, .007f, 0, 16)]
    [TestCase(1, float.NaN, .007f, 0, 5)]
    [TestCase(1, .2f, float.PositiveInfinity, 0, 5)]
    [TestCase(1, .2f, 0f, 0, 5)]
    public void RejectsUnsupportedPatternInputs(int count, float side, float thickness, int offset, int mortar)
    {
        var error = Assert.Throws<TargetInvocationException>(() =>
            Activator.CreateInstance(PatternType, count, side, thickness, offset, mortar));
        Assert.That(error.InnerException, Is.TypeOf<ArgumentOutOfRangeException>());
    }

    [Test]
    public void RenderPreviewKeepsPhysicalScaleBesideAScaledCanvas()
    {
        var menu = new GameObject("Scaled menu test owner");
        Component visual = null;
        MeshRenderer renderer = null;
        Mesh mesh = null;
        Material material = null;
        const BindingFlags privateInstance = BindingFlags.NonPublic | BindingFlags.Instance;
        Type visualType = Type.GetType("FloorTilePatternVisual, Assembly-CSharp", true);
        try
        {
            menu.transform.localScale = Vector3.one * .001f;
            visual = menu.AddComponent(visualType);
            object pattern = Pattern(5, offset: 1, mortar: 10);
            var menuPose = new Pose(new Vector3(1f, 2f, 3f), Quaternion.Euler(90f, 15f, 0f));
            MethodInfo show = visualType.GetMethod("Show");
            show.Invoke(visual, new object[] { pattern, menuPose, .5f });
            renderer = (MeshRenderer)visualType.GetField("_renderer", privateInstance).GetValue(visual);
            mesh = renderer.GetComponent<MeshFilter>().sharedMesh;
            material = renderer.sharedMaterial;
            Assert.That(mesh.vertexCount, Is.EqualTo(40));
            AssertPoint(mesh.bounds.size, .62f, .007f, .41f);
            AssertPoint(renderer.transform.position, 1f, 2f, 3f);
            AssertPoint(renderer.transform.lossyScale, .5f, .5f, .5f);
            Assert.That(Quaternion.Angle(renderer.transform.rotation, menuPose.rotation), Is.LessThan(.01f));
            Assert.That(renderer.GetComponentInChildren<Collider>(true), Is.Null);

            show.Invoke(visual, new object[] { pattern, new Pose(Vector3.zero, Quaternion.identity), 1f });
            Assert.That(renderer.GetComponent<MeshFilter>().sharedMesh, Is.SameAs(mesh));
            Assert.That(renderer.sharedMaterial, Is.SameAs(material));
            AssertPoint(renderer.bounds.size, .62f, .007f, .41f);
            visualType.GetMethod("Hide").Invoke(visual, null);
            Assert.That(renderer.enabled, Is.False);
        }
        finally
        {
            if (visual != null) visualType.GetMethod("OnDestroy", privateInstance).Invoke(visual, null);
            UnityEngine.Object.DestroyImmediate(menu);
        }
        Assert.That(renderer == null && mesh == null && material == null, Is.True);
    }

    private static object Pattern(int count, int offset = 0, int mortar = 5) =>
        Activator.CreateInstance(PatternType, count, .2f, .007f, offset, mortar);

    private static T Read<T>(object pattern, string name) => (T)PatternType.GetProperty(name).GetValue(pattern);

    private static Vector3 Center(object pattern, int index) =>
        (Vector3)PatternType.GetMethod("GetTileCenter").Invoke(pattern, new object[] { index });

    private static void AssertPoint(Vector3 actual, float x, float y, float z)
    {
        Assert.That(actual.x, Is.EqualTo(x).Within(1e-6f));
        Assert.That(actual.y, Is.EqualTo(y).Within(1e-6f));
        Assert.That(actual.z, Is.EqualTo(z).Within(1e-6f));
    }
}
