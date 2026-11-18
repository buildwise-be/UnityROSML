using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public sealed class PickSurfaceGuideAnimationTests
{
    private const BindingFlags Instance = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    private readonly List<GameObject> _previousMainCameras = new List<GameObject>();
    private GameObject _viewer, _owner;
    private Component _guide;

    [SetUp]
    public void OpenGuideWithAnUnambiguousViewer()
    {
        foreach (Camera camera in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (!camera.CompareTag("MainCamera")) continue;
            _previousMainCameras.Add(camera.gameObject);
            camera.tag = "Untagged";
        }
        _viewer = new GameObject("Item-guide viewer", typeof(Camera)) { tag = "MainCamera" };
        _viewer.transform.position = new Vector3(1f, 1.5f, 2f);
        _owner = new GameObject("Item-guide test");
        _guide = _owner.AddComponent(Runtime("PickSurfaceGuide"));
    }

    [TearDown]
    public void Cleanup()
    {
        if (_owner != null) UnityEngine.Object.DestroyImmediate(_owner);
        if (_viewer != null) UnityEngine.Object.DestroyImmediate(_viewer);
        foreach (GameObject camera in _previousMainCameras) if (camera != null) camera.tag = "MainCamera";
        _previousMainCameras.Clear();
    }

    [Test]
    public void GuideUsesDoublePhysicalSizeAndKeepsItsAnchorAcrossCaptures()
    {
        var preset = new PickItemPreset("Fixture tile", .30d, .24d, .02d, 2400d, 0d);
        _viewer.transform.rotation = Quaternion.Euler(30f, 45f, 6f);
        Refresh(0, preset);
        Bounds bounds = (Bounds)_guide.GetType().GetProperty("LayoutBounds").GetValue(_guide);
        Vector3 layoutCenter = _guide.transform.TransformPoint(bounds.center);
        Vector3 screen = _viewer.GetComponent<Camera>().WorldToViewportPoint(layoutCenter);
        Assert.That(screen.x, Is.EqualTo(.5f).Within(.001f));
        Assert.That(screen.y, Is.EqualTo(.5f).Within(.001f));
        GameObject geometry = Read<GameObject>("_geometry");
        Transform body = geometry.transform.Find("Nominal item body");
        Assert.That(body.localScale.x, Is.EqualTo(.24f));
        Assert.That(body.localScale.y, Is.EqualTo(.02f));
        Assert.That(body.localScale.z, Is.EqualTo(.30f));
        Assert.That(geometry.transform.lossyScale.x, Is.EqualTo(2f).Within(1e-6f));
        Assert.That(geometry.transform.lossyScale.y, Is.EqualTo(2f).Within(1e-6f));
        Assert.That(geometry.transform.lossyScale.z, Is.EqualTo(2f).Within(1e-6f));
        Pose anchored = new Pose(_guide.transform.position, _guide.transform.rotation);
        Quaternion startAngle = geometry.transform.localRotation;
        SetTime(3.8f);
        Assert.That(Quaternion.Angle(startAngle, geometry.transform.localRotation), Is.GreaterThan(20f));
        _viewer.transform.SetPositionAndRotation(new Vector3(5f, 3f, -1f), Quaternion.Euler(10f, 45f, 0f));
        Refresh(1, preset);
        Assert.That(Read<float>("_elapsedSeconds"), Is.EqualTo(3.8f));
        Assert.That(Vector3.Distance(_guide.transform.position, anchored.position), Is.LessThan(1e-6f));
        Assert.That(Quaternion.Angle(_guide.transform.rotation, anchored.rotation), Is.LessThan(.01f));
        Assert.That(_guide.GetComponentsInChildren<Collider>(true), Is.Empty);
        Assert.That(_guide.GetComponentsInChildren<ArticulationBody>(true), Is.Empty);
    }

    [TestCase(0f, 0, false, false)]
    [TestCase(2.1f, 1, false, false)]
    [TestCase(3.6f, 2, false, false)]
    [TestCase(5.1f, 3, false, false)]
    [TestCase(7.3f, 3, true, false)]
    [TestCase(9.1f, 3, false, true)]
    [TestCase(10.1f, 0, false, false)]
    public void ExplanationShowsConsecutiveCornersThenCircularOrderAndNormal(
        float seconds, int visibleCorners, bool roundArrow, bool normalArrow)
    {
        Refresh(0, PickSurfaceGeometry.GetPreset(PickItem.Tile));
        SetTime(seconds);
        GameObject geometry = Read<GameObject>("_geometry");
        int visible = Enumerable.Range(1, 3).Count(index => geometry.transform.Find("Capture corner " + index).gameObject.activeSelf);
        Assert.That(visible, Is.EqualTo(visibleCorners));
        Assert.That(Read<GameObject>("_roundArrow").activeSelf, Is.EqualTo(roundArrow));
        Assert.That(Read<GameObject>("_normalArrow").activeSelf, Is.EqualTo(normalArrow));
        if (roundArrow)
        {
            LineRenderer line = Read<LineRenderer>("_roundLine");
            Vector3 before = line.GetPosition(20);
            Vector3 after = line.GetPosition(21);
            Assert.That(before.x * after.z - before.z * after.x, Is.GreaterThan(0f));
        }
        if (normalArrow)
        {
            LineRenderer shaft = Read<GameObject>("_normalArrow").transform.Find("Downward normal shaft").GetComponent<LineRenderer>();
            Assert.That(shaft.GetPosition(0).y, Is.GreaterThan(shaft.GetPosition(1).y));
        }
    }

    [Test]
    public void OkReturnsFromTheGuideAndKeepsItsTriggerConsumedForTheFrame()
    {
        Refresh(0, PickSurfaceGeometry.GetPreset(PickItem.Tile));
        int dismissed = 0;
        Action callback = () => dismissed++;
        _guide.GetType().GetEvent("DismissRequested").AddEventHandler(_guide, callback);
        _guide.GetType().GetMethod("SetCaptureStatus").Invoke(_guide, new object[] { "Fit rejected. Start again." });
        Component caption = Read<Component>("_caption");
        Assert.That(caption.GetType().GetProperty("text").GetValue(caption), Does.Contain("Fit rejected. Start again."));
        Component button = _guide.GetComponentInChildren(Runtime("UnityEngine.UI.Button"), true);
        object clicked = button.GetType().GetProperty("onClick").GetValue(button);
        clicked.GetType().GetMethod("Invoke").Invoke(clicked, null);
        Assert.That(dismissed, Is.EqualTo(1));
        Assert.That(_owner.activeSelf, Is.False);
        Assert.That(_guide.GetType().GetProperty("IsPointerOverControls").GetValue(null), Is.True);
    }

    [Test]
    public void RejectedThirdCornerShowsAllThreeCapturedContacts()
    {
        Refresh(3, PickSurfaceGeometry.GetPreset(PickItem.Tile));
        _guide.GetType().GetMethod("SetCaptureStatus").Invoke(_guide, new object[] { "Fit rejected. Press OK to start again." });
        Component caption = Read<Component>("_caption");
        Assert.That(caption.GetType().GetProperty("text").GetValue(caption), Does.Contain("3/3 captured"));
        Assert.That(_guide.GetType().GetProperty("SelectedPointIndex").GetValue(_guide), Is.EqualTo(2));
    }

    [Test]
    public void TopFaceHasContrastAndTheNormalArrowHeadHasDepth()
    {
        Refresh(0, PickSurfaceGeometry.GetPreset(PickItem.Brick));
        Transform geometry = Read<GameObject>("_geometry").transform;
        MeshRenderer body = geometry.Find("Nominal item body").GetComponent<MeshRenderer>();
        Material[] materials = body.sharedMaterials;
        Assert.That(materials[2].GetColor("_BaseColor").grayscale,
            Is.GreaterThan(materials[0].GetColor("_BaseColor").grayscale + .1f));
        Assert.That(geometry.Find("Top face perimeter").GetComponent<LineRenderer>().loop, Is.True);
        SetTime(9.1f);
        Mesh head = Read<GameObject>("_normalArrow").transform.Find("Downward normal head").GetComponent<MeshFilter>().sharedMesh;
        Assert.That(head.bounds.size.x, Is.GreaterThan(.02f));
        Assert.That(head.bounds.size.z, Is.GreaterThan(.02f));
        Assert.That(head.bounds.size.y, Is.GreaterThan(.02f));
    }

    [TestCase(.2d, .2d, .007d)]
    [TestCase(.28d, .185d, .135d)]
    [TestCase(.6d, .6d, .02d)]
    [TestCase(2d, 2d, .1d)]
    public void RotatingBodyStaysBetweenCaptionAndOkAcrossConfiguredSizes(double length, double width, double thickness)
    {
        Refresh(0, new PickItemPreset("Configured tile", length, width, thickness, 2400d, 0d));
        Transform body = Read<GameObject>("_geometry").transform.Find("Nominal item body");
        Bounds meshBounds = body.GetComponent<MeshFilter>().sharedMesh.bounds;
        float captionBottom = RectYBounds((RectTransform)Read<Component>("_caption").transform).x;
        float controlsTop = RectYBounds((RectTransform)Read<GameObject>("_controls").transform).y;
        for (int step = 0; step < 12; step++)
        {
            SetTime(step * 2.5f);
            for (int corner = 0; corner < 8; corner++)
            {
                Vector3 local = new Vector3(
                    (corner & 1) == 0 ? meshBounds.min.x : meshBounds.max.x,
                    (corner & 2) == 0 ? meshBounds.min.y : meshBounds.max.y,
                    (corner & 4) == 0 ? meshBounds.min.z : meshBounds.max.z);
                Vector3 point = _guide.transform.InverseTransformPoint(body.TransformPoint(local));
                Assert.That(point.y, Is.GreaterThan(controlsTop + .02f));
                Assert.That(point.y, Is.LessThan(captionBottom - .02f));
            }
        }
        float distance = Vector3.ProjectOnPlane(_guide.transform.position - _viewer.transform.position, Vector3.up).magnitude;
        Assert.That(distance, Is.GreaterThanOrEqualTo(1f));
        if (length == 2d) Assert.That(distance, Is.GreaterThan(4f));
    }

    private Vector2 RectYBounds(RectTransform rect)
    {
        var corners = new Vector3[4];
        rect.GetWorldCorners(corners);
        float minimum = float.PositiveInfinity;
        float maximum = float.NegativeInfinity;
        foreach (Vector3 corner in corners)
        {
            float y = _guide.transform.InverseTransformPoint(corner).y;
            minimum = Mathf.Min(minimum, y);
            maximum = Mathf.Max(maximum, y);
        }
        return new Vector2(minimum, maximum);
    }

    private void Refresh(int point, PickItemPreset preset) => _guide.GetType().GetMethod("Refresh",
        new[] { typeof(PickItem), typeof(int), typeof(PickItemPreset) }).Invoke(_guide, new object[] { PickItem.Tile, point, preset });

    private void SetTime(float seconds)
    {
        _guide.GetType().GetField("_elapsedSeconds", Instance).SetValue(_guide, seconds);
        _guide.GetType().GetMethod("PresentSequence", Instance).Invoke(_guide, new object[] { false });
    }

    private T Read<T>(string field) => (T)_guide.GetType().GetField(field, Instance).GetValue(_guide);

    private static Type Runtime(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).First(type => type != null);
}
