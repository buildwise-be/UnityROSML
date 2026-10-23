using System;
using System.IO;
using NUnit.Framework;
using UnityEngine;

public sealed class CalibrationProfileTests
{
    private string _directory;
    private RobotCalibrationStore _store;
    private static readonly Pose Calibration = new Pose(new Vector3(-0.52f, 0.19f, 0.41f), Quaternion.Euler(-87f, 12f, 3f));

    [SetUp]
    public void SetUp()
    {
        _directory = Path.GetFullPath(Path.Combine(Path.GetTempPath(), "UnityROSML-calibration-" + Guid.NewGuid().ToString("N")));
        _store = new RobotCalibrationStore(_directory);
    }

    [TearDown]
    public void TearDown()
    {
        string parent = Path.GetFullPath(Path.GetTempPath()).TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar;
        Assert.That(_directory.StartsWith(parent, StringComparison.OrdinalIgnoreCase), Is.True);
        if (Directory.Exists(_directory)) Directory.Delete(_directory, true);
        else if (File.Exists(_directory)) File.Delete(_directory);
    }

    [Test]
    public void ReloadedCalibrationFollowsTheQrUnderADifferentXrOrigin()
    {
        var profile = RobotCalibrationProfile.Create("robot-A", "tag/with:payload?x=1", 0.087f, Calibration);
        Assert.That(_store.TrySave(profile, out string error), Is.True, error);
        Assert.That(new RobotCalibrationStore(_directory).TryLoad("robot-A", profile.markerPayload, 0.087f, out var loaded, out error), Is.True, error);
        Pose markerInTracking = new Pose(new Vector3(0.2f, -0.4f, 0.7f), Quaternion.Euler(90f, 30f, 0f));
        Pose newOrigin = new Pose(new Vector3(10f, 2f, -4f), Quaternion.Euler(0f, -75f, 0f));
        Pose markerWorld = RobotFrameMath.Compose(newOrigin, markerInTracking);
        Pose actualBase = RobotFrameMath.Compose(markerWorld, loaded.PoseInMarker);
        Pose expectedBase = RobotFrameMath.Compose(markerWorld, Calibration);
        Assert.That(Vector3.Distance(expectedBase.position, actualBase.position), Is.LessThan(0.00001f));
        Assert.That(Quaternion.Angle(expectedBase.rotation, actualBase.rotation), Is.LessThan(0.01f));
    }

    [Test]
    public void DifferentMarkersAndRobotProfilesDoNotOverwriteEachOtherOrEscapeTheDirectory()
    {
        string payload = "../../other/path\\file.json";
        string first = _store.PathFor("robot-A", payload);
        string second = _store.PathFor("robot-B", payload);
        string third = _store.PathFor("robot-A", "other marker");
        Assert.That(Path.GetDirectoryName(first), Is.EqualTo(_directory));
        Assert.That(Path.GetFileNameWithoutExtension(first).Length, Is.EqualTo(64));
        Assert.That(first, Is.Not.EqualTo(second));
        Assert.That(first, Is.Not.EqualTo(third));
        Assert.That(_store.TrySave(RobotCalibrationProfile.Create("robot-A", payload, 0.087f, Calibration), out _), Is.True);
        Assert.That(_store.TryLoad("robot-B", payload, 0.087f, out _, out string error), Is.False);
        Assert.That(error, Is.Null, "A missing independent profile is normal.");
    }

    [Test]
    public void UpdatingAProfileReplacesTheCompleteRecordAndLeavesNoTemporaryFiles()
    {
        Assert.That(_store.TrySave(RobotCalibrationProfile.Create("robot", "tag", 0.087f, Calibration), out string error), Is.True, error);
        Pose changed = new Pose(Calibration.position + Vector3.right * 0.05f, Quaternion.Euler(10f, -30f, 80f));
        Assert.That(_store.TrySave(RobotCalibrationProfile.Create("robot", "tag", 0.087f, changed), out error), Is.True, error);
        Assert.That(_store.TryLoad("robot", "tag", 0.087f, out var loaded, out error), Is.True, error);
        Assert.That(Vector3.Distance(changed.position, loaded.basePositionInMarker), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(changed.rotation, loaded.baseRotationInMarker), Is.LessThan(0.01f));
        Assert.That(Directory.GetFiles(_directory), Has.Length.EqualTo(1));
    }

    [TestCase("version")]
    [TestCase("profile")]
    [TestCase("payload")]
    [TestCase("size")]
    [TestCase("rotation")]
    [TestCase("time")]
    public void LoadedFilesWithIncompatibleOrMalformedMetadataAreRejected(string change)
    {
        RobotCalibrationProfile profile = RobotCalibrationProfile.Create("robot", "tag", 0.087f, Calibration);
        switch (change)
        {
            case "version": profile.schemaVersion = 55; break;
            case "profile": profile.profileId = "other robot"; break;
            case "payload": profile.markerPayload = "other tag"; break;
            case "size": profile.markerSizeMeters = 0.1f; break;
            case "rotation": profile.baseRotationInMarker = new Quaternion(0f, 0f, 0f, 2f); break;
            case "time": profile.savedUtc = "invalid"; break;
        }
        Directory.CreateDirectory(_directory);
        File.WriteAllText(_store.PathFor("robot", "tag"), JsonUtility.ToJson(profile));
        Assert.That(_store.TryLoad("robot", "tag", 0.087f, out var loaded, out string error), Is.False);
        Assert.That(loaded, Is.Null);
        Assert.That(error, Is.Not.Empty);
    }

    [Test]
    public void InvalidNewPoseDoesNotDamageThePreviousSavedProfile()
    {
        RobotCalibrationProfile profile = RobotCalibrationProfile.Create("robot", "tag", 0.087f, Calibration);
        Assert.That(_store.TrySave(profile, out _), Is.True);
        string path = _store.PathFor("robot", "tag");
        string previous = File.ReadAllText(path);
        profile.basePositionInMarker = new Vector3(float.NaN, 0f, 0f);
        Assert.That(_store.TrySave(profile, out string error), Is.False);
        Assert.That(error, Is.Not.Empty);
        Assert.That(File.ReadAllText(path), Is.EqualTo(previous));
        profile.basePositionInMarker = Vector3.zero;
        profile.baseRotationInMarker = new Quaternion();
        Assert.That(_store.TrySave(profile, out error), Is.False);
        Assert.That(File.ReadAllText(path), Is.EqualTo(previous));
    }

    [Test]
    public void CorruptJsonAndUnavailableStorageReturnErrorsInsteadOfThrowing()
    {
        Directory.CreateDirectory(_directory);
        string path = _store.PathFor("robot", "tag");
        File.WriteAllText(path, "{not valid json");
        Assert.That(_store.TryLoad("robot", "tag", 0.087f, out _, out string error), Is.False);
        Assert.That(error, Is.Not.Empty);
        File.Delete(path);
        Directory.Delete(_directory);
        File.WriteAllText(_directory, "blocks directory creation");
        Assert.That(_store.TrySave(RobotCalibrationProfile.Create("robot", "tag", 0.087f, Calibration), out error), Is.False);
        Assert.That(error, Is.Not.Empty);
    }
}
