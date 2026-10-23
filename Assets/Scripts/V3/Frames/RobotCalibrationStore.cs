using System;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

/// <summary>A QR-relative robot base pose. It contains no XR-session world coordinates.</summary>
[Serializable]
public sealed class RobotCalibrationProfile
{
    public int schemaVersion;
    public string profileId;
    public string markerPayload;
    public float markerSizeMeters;
    public Vector3 basePositionInMarker;
    public Quaternion baseRotationInMarker;
    public string savedUtc;

    public Pose PoseInMarker => new Pose(basePositionInMarker, baseRotationInMarker);

    public static RobotCalibrationProfile Create(string profileId, string markerPayload, float markerSizeMeters, Pose pose)
    {
        if (!RobotFrameMath.IsValidPose(pose)) throw new ArgumentException("A finite robot-base pose is required.", nameof(pose));
        return new RobotCalibrationProfile
        {
            schemaVersion = 1,
            profileId = profileId,
            markerPayload = markerPayload,
            markerSizeMeters = markerSizeMeters,
            basePositionInMarker = pose.position,
            baseRotationInMarker = pose.rotation.normalized,
            savedUtc = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture)
        };
    }

    public bool Matches(string expectedProfileId, string expectedPayload, float expectedSize, out string error)
    {
        error = null;
        if (schemaVersion != 1) error = "Unsupported calibration file version.";
        else if (string.IsNullOrWhiteSpace(profileId) || profileId != expectedProfileId)
            error = "Calibration belongs to another robot profile.";
        else if (string.IsNullOrWhiteSpace(markerPayload) || markerPayload != expectedPayload)
            error = "Calibration belongs to another QR marker.";
        else if (!(markerSizeMeters > 0f) || float.IsInfinity(markerSizeMeters) ||
                 !(expectedSize > 0f) || float.IsInfinity(expectedSize) ||
                 Mathf.Abs(markerSizeMeters - expectedSize) > 0.000001f)
            error = "Calibration uses a different QR marker size.";
        else if (!RobotFrameMath.IsValidPose(PoseInMarker)) error = "Calibration pose is invalid.";
        else
        {
            Quaternion q = baseRotationInMarker;
            double normSquared = (double)q.x * q.x + (double)q.y * q.y + (double)q.z * q.z + (double)q.w * q.w;
            if (Math.Abs(normSquared - 1.0) > 0.0001) error = "Calibration rotation must be a unit quaternion.";
            else if (!DateTime.TryParse(savedUtc, CultureInfo.InvariantCulture, DateTimeStyles.RoundtripKind, out _))
                error = "Calibration save time is invalid.";
        }
        return error == null;
    }
}

/// <summary>App-local profiles, keyed by robot identity and exact decoded QR payload.</summary>
public sealed class RobotCalibrationStore
{
    private readonly string _directory;

    public RobotCalibrationStore(string directory)
    {
        if (string.IsNullOrWhiteSpace(directory)) throw new ArgumentException("A calibration directory is required.", nameof(directory));
        _directory = Path.GetFullPath(directory);
    }

    public string PathFor(string profileId, string markerPayload)
    {
        if (string.IsNullOrWhiteSpace(profileId) || string.IsNullOrWhiteSpace(markerPayload))
            throw new ArgumentException("A robot profile and decoded QR payload are required.");
        // Length-prefixing prevents different profile/payload pairs from sharing a key.
        string identity = profileId.Length.ToString(CultureInfo.InvariantCulture) + ":" + profileId + markerPayload;
        using (SHA256 hash = SHA256.Create())
        {
            byte[] digest = hash.ComputeHash(Encoding.UTF8.GetBytes(identity));
            StringBuilder name = new StringBuilder(64);
            foreach (byte value in digest) name.Append(value.ToString("x2", CultureInfo.InvariantCulture));
            return Path.Combine(_directory, name + ".json");
        }
    }

    /// <summary>A missing profile is normal and returns no error. Invalid/unreadable files are reported.</summary>
    public bool TryLoad(string profileId, string markerPayload, float markerSizeMeters,
        out RobotCalibrationProfile profile, out string error)
    {
        profile = null;
        error = null;
        try
        {
            string path = PathFor(profileId, markerPayload);
            if (!File.Exists(path)) return false;
            if (new FileInfo(path).Length > 65536) { error = "Calibration file is too large."; return false; }
            RobotCalibrationProfile loaded = JsonUtility.FromJson<RobotCalibrationProfile>(File.ReadAllText(path, Encoding.UTF8));
            if (loaded == null) { error = "Calibration file is empty."; return false; }
            if (!loaded.Matches(profileId, markerPayload, markerSizeMeters, out error)) return false;
            profile = loaded;
            return true;
        }
        catch (Exception exception) when (exception is IOException || exception is UnauthorizedAccessException ||
                                          exception is ArgumentException || exception is NotSupportedException)
        {
            error = "Calibration could not be read: " + exception.Message;
            return false;
        }
    }

    /// <summary>Replace atomically in the same directory; a failed write preserves the previous profile.</summary>
    public bool TrySave(RobotCalibrationProfile profile, out string error)
    {
        error = null;
        string temporary = null;
        try
        {
            if (profile == null) { error = "A calibration profile is required."; return false; }
            if (!profile.Matches(profile.profileId, profile.markerPayload, profile.markerSizeMeters, out error)) return false;
            string destination = PathFor(profile.profileId, profile.markerPayload);
            Directory.CreateDirectory(_directory);
            temporary = destination + "." + Guid.NewGuid().ToString("N") + ".tmp";
            byte[] bytes = new UTF8Encoding(false).GetBytes(JsonUtility.ToJson(profile, true));
            if (bytes.Length > 65536) { error = "Calibration file is too large."; return false; }
            using (FileStream stream = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None))
            {
                stream.Write(bytes, 0, bytes.Length);
                stream.Flush(true);
            }
            if (File.Exists(destination)) File.Replace(temporary, destination, null);
            else File.Move(temporary, destination);
            temporary = null;
            return true;
        }
        catch (Exception exception) when (exception is IOException || exception is UnauthorizedAccessException ||
                                          exception is ArgumentException || exception is NotSupportedException)
        {
            error = "Calibration could not be saved: " + exception.Message;
            return false;
        }
        finally
        {
            if (temporary != null)
                try { if (File.Exists(temporary)) File.Delete(temporary); }
                catch (IOException) { }
                catch (UnauthorizedAccessException) { }
        }
    }
}
