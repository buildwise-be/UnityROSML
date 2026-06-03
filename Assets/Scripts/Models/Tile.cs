using UnityEngine;

/// <summary>
/// Component attached to tile GameObjects.
/// Stores tile dimensions, pick/place face information, and manages tile state.
/// </summary>
public class Tile : MonoBehaviour
{
    public enum TileType
    {
        Pick,
        Place
    }

    [SerializeField]
    private TileType _tileType = TileType.Pick;

    [SerializeField]
    private Vector3 _pickFaceNormal = Vector3.zero;

    [SerializeField]
    private Vector3 _placeFaceNormal = Vector3.zero;

    [SerializeField]
    private Vector3 _dimensions = new Vector3(0.197f, 0.006f, 0.197f); // Default tile dimensions

    private bool _pickFaceSelected = false;
    private bool _placeFaceSelected = false;

    public TileType Type
    {
        get => _tileType;
        set => _tileType = value;
    }

    public Vector3 PickFaceNormal
    {
        get => _pickFaceNormal;
        private set => _pickFaceNormal = value;
    }

    public Vector3 PlaceFaceNormal
    {
        get => _placeFaceNormal;
        private set => _placeFaceNormal = value;
    }

    public Vector3 Dimensions
    {
        get => _dimensions;
        set => _dimensions = value;
    }

    public bool PickFaceSelected => _pickFaceSelected;
    public bool PlaceFaceSelected => _placeFaceSelected;

    /// <summary>
    /// Set the pick face normal vector (in world space).
    /// </summary>
    public void SetPickFace(Vector3 faceNormal)
    {
        PickFaceNormal = faceNormal.normalized;
        _pickFaceSelected = true;
        Debug.Log($"Tile {gameObject.name}: Pick face set to {PickFaceNormal}");
    }

    /// <summary>
    /// Set the place face normal vector (in world space).
    /// </summary>
    public void SetPlaceFace(Vector3 faceNormal)
    {
        PlaceFaceNormal = faceNormal.normalized;
        _placeFaceSelected = true;
        Debug.Log($"Tile {gameObject.name}: Place face set to {PlaceFaceNormal}");
    }

    /// <summary>
    /// Get the pose for the specified face (pick or place).
    /// The pose is positioned at the center of the face, with rotation aligned to the face normal.
    /// </summary>
    public Pose GetPoseForFace(bool isPickFace)
    {
        Vector3 faceNormal = isPickFace ? PickFaceNormal : PlaceFaceNormal;
        
        if (faceNormal == Vector3.zero)
        {
            Debug.LogWarning($"Tile {gameObject.name}: Face normal not set!");
            return new Pose(transform.position, transform.rotation);
        }

        // Calculate face center position
        // Face is offset by half the dimension along the face normal
        Vector3 offset = faceNormal * (GetFaceDimension(faceNormal) / 2.0f);
        Vector3 faceCenter = transform.position + transform.TransformDirection(offset);

        // Calculate rotation: align up vector with face normal
        // For a face normal, we want the Z+ axis (forward) to point along the normal
        Quaternion faceRotation = Quaternion.LookRotation(faceNormal, Vector3.up);

        return new Pose(faceCenter, faceRotation);
    }

    /// <summary>
    /// Get the dimension along the specified face normal.
    /// </summary>
    private float GetFaceDimension(Vector3 faceNormal)
    {
        // Transform face normal to local space
        Vector3 localNormal = transform.InverseTransformDirection(faceNormal);
        
        // Find which axis the normal is closest to
        float absX = Mathf.Abs(localNormal.x);
        float absY = Mathf.Abs(localNormal.y);
        float absZ = Mathf.Abs(localNormal.z);

        if (absX > absY && absX > absZ)
        {
            return _dimensions.x;
        }
        else if (absY > absX && absY > absZ)
        {
            return _dimensions.y;
        }
        else
        {
            return _dimensions.z;
        }
    }

    /// <summary>
    /// Check if this tile is ready for pick/place operations.
    /// </summary>
    public bool IsReady()
    {
        if (_tileType == TileType.Pick)
        {
            return _pickFaceSelected;
        }
        else if (_tileType == TileType.Place)
        {
            return _placeFaceSelected;
        }
        return false;
    }

    /// <summary>
    /// Reset face selections.
    /// </summary>
    public void ResetFaces()
    {
        _pickFaceSelected = false;
        _placeFaceSelected = false;
        PickFaceNormal = Vector3.zero;
        PlaceFaceNormal = Vector3.zero;
    }
}

