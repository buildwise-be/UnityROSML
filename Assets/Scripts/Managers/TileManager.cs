using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// <summary>
/// Manages tile spawning, tracking, and lifecycle.
/// Handles tile selection and face assignment.
/// Replaces PrimitivesSpawner functionality.
/// </summary>
public class TileManager : MonoBehaviour
{
    [SerializeField]
    [Tooltip("The prefab to use for spawning tiles")]
    private GameObject _tilePrefab;

    [SerializeField]
    [Tooltip("The material to apply to spawned tiles")]
    private Material _tileMaterial;

    [SerializeField]
    [Tooltip("The material to apply to hovered tile faces when selecting the pick or place face")]
    private Material _hoverMaterial;

    [SerializeField]
    [Tooltip("Default tile dimensions (width, height, depth) in meters")]
    private Vector3 _defaultTileDimensions = new Vector3(0.197f, 0.006f, 0.197f);

    [SerializeField]
    [Tooltip("Reference to UIManager for showing tile UI")]
    private UIManager _uiManager;

    private List<GameObject> _pickTiles = new List<GameObject>();
    private List<GameObject> _placeTiles = new List<GameObject>();

    /// <summary>
    /// Spawn a pick tile in front of the camera.
    /// </summary>
    public void SpawnPickTile()
    {
        GameObject tile = SpawnTile(Tile.TileType.Pick);
        if (tile != null)
        {
            _pickTiles.Add(tile);
        }
    }

    /// <summary>
    /// Spawn a place tile in front of the camera.
    /// </summary>
    public void SpawnPlaceTile()
    {
        GameObject tile = SpawnTile(Tile.TileType.Place);
        if (tile != null)
        {
            _placeTiles.Add(tile);
        }
    }

    private GameObject SpawnTile(Tile.TileType tileType)
    {
        if (_tilePrefab == null)
        {
            Debug.LogError("TileManager: Tile prefab not assigned!");
            return null;
        }

        GameObject tileInstance = Instantiate(_tilePrefab);
        tileInstance.name = $"{tileType}Tile_{tileInstance.GetInstanceID()}";

        // Add Tile component
        Tile tileComponent = tileInstance.AddComponent<Tile>();
        tileComponent.Type = tileType;
        tileComponent.Dimensions = _defaultTileDimensions;

        // Set material
        if (_tileMaterial != null)
        {
            var renderer = tileInstance.GetComponent<MeshRenderer>();
            if (renderer != null)
            {
                renderer.materials = new Material[] { _tileMaterial, _tileMaterial, _tileMaterial, _tileMaterial, _tileMaterial, _tileMaterial };
            }
        }

        // Position tile in front of camera
        tileInstance.transform.position = Camera.main.transform.position + Camera.main.transform.forward * 1.0f;
        tileInstance.transform.localScale = _defaultTileDimensions;

        // Add XR interaction components
        SetupXRInteraction(tileInstance);

        // Add MeshFaceColorer for face selection visualization
        if (!tileInstance.TryGetComponent<MeshFaceColorer>(out _))
        {
            var mfc = tileInstance.AddComponent<MeshFaceColorer>();
            mfc.HoverMaterial = _hoverMaterial;
        }

        Debug.Log($"TileManager: Spawned {tileType} tile at {tileInstance.transform.position}");
        Debug.Log($"Main camera is currently at location {Camera.main.transform.position}");
        return tileInstance;
    }

    private void SetupXRInteraction(GameObject tile)
    {
        // Add XRGrabInteractable if not present
        XRGrabInteractable xrgi = tile.GetComponent<XRGrabInteractable>();
        if (xrgi == null)
        {
            xrgi = tile.AddComponent<XRGrabInteractable>();
        }

        // Configure XR interaction
        xrgi.trackPosition = true;
        xrgi.trackRotation = false;
        xrgi.trackScale = false;
        xrgi.throwOnDetach = false;

        // Setup rigidbody
        Rigidbody rb = tile.GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = tile.AddComponent<Rigidbody>();
        }
        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        // Wire up activation event to show UI
        xrgi.activated.AddListener((args) => OnTileActivated(tile));
    }

    private void OnTileActivated(GameObject tile)
    {
        if (_uiManager != null)
        {
            _uiManager.ShowTileUI(tile);
        }
        
        // Visual feedback: change color
        /*
        var renderer = tile.GetComponent<MeshRenderer>();
        if (renderer != null)
        {
            renderer.material.color = Color.green;
        }
        */
    }

    /// <summary>
    /// Get all pick tiles.
    /// </summary>
    public List<GameObject> GetPickTiles()
    {
        return new List<GameObject>(_pickTiles);
    }

    /// <summary>
    /// Get all place tiles.
    /// </summary>
    public List<GameObject> GetPlaceTiles()
    {
        return new List<GameObject>(_placeTiles);
    }

    /// <summary>
    /// Get the first ready pick tile (with face selected).
    /// </summary>
    public Tile GetReadyPickTile()
    {
        foreach (var tileObj in _pickTiles)
        {
            if (tileObj == null) continue;
            Tile tile = tileObj.GetComponent<Tile>();
            if (tile != null && tile.IsReady())
            {
                return tile;
            }
        }
        return null;
    }

    /// <summary>
    /// Get the first ready place tile (with face selected).
    /// </summary>
    public Tile GetReadyPlaceTile()
    {
        foreach (var tileObj in _placeTiles)
        {
            if (tileObj == null) continue;
            Tile tile = tileObj.GetComponent<Tile>();
            if (tile != null && tile.IsReady())
            {
                return tile;
            }
        }
        return null;
    }

    /// <summary>
    /// Clear all tiles.
    /// </summary>
    public void ClearAllTiles()
    {
        foreach (var tile in _pickTiles)
        {
            if (tile != null)
            {
                Destroy(tile);
            }
        }
        foreach (var tile in _placeTiles)
        {
            if (tile != null)
            {
                Destroy(tile);
            }
        }
        _pickTiles.Clear();
        _placeTiles.Clear();
    }
}

