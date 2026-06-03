using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// <summary>
/// Updated UIManager to support tile face selection workflow.
/// Works with TileUI and FaceSelectionController.
/// </summary>
public class UIManager : MonoBehaviour
{
    [SerializeField]
    [Tooltip("The UI that will appear above a tile upon creation.")]
    private GameObject _tileUIPrefab;

    [SerializeField]
    [Tooltip("The distance the UI will appear above the tile.")]
    private float _distanceAboveTile = 0.1f;

    [SerializeField]
    [Tooltip("Reference to FaceSelectionController for face selection")]
    private FaceSelectionController _faceSelectionController;

    [HideInInspector]
    public UnityEvent<GameObject> OnSendingObjectAsROSObstacle;

    private GameObject _currentTileUIInstance;
    private GameObject _currentTile;

    private void Awake()
    {
        OnSendingObjectAsROSObstacle = new UnityEvent<GameObject>();
    }

    /// <summary>
    /// Show tile UI for a specific tile.
    /// </summary>
    public void ShowTileUI(GameObject tile)
    {
        if (tile == null)
        {
            Debug.LogError("UIManager: Tile is null!");
            return;
        }

        // Remove existing UI if any
        if (_currentTileUIInstance != null)
        {
            Destroy(_currentTileUIInstance);
        }

        Debug.Log("UIManager: Showing tile UI");
        _currentTile = tile;
        _currentTileUIInstance = Instantiate(_tileUIPrefab);
        _currentTileUIInstance.transform.position = tile.transform.position + Vector3.up * _distanceAboveTile;

        // Initialize TileUI component if present
        TileUI tileUIComponent = _currentTileUIInstance.GetComponent<TileUI>();
        if (tileUIComponent != null && _faceSelectionController != null)
        {
            tileUIComponent.Initialize(tile, _faceSelectionController);
        }
        else
        {
            // Fallback to old event wiring if TileUI component not present
            WireTileUIEvents(_currentTileUIInstance);
        }
    }

    /// <summary>
    /// Hide the current tile UI.
    /// </summary>
    public void HideTileUI()
    {
        if (_currentTileUIInstance != null)
        {
            Destroy(_currentTileUIInstance);
            _currentTileUIInstance = null;
        }
        _currentTile = null;
    }

    private void WireTileUIEvents(GameObject tileUI)
    {
        Debug.Log("UIManager: Wiring tile UI Events (legacy mode)");
        var buttons = tileUI.transform.GetComponentsInChildren<Button>();

        foreach (var button in buttons)
        {
            if (button.name == "SendObstacleToSceneButton")
            {
                button.onClick.AddListener(() => OnSendingObjectAsROSObstacle.Invoke(_currentTile));
            }
            else if (button.name == "SelectPickFaceButton")
            {
                Debug.Log("UIManager: Found SelectPickFaceButton (legacy)");
                button.onClick.AddListener(() => PromptUserToSelectPickFace());
            }
        }
    }

    private void PromptUserToSelectPickFace()
    {
        if (_currentTile == null || _faceSelectionController == null)
        {
            Debug.LogError("UIManager: Cannot prompt face selection - tile or controller is null!");
            return;
        }

        Debug.Log("UIManager: Prompting user to select pick face");
        Tile tileComponent = _currentTile.GetComponent<Tile>();
        if (tileComponent != null)
        {
            _faceSelectionController.StartFaceSelection(_currentTile, true);
        }
    }
}
