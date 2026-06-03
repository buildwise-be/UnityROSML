using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// Component for tile-specific UI.
/// Shows face selection options and tile status.
/// </summary>
public class TileUI : MonoBehaviour
{
    [SerializeField]
    private Button _selectPickFaceButton;
    [SerializeField]
    private Button _selectPlaceFaceButton;
    [SerializeField]
    private TextMeshProUGUI _tileStatusText;
    [SerializeField]
    private TextMeshProUGUI _faceSelectionStatusText;

    private GameObject _currentTile;
    private Tile _tileComponent;
    private FaceSelectionController _faceSelectionController;

    /// <summary>
    /// Initialize the UI for a specific tile.
    /// </summary>
    public void Initialize(GameObject tile, FaceSelectionController faceSelectionController)
    {
        _currentTile = tile;
        _tileComponent = tile?.GetComponent<Tile>();
        _faceSelectionController = faceSelectionController;

        if (_tileComponent == null)
        {
            Debug.LogError("TileUI: Tile component not found!");
            return;
        }

        UpdateUI();
        WireUpButtons();
    }

    private void WireUpButtons()
    {
        if (_selectPickFaceButton != null)
        {
            _selectPickFaceButton.onClick.RemoveAllListeners();
            _selectPickFaceButton.onClick.AddListener(() => StartPickFaceSelection());
        }

        if (_selectPlaceFaceButton != null)
        {
            _selectPlaceFaceButton.onClick.RemoveAllListeners();
            _selectPlaceFaceButton.onClick.AddListener(() => StartPlaceFaceSelection());
        }
    }

    private void StartPickFaceSelection()
    {
        if (_currentTile == null || _faceSelectionController == null)
        {
            Debug.LogError("TileUI: Cannot start face selection - tile or controller is null!");
            return;
        }

        if (_tileComponent.Type != Tile.TileType.Pick)
        {
            Debug.LogWarning("TileUI: Cannot select pick face on a place tile!");
            return;
        }

        _faceSelectionController.StartFaceSelection(_currentTile, true);
        UpdateUI();
    }

    private void StartPlaceFaceSelection()
    {
        if (_currentTile == null || _faceSelectionController == null)
        {
            Debug.LogError("TileUI: Cannot start face selection - tile or controller is null!");
            return;
        }

        if (_tileComponent.Type != Tile.TileType.Place)
        {
            Debug.LogWarning("TileUI: Cannot select place face on a pick tile!");
            return;
        }

        _faceSelectionController.StartFaceSelection(_currentTile, false);
        UpdateUI();
    }

    private void UpdateUI()
    {
        if (_tileComponent == null) return;

        // Update tile status text
        if (_tileStatusText != null)
        {
            _tileStatusText.text = $"Tile Type: {_tileComponent.Type}";
        }

        // Update face selection status
        if (_faceSelectionStatusText != null)
        {
            string status = "";
            if (_tileComponent.Type == Tile.TileType.Pick)
            {
                status = _tileComponent.PickFaceSelected 
                    ? "Pick Face: Selected" 
                    : "Pick Face: Not Selected";
            }
            else if (_tileComponent.Type == Tile.TileType.Place)
            {
                status = _tileComponent.PlaceFaceSelected 
                    ? "Place Face: Selected" 
                    : "Place Face: Not Selected";
            }
            _faceSelectionStatusText.text = status;
        }

        // Update button states
        if (_selectPickFaceButton != null)
        {
            _selectPickFaceButton.interactable = 
                _tileComponent.Type == Tile.TileType.Pick && !_tileComponent.PickFaceSelected;
        }

        if (_selectPlaceFaceButton != null)
        {
            _selectPlaceFaceButton.interactable = 
                _tileComponent.Type == Tile.TileType.Place && !_tileComponent.PlaceFaceSelected;
        }
    }

    private void Update()
    {
        // Update UI periodically to reflect tile state changes
        if (_tileComponent != null)
        {
            UpdateUI();
        }
    }
}

