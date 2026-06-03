using System;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

/// <summary>
/// Handles raycast-based face selection on tiles.
/// Provides visual feedback (highlighting, plane representation) and stores selected face in Tile component.
/// </summary>
public class FaceSelectionController : MonoBehaviour
{
    private bool _isSelectionModeActive = false;
    private Tile _currentTile = null;
    private bool _selectingPickFace = false;
    private XRRayInteractor _rayInteractor = null;
    private XRGrabInteractable _tileInteractable = null;
    private GameObject _hitPlaneRepresentation = null;
    private MeshFaceColorer _meshFaceColorer = null;

    /// <summary>
    /// Enter face selection mode for a tile.
    /// </summary>
    /// <param name="tile">The tile to select a face on</param>
    /// <param name="isPickFace">True if selecting pick face, false for place face</param>
    public void StartFaceSelection(GameObject tile, bool isPickFace)
    {
        if (tile == null)
        {
            Debug.LogError("FaceSelectionController: Tile is null!");
            return;
        }

        Tile tileComponent = tile.GetComponent<Tile>();
        if (tileComponent == null)
        {
            Debug.LogError("FaceSelectionController: Tile component not found!");
            return;
        }

        _currentTile = tileComponent;
        _selectingPickFace = isPickFace;
        _isSelectionModeActive = true;

        // Get or add MeshFaceColorer
        _meshFaceColorer = tile.GetComponent<MeshFaceColorer>();
        if (_meshFaceColorer == null)
        {
            _meshFaceColorer = tile.AddComponent<MeshFaceColorer>();
        }

        // Get XRGrabInteractable for hover events
        _tileInteractable = tile.GetComponent<XRGrabInteractable>();
        if (_tileInteractable != null)
        {
            _tileInteractable.hoverEntered.AddListener(OnTileHoverEntered);
            _tileInteractable.hoverExited.AddListener(OnTileHoverExited);
        }

        Debug.Log($"FaceSelectionController: Started selection mode for {(isPickFace ? "pick" : "place")} face");
    }

    /// <summary>
    /// Exit face selection mode.
    /// </summary>
    public void StopFaceSelection()
    {
        _isSelectionModeActive = false;
        _currentTile = null;

        if (_tileInteractable != null)
        {
            _tileInteractable.hoverEntered.RemoveListener(OnTileHoverEntered);
            _tileInteractable = null;
        }

        if (_hitPlaneRepresentation != null)
        {
            Destroy(_hitPlaneRepresentation);
            _hitPlaneRepresentation = null;
        }

        _rayInteractor = null;
        Debug.Log("FaceSelectionController: Stopped selection mode");
    }

    private void OnTileHoverEntered(HoverEnterEventArgs args)
    {
        if (!_isSelectionModeActive) return;

        _rayInteractor = args.interactorObject as XRRayInteractor;
        if (_meshFaceColorer != null && _rayInteractor != null)
        {
            _meshFaceColorer.SetColoringModeOnOff(true, args);
        }
    }

    private void OnTileHoverExited(HoverExitEventArgs arg0)
    {
        if (_meshFaceColorer != null)
        {
            _meshFaceColorer.SetColoringModeOnOff(false, null);
        }
    }

    private void Update()
    {
        /*
        if (!_isSelectionModeActive || _currentTile == null) return;

        // Check for raycast hit on tile
        if (_rayInteractor != null && _rayInteractor.TryGetCurrent3DRaycastHit(out RaycastHit hit))
        {
            GameObject tileObject = _currentTile.gameObject;
            
            // Check if hit is on the current tile
            if (hit.transform.IsChildOf(tileObject.transform) || hit.transform == tileObject.transform)
            {
                UpdatePlaneRepresentation(hit);

                // Check for selection input (trigger press or click)
                // Note: XR interaction selection is handled via XRInteractionManager events
                // For now, use mouse click as fallback for testing
                if (Input.GetMouseButtonDown(0))
                {
                    SelectFace(hit);
                }
                // TODO: Add XR controller trigger detection via XRInteractionManager
            }
            else
            {
                HidePlaneRepresentation();
            }
        }
        else
        {
            HidePlaneRepresentation();
        }
        */
    }

    private void SelectFace(RaycastHit hit)
    {
        if (_currentTile == null) return;

        // Get face normal in world space
        Vector3 faceNormal = hit.normal;

        // Transform to world space if needed (hit.normal is already in world space)
        // But we need to account for tile rotation
        Vector3 worldNormal = _currentTile.transform.TransformDirection(
            _currentTile.transform.InverseTransformDirection(faceNormal)
        );

        // Store the face normal in the tile
        if (_selectingPickFace)
        {
            _currentTile.SetPickFace(faceNormal);
        }
        else
        {
            _currentTile.SetPlaceFace(faceNormal);
        }

        Debug.Log($"FaceSelectionController: Selected {( _selectingPickFace ? "pick" : "place")} face with normal {faceNormal}");

        // Exit selection mode
        StopFaceSelection();
    }
}

