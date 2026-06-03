using UnityEngine;

/// <summary>
/// DEPRECATED: This component has been replaced by the Tile component.
/// Use Tile component for pick/place location management.
/// This file is kept for backward compatibility with old PickAndPlaceManager.
/// </summary>
[System.Obsolete("Use Tile component instead")]
public class PickOrPlaceLocationInstance : MonoBehaviour
{
    [SerializeField]
    private LocationType _locationType;
    public LocationType LocationType { get => _locationType; set => _locationType = value; }
}
