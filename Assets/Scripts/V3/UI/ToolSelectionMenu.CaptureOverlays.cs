using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public sealed partial class ToolSelectionMenu
{
    private readonly Dictionary<HudPanel, bool> _guideHudStates = new Dictionary<HudPanel, bool>();
    private float _nextGuideHudSearchAt;

    private void RefreshGuideOverlays(bool guideActive)
    {
        if (!guideActive)
        {
            foreach (KeyValuePair<HudPanel, bool> state in _guideHudStates)
                if (state.Key != null) state.Key.enabled = state.Value;
            _guideHudStates.Clear();
            _nextGuideHudSearchAt = 0f;
            return;
        }

        if (Time.realtimeSinceStartup < _nextGuideHudSearchAt) return;
        _nextGuideHudSearchAt = Time.realtimeSinceStartup + .5f;
        // ROS creates its HUD in Start, so it can appear after capture has begun.
        foreach (HudPanel hud in FindObjectsByType<HudPanel>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (!_guideHudStates.ContainsKey(hud)) _guideHudStates.Add(hud, hud.enabled);
            hud.enabled = false;
        }
    }
}
