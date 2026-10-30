using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>A renderer feature can withdraw availability immediately when disabled or unsupported.</summary>
public interface IRobotContourRenderingProvider
{
    bool IsContourRenderingAvailable { get; }
}

/// <summary>One generated visual submesh of the placed live robot, in its current world pose.</summary>
public readonly struct RobotContourDraw
{
    public readonly Mesh Mesh;
    public readonly Matrix4x4 LocalToWorld;
    public readonly int SubMeshIndex;

    public RobotContourDraw(Mesh mesh, Matrix4x4 localToWorld, int subMeshIndex)
    {
        Mesh = mesh;
        LocalToWorld = localToWorld;
        SubMeshIndex = subMeshIndex;
    }
}

/// <summary>
/// Presentation state for the explicitly placed robot. Trajectory rigs never register here.
/// Suppressing its normal draws leaves materials, physics, planning and visibility owners intact.
/// </summary>
public static class RobotContourDisplay
{
    private static RosRobotRig _liveRig;
    private static bool _enabled;
    private static bool _renderingAvailable;
    private static bool _cameraEventsSubscribed;
    private static int _cameraScopeDepth;
    private static readonly List<MeshRenderer> Renderers = new List<MeshRenderer>();
    private static readonly Dictionary<Renderer, bool> OriginalForceRenderingOff = new Dictionary<Renderer, bool>();
    private static readonly HashSet<UnityEngine.Object> RenderingProviders = new HashSet<UnityEngine.Object>();
    private static readonly List<CameraScope> CameraScopes = new List<CameraScope>();

    // Reuse scopes and their sets after the first frame, including nested Camera.Render calls.
    private sealed class CameraScope
    {
        public Camera Camera;
        public readonly HashSet<UnityEngine.Object> Providers = new HashSet<UnityEngine.Object>();
    }

    public static bool Enabled => _enabled;
    public static bool IsRenderingAvailable
    {
        get { RefreshAvailability(); return _renderingAvailable; }
    }
    public static float LineWidthPixels => 2f;
    public static Color LineColor => new Color(0.35f, 0.9f, 1f, RobotExecutionState.IsBusy ? 0.03f : 0.9f);
    public static event Action StateChanged;

    public static void SetEnabled(bool enabled)
    {
        bool changed = _enabled != enabled;
        _enabled = enabled;
        UpdateCameraSubscription();
        RefreshRenderers();
        if (changed) StateChanged?.Invoke();
    }

    /// <summary>Availability belongs to active renderer features, not the checkbox selection.</summary>
    public static void SetRenderingAvailable(UnityEngine.Object owner, bool available)
    {
        if (ReferenceEquals(owner, null)) return;
        if (available && owner != null) RenderingProviders.Add(owner);
        else RenderingProviders.Remove(owner);
        RenderingProviders.RemoveWhere(provider => provider == null);
        UpdateCameraSubscription();
        RefreshAvailability();
    }

    /// <summary>Only a feature on this camera's renderer can suppress its normal filled robot.</summary>
    public static void SetCameraRenderingAvailable(UnityEngine.Object owner, Camera camera, bool available)
    {
        if (owner == null || camera == null || _cameraScopeDepth == 0) return;
        CameraScope scope = CameraScopes[_cameraScopeDepth - 1];
        if (!ReferenceEquals(scope.Camera, camera)) return;
        if (available)
        {
            RenderingProviders.Add(owner);
            scope.Providers.Add(owner);
        }
        else scope.Providers.Remove(owner);
        RefreshAvailability(false);
    }

    /// <summary>Called only by placement. IsGhost is deliberately not used as robot identity.</summary>
    public static void SetLiveRobot(RosRobotRig rig)
    {
        if (_liveRig != null) _liveRig.ModelChanged -= RefreshRenderers;
        RestoreRendererFlags();
        _liveRig = rig;
        if (_liveRig != null) _liveRig.ModelChanged += RefreshRenderers;
        RefreshRenderers();
        StateChanged?.Invoke();
    }

    /// <summary>Render passes draw these meshes into private depth, never the camera's scene depth.</summary>
    public static bool TryGetDraws(List<RobotContourDraw> draws)
    {
        if (draws == null) throw new ArgumentNullException(nameof(draws));
        draws.Clear();
        if (!_enabled || !IsRenderingAvailable || _liveRig == null || !_liveRig.isActiveAndEnabled) return false;
        foreach (MeshRenderer renderer in Renderers)
        {
            if (renderer == null || !renderer.enabled || !renderer.gameObject.activeInHierarchy ||
                !OriginalForceRenderingOff.TryGetValue(renderer, out bool originallyOff) || originallyOff) continue;
            MeshFilter filter = renderer.GetComponent<MeshFilter>();
            Mesh mesh = filter != null ? filter.sharedMesh : null;
            if (mesh == null) continue;
            Matrix4x4 matrix = renderer.localToWorldMatrix;
            for (int index = 0; index < mesh.subMeshCount; index++)
                draws.Add(new RobotContourDraw(mesh, matrix, index));
        }
        return draws.Count > 0;
    }

    // These hooks keep forceRenderingOff owned by this presentation option,
    // including model replacement that temporarily disables the rig hierarchy.
    public static void OnRigEnabled(RosRobotRig rig)
    {
        if (ReferenceEquals(_liveRig, rig)) RefreshRenderers();
    }

    public static void OnRigDisabled(RosRobotRig rig)
    {
        if (ReferenceEquals(_liveRig, rig)) RestoreRendererFlags();
    }

    public static void OnRigDestroyed(RosRobotRig rig)
    {
        if (!ReferenceEquals(_liveRig, rig)) return;
        _liveRig.ModelChanged -= RefreshRenderers;
        RestoreRendererFlags();
        Renderers.Clear();
        _liveRig = null;
        StateChanged?.Invoke();
    }

    private static void RefreshRenderers()
    {
        _renderingAvailable = HasAvailableProvider();
        RestoreRendererFlags();
        Renderers.Clear();
        if (_liveRig == null || _liveRig.Model == null) return;
        foreach (GameObject visual in _liveRig.VisualRoots)
        {
            if (visual == null) continue;
            foreach (MeshRenderer renderer in visual.GetComponentsInChildren<MeshRenderer>(true))
                if (renderer != null && !Renderers.Contains(renderer)) Renderers.Add(renderer);
        }
        ApplyRendererFlags();
    }

    private static void ApplyRendererFlags()
    {
        RestoreRendererFlags();
        if (!_enabled || !_renderingAvailable || _liveRig == null || !_liveRig.isActiveAndEnabled) return;
        foreach (MeshRenderer renderer in Renderers)
        {
            if (renderer == null) continue;
            OriginalForceRenderingOff.Add(renderer, renderer.forceRenderingOff);
            renderer.forceRenderingOff = true;
        }
    }

    private static bool HasAvailableProvider()
    {
        CameraScope scope = _cameraScopeDepth > 0 ? CameraScopes[_cameraScopeDepth - 1] : null;
        foreach (UnityEngine.Object provider in RenderingProviders)
            if (provider != null && (scope == null || scope.Providers.Contains(provider)) &&
                (!(provider is IRobotContourRenderingProvider feature) || feature.IsContourRenderingAvailable))
                return true;
        return false;
    }

    private static void RefreshAvailability(bool notify = true)
    {
        bool available = HasAvailableProvider();
        if (_renderingAvailable == available) return;
        _renderingAvailable = available;
        ApplyRendererFlags();
        if (notify) StateChanged?.Invoke();
    }

    private static void OnBeginCameraRendering(ScriptableRenderContext context, Camera camera)
    {
        if (camera == null) { RefreshAvailability(); return; }
        if (_cameraScopeDepth == CameraScopes.Count) CameraScopes.Add(new CameraScope());
        CameraScope scope = CameraScopes[_cameraScopeDepth++];
        scope.Camera = camera;
        scope.Providers.Clear();
        // OnCameraPreCull confirms the actual renderer before URP performs culling.
        // Providers belonging to another pipeline, renderer index or camera cannot hide this model.
        RefreshAvailability(false);
    }

    private static void OnEndCameraRendering(ScriptableRenderContext context, Camera camera)
    {
        int index = _cameraScopeDepth - 1;
        while (index >= 0 && !ReferenceEquals(CameraScopes[index].Camera, camera)) index--;
        if (index < 0) return;
        while (_cameraScopeDepth > index)
        {
            CameraScope scope = CameraScopes[--_cameraScopeDepth];
            scope.Camera = null;
            scope.Providers.Clear();
        }
        RefreshAvailability(false);
        UpdateCameraSubscription();
    }

    private static void UpdateCameraSubscription()
    {
        bool subscribe = _enabled || RenderingProviders.Count != 0 || _cameraScopeDepth != 0;
        if (_cameraEventsSubscribed == subscribe) return;
        _cameraEventsSubscribed = subscribe;
        if (subscribe)
        {
            RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
            RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
        }
        else
        {
            RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;
            RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
        }
    }

    private static void RestoreRendererFlags()
    {
        foreach (KeyValuePair<Renderer, bool> entry in OriginalForceRenderingOff)
            if (entry.Key != null) entry.Key.forceRenderingOff = entry.Value;
        OriginalForceRenderingOff.Clear();
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Reset()
    {
        RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;
        RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
        if (_liveRig != null) _liveRig.ModelChanged -= RefreshRenderers;
        RestoreRendererFlags();
        Renderers.Clear();
        _liveRig = null;
        _enabled = false;
        _renderingAvailable = false;
        _cameraEventsSubscribed = false;
        _cameraScopeDepth = 0;
        RenderingProviders.Clear();
        CameraScopes.Clear();
        StateChanged = null;
    }
}
