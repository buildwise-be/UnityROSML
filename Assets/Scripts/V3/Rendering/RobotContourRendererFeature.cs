using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

/// <summary>
/// Visible contours of the explicitly registered live robot. Private depth hides
/// rear robot edges without introducing an invisible occluder for trajectory ghosts.
/// </summary>
public sealed class RobotContourRendererFeature : ScriptableRendererFeature, IRobotContourRenderingProvider
{
    [SerializeField] private Material _depthNormalsMaterial;
    [SerializeField] private Material _compositeMaterial;
    private ContourPass _pass;

    public bool IsContourRenderingAvailable => isActive && _pass != null && _pass.IsSupported;

    public override void Create()
    {
        _pass?.Dispose();
        Material depth = _depthNormalsMaterial != null ? _depthNormalsMaterial : Resources.Load<Material>("RobotContours/DepthNormalsMaterial");
        Material composite = _compositeMaterial != null ? _compositeMaterial : Resources.Load<Material>("RobotContours/CompositeMaterial");
        _pass = depth != null && composite != null ? new ContourPass(depth, composite) : null;
        RobotContourDisplay.SetRenderingAvailable(this, IsContourRenderingAvailable);
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        bool available = IsContourRenderingAvailable;
        RobotContourDisplay.SetRenderingAvailable(this, available);
        if (!RobotContourDisplay.Enabled || renderingData.cameraData.renderType != CameraRenderType.Base) return;
        CameraType type = renderingData.cameraData.camera.cameraType;
        if (type != CameraType.Game && type != CameraType.SceneView) return;
        if (!available) return;
        if (_pass.Prepare()) renderer.EnqueuePass(_pass);
    }

    public override void OnCameraPreCull(ScriptableRenderer renderer, in CameraData cameraData)
    {
        CameraType type = cameraData.camera.cameraType;
        RobotContourDisplay.SetCameraRenderingAvailable(this, cameraData.camera,
            IsContourRenderingAvailable && cameraData.renderType == CameraRenderType.Base &&
            (type == CameraType.Game || type == CameraType.SceneView));
    }

    protected override void Dispose(bool disposing)
    {
        RobotContourDisplay.SetRenderingAvailable(this, false);
        _pass?.Dispose();
        _pass = null;
    }

    private sealed class ContourPass : ScriptableRenderPass
    {
        private static readonly int DepthRangeId = Shader.PropertyToID("_ContourDepthRange");
        private static readonly int ColorId = Shader.PropertyToID("_ContourColor");
        private static readonly int TexelId = Shader.PropertyToID("_ContourTexelSize");
        private static readonly int WidthId = Shader.PropertyToID("_ContourWidth");
        private static readonly Vector4 UnitScaleBias = new Vector4(1f, 1f, 0f, 0f);
        private readonly Material _depthMaterial;
        private readonly Material _compositeMaterial;
        private readonly List<RobotContourDraw> _draws = new List<RobotContourDraw>();
        private RTHandle _normalDepth;
        private RTHandle _privateDepth;

        public bool IsSupported => _depthMaterial != null && _compositeMaterial != null &&
            _depthMaterial.shader != null && _compositeMaterial.shader != null &&
            _depthMaterial.shader.isSupported && _compositeMaterial.shader.isSupported;

        public ContourPass(Material depth, Material composite)
        {
            _depthMaterial = new Material(depth) { name = "Live robot private normals/depth", hideFlags = HideFlags.HideAndDontSave };
            _compositeMaterial = new Material(composite) { name = "Live robot visible contour composite", hideFlags = HideFlags.HideAndDontSave };
            renderPassEvent = RenderPassEvent.BeforeRenderingTransparents;
            // Match private-buffer orientation to the camera color target. URP
            // performs the final XR/backbuffer flip and tracks the native stereo
            // matrices for late latching; a cached material VP cannot do either.
            requiresIntermediateTexture = true;
        }

        public bool Prepare() => RobotContourDisplay.TryGetDraws(_draws);

        private static RenderTextureDescriptor ColorDescriptor(RenderTextureDescriptor camera)
        {
            camera.msaaSamples = 1;
            camera.bindMS = false;
            camera.depthStencilFormat = GraphicsFormat.None;
            camera.graphicsFormat = GraphicsFormat.R8G8B8A8_UNorm;
            camera.enableRandomWrite = false;
            camera.useMipMap = false;
            camera.autoGenerateMips = false;
            return camera;
        }

        private static RenderTextureDescriptor DepthDescriptor(RenderTextureDescriptor color)
        {
            color.graphicsFormat = GraphicsFormat.None;
            color.depthStencilFormat = SystemInfo.GetGraphicsFormat(DefaultFormat.DepthStencil);
            return color;
        }

        private void SetCompositeProperties(int width, int height, float depthRange, Color color, float lineWidth)
        {
            _compositeMaterial.SetColor(ColorId, color);
            _compositeMaterial.SetVector(TexelId, new Vector4(1f / Mathf.Max(1, width), 1f / Mathf.Max(1, height), width, height));
            _compositeMaterial.SetFloat(DepthRangeId, depthRange);
            _compositeMaterial.SetFloat(WidthId, lineWidth);
        }

#pragma warning disable 618, 672 // Compatibility mode remains supported beside RenderGraph.
        public override void OnCameraSetup(CommandBuffer cmd, ref RenderingData renderingData)
        {
            RenderTextureDescriptor color = ColorDescriptor(renderingData.cameraData.cameraTargetDescriptor);
            RenderTextureDescriptor depth = DepthDescriptor(color);
            RenderingUtils.ReAllocateHandleIfNeeded(ref _normalDepth, color, FilterMode.Point, TextureWrapMode.Clamp, name: "_RobotPrivateNormalDepth");
            RenderingUtils.ReAllocateHandleIfNeeded(ref _privateDepth, depth, FilterMode.Point, TextureWrapMode.Clamp, name: "_RobotPrivateDepth");
        }

        public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
        {
            if (!RobotContourDisplay.Enabled || _normalDepth == null || !Prepare()) return;
            CameraData cameraData = renderingData.cameraData;
            float range = Mathf.Clamp(cameraData.camera.farClipPlane, 1f, 100f);
            _depthMaterial.SetFloat(DepthRangeId, range);
            SetCompositeProperties(cameraData.cameraTargetDescriptor.width, cameraData.cameraTargetDescriptor.height,
                range, RobotContourDisplay.LineColor, RobotContourDisplay.LineWidthPixels);
            CommandBuffer cmd = CommandBufferPool.Get("Live robot visible contours");
            try
            {
                CoreUtils.SetRenderTarget(cmd, _normalDepth, _privateDepth, ClearFlag.All, Color.clear);
                foreach (RobotContourDraw draw in _draws)
                    if (draw.Mesh != null) cmd.DrawMesh(draw.Mesh, draw.LocalToWorld, _depthMaterial, draw.SubMeshIndex, 0);
                // Main depth is deliberately not attached or written by the composite.
                CoreUtils.SetRenderTarget(cmd, renderingData.cameraData.renderer.cameraColorTargetHandle, ClearFlag.None);
                Blitter.BlitTexture(cmd, _normalDepth, UnitScaleBias, _compositeMaterial, 0);
                // Compatibility URP caches its last attachments. Restore that binding
                // after our private targets so later transparents retain normal scene depth.
                CoreUtils.SetRenderTarget(cmd, renderingData.cameraData.renderer.cameraColorTargetHandle,
                    renderingData.cameraData.renderer.cameraDepthTargetHandle, ClearFlag.None);
                context.ExecuteCommandBuffer(cmd);
            }
            finally { CommandBufferPool.Release(cmd); }
        }
#pragma warning restore 618, 672

        private sealed class DepthPassData
        {
            public RobotContourDraw[] Draws;
            public Material Material;
            public float DepthRange;
        }

        private sealed class CompositePassData
        {
            public TextureHandle Source;
            public Material Material;
            public Color Color;
            public float Width;
            public float DepthRange;
            public int PixelWidth;
            public int PixelHeight;
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            if (!RobotContourDisplay.Enabled || !Prepare()) return;
            UniversalCameraData cameraData = frameData.Get<UniversalCameraData>();
            UniversalResourceData resources = frameData.Get<UniversalResourceData>();
            RenderTextureDescriptor color = ColorDescriptor(cameraData.cameraTargetDescriptor);
            // Alpha zero is the no-surface sentinel in the packed depth; TextureDesc's
            // default black has alpha one, so explicitly request a transparent clear.
            TextureHandle normalDepth = renderGraph.CreateTexture(new TextureDesc(color)
            {
                name = "Robot private normals/depth", clearBuffer = true, clearColor = Color.clear,
                filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp
            });
            TextureHandle privateDepth = UniversalRenderer.CreateRenderGraphTexture(renderGraph, DepthDescriptor(color), "Robot private visibility depth", true);
            float depthRange = Mathf.Clamp(cameraData.camera.farClipPlane, 1f, 100f);
            using (var builder = renderGraph.AddRasterRenderPass<DepthPassData>("Live robot private depth", out DepthPassData data))
            {
                data.Draws = _draws.ToArray();
                data.Material = _depthMaterial;
                data.DepthRange = depthRange;
                builder.SetRenderAttachment(normalDepth, 0, AccessFlags.Write);
                builder.SetRenderAttachmentDepth(privateDepth, AccessFlags.Write);
                builder.SetRenderFunc((DepthPassData pass, RasterGraphContext context) =>
                {
                    pass.Material.SetFloat(DepthRangeId, pass.DepthRange);
                    foreach (RobotContourDraw draw in pass.Draws)
                        if (draw.Mesh != null) context.cmd.DrawMesh(draw.Mesh, draw.LocalToWorld, pass.Material, draw.SubMeshIndex, 0);
                });
            }
            using (var builder = renderGraph.AddRasterRenderPass<CompositePassData>("Live robot visible contour lines", out CompositePassData data))
            {
                data.Source = normalDepth;
                data.Material = _compositeMaterial;
                data.Color = RobotContourDisplay.LineColor;
                data.Width = RobotContourDisplay.LineWidthPixels;
                data.DepthRange = depthRange;
                data.PixelWidth = color.width;
                data.PixelHeight = color.height;
                builder.UseTexture(normalDepth, AccessFlags.Read);
                // Blend into existing color. No camera depth attachment, no camera-color sampling.
                builder.SetRenderAttachment(resources.activeColorTexture, 0, AccessFlags.ReadWrite);
                builder.SetRenderFunc((CompositePassData pass, RasterGraphContext context) =>
                {
                    pass.Material.SetColor(ColorId, pass.Color);
                    pass.Material.SetFloat(WidthId, pass.Width);
                    pass.Material.SetFloat(DepthRangeId, pass.DepthRange);
                    pass.Material.SetVector(TexelId, new Vector4(1f / pass.PixelWidth, 1f / pass.PixelHeight, pass.PixelWidth, pass.PixelHeight));
                    Blitter.BlitTexture(context.cmd, pass.Source, UnitScaleBias, pass.Material, 0);
                });
            }
        }

        public void Dispose()
        {
            _normalDepth?.Release();
            _privateDepth?.Release();
            CoreUtils.Destroy(_depthMaterial);
            CoreUtils.Destroy(_compositeMaterial);
        }
    }
}
