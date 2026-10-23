#!/usr/bin/env python3
"""Prepare a minimal isolated Android Gradle export to verify Unity's icon layers.

Copies the current icon settings and four branded textures. The isolated export
uses Mono/ARMv7 to avoid compiling the real app or invoking IL2CPP/Gradle. It never
opens the active project, installs packages, deploys an APK or starts hardware.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil


PROJECT = Path(__file__).resolve().parents[2]

EDITOR_SOURCE = r'''using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using UnityEditor;
using UnityEditor.Android;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class AndroidIconExportValidation
{
    [Serializable] private sealed class PixelStats {
        public string path; public int width, height, uniqueColors, minAlpha = 255, maxAlpha;
        public int firstR, firstG, firstB, firstA; public string sha256;
    }
    [Serializable] private sealed class Slot {
        public int width, height; public string layer0Background, layer1Foreground;
    }
    [Serializable] private sealed class Result {
        public bool success; public string completedUtc, unityVersion, exportResult, error;
        public Slot[] adaptiveSlots; public PixelStats foreground, background;
        public string adaptiveXml, adaptiveXmlPath;
        public bool gradleInvoked = false, deviceDeployed = false;
    }

    public static void Export()
    {
        string project = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        Result result = new Result { unityVersion = Application.unityVersion };
        try
        {
            PlatformIcon[] icons = PlayerSettings.GetPlatformIcons(NamedBuildTarget.Android, AndroidPlatformIconKind.Adaptive);
            if (icons.Length != 6) throw new InvalidOperationException("Expected all six adaptive icon density slots.");
            List<Slot> slots = new List<Slot>();
            foreach (PlatformIcon icon in icons)
            {
                Texture2D background = icon.GetTexture(0), foreground = icon.GetTexture(1);
                slots.Add(new Slot { width = icon.width, height = icon.height,
                    layer0Background = background != null ? background.name : null,
                    layer1Foreground = foreground != null ? foreground.name : null });
                if (background == null || background.name != "UnityROSML-Adaptive-Background" ||
                    foreground == null || foreground.name != "UnityROSML-Adaptive-Foreground")
                    throw new InvalidOperationException("Adaptive slot does not use background layer 0 and foreground layer 1.");
            }
            result.adaptiveSlots = slots.ToArray();
            PlayerSettings.productName = "UnityROSML Icon Validation";
            PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Android, "com.buildwise.unityrosml.iconvalidation");
            PlayerSettings.SetScriptingBackend(NamedBuildTarget.Android, ScriptingImplementation.Mono2x);
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARMv7;
            PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.Android.targetSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.Android.useCustomKeystore = false;
            EditorUserBuildSettings.exportAsGoogleAndroidProject = true;
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            string scenePath = "Assets/IconValidation.unity";
            EditorSceneManager.SaveScene(scene, scenePath);
            string export = Path.Combine(project, "icon-export");
            BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
                scenes = new[] { scenePath }, target = BuildTarget.Android, locationPathName = export,
                options = BuildOptions.AcceptExternalModificationsToPlayer
            });
            result.exportResult = report.summary.result.ToString();
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException("Isolated Android export failed; see the editor log.");
            string density = Path.Combine(export, "launcher/src/main/res/mipmap-xxxhdpi");
            result.foreground = Inspect(Path.Combine(density, "ic_launcher_foreground.png"));
            result.background = Inspect(Path.Combine(density, "ic_launcher_background.png"));
            result.adaptiveXmlPath = Path.Combine(export, "launcher/src/main/res/mipmap-anydpi-v26/app_icon.xml");
            result.adaptiveXml = File.ReadAllText(result.adaptiveXmlPath);
            if (!result.adaptiveXml.Contains("<background android:drawable=\"@mipmap/ic_launcher_background\"") ||
                !result.adaptiveXml.Contains("<foreground android:drawable=\"@mipmap/ic_launcher_foreground\""))
                throw new InvalidOperationException("Adaptive XML references are incorrect.");
            if (result.foreground.uniqueColors < 100 || result.foreground.minAlpha != 0 || result.foreground.maxAlpha != 255)
                throw new InvalidOperationException("Exported foreground does not contain the transparent robot artwork.");
            if (result.background.uniqueColors != 1 || result.background.minAlpha != 255 ||
                result.background.firstR != 16 || result.background.firstG != 38 || result.background.firstB != 56)
                throw new InvalidOperationException("Exported background is not solid opaque #102638 navy.");
            result.success = true;
            Debug.Log("AndroidIconValidation: generated foreground is transparent robot; background is opaque navy.");
        }
        catch (Exception exception)
        {
            result.error = exception.ToString();
            Debug.LogError(exception);
        }
        finally
        {
            result.completedUtc = DateTime.UtcNow.ToString("O");
            File.WriteAllText(Path.Combine(project, "icon-validation-result.json"), JsonUtility.ToJson(result, true), new UTF8Encoding(false));
        }
        if (!result.success) EditorApplication.Exit(1);
    }

    private static PixelStats Inspect(string path)
    {
        byte[] bytes = File.ReadAllBytes(path);
        Texture2D texture = new Texture2D(2, 2, TextureFormat.RGBA32, false, true);
        try
        {
            if (!ImageConversion.LoadImage(texture, bytes, false)) throw new InvalidOperationException("Cannot decode generated PNG " + path);
            PixelStats stats = new PixelStats { path = path, width = texture.width, height = texture.height };
            HashSet<uint> colors = new HashSet<uint>();
            Color32[] pixels = texture.GetPixels32();
            Color32 first = pixels[0]; stats.firstR = first.r; stats.firstG = first.g; stats.firstB = first.b; stats.firstA = first.a;
            foreach (Color32 pixel in pixels)
            {
                colors.Add((uint)(pixel.r | pixel.g << 8 | pixel.b << 16 | pixel.a << 24));
                stats.minAlpha = Math.Min(stats.minAlpha, pixel.a); stats.maxAlpha = Math.Max(stats.maxAlpha, pixel.a);
            }
            stats.uniqueColors = colors.Count;
            using (SHA256 hash = SHA256.Create()) {
                StringBuilder value = new StringBuilder();
                foreach (byte b in hash.ComputeHash(bytes)) value.Append(b.ToString("x2"));
                stats.sha256 = value.ToString();
            }
            return stats;
        }
        finally { UnityEngine.Object.DestroyImmediate(texture); }
    }
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=PROJECT / "Library/AndroidIconValidation")
    args = parser.parse_args()
    destination = args.destination.resolve()
    if destination == PROJECT or (destination / "Assets/Scenes/UR16eInteraction.unity").exists():
        raise ValueError("Refusing to prepare over the active project")
    settings = destination / "ProjectSettings"
    settings.mkdir(parents=True, exist_ok=True)
    original = PROJECT / "ProjectSettings/ProjectSettings.asset"
    content = original.read_text(encoding="utf-8-sig")
    # Only these isolated validation settings differ: no custom manifest, legacy input,
    # no IL2CPP compile. Icon YAML remains an exact copy of the approved source.
    content = re.sub(r"(?m)^(  useCustomMainManifest: )1$", r"\g<1>0", content)
    content = re.sub(r"(?m)^(  activeInputHandler: )\d+$", r"\g<1>0", content)
    (settings / "ProjectSettings.asset").write_text(content, encoding="utf-8")
    shutil.copy2(PROJECT / "ProjectSettings/ProjectVersion.txt", settings / "ProjectVersion.txt")
    plugins = destination / "Packages"
    plugins.mkdir(parents=True, exist_ok=True)
    (plugins / "manifest.json").write_text(json.dumps({"dependencies": {
        "com.unity.modules.androidjni": "1.0.0", "com.unity.modules.imageconversion": "1.0.0",
        "com.unity.modules.jsonserialize": "1.0.0", "com.unity.modules.ui": "1.0.0"}}, indent=2) + "\n")
    editor = destination / "Assets/Editor"
    editor.mkdir(parents=True, exist_ok=True)
    (editor / "AndroidIconExportValidation.cs").write_text(EDITOR_SOURCE, encoding="utf-8")
    sources = []
    for source in (PROJECT / "Assets/Branding/UnityROSML").glob("*.png"):
        target = destination / source.relative_to(PROJECT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        shutil.copy2(Path(str(source) + ".meta"), Path(str(target) + ".meta"))
        sources.append({"path": source.relative_to(PROJECT).as_posix(), "sha256": hashlib.sha256(source.read_bytes()).hexdigest()})
    (destination / "preparation-report.json").write_text(json.dumps({
        "sourceProject": str(PROJECT), "sourceSettingsSha256": hashlib.sha256(original.read_bytes()).hexdigest(),
        "sourceIconTextures": sources,
        "changesInIsolatedCopyOnly": ["custom Android manifest disabled", "legacy input", "Mono ARMv7 Gradle export"],
        "gradleBuildRequested": False, "deviceDeploymentRequested": False
    }, indent=2) + "\n")
    print(f"Prepared minimal Android icon export validation: {destination}")


if __name__ == "__main__":
    main()
