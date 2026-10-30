#!/usr/bin/env python3
"""Record contour alignment, ghost looping, registration guide and Android shader checks."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

from record_execution_improvements_validation import imported_material_text


PROJECT = Path(__file__).resolve().parents[2]
ISOLATED = PROJECT / "Library/PointRegistrationValidation"
PLAYER = PROJECT / "Library/RobotContourPlayerValidation"
OUTPUT = PROJECT / "Tools/RobotFrames/ContourValidation"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_copy(relative, destination):
    source, tested = PROJECT / relative, destination / relative
    record = {"path": relative.as_posix(), "sha256": digest(source)}
    if digest(source) != digest(tested):
        reason = None
        if source.suffix == ".mat" and imported_material_text(source) == imported_material_text(tested):
            reason = "Only URP importer-added disabled MOTIONVECTORS/SHADOWCASTER passes differ."
        elif source.name == "UniversalRenderPipelineGlobalSettings.asset":
            # Unity regenerates the runtime-only settings reference list on
            # editor import. All actual settings and references must still match.
            def normalize(path):
                return re.sub(r"(    m_RuntimeSettings:\n)      m_List:(?: \[\])?(?:\n      - rid: \d+)*",
                              r"\1      m_List: []", path.read_text(encoding="utf-8-sig"))
            if normalize(source) == normalize(tested):
                reason = "Only Unity's regenerated m_RuntimeSettings reference list differs; actual settings match."
        elif source.name == "Mobile_RPAsset.asset" and destination == PLAYER:
            # URP recalculates these build-time keyword prefilters. This harness
            # has no XR loader, so record the exact differences and the limit.
            fields = ("m_PrefilteringModeAdditionalLight", "m_PrefilterXRKeywords")
            pattern = r"(?m)^  (" + "|".join(fields) + r"): (\d+)$"
            original = source.read_text(encoding="utf-8-sig")
            built = tested.read_text(encoding="utf-8-sig")
            if re.sub(pattern, r"  \1: GENERATED", original) == re.sub(pattern, r"  \1: GENERATED", built):
                record["buildGeneratedPrefilters"] = {
                    "source": dict(re.findall(pattern, original)), "tested": dict(re.findall(pattern, built))}
                reason = "Only URP-generated additional-light/XR keyword prefilters differ for the isolated non-XR Android export."
        if reason is None:
            raise ValueError("Current source differs from tested copy: " + str(relative))
        record["testedSha256"] = digest(tested)
        record["importNormalization"] = reason
    return record


def main():
    results = ISOLATED / "contour-graphics-results.xml"
    run = ET.parse(results).getroot()
    if run.get("result") != "Passed" or int(run.get("failed", "1")) or int(run.get("skipped", "1")):
        raise ValueError("A complete graphics-enabled passing Unity suite is required")
    graphics_log = ISOLATED / "contour-previews.log"
    log = graphics_log.read_text(encoding="utf-8-sig", errors="replace")
    if "RobotContourGraphicsValidation passed:" not in log:
        raise ValueError("The actual installed contour feature must pass its pixel checks")
    if "RobotInteractionPreviews passed:" not in log or len(re.findall(r"RobotContourWorldAnchoring ", log)) != 4:
        raise ValueError("All four camera-motion checks and the integrated guide previews are required")
    negative_path = ISOLATED / "contour-old-shader-negative-results.xml"
    negative = ET.parse(negative_path).getroot()
    negative_cases = [case for case in negative.iter("test-case") if case.get("result") == "Failed"]
    if (int(negative.get("failed", "0")) != 1 or len(negative_cases) != 1 or
            negative_cases[0].get("name") != "DepthShaderUsesTheRenderTimeNativeViewInsteadOfPreparedMaterialMatrices" or
            "But was:  0" not in negative_cases[0].findtext("failure/message", "")):
        raise ValueError("The previous shader must fail the render-time view regression")
    player_path = PLAYER / "contour-player-validation-result.json"
    player = json.loads(player_path.read_text(encoding="utf-8-sig"))
    if not player["success"] or player["buildResult"] != "Succeeded":
        raise ValueError("A successful full offline Android shader export is required")

    paths = []
    for folder in ("Assets/Scripts/V3", "Assets/Tests/RobotFrames"):
        paths += [path.relative_to(PROJECT) for path in (PROJECT / folder).rglob("*.cs")
                  if (ISOLATED / path.relative_to(PROJECT)).is_file()]
    for folder in ("Assets/Resources/RobotContours", "Assets/Resources/RobotTargetMarkers"):
        paths += [path.relative_to(PROJECT) for path in (PROJECT / folder).rglob("*") if path.is_file()]
    paths += [Path("Assets/Resources/RobotModels/Models.json"),
              Path("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab")]
    for asset in ("Mobile_RPAsset.asset", "Mobile_Renderer.asset", "PC_RPAsset.asset", "PC_Renderer.asset",
                  "UniversalRenderPipelineGlobalSettings.asset"):
        paths += [Path("Assets/Settings") / asset, Path("Assets/Settings") / (asset + ".meta")]
    hashes = [checked_copy(path, ISOLATED) for path in sorted(set(paths))]
    player_hashes = [checked_copy(path, PLAYER) for path in sorted(set(paths))
                     if path.parts[:3] == ("Assets", "Resources", "RobotContours")
                     or path.parts[:4] == ("Assets", "Scripts", "V3", "Rendering")
                     or path.as_posix() in ("Assets/Settings/Mobile_Renderer.asset", "Assets/Settings/Mobile_RPAsset.asset")]
    helper = PROJECT / "Tools/RobotFrames/RobotContourPlayerValidation.cs"
    if digest(helper) != digest(PLAYER / "Assets/Editor/RobotContourPlayerValidation.cs"):
        raise ValueError("Android export helper differs from the tested copy")
    compile_path = PROJECT / "Tools/RobotFrames/runtime_compile_summary.json"
    compile_summary = json.loads(compile_path.read_text(encoding="utf-8-sig"))
    for source in compile_summary["validatedSources"]:
        if digest(PROJECT / source["path"]) != source["sha256"]:
            raise ValueError("Runtime compilation is stale: " + source["path"])

    OUTPUT.mkdir(exist_ok=True)
    images = []
    states = ("hidden-rear", "visible-rear", "execution", "disabled-feature-fallback",
              "ur16e-normal", "ur16e-contours", "ur16e-contours-and-path-ghost")
    previews = [ISOLATED / "RobotContourPreviews" / f"{renderer}-{mode}-{state}.png"
                for renderer in ("Mobile", "PC") for mode in ("RenderGraph", "Compatibility") for state in states]
    previews += [ISOLATED / "calibration-menu-task.png", ISOLATED / "calibration-menu-pick-place.png"]
    previews += [ISOLATED / "point-placement-menu.png"]
    previews += [ISOLATED / "RegistrationGuidePreviews" / f"step-{step}.png" for step in range(1, 5)]
    if any(not path.is_file() for path in previews):
        raise ValueError("Contour and main-menu image evidence is incomplete")
    for source in sorted(previews):
        target = OUTPUT / ("registration-" + source.name if source.parent.name == "RegistrationGuidePreviews" else source.name)
        shutil.copy2(source, target)
        images.append({"path": target.relative_to(PROJECT).as_posix(), "sha256": digest(target)})
    shutil.copy2(player_path, OUTPUT / "android-player-result.json")
    shutil.copy2(negative_path, OUTPUT / "old-shader-negative-results.xml")
    summary = {
        "schemaVersion": 1,
        "completedUtc": datetime.now(timezone.utc).isoformat(),
        "scope": "Tracked contour alignment with private hidden-line depth, optional ghost playback loop and step-by-step UR16e cover miniature.",
        "unityVersion": "6000.0.84f1",
        "unityTests": {key: int(run.get(key, "0")) for key in ("total", "passed", "failed", "skipped")},
        "fixtures": sorted({case.get("classname") for case in run.iter("test-case")}),
        "testResults": {"path": results.relative_to(PROJECT).as_posix(), "sha256": digest(results)},
        "graphics": {"log": graphics_log.relative_to(PROJECT).as_posix(), "sha256": digest(graphics_log),
                     "pixelChecks": re.findall(r"RobotContourPixels [^\r\n]+", log),
                     "cameraMotionChecks": re.findall(r"RobotContourWorldAnchoring [^\r\n]+", log),
                     "guideChecks": re.findall(r"RegistrationGuidePreview: [^\r\n]+", log),
                     "oldShaderNegativeControl": {"result": "ContourValidation/old-shader-negative-results.xml",
                                                  "resultSha256": digest(negative_path), "expectedFailures": 1},
                     "images": images},
        "validatedSources": hashes,
        "runtimeCompile": {"sourceCount": compile_summary["helperSourceCount"] + compile_summary["mainSourceCount"],
                           "errors": compile_summary["errorCount"], "summarySha256": digest(compile_path)},
        "android": {"result": "ContourValidation/android-player-result.json", "sha256": digest(player_path),
                    "validatedSources": player_hashes, "helperSha256": digest(helper)},
        "limits": [
            "Pixel checks use desktop D3D11 through both installed URP renderer assets, not an ML2 display.",
            "Android Vulkan player-data export compiles and retains shaders; it does not exercise headset stereo or device performance.",
            "Contours remove lines hidden by other live-robot surfaces using private depth. They do not reconstruct real-world occlusion.",
            "ROS transport, hardware motion, deployment and app installation were not exercised.",
        ],
        "hardwareExercised": False,
        "robotCommandsSent": False,
        "deployed": False,
    }
    path = PROJECT / "Tools/RobotFrames/contour_validation_summary.json"
    path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {summary['unityTests']['passed']} Unity checks, {len(images)} previews and Android shader retention")


if __name__ == "__main__":
    main()
