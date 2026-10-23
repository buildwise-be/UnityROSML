UnityROSML — UR16e app logo

A simplified white-and-cyan UR16e arm in the reference robot's bent Z pose.
The icon contains no text, so it stays readable at small launcher sizes.

Files
- UnityROSML-Launcher.png: opaque navy square, 1254 x 1254; assigned to
  all Android Legacy and Round icon slots. Corners are not pre-masked.
- UnityROSML-Logo-Transparent.png: general-purpose transparent logo,
  1254 x 1254. Assigned to the task/tool and robot alignment menu headers
  in Assets/Scenes/UR16eInteraction.unity. Also usable for documentation
  and presentations.
- UnityROSML-Adaptive-Foreground.png: transparent foreground with extra
  padding, 1254 x 1254; assigned as adaptive layer 1 (Foreground).
- UnityROSML-Adaptive-Background.png: solid #102638 background,
  1280 x 1280; assigned as adaptive layer 0 (Background).
- Design-Prompts.txt: final artwork prompts. Built-in image_gen was used;
  the CLI/API fallback was not used. Background is a solid-color PNG.

Unity / Magic Leap 2
The Android icon slots are assigned in ProjectSettings/ProjectSettings.asset.
Unity stores Android adaptive layers as Background first, Foreground second.
The earlier reversed assignment made the opaque navy layer cover the robot.
Unity scales the source textures to its Android density slots. Texture
imports retain alpha, disable compression and mipmaps, and use clamp edges.
The XR menu uses the existing transparent texture directly; its UV rectangle
omits empty padding so the robot stays legible in the small header. The image
pixels are unchanged, and the logo does not intercept controller clicks.
No administrator rights, additional packages or manifest changes are needed.

The adaptive artwork was checked against Android's 66/108 safe-circle
diameter: meaningful artwork extends to radius 0.2412 of the image width,
inside the 0.3056 safe radius. Its transparent padding is intentional.
After correcting the layer order, an isolated Unity 6000.0.84f1 Android Gradle
export verified the six adaptive slots and freshly generated resources:
transparent robot foreground, opaque navy background, correct adaptive XML.
See Tools/RobotFrames/android_icon_validation_summary.json. This verification
did not build the real app APK or launch it on a Magic Leap 2 device.

Official compatibility references
https://developer.android.com/develop/ui/compose/system/icon_design_adaptive
https://docs.unity3d.com/6000.0/Documentation/Manual/class-PlayerSettingsAndroid.html
https://developer-docs.magicleap.cloud/docs/guides/openxr/getting-started/android-manifest-settings/

The supplied robot photograph was a visual reference. The Universal Robots
wordmark was not included in the app logo.
