using System;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>The three app scenes share transport, but never a reviewed plan or XR alignment.</summary>
public static class RobotAppNavigation
{
    public const string WelcomeSceneName = "Welcome";
    public const string SimpleSceneName = "SimplePickAndPlace";
    public const string RepetitiveSceneName = "RepetitivePickAndPlace";
    private static bool _changingScene;
    public static bool CanChangeScene => !_changingScene && !RobotExecutionState.IsBusy && MTCPlansManager.ActiveExecution == null;
    public static bool OpenSimplePickAndPlace() => TryNavigate(SimpleSceneName, LoadScene);
    public static bool OpenRepetitivePickAndPlace() => TryNavigate(RepetitiveSceneName, LoadScene);
    public static bool ReturnToWelcome() => TryNavigate(WelcomeSceneName, LoadScene);

    private static bool TryNavigate(string sceneName, Action<string> load)
    {
        if (!CanChangeScene || load == null || (sceneName != WelcomeSceneName && sceneName != SimpleSceneName && sceneName != RepetitiveSceneName)) return false;
        _changingScene = true;
        try
        {
            TargetPoseContextMenu.CloseAll();
            TargetRotationHandles.CancelAllInteractions();
            ToolSelectionMenu.CancelAllInteractions();
            foreach (RobotStateService state in UnityEngine.Object.FindObjectsByType<RobotStateService>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                state.ResetForSceneChange();
            foreach (RobotCoordinateService coordinates in UnityEngine.Object.FindObjectsByType<RobotCoordinateService>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                coordinates.ResetForSceneChange();
            SceneManager.sceneLoaded -= OnSceneLoaded;
            SceneManager.sceneLoaded += OnSceneLoaded;
            load(sceneName);
            return true;
        }
        catch (Exception error)
        {
            _changingScene = false;
            SceneManager.sceneLoaded -= OnSceneLoaded;
            Debug.LogError("Could not open the selected workflow: " + error.Message);
            return false;
        }
    }

    private static void LoadScene(string name) => SceneManager.LoadScene(name, LoadSceneMode.Single);
    private static void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        _changingScene = false;
        SceneManager.sceneLoaded -= OnSceneLoaded;
    }
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Reset()
    {
        _changingScene = false;
        SceneManager.sceneLoaded -= OnSceneLoaded;
    }
}
