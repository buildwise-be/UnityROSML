using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>XR welcome screen. Opening it does not create robot controllers or ROS transport.</summary>
public sealed class RobotWelcomeMenu : MonoBehaviour
{
    [SerializeField] private Texture2D _appLogo;
    private Canvas _canvas;
    private Button _simpleButton, _repetitiveButton;
    private bool _positioned;

    private void Start() => BuildMenu();

    private void BuildMenu()
    {
        if (_canvas != null) return;
        GameObject owner = new GameObject("Welcome workflow choices", typeof(RectTransform), typeof(Canvas), typeof(CanvasScaler), typeof(TrackedDeviceGraphicRaycaster));
        owner.transform.SetParent(transform, false);
        _canvas = owner.GetComponent<Canvas>();
        _canvas.renderMode = RenderMode.WorldSpace;
        RectTransform canvas = (RectTransform)_canvas.transform;
        canvas.sizeDelta = new Vector2(520, 400);
        canvas.localScale = Vector3.one * .001f;
        owner.GetComponent<CanvasScaler>().dynamicPixelsPerUnit = 10;
        RectTransform panel = Rect("Welcome", canvas, Vector2.zero, new Vector2(520, 400));
        panel.anchorMin = panel.anchorMax = new Vector2(.5f,.5f);
        panel.pivot = new Vector2(.5f,.5f);
        panel.gameObject.AddComponent<Image>().color = new Color(.035f,.06f,.09f,.97f);
        Outline border = panel.gameObject.AddComponent<Outline>();
        border.effectColor = new Color(.2f,.44f,.53f,.8f); border.effectDistance = new Vector2(1,-1);
        if (_appLogo != null)
        {
            RawImage logo = Rect("App logo", panel, new Vector2(22,-20), new Vector2(55,55)).gameObject.AddComponent<RawImage>();
            logo.texture = _appLogo; logo.raycastTarget = false;
        }
        Text("Welcome title", panel, new Vector2(92,-20),new Vector2(400,40),"Welcome",29);
        Text("Welcome instructions", panel,new Vector2(24,-85),new Vector2(472,32),"Choose your pick-and-place workflow.",18);
        _simpleButton = Choice(panel,"Simple Pick and Place",132,"Move one item using TCP targets or pick/place contacts.",OpenSimplePickAndPlace);
        _repetitiveButton = Choice(panel,"Repetitive Pick and Place",238,"Place a tile pattern using the alignment fixture.",OpenRepetitivePickAndPlace);
        Text("Alignment instructions",panel,new Vector2(24,-347),new Vector2(472,33),"Set up robot alignment inside your selected workflow.",15);
        PositionMenu();
    }

    public void OpenSimplePickAndPlace() => RobotAppNavigation.OpenSimplePickAndPlace();
    public void OpenRepetitivePickAndPlace() => RobotAppNavigation.OpenRepetitivePickAndPlace();

    private void LateUpdate()
    {
        if (_canvas == null) return;
        PositionMenu();
        bool enabled = RobotAppNavigation.CanChangeScene;
        _simpleButton.interactable = _repetitiveButton.interactable = enabled;
    }
    private void PositionMenu()
    {
        Camera camera = Camera.main;
        if (camera == null || _canvas == null) return;
        _canvas.worldCamera = camera;
        if (!_positioned)
        {
            Vector3 forward = Vector3.ProjectOnPlane(camera.transform.forward,Vector3.up);
            if (forward.sqrMagnitude < .001f) forward = Vector3.forward;
            transform.position = camera.transform.position + forward.normalized * 1.1f + Vector3.down * .12f;
            _positioned = true;
        }
        Vector3 facing = transform.position - camera.transform.position;
        if (facing.sqrMagnitude > .001f) transform.rotation = Quaternion.LookRotation(facing,Vector3.up);
    }
    private static Button Choice(Transform parent,string title,float y,string detail,UnityEngine.Events.UnityAction action)
    {
        RectTransform root = Rect(title,parent,new Vector2(24,-y),new Vector2(472,88));
        Image background = root.gameObject.AddComponent<Image>(); background.color = new Color(.10f,.26f,.33f,1);
        Button button = root.gameObject.AddComponent<Button>();button.targetGraphic = background;
        Navigation navigation = button.navigation;navigation.mode = Navigation.Mode.None;button.navigation = navigation;
        button.onClick.AddListener(action);
        Text(title+" label",root,new Vector2(18,-12),new Vector2(436,32),title,23);
        Text(title+" description",root,new Vector2(18,-49),new Vector2(436,27),detail,15);
        return button;
    }
    private static RectTransform Rect(string name,Transform parent,Vector2 position,Vector2 size)
    {
        RectTransform rect = new GameObject(name,typeof(RectTransform)).GetComponent<RectTransform>();
        rect.SetParent(parent,false);rect.anchorMin = rect.anchorMax = new Vector2(0,1);rect.pivot = new Vector2(0,1);
        rect.anchoredPosition = position;rect.sizeDelta = size;return rect;
    }
    private static void Text(string name,Transform parent,Vector2 position,Vector2 size,string value,float fontSize)
    {
        TextMeshProUGUI text = Rect(name,parent,position,size).gameObject.AddComponent<TextMeshProUGUI>();
        text.text = value;text.fontSize = fontSize;text.color = new Color(.87f,.93f,.96f);text.raycastTarget = false;
        text.alignment = TextAlignmentOptions.MidlineLeft;
    }
    private void OnDestroy()
    {
        if (_simpleButton != null) _simpleButton.onClick.RemoveListener(OpenSimplePickAndPlace);
        if (_repetitiveButton != null) _repetitiveButton.onClick.RemoveListener(OpenRepetitivePickAndPlace);
    }
}
