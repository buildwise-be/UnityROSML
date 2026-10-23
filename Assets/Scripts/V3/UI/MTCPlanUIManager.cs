using TMPro;
using UnityEngine;

/// <summary>Keeps legacy scene status in sync while the shared robot menu owns task controls.</summary>
public class MTCPlanUIManager : MonoBehaviour
{
    [SerializeField] private MTCPlansManager _MTCPlansManager;
    [SerializeField] private URInputManager _input;
    [SerializeField] private TcpPlanningController _tcpPlanning;
    private MTCPlansManager _subscribedPlans;
    private TcpPlanningController _subscribedTcp;
    private TextMeshProUGUI _textBox;
    private ToolSelectionMenu _sharedMenu;
    private CanvasGroup _visibility;
    private bool? _previousTcpMode;
    private bool _wasSharedMenuVisible;
    private bool _hasStarted;

    private void Start()
    {
        _hasStarted = true;
        _textBox = GetComponentInChildren<TextMeshProUGUI>(true);
        _visibility = GetComponent<CanvasGroup>();
        if (_visibility == null) _visibility = gameObject.AddComponent<CanvasGroup>();
        Bind();
        RefreshPanel();
    }

    private void OnEnable() { if (_hasStarted) { Bind(); RefreshPanel(); } }

    private void Update()
    {
        if (_input == null || _tcpPlanning == null || _MTCPlansManager == null) Bind();
        if (_sharedMenu == null) _sharedMenu = FindFirstObjectByType<ToolSelectionMenu>();
        bool sharedVisible = _sharedMenu != null && _sharedMenu.IsVisible;
        bool tcp = _input == null || _input.IsTcpMode;
        if (_previousTcpMode != tcp || _wasSharedMenuVisible != sharedVisible) RefreshPanel();
    }

    private void Bind()
    {
        if (_input == null) _input = FindFirstObjectByType<URInputManager>();
        if (_tcpPlanning == null) _tcpPlanning = _input != null
            ? _input.GetComponent<TcpPlanningController>() : FindFirstObjectByType<TcpPlanningController>();
        if (_MTCPlansManager == null) _MTCPlansManager = FindFirstObjectByType<MTCPlansManager>();
        if (_subscribedPlans != _MTCPlansManager)
        {
            if (_subscribedPlans != null) _subscribedPlans.OnSuccessfulPlan.RemoveListener(UpdatePlanStatusPanel);
            _subscribedPlans = _MTCPlansManager;
            if (_subscribedPlans != null) _subscribedPlans.OnSuccessfulPlan.AddListener(UpdatePlanStatusPanel);
        }
        if (_subscribedTcp != _tcpPlanning)
        {
            if (_subscribedTcp != null) _subscribedTcp.StateChanged -= RefreshPanel;
            _subscribedTcp = _tcpPlanning;
            if (_subscribedTcp != null) _subscribedTcp.StateChanged += RefreshPanel;
        }
    }

    private void UpdatePlanStatusPanel(bool success)
    {
        RefreshPanel();
    }

    public void ResetPlanStatusPanel()
    {
        // Historical button listeners can still call this after Plan. TCP status
        // belongs to its request controller and must never be replaced by Waiting.
        RefreshPanel();
    }

    private void RefreshPanel()
    {
        bool tcp = _input == null || _input.IsTcpMode;
        _previousTcpMode = tcp;
        if (_textBox != null)
            _textBox.text = tcp
                ? (_tcpPlanning != null ? _tcpPlanning.StatusText : "Capture TCP A and B, then plan a preview.")
                : (_MTCPlansManager != null ? _MTCPlansManager.StatusText : "Capture pick and place surfaces, then Plan.");
        if (_sharedMenu == null) _sharedMenu = FindFirstObjectByType<ToolSelectionMenu>();
        _wasSharedMenuVisible = _sharedMenu != null && _sharedMenu.IsVisible;
        if (_visibility != null)
        {
            _visibility.alpha = _wasSharedMenuVisible ? 0f : 1f;
            _visibility.blocksRaycasts = !_wasSharedMenuVisible;
            _visibility.interactable = !_wasSharedMenuVisible;
        }
    }

    private void Unsubscribe()
    {
        if (_subscribedPlans != null) _subscribedPlans.OnSuccessfulPlan.RemoveListener(UpdatePlanStatusPanel);
        if (_subscribedTcp != null) _subscribedTcp.StateChanged -= RefreshPanel;
        _subscribedPlans = null;
        _subscribedTcp = null;
    }

    private void OnDisable() => Unsubscribe();
    private void OnDestroy() => Unsubscribe();
}
