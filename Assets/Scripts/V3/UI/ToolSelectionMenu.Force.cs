using System;
using System.Globalization;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

/// <summary>Read-only force telemetry beside the task controls in either workflow.</summary>
public sealed partial class ToolSelectionMenu
{
    [FormerlySerializedAs("_forceGaugeInitialMaximum")]
    [SerializeField, Min(0.01f), Tooltip("Fixed force gauge maximum in N. Values above this range remain visible numerically.")]
    private float _forceGaugeMaximum = 100f;
    [FormerlySerializedAs("_torqueGaugeInitialMaximum")]
    [SerializeField, Min(0.01f), Tooltip("Fixed torque gauge maximum in Nm. Values above this range remain visible numerically.")]
    private float _torqueGaugeMaximum = 20f;

    private sealed class MagnitudeGauge
    {
        public TextMeshProUGUI CurrentValue;
        public TextMeshProUGUI PeakValue;
        public TextMeshProUGUI MaximumValue;
        public RectTransform Fill;
        public RectTransform PeakMarker;
        public double Maximum;
        public string Unit;
    }

    private ForceTorqueSubscriber _forceTorque;
    private TextMeshProUGUI _forceStatus;
    private TextMeshProUGUI _forceReading;
    private TextMeshProUGUI _forcePeakHint;
    private MagnitudeGauge _forceMagnitudeGauge;
    private MagnitudeGauge _torqueMagnitudeGauge;
    private float _nextForceReadoutAt;
    private const float MagnitudeGaugeWidth = 264f;
    private static readonly Color ExecutionPeakColor = new Color(1f, 0.78f, 0.38f);

    private void BuildForceReadout()
    {
        // A side card preserves the space used by planning and measured alignment.
        RectTransform card = Rect("UR16e force readout", _taskView,
            new Vector2(456f, -44f), new Vector2(300f, 390f));
        Image background = card.gameObject.AddComponent<Image>();
        background.color = new Color(0.035f, 0.06f, 0.09f, 0.97f);
        Outline border = card.gameObject.AddComponent<Outline>();
        border.effectColor = new Color(0.2f, 0.44f, 0.53f, 0.8f);
        border.effectDistance = new Vector2(1f, -1f);
        AddHoverTracking(card.gameObject);
        Text("Force readout title", card, new Vector2(12f, -10f),
            new Vector2(276f, 26f), "UR16e force / torque", 18f);
        _forceStatus = Text("Force sample status", card, new Vector2(12f, -41f),
            new Vector2(276f, 38f), "Waiting for force samples", 13f);
        _forceStatus.richText = false;
        _forceStatus.alignment = TextAlignmentOptions.TopLeft;
        _forceStatus.color = ExecutionPeakColor;
        _forceMagnitudeGauge = BuildMagnitudeGauge(card, "Force", "N", 84f,
            _forceGaugeMaximum, 100d, new Color(0.23f, 0.77f, 0.85f));
        GaugeImage("Gauge divider", card, new Vector2(12f, -197f), new Vector2(276f, 1f),
            new Color(0.16f, 0.24f, 0.3f));
        _torqueMagnitudeGauge = BuildMagnitudeGauge(card, "Torque", "Nm", 207f,
            _torqueGaugeMaximum, 20d, new Color(0.61f, 0.57f, 0.91f));
        _forceReading = Text("Force sample frame", card, new Vector2(12f, -329f),
            new Vector2(276f, 20f), "Frame: —", 12f);
        _forceReading.richText = false;
        _forceReading.alignment = TextAlignmentOptions.TopLeft;
        _forceReading.enableAutoSizing = true;
        _forceReading.fontSizeMin = 10f;
        _forceReading.fontSizeMax = 12f;
        _forceReading.color = new Color(0.65f, 0.74f, 0.8f);
        _forcePeakHint = Text("Execution peak hint", card, new Vector2(12f, -353f),
            new Vector2(276f, 29f), "Peak starts with Execute", 12f);
        _forcePeakHint.richText = false;
        _forcePeakHint.color = new Color(0.65f, 0.74f, 0.8f);
    }

    private MagnitudeGauge BuildMagnitudeGauge(Transform card, string title, string unit, float top,
        float maximum, double fallbackMaximum, Color fillColor)
    {
        RectTransform root = Rect(title + " magnitude gauge", card,
            new Vector2(12f, -top), new Vector2(276f, 109f));
        Text(title + " magnitude title", root, Vector2.zero, new Vector2(116f, 27f), title, 17f);
        var gauge = new MagnitudeGauge {
            Unit = unit,
            Maximum = maximum > 0f && !float.IsInfinity(maximum) ? maximum : fallbackMaximum
        };
        gauge.CurrentValue = Text("Current magnitude", root, new Vector2(117f, 0f), new Vector2(159f, 27f),
            "— " + unit, 20f);
        gauge.CurrentValue.alignment = TextAlignmentOptions.MidlineRight;
        gauge.CurrentValue.color = fillColor;
        gauge.CurrentValue.richText = false;
        gauge.CurrentValue.enableAutoSizing = true;
        gauge.CurrentValue.fontSizeMin = 13f;
        gauge.CurrentValue.fontSizeMax = 20f;
        Image track = GaugeImage("Gauge track", root, new Vector2(6f, -39f),
            new Vector2(MagnitudeGaugeWidth, 16f), new Color(0.1f, 0.17f, 0.22f));
        gauge.Fill = GaugeImage("Current fill", track.transform, Vector2.zero,
            new Vector2(0f, 16f), fillColor).rectTransform;
        gauge.Fill.gameObject.SetActive(false);
        gauge.PeakMarker = GaugeImage("Execution peak marker", track.transform, new Vector2(0f, 4f),
            new Vector2(2f, 24f), ExecutionPeakColor).rectTransform;
        gauge.PeakMarker.gameObject.SetActive(false);
        Text("Scale minimum", root, new Vector2(6f, -59f), new Vector2(75f, 18f), "0", 11f)
            .color = new Color(0.65f, 0.74f, 0.8f);
        gauge.MaximumValue = Text("Scale maximum", root, new Vector2(91f, -59f), new Vector2(179f, 18f),
            GaugeNumber(gauge.Maximum) + " " + unit, 11f);
        gauge.MaximumValue.alignment = TextAlignmentOptions.MidlineRight;
        gauge.MaximumValue.richText = false;
        gauge.MaximumValue.color = new Color(0.65f, 0.74f, 0.8f);
        gauge.PeakValue = Text("Execution peak value", root, new Vector2(0f, -82f), new Vector2(276f, 23f),
            "Peak —", 13f);
        gauge.PeakValue.richText = false;
        gauge.PeakValue.color = ExecutionPeakColor;
        return gauge;
    }

    private static Image GaugeImage(string name, Transform parent, Vector2 position, Vector2 size, Color color)
    {
        Image image = Rect(name, parent, position, size).gameObject.AddComponent<Image>();
        image.color = color;
        image.raycastTarget = false;
        return image;
    }

    private static string GaugeNumber(double value) => value.ToString(value < 10000d ? "0.00" : "0.###E+0",
        CultureInfo.InvariantCulture);

    private static void RefreshMagnitudeGauge(MagnitudeGauge gauge, bool fresh, double current,
        bool hasPeak, double peak)
    {
        if (gauge == null) return;
        // Clamp visual positions to the fixed scale; retain actual numeric readings and peaks.
        gauge.MaximumValue.text = GaugeNumber(gauge.Maximum) + " " + gauge.Unit;
        gauge.CurrentValue.text = fresh ? GaugeNumber(current) + " " + gauge.Unit : "— " + gauge.Unit;
        gauge.Fill.gameObject.SetActive(fresh);
        gauge.Fill.sizeDelta = new Vector2(fresh ? (float)Math.Min(current / gauge.Maximum, 1d) * MagnitudeGaugeWidth : 0f, 16f);
        gauge.PeakMarker.gameObject.SetActive(hasPeak);
        if (hasPeak)
            gauge.PeakMarker.anchoredPosition = new Vector2((float)Math.Min(peak / gauge.Maximum, 1d) * (MagnitudeGaugeWidth - 2f), 4f);
        gauge.PeakValue.text = hasPeak ? "Peak " + GaugeNumber(peak) + " " + gauge.Unit : "Peak —";
    }

    private void BindForceReadout()
    {
        // BuildMenu is also used by offline UI tests. Only starting the live
        // menu creates a subscriber; it shares the app's existing ROS transport.
        _forceTorque = GetComponent<ForceTorqueSubscriber>();
        if (_forceTorque == null) _forceTorque = gameObject.AddComponent<ForceTorqueSubscriber>();
        _nextForceReadoutAt = 0f;
        RefreshForceReadout();
    }

    private void RefreshForceReadout()
    {
        if (_forceStatus == null || _forceReading == null || _forceTorque == null ||
            Time.unscaledTime < _nextForceReadoutAt) return;
        _nextForceReadoutAt = Time.unscaledTime + 0.1f;
        _forceStatus.text = _forceTorque.StatusText;
        _forceReading.text = "Frame: " + (_forceTorque.IsFresh ? _forceTorque.FrameId : "—");
        _forceStatus.color = _forceTorque.IsFresh
            ? new Color(0.45f, 0.91f, 0.72f) : ExecutionPeakColor;
        RefreshMagnitudeGauge(_forceMagnitudeGauge, _forceTorque.IsFresh, _forceTorque.ForceMagnitude,
            _forceTorque.HasExecutionPeak, _forceTorque.PeakForceMagnitude);
        RefreshMagnitudeGauge(_torqueMagnitudeGauge, _forceTorque.IsFresh, _forceTorque.TorqueMagnitude,
            _forceTorque.HasExecutionPeak, _forceTorque.PeakTorqueMagnitude);
        _forcePeakHint.text = _forceTorque.IsRecordingExecutionPeak ? "Recording execution peaks"
            : _forceTorque.HasExecutionPeak ? "Peak held · resets on Plan or Execute" : "Peak starts with Execute";
    }
}
