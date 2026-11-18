using UnityEngine;

/// <summary>Owns a quiet synthesized contact cue shared by the interactive guides.</summary>
[DisallowMultipleComponent]
public sealed class GuideChime : MonoBehaviour
{
    private AudioSource _source;
    private AudioClip _clip;

    public void PlayCue()
    {
        if (!isActiveAndEnabled) return;
        if (_source == null)
        {
            const int frequency = 44100;
            const float duration = .24f;
            var samples = new float[(int)(frequency * duration)];
            for (int i = 0; i < samples.Length; i++)
            {
                float time = i / (float)frequency;
                float attack = Mathf.Clamp01(time / .008f);
                float envelope = attack * Mathf.Exp(-time * 19f) * Mathf.Clamp01((duration - time) / .025f);
                samples[i] = envelope * (.24f * Mathf.Sin(2f * Mathf.PI * 740f * time) +
                    .08f * Mathf.Sin(2f * Mathf.PI * 1110f * time));
            }
            _clip = AudioClip.Create("Guide contact chime", samples.Length, 1, frequency, false);
            _clip.SetData(samples, 0);
            _source = gameObject.AddComponent<AudioSource>();
            _source.playOnAwake = false;
            _source.spatialBlend = 0f;
            _source.volume = .32f;
        }
        _source.PlayOneShot(_clip);
    }

    private void OnDisable()
    {
        if (_source != null) _source.Stop();
    }

    private void OnDestroy()
    {
        if (_source != null) Dispose(_source);
        if (_clip != null) Dispose(_clip);
    }

    private static void Dispose(Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }
}
