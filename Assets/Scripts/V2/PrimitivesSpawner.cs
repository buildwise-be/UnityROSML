using System;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

public class PrimitivesSpawner : MonoBehaviour
{
    [SerializeField]
    private Canvas PrimitiveRotationMenu;
    [SerializeField]
    private Material primitiveMaterial;
    [SerializeField]
    private UIManager _uiManager;

    [SerializeField]
    private GameObject _primitivePrefab;

    [SerializeField]
    [Tooltip("The diameter in meters of spawned spheres")]
    private float _sphereDiameter;

    private bool hasSpawned = false;
    private XRGrabInteractable xrgi;
    private Rigidbody rb;
    private GameObject primitiveInstance;
    private Color initialCubeColor;
    private Vector3 initialCubePosition;

    /// <summary>
    /// Spawn a sphere of diameter _sphereDiameter, 50cm in front of main camera.
    /// </summary>
    public void SpawnSphere()
    {
        var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.position = Camera.main.transform.position + 0.5f * Camera.main.transform.forward;
        sphere.transform.localScale = _sphereDiameter * Vector3.one;
    }

    public void SpawnTile()
    {
        SpawnBox(0.197f, 0.006f, 0.197f);
    }
    public void SpawnBox(float sizex, float sizey, float sizez)
    {
        if (hasSpawned)
        {
            return;
        }
        //primitiveInstance = GameObject.CreatePrimitive(PrimitiveType.Cube);
        primitiveInstance = GameObject.Instantiate(_primitivePrefab);

        primitiveInstance.AddComponent<MeshFaceColorer>();

        primitiveInstance.GetComponent<MeshRenderer>().material = primitiveMaterial;
        initialCubeColor = primitiveInstance.GetComponent<MeshRenderer>().material.color;
        primitiveInstance.transform.position = Camera.main.transform.position + Camera.main.transform.forward;
        primitiveInstance.transform.localScale = new Vector3(sizex, sizey, sizez);
        
        xrgi = primitiveInstance.AddComponent<XRGrabInteractable>();
        if (primitiveInstance.TryGetComponent(out rb))
        {
            rb.useGravity = false;

        }
        else
        {
            rb = primitiveInstance.AddComponent<Rigidbody>();
            //rb.isKinematic = true; //Concave Mesh Colliders are not supported when used with dynamic Rigidbody GameObjects.
            rb.useGravity = false;
        }

        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        xrgi.trackPosition = true;
        xrgi.trackRotation = false;
        xrgi.trackScale = false;
        xrgi.throwOnDetach = false;

        xrgi.activated.AddListener(EnablePrimitiveRotationAndTranslation);
        xrgi.deactivated.AddListener(DisablePrimitiveRotationAndTranslation);

        hasSpawned = true;
    }

    private void DisablePrimitiveRotationAndTranslation(DeactivateEventArgs arg0)
    {
        /*
        primitiveRotationMenuInstance.gameObject.SetActive(false);

        rb.constraints = RigidbodyConstraints.None;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        xrgi.enabled = true;

        xrgi.trackPosition = true;
        xrgi.trackRotation = false;
        xrgi.trackScale = false;
        xrgi.throwOnDetach = false;

        primitiveInstance.GetComponent<MeshRenderer>().material.color = initialCubeColor;
        */
    }

    private void EnablePrimitiveRotationAndTranslation(ActivateEventArgs arg0)
    {
        _uiManager.ShowTileUI(primitiveInstance);
        primitiveInstance.GetComponent<MeshRenderer>().material.color = Color.green;

        /*
        if (primitiveRotationMenuInstance == null)
        {
            primitiveRotationMenuInstance = GameObject.Instantiate(PrimitiveRotationMenu);
        }
        else
        {
            primitiveRotationMenuInstance.gameObject.SetActive(true);
        }

        initialCubePosition = primitiveInstance.transform.position;
        primitiveRotationMenuInstance.transform.position = primitiveInstance.transform.position + Vector3.up * 0.1f;


        xrgi.enabled = false;
        rb.constraints = RigidbodyConstraints.None;

        Slider[] sliders = primitiveRotationMenuInstance.GetComponentsInChildren<Slider>();
        foreach (Slider sl in sliders)
        {
            if (sl.gameObject.name == "RotationSlider")
            {
                sl.onValueChanged.AddListener(RotatePrimitiveAroundY);
            }
            else if (sl.gameObject.name == "HeightSlider")
            {
                sl.onValueChanged.AddListener(MovePrimitiveAlongVerticalAxis);
            }
        }

        Button sendButton = primitiveRotationMenuInstance.GetComponentInChildren<Button>();
        var addBox = sendButton.GetComponent<AddBoxToScene>();
        sendButton.onClick.AddListener(() => addBox.CallSceneBoxTopic(primitiveInstance));
        float pickOffset = primitiveInstance.transform.localScale.y / 2.0f;
        sendButton.onClick.AddListener(() => pickAndPlaceScriptableObject.SetPose(primitiveInstance, Vector3.up, pickOffset, "base_link" ,PoseType.PickPose));
        */
    }

    private void RotatePrimitiveAroundY(float angle)
    {
        var rotation = Quaternion.Euler(0.0f, angle, 0.0f);
        primitiveInstance.transform.SetPositionAndRotation(primitiveInstance.transform.position, rotation);
    }

    private void MovePrimitiveAlongVerticalAxis(float value)
    {
        primitiveInstance.transform.SetPositionAndRotation(initialCubePosition + new Vector3(0, value/1000.0f, 0), primitiveInstance.transform.rotation);
    }
}
