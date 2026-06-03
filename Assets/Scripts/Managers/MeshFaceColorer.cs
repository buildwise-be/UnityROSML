using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

public class MeshFaceColorer : MonoBehaviour
{
    private bool _coloring = false;
    private XRRayInteractor _rayInteractor;
    private XRGrabInteractable _rayInteractable;

    private Material _hoverMaterial;
    public Material HoverMaterial { set { _hoverMaterial = value; } }

    void Update()
    {
        if (_coloring)
        {
            if (_rayInteractor.TryGetCurrent3DRaycastHit(out RaycastHit hit))
            {
                if (hit.transform.IsChildOf(_rayInteractable.transform))
                {
                    int faceIndex = GetFaceIndexFromHitNormal(hit.normal);
                    AssignMaterialToSubMesh(gameObject, faceIndex, _hoverMaterial);
                }
            }
        }
    }

    private void Awake()
    {
        Mesh dividedMesh = DivideCubeMeshIntoSubmeshes(GetComponent<MeshFilter>().mesh);
        gameObject.GetComponent<MeshFilter>().mesh = dividedMesh;
    }

    private Mesh DivideCubeMeshIntoSubmeshes(Mesh source)
    {
        Mesh dividedMesh = Object.Instantiate(source);
        dividedMesh.name = source.name + "_SplitSubmeshes";

        var tris = dividedMesh.triangles;

        // Helper: map a normal to a face index
        int FaceIndexFromNormal(Vector3 n)
        {
            // Pick the dominant axis (cube faces should be axis-aligned).
            n = n.normalized;
            float ax = Mathf.Abs(n.x);
            float ay = Mathf.Abs(n.y);
            float az = Mathf.Abs(n.z);

            if (ax >= ay && ax >= az) return n.x >= 0 ? 0 : 1; // +X / -X
            if (ay >= ax && ay >= az) return n.y >= 0 ? 2 : 3; // +Y / -Y
            return n.z >= 0 ? 4 : 5;                           // +Z / -Z
        }

        var subMeshesTriangles = new List<int>[6];
        for (int i = 0; i < subMeshesTriangles.Length; i++)
        {
            subMeshesTriangles[i] = new List<int>(tris.Length / 6);
        }

        for (int i = 0; i < source.triangles.Length; i += 3)
        {
            var i1 = source.triangles[i];
            var i2 = source.triangles[i+1];
            var i3 = source.triangles[i+2];

            var v1 = source.vertices[i1];
            var v2 = source.vertices[i2];
            var v3 = source.vertices[i3];

            var triangleNormal = Vector3.Cross(v2 - v1, v3 - v1).normalized;
            var subMeshIndex = FaceIndexFromNormal(triangleNormal);

            subMeshesTriangles[subMeshIndex].Add(i1);
            subMeshesTriangles[subMeshIndex].Add(i2);
            subMeshesTriangles[subMeshIndex].Add(i3);
        }

        dividedMesh.subMeshCount = 6;
        for (int i = 0; i < subMeshesTriangles.Length; i++)
        {
            dividedMesh.SetTriangles(subMeshesTriangles[i], i, true);
        }

        dividedMesh.RecalculateBounds();
        
        return dividedMesh;
    }

    public void AssignMaterialToSubMesh(GameObject go, int submeshIndex, Material mat)
    {
        try
        {
            var tris = go.GetComponent<MeshFilter>().mesh.GetTriangles(submeshIndex);
            var materials = go.GetComponent<MeshRenderer>().materials;
            materials[submeshIndex] = mat;
            go.GetComponent<MeshRenderer>().materials = materials;
        }
        catch 
        {
            Debug.LogError($"No submesh with index {submeshIndex} is found on mesh");
            return;
        }
    }

    private int GetFaceIndexFromHitNormal(Vector3 normal)
    {
        normal = transform.InverseTransformDirection(normal).normalized;
        float ax = Mathf.Abs(normal.x);
        float ay = Mathf.Abs(normal.y);
        float az = Mathf.Abs(normal.z);

        if (ax >= ay && ax >= az) return normal.x >= 0 ? 0 : 1;
        if (ay >= ax && ay >= az) return normal.y >= 0 ? 2 : 3;
        return normal.z >= 0 ? 4 : 5;
    }

    public void SetColoringModeOnOff(bool on, HoverEnterEventArgs hoverArgs)
    {
        if (on)
        {
            _rayInteractor = (XRRayInteractor)hoverArgs.interactorObject;
            _rayInteractable = (XRGrabInteractable)hoverArgs.interactableObject;
        }

        Debug.Log($"Ray interactable is {_rayInteractable.name}");

        _coloring = on;
    }
}