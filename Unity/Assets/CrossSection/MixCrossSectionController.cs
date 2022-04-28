using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;

/// <summary>
/// Simple demo For cross section shader controller. you can use AAplane, Box ,Boxs difference way to cutoff mesh 
/// kaser@toydea.com
/// 2022/04/28
/// </summary>
public class MixCrossSectionController : MonoBehaviour
{

    [Header("ThreeAAplaneCross")]
    public Transform planeYZ;
    public Transform planeXZ;
    public Transform planeXY;

    [Header("SingleBoxCross")]
    public Transform Box;

    [Header("MultiBoxCross")]
    public List<Transform> Boxs;
    public enum CrossMode
    {
        ThreeAAplaneCross,
        SingleBoxCross,
        MultiBoxCross
    }

    public CrossMode UseCrossMode = CrossMode.ThreeAAplaneCross;
    public bool ShowCrossArea;

    /// <summary>
    ///Tthis number is default of shader array size. If you want to more range to use. You need to change the declare of GlobalArray length in shader and this property.
    /// CrossCubeRotationMatrixArray[24] , CrossCubePosArray[24] , CrossCubeExtentArray[24]
    /// </summary>
    public const int MaxCrossRange = 24;

    /// <summary>
    /// Always keep sended array data length equal MaxCrossRange [Off to save memory] 
    /// </summary>
    public bool AlwaysKeepArrayLength = true;

    /// <summary>
    /// If False it will not update.
    /// </summary>
    public bool UseUpdate = false;

    public Renderer Renderer;
    // Use this for initialization

    void Start()
    {
        if (Renderer == null)
        {
            Renderer = GetComponent<Renderer>();
        }
        UpdateStaticShaderProperties();
        UpdateShaderProperties();
    }
    void Update()
    {
        if (UseUpdate)
        {
            UpdateShaderProperties();
        }
    }

    /// <summary>
    /// Update Static Shader Properties
    /// </summary>
    void UpdateStaticShaderProperties()
    {
        if (Renderer == null)
        {
            return;
        }
        Renderer.material.SetFloat("_ShowCrossArea", Convert.ToInt32(ShowCrossArea));
        Renderer.material.SetFloat("_CrossMode", (float)UseCrossMode);
    }
    private void UpdateShaderProperties()
    {
        switch (UseCrossMode)
        {
            case CrossMode.ThreeAAplaneCross: ThreeAAplaneCrossProcess(); break;
            case CrossMode.SingleBoxCross: SingleBoxCrossProcess(); break;
            case CrossMode.MultiBoxCross: MultiBoxsCrossProcess(); break;
        };
    }
    /// <summary>
    /// AAplane Cross Process
    /// </summary>
    void ThreeAAplaneCrossProcess()
    {
        if (nullCheck(planeYZ) || nullCheck(planeXZ) || nullCheck(planeXY))
        {
            return;
        }
        Renderer.material.SetVector("_PosYZ", planeYZ.position);
        Renderer.material.SetVector("_PosXZ", planeXZ.position);
        Renderer.material.SetVector("_PosXY", planeXY.position);
    }
    /// <summary>
    /// Single Box Cross Process
    /// </summary>
    void SingleBoxCrossProcess()
    {

        if (nullCheck(Box))
        {
            return;
        }
        Matrix4x4 boxRotationMatrix = Matrix4x4.TRS(-Box.position, Box.rotation, Vector3.one);

        Renderer.material.SetMatrix("_BoxRotationMatrix", boxRotationMatrix);
        Renderer.material.SetVector("_Cubepos", Box.position);
        Renderer.material.SetVector("_CubeExtent", Box.localScale / 2.0f);
    }
    /// <summary>
    /// Multi Boxs Cross Process
    /// </summary>
    void MultiBoxsCrossProcess()
    {
        Boxs.RemoveAll(i => i == null);
        var boxLength = Boxs.Count;
        if (boxLength <= 0)
        {
            return;
        }
        Renderer.material.SetFloat("_CrossCubsArrayLength", boxLength);
        var RotationMatrixArray = Boxs.Select(i => Matrix4x4.TRS(-i.position, i.rotation, Vector3.one)).ToList();
        var CrossCubePosArray = Boxs.Select(i => (Vector4)i.position).ToList();
        var CrossCubeExtentArray = Boxs.Select(i => (Vector4)i.localScale / 2.0f).ToList();

        fillList(24, RotationMatrixArray, new Matrix4x4());
        fillList(24, CrossCubePosArray, new Vector4());
        fillList(24, CrossCubeExtentArray, new Vector4());

        Renderer.material.SetMatrixArray("CrossCubeRotationMatrixArray", RotationMatrixArray);
        Renderer.material.SetVectorArray("CrossCubePosArray", CrossCubePosArray);
        Renderer.material.SetVectorArray("CrossCubeExtentArray", CrossCubeExtentArray);
    }

    bool nullCheck(Transform target)
    {
        return target == null;
    }
    /// <summary>
    /// make array length always equal declared array size
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="targetLength"></param>
    /// <param name="list"></param>
    /// <param name="newValue"></param>
    void fillList<T>(int targetLength, List<T> list, object newValue)
    {
        if (!AlwaysKeepArrayLength)
        {
            return;
        }

        if (list.Count() < targetLength)
        {
            var fillDataLength = targetLength - list.Count();
            for (var i = 0; i < fillDataLength; i++)
            {
                list.Add((T)newValue);
            }
        }
    }
    private void OnValidate()
    {
        UpdateStaticShaderProperties();
    }


}
