// Made with Amplify Shader Editor
// Available at the Unity Asset Store - http://u3d.as/y3X 
Shader "Dmp/URP/Effects/CrossSection/MixCrossSection"
{
	Properties
	{
		[HideInInspector] _AlphaCutoff("Alpha Cutoff ", Range(0, 1)) = 0.5
		[HideInInspector] _EmissionColor("Emission Color", Color) = (1,1,1,1)
		[ASEBegin][Enum(UnityEngine.Rendering.CullMode)]_CullMode("CullMode", Float) = 0
		[IntRange][Enum(ThreeAAplane,0,SingleBoxCross,1,MultiBoxCross,2)]_CrossMode("CrossMode", Float) = 0
		[ToggleUI]_ShowCrossArea("ShowCrossArea", Float) = 0
		_MainTex("MainTex", 2D) = "white" {}
		_MainColor("MainColor", Color) = (1,1,1,0)
		_CubeExtent("CubeExtent", Vector) = (0,0,0,0)
		_Cubepos("Cubepos", Vector) = (0,0,0,0)
		_CrossColor("CrossColor", Color) = (0,0,0,0)
		_PosXZ("PosXZ", Vector) = (0,0,0,0)
		_PosXY("PosXY", Vector) = (0,0,0,0)
		_PosYZ("PosYZ", Vector) = (0,0,0,0)
		[ASEEnd]_CrossCubsArrayLength("CrossCubsArrayLength", Float) = 0
		[HideInInspector] _texcoord( "", 2D ) = "white" {}

		//_TransmissionShadow( "Transmission Shadow", Range( 0, 1 ) ) = 0.5
		//_TransStrength( "Trans Strength", Range( 0, 50 ) ) = 1
		//_TransNormal( "Trans Normal Distortion", Range( 0, 1 ) ) = 0.5
		//_TransScattering( "Trans Scattering", Range( 1, 50 ) ) = 2
		//_TransDirect( "Trans Direct", Range( 0, 1 ) ) = 0.9
		//_TransAmbient( "Trans Ambient", Range( 0, 1 ) ) = 0.1
		//_TransShadow( "Trans Shadow", Range( 0, 1 ) ) = 0.5
		//_TessPhongStrength( "Tess Phong Strength", Range( 0, 1 ) ) = 0.5
		//_TessValue( "Tess Max Tessellation", Range( 1, 32 ) ) = 16
		//_TessMin( "Tess Min Distance", Float ) = 10
		//_TessMax( "Tess Max Distance", Float ) = 25
		//_TessEdgeLength ( "Tess Edge length", Range( 2, 50 ) ) = 16
		//_TessMaxDisp( "Tess Max Displacement", Float ) = 25
	}

	SubShader
	{
		LOD 0

		

		Tags { "RenderPipeline"="UniversalPipeline" "RenderType"="Opaque" "Queue"="Geometry" }
		Cull [_CullMode]
		AlphaToMask Off
		
		HLSLINCLUDE
		#pragma target 3.0

		#pragma prefer_hlslcc gles
		#pragma exclude_renderers d3d11_9x 

		#ifndef ASE_TESS_FUNCS
		#define ASE_TESS_FUNCS
		float4 FixedTess( float tessValue )
		{
			return tessValue;
		}
		
		float CalcDistanceTessFactor (float4 vertex, float minDist, float maxDist, float tess, float4x4 o2w, float3 cameraPos )
		{
			float3 wpos = mul(o2w,vertex).xyz;
			float dist = distance (wpos, cameraPos);
			float f = clamp(1.0 - (dist - minDist) / (maxDist - minDist), 0.01, 1.0) * tess;
			return f;
		}

		float4 CalcTriEdgeTessFactors (float3 triVertexFactors)
		{
			float4 tess;
			tess.x = 0.5 * (triVertexFactors.y + triVertexFactors.z);
			tess.y = 0.5 * (triVertexFactors.x + triVertexFactors.z);
			tess.z = 0.5 * (triVertexFactors.x + triVertexFactors.y);
			tess.w = (triVertexFactors.x + triVertexFactors.y + triVertexFactors.z) / 3.0f;
			return tess;
		}

		float CalcEdgeTessFactor (float3 wpos0, float3 wpos1, float edgeLen, float3 cameraPos, float4 scParams )
		{
			float dist = distance (0.5 * (wpos0+wpos1), cameraPos);
			float len = distance(wpos0, wpos1);
			float f = max(len * scParams.y / (edgeLen * dist), 1.0);
			return f;
		}

		float DistanceFromPlane (float3 pos, float4 plane)
		{
			float d = dot (float4(pos,1.0f), plane);
			return d;
		}

		bool WorldViewFrustumCull (float3 wpos0, float3 wpos1, float3 wpos2, float cullEps, float4 planes[6] )
		{
			float4 planeTest;
			planeTest.x = (( DistanceFromPlane(wpos0, planes[0]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[0]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[0]) > -cullEps) ? 1.0f : 0.0f );
			planeTest.y = (( DistanceFromPlane(wpos0, planes[1]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[1]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[1]) > -cullEps) ? 1.0f : 0.0f );
			planeTest.z = (( DistanceFromPlane(wpos0, planes[2]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[2]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[2]) > -cullEps) ? 1.0f : 0.0f );
			planeTest.w = (( DistanceFromPlane(wpos0, planes[3]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[3]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[3]) > -cullEps) ? 1.0f : 0.0f );
			return !all (planeTest);
		}

		float4 DistanceBasedTess( float4 v0, float4 v1, float4 v2, float tess, float minDist, float maxDist, float4x4 o2w, float3 cameraPos )
		{
			float3 f;
			f.x = CalcDistanceTessFactor (v0,minDist,maxDist,tess,o2w,cameraPos);
			f.y = CalcDistanceTessFactor (v1,minDist,maxDist,tess,o2w,cameraPos);
			f.z = CalcDistanceTessFactor (v2,minDist,maxDist,tess,o2w,cameraPos);

			return CalcTriEdgeTessFactors (f);
		}

		float4 EdgeLengthBasedTess( float4 v0, float4 v1, float4 v2, float edgeLength, float4x4 o2w, float3 cameraPos, float4 scParams )
		{
			float3 pos0 = mul(o2w,v0).xyz;
			float3 pos1 = mul(o2w,v1).xyz;
			float3 pos2 = mul(o2w,v2).xyz;
			float4 tess;
			tess.x = CalcEdgeTessFactor (pos1, pos2, edgeLength, cameraPos, scParams);
			tess.y = CalcEdgeTessFactor (pos2, pos0, edgeLength, cameraPos, scParams);
			tess.z = CalcEdgeTessFactor (pos0, pos1, edgeLength, cameraPos, scParams);
			tess.w = (tess.x + tess.y + tess.z) / 3.0f;
			return tess;
		}

		float4 EdgeLengthBasedTessCull( float4 v0, float4 v1, float4 v2, float edgeLength, float maxDisplacement, float4x4 o2w, float3 cameraPos, float4 scParams, float4 planes[6] )
		{
			float3 pos0 = mul(o2w,v0).xyz;
			float3 pos1 = mul(o2w,v1).xyz;
			float3 pos2 = mul(o2w,v2).xyz;
			float4 tess;

			if (WorldViewFrustumCull(pos0, pos1, pos2, maxDisplacement, planes))
			{
				tess = 0.0f;
			}
			else
			{
				tess.x = CalcEdgeTessFactor (pos1, pos2, edgeLength, cameraPos, scParams);
				tess.y = CalcEdgeTessFactor (pos2, pos0, edgeLength, cameraPos, scParams);
				tess.z = CalcEdgeTessFactor (pos0, pos1, edgeLength, cameraPos, scParams);
				tess.w = (tess.x + tess.y + tess.z) / 3.0f;
			}
			return tess;
		}
		#endif //ASE_TESS_FUNCS

		ENDHLSL

		
		Pass
		{
			
			Name "Forward"
			Tags { "LightMode"="UniversalForward" }
			
			Blend One Zero, One Zero
			ZWrite On
			ZTest LEqual
			Offset 0 , 0
			ColorMask RGBA
			

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define ASE_SRP_VERSION 110000

			
			#pragma multi_compile _ _SCREEN_SPACE_OCCLUSION
			#pragma multi_compile _ _MAIN_LIGHT_SHADOWS
			#pragma multi_compile _ _MAIN_LIGHT_SHADOWS_CASCADE
			#pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS _ADDITIONAL_OFF
			#pragma multi_compile _ _ADDITIONAL_LIGHT_SHADOWS
			#pragma multi_compile _ _SHADOWS_SOFT
			#pragma multi_compile _ _MIXED_LIGHTING_SUBTRACTIVE
			
			#pragma multi_compile _ LIGHTMAP_SHADOW_MIXING
			#pragma multi_compile _ SHADOWS_SHADOWMASK

			#pragma multi_compile _ DIRLIGHTMAP_COMBINED
			#pragma multi_compile _ LIGHTMAP_ON

			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_FORWARD

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			
			#if ASE_SRP_VERSION <= 70108
			#define REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR
			#endif

			#if defined(UNITY_INSTANCING_ENABLED) && defined(_TERRAIN_INSTANCED_PERPIXEL_NORMAL)
			    #define ENABLE_TERRAIN_PERPIXEL_NORMAL
			#endif

			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma multi_compile_instancing


			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_tangent : TANGENT;
				float4 texcoord1 : TEXCOORD1;
				float4 texcoord : TEXCOORD0;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				float4 lightmapUVOrVertexSH : TEXCOORD0;
				half4 fogFactorAndVertexLight : TEXCOORD1;
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
				float4 shadowCoord : TEXCOORD2;
				#endif
				float4 tSpace0 : TEXCOORD3;
				float4 tSpace1 : TEXCOORD4;
				float4 tSpace2 : TEXCOORD5;
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				float4 screenPos : TEXCOORD6;
				#endif
				float4 ase_texcoord7 : TEXCOORD7;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _MainColor;
			float4 _CrossColor;
			float4 _PosYZ;
			float4 _PosXZ;
			float4 _PosXY;
			float3 _Cubepos;
			float3 _CubeExtent;
			float _ShowCrossArea;
			float _CullMode;
			float _CrossCubsArrayLength;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			float4 CrossCubeExtentArray[24];
			float4x4 CrossCubeRotationMatrixArray[24];
			float4 CrossCubePosArray[24];
			sampler2D _MainTex;
			UNITY_INSTANCING_BUFFER_START(DmpURPEffectsCrossSectionMixCrossSection)
				UNITY_DEFINE_INSTANCED_PROP(float4x4, _BoxRotationMatrix)
				UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST)
				UNITY_DEFINE_INSTANCED_PROP(float, _CrossMode)
			UNITY_INSTANCING_BUFFER_END(DmpURPEffectsCrossSectionMixCrossSection)


			float CheckCrossResult( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4x4 Inverse4x4(float4x4 input)
			{
				#define minor(a,b,c) determinant(float3x3(input.a, input.b, input.c))
				float4x4 cofactors = float4x4(
				minor( _22_23_24, _32_33_34, _42_43_44 ),
				-minor( _21_23_24, _31_33_34, _41_43_44 ),
				minor( _21_22_24, _31_32_34, _41_42_44 ),
				-minor( _21_22_23, _31_32_33, _41_42_43 ),
			
				-minor( _12_13_14, _32_33_34, _42_43_44 ),
				minor( _11_13_14, _31_33_34, _41_43_44 ),
				-minor( _11_12_14, _31_32_34, _41_42_44 ),
				minor( _11_12_13, _31_32_33, _41_42_43 ),
			
				minor( _12_13_14, _22_23_24, _42_43_44 ),
				-minor( _11_13_14, _21_23_24, _41_43_44 ),
				minor( _11_12_14, _21_22_24, _41_42_44 ),
				-minor( _11_12_13, _21_22_23, _41_42_43 ),
			
				-minor( _12_13_14, _22_23_24, _32_33_34 ),
				minor( _11_13_14, _21_23_24, _31_33_34 ),
				-minor( _11_12_14, _21_22_24, _31_32_34 ),
				minor( _11_12_13, _21_22_23, _31_32_33 ));
				#undef minor
				return transpose( cofactors ) / determinant( input );
			}
			
			float returnPosYZ0PosXZ0PosXY0115( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4 MultiBoxCrossProcess( float4 DefaultColor, float3 WorldPosition, int ArrayLength )
			{
				bool crossArea[24];
				for(int i=0;i<ArrayLength;i++)
				{
				crossArea[i] = 1;
				float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz;
				float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i];
				float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i];
				int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0;
				int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0;
				int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0;
				bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0;
				if(isCross && _ShowCrossArea)
				{
				return _CrossColor;
				}
				if(_CrossMode == 2)
				{
				clip( -(isCross));
				}
				}
				return DefaultColor;
			}
			
			float4 FinalCrossModeCheck188( int CurrentPassMode, float4 ThreeAAplane, float4 SingleBoxCross, float4 MultiBoxCross )
			{
				 if(CurrentPassMode == 0)
				{
				return ThreeAAplane;
				}
				 if(CurrentPassMode == 1)
				{
				return SingleBoxCross;
				}
				 if(CurrentPassMode == 2)
				{
				return MultiBoxCross;
				}
				return float4(1,1,1,1);
			}
			

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				o.ase_texcoord7.xy = v.texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord7.zw = 0;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif
				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float3 positionVS = TransformWorldToView( positionWS );
				float4 positionCS = TransformWorldToHClip( positionWS );

				VertexNormalInputs normalInput = GetVertexNormalInputs( v.ase_normal, v.ase_tangent );

				o.tSpace0 = float4( normalInput.normalWS, positionWS.x);
				o.tSpace1 = float4( normalInput.tangentWS, positionWS.y);
				o.tSpace2 = float4( normalInput.bitangentWS, positionWS.z);

				OUTPUT_LIGHTMAP_UV( v.texcoord1, unity_LightmapST, o.lightmapUVOrVertexSH.xy );
				OUTPUT_SH( normalInput.normalWS.xyz, o.lightmapUVOrVertexSH.xyz );

				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					o.lightmapUVOrVertexSH.zw = v.texcoord;
					o.lightmapUVOrVertexSH.xy = v.texcoord * unity_LightmapST.xy + unity_LightmapST.zw;
				#endif

				half3 vertexLight = VertexLighting( positionWS, normalInput.normalWS );
				#ifdef ASE_FOG
					half fogFactor = ComputeFogFactor( positionCS.z );
				#else
					half fogFactor = 0;
				#endif
				o.fogFactorAndVertexLight = half4(fogFactor, vertexLight);
				
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
				VertexPositionInputs vertexInput = (VertexPositionInputs)0;
				vertexInput.positionWS = positionWS;
				vertexInput.positionCS = positionCS;
				o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				
				o.clipPos = positionCS;
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				o.screenPos = ComputeScreenPos(positionCS);
				#endif
				return o;
			}
			
			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_tangent : TANGENT;
				float4 texcoord : TEXCOORD0;
				float4 texcoord1 : TEXCOORD1;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_tangent = v.ase_tangent;
				o.texcoord = v.texcoord;
				o.texcoord1 = v.texcoord1;
				
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_tangent = patch[0].ase_tangent * bary.x + patch[1].ase_tangent * bary.y + patch[2].ase_tangent * bary.z;
				o.texcoord = patch[0].texcoord * bary.x + patch[1].texcoord * bary.y + patch[2].texcoord * bary.z;
				o.texcoord1 = patch[0].texcoord1 * bary.x + patch[1].texcoord1 * bary.y + patch[2].texcoord1 * bary.z;
				
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif

			half4 frag ( VertexOutput IN 
						#ifdef ASE_DEPTH_WRITE_ON
						,out float outputDepth : ASE_SV_DEPTH
						#endif
						, FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_Target
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(IN);

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif

				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					float2 sampleCoords = (IN.lightmapUVOrVertexSH.zw / _TerrainHeightmapRecipSize.zw + 0.5f) * _TerrainHeightmapRecipSize.xy;
					float3 WorldNormal = TransformObjectToWorldNormal(normalize(SAMPLE_TEXTURE2D(_TerrainNormalmapTexture, sampler_TerrainNormalmapTexture, sampleCoords).rgb * 2 - 1));
					float3 WorldTangent = -cross(GetObjectToWorldMatrix()._13_23_33, WorldNormal);
					float3 WorldBiTangent = cross(WorldNormal, -WorldTangent);
				#else
					float3 WorldNormal = normalize( IN.tSpace0.xyz );
					float3 WorldTangent = IN.tSpace1.xyz;
					float3 WorldBiTangent = IN.tSpace2.xyz;
				#endif
				float3 WorldPosition = float3(IN.tSpace0.w,IN.tSpace1.w,IN.tSpace2.w);
				float3 WorldViewDirection = _WorldSpaceCameraPos.xyz  - WorldPosition;
				float4 ShadowCoords = float4( 0, 0, 0, 0 );
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				float4 ScreenPos = IN.screenPos;
				#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
					ShadowCoords = IN.shadowCoord;
				#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
					ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
				#endif
	
				WorldViewDirection = SafeNormalize( WorldViewDirection );

				float _CrossMode_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_CrossMode);
				float CrossMode198 = _CrossMode_Instance;
				int CurrentPassMode188 = (int)CrossMode198;
				float4 _MainTex_ST_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_MainTex_ST);
				float2 uv_MainTex = IN.ase_texcoord7.xy * _MainTex_ST_Instance.xy + _MainTex_ST_Instance.zw;
				float4 AlbedoResult116 = ( tex2D( _MainTex, uv_MainTex ) * _MainColor );
				float4 CorssColor134 = _CrossColor;
				float4 switchResult81 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float dotResult46 = dot( float3(1,0,0) , ( WorldPosition - (_PosYZ).xyz ) );
				float PosYZ57 = ( dotResult46 + _PosYZ.w );
				float dotResult47 = dot( float3(0,1,0) , ( WorldPosition - (_PosXZ).xyz ) );
				float PosXZ57 = ( dotResult47 + _PosXZ.w );
				float dotResult48 = dot( float3(0,0,1) , ( WorldPosition - (_PosXY).xyz ) );
				float PosXY57 = ( dotResult48 + _PosXY.w );
				float localCheckCrossResult57 = CheckCrossResult( PosYZ57 , PosXZ57 , PosXY57 );
				clip( -( CrossMode198 == 0.0 ? localCheckCrossResult57 : 0.0 ) );
				float4 ThreeAAplaneResult111 = switchResult81;
				float4 ThreeAAplane188 = ThreeAAplaneResult111;
				float4 switchResult133 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4x4 _BoxRotationMatrix_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_BoxRotationMatrix);
				float4x4 invertVal91 = Inverse4x4( _BoxRotationMatrix_Instance );
				float3 break98 = ( mul( invertVal91, float4( ( WorldPosition - _Cubepos ) , 0.0 ) ).xyz + _Cubepos );
				float3 break100 = ( _Cubepos - _CubeExtent );
				float3 break99 = ( _Cubepos + _CubeExtent );
				float PosYZ115 = (( break98.x >= break100.x && break98.x <= break99.x ) ? 1.0 :  0.0 );
				float PosXZ115 = (( break98.y >= break100.y && break98.y <= break99.y ) ? 1.0 :  0.0 );
				float PosXY115 = (( break98.z >= break100.z && break98.z <= break99.z ) ? 1.0 :  0.0 );
				float localreturnPosYZ0PosXZ0PosXY0115 = returnPosYZ0PosXZ0PosXY0115( PosYZ115 , PosXZ115 , PosXY115 );
				clip( -( CrossMode198 == 1.0 ? localreturnPosYZ0PosXZ0PosXY0115 : 0.0 ) );
				float4 BoxCrossResult118 = switchResult133;
				float4 SingleBoxCross188 = BoxCrossResult118;
				float4 switchResult162 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4 DefaultColor156 = switchResult162;
				float3 WorldPosition156 = WorldPosition;
				int ArrayLength156 = (int)_CrossCubsArrayLength;
				float4 localMultiBoxCrossProcess156 = MultiBoxCrossProcess( DefaultColor156 , WorldPosition156 , ArrayLength156 );
				float4 MultiBoxCrossResult159 = localMultiBoxCrossProcess156;
				float4 MultiBoxCross188 = MultiBoxCrossResult159;
				float4 localFinalCrossModeCheck188 = FinalCrossModeCheck188( CurrentPassMode188 , ThreeAAplane188 , SingleBoxCross188 , MultiBoxCross188 );
				
				float3 Albedo = localFinalCrossModeCheck188.xyz;
				float3 Normal = float3(0, 0, 1);
				float3 Emission = 0;
				float3 Specular = 0.5;
				float Metallic = 0;
				float Smoothness = 0.5;
				float Occlusion = 1;
				float Alpha = localFinalCrossModeCheck188.x;
				float AlphaClipThreshold = 0.5;
				float AlphaClipThresholdShadow = 0.5;
				float3 BakedGI = 0;
				float3 RefractionColor = 1;
				float RefractionIndex = 1;
				float3 Transmission = 1;
				float3 Translucency = 1;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				InputData inputData;
				inputData.positionWS = WorldPosition;
				inputData.viewDirectionWS = WorldViewDirection;
				inputData.shadowCoord = ShadowCoords;

				#ifdef _NORMALMAP
					#if _NORMAL_DROPOFF_TS
					inputData.normalWS = TransformTangentToWorld(Normal, half3x3( WorldTangent, WorldBiTangent, WorldNormal ));
					#elif _NORMAL_DROPOFF_OS
					inputData.normalWS = TransformObjectToWorldNormal(Normal);
					#elif _NORMAL_DROPOFF_WS
					inputData.normalWS = Normal;
					#endif
					inputData.normalWS = NormalizeNormalPerPixel(inputData.normalWS);
				#else
					inputData.normalWS = WorldNormal;
				#endif

				#ifdef ASE_FOG
					inputData.fogCoord = IN.fogFactorAndVertexLight.x;
				#endif

				inputData.vertexLighting = IN.fogFactorAndVertexLight.yzw;
				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					float3 SH = SampleSH(inputData.normalWS.xyz);
				#else
					float3 SH = IN.lightmapUVOrVertexSH.xyz;
				#endif

				inputData.bakedGI = SAMPLE_GI( IN.lightmapUVOrVertexSH.xy, SH, inputData.normalWS );
				#ifdef _ASE_BAKEDGI
					inputData.bakedGI = BakedGI;
				#endif
				
				inputData.normalizedScreenSpaceUV = GetNormalizedScreenSpaceUV(IN.clipPos);
				inputData.shadowMask = SAMPLE_SHADOWMASK(IN.lightmapUVOrVertexSH.xy);

				half4 color = UniversalFragmentPBR(
					inputData, 
					Albedo, 
					Metallic, 
					Specular, 
					Smoothness, 
					Occlusion, 
					Emission, 
					Alpha);

				#ifdef _TRANSMISSION_ASE
				{
					float shadow = _TransmissionShadow;

					Light mainLight = GetMainLight( inputData.shadowCoord );
					float3 mainAtten = mainLight.color * mainLight.distanceAttenuation;
					mainAtten = lerp( mainAtten, mainAtten * mainLight.shadowAttenuation, shadow );
					half3 mainTransmission = max(0 , -dot(inputData.normalWS, mainLight.direction)) * mainAtten * Transmission;
					color.rgb += Albedo * mainTransmission;

					#ifdef _ADDITIONAL_LIGHTS
						int transPixelLightCount = GetAdditionalLightsCount();
						for (int i = 0; i < transPixelLightCount; ++i)
						{
							Light light = GetAdditionalLight(i, inputData.positionWS);
							float3 atten = light.color * light.distanceAttenuation;
							atten = lerp( atten, atten * light.shadowAttenuation, shadow );

							half3 transmission = max(0 , -dot(inputData.normalWS, light.direction)) * atten * Transmission;
							color.rgb += Albedo * transmission;
						}
					#endif
				}
				#endif

				#ifdef _TRANSLUCENCY_ASE
				{
					float shadow = _TransShadow;
					float normal = _TransNormal;
					float scattering = _TransScattering;
					float direct = _TransDirect;
					float ambient = _TransAmbient;
					float strength = _TransStrength;

					Light mainLight = GetMainLight( inputData.shadowCoord );
					float3 mainAtten = mainLight.color * mainLight.distanceAttenuation;
					mainAtten = lerp( mainAtten, mainAtten * mainLight.shadowAttenuation, shadow );

					half3 mainLightDir = mainLight.direction + inputData.normalWS * normal;
					half mainVdotL = pow( saturate( dot( inputData.viewDirectionWS, -mainLightDir ) ), scattering );
					half3 mainTranslucency = mainAtten * ( mainVdotL * direct + inputData.bakedGI * ambient ) * Translucency;
					color.rgb += Albedo * mainTranslucency * strength;

					#ifdef _ADDITIONAL_LIGHTS
						int transPixelLightCount = GetAdditionalLightsCount();
						for (int i = 0; i < transPixelLightCount; ++i)
						{
							Light light = GetAdditionalLight(i, inputData.positionWS);
							float3 atten = light.color * light.distanceAttenuation;
							atten = lerp( atten, atten * light.shadowAttenuation, shadow );

							half3 lightDir = light.direction + inputData.normalWS * normal;
							half VdotL = pow( saturate( dot( inputData.viewDirectionWS, -lightDir ) ), scattering );
							half3 translucency = atten * ( VdotL * direct + inputData.bakedGI * ambient ) * Translucency;
							color.rgb += Albedo * translucency * strength;
						}
					#endif
				}
				#endif

				#ifdef _REFRACTION_ASE
					float4 projScreenPos = ScreenPos / ScreenPos.w;
					float3 refractionOffset = ( RefractionIndex - 1.0 ) * mul( UNITY_MATRIX_V, float4( WorldNormal,0 ) ).xyz * ( 1.0 - dot( WorldNormal, WorldViewDirection ) );
					projScreenPos.xy += refractionOffset.xy;
					float3 refraction = SHADERGRAPH_SAMPLE_SCENE_COLOR( projScreenPos.xy ) * RefractionColor;
					color.rgb = lerp( refraction, color.rgb, color.a );
					color.a = 1;
				#endif

				#ifdef ASE_FINAL_COLOR_ALPHA_MULTIPLY
					color.rgb *= color.a;
				#endif

				#ifdef ASE_FOG
					#ifdef TERRAIN_SPLAT_ADDPASS
						color.rgb = MixFogColor(color.rgb, half3( 0, 0, 0 ), IN.fogFactorAndVertexLight.x );
					#else
						color.rgb = MixFog(color.rgb, IN.fogFactorAndVertexLight.x);
					#endif
				#endif

				#ifdef ASE_DEPTH_WRITE_ON
					outputDepth = DepthValue;
				#endif

				return color;
			}

			ENDHLSL
		}

		
		Pass
		{
			
			Name "ShadowCaster"
			Tags { "LightMode"="ShadowCaster" }

			ZWrite On
			ZTest LEqual
			AlphaToMask Off
			ColorMask 0

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define ASE_SRP_VERSION 110000

			
			#pragma vertex vert
			#pragma fragment frag
#if ASE_SRP_VERSION >= 110000
			#pragma multi_compile _ _CASTING_PUNCTUAL_LIGHT_SHADOW
#endif
			#define SHADERPASS_SHADOWCASTER

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"

			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma multi_compile_instancing


			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				float4 ase_texcoord2 : TEXCOORD2;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _MainColor;
			float4 _CrossColor;
			float4 _PosYZ;
			float4 _PosXZ;
			float4 _PosXY;
			float3 _Cubepos;
			float3 _CubeExtent;
			float _ShowCrossArea;
			float _CullMode;
			float _CrossCubsArrayLength;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			float4 CrossCubeExtentArray[24];
			float4x4 CrossCubeRotationMatrixArray[24];
			float4 CrossCubePosArray[24];
			sampler2D _MainTex;
			UNITY_INSTANCING_BUFFER_START(DmpURPEffectsCrossSectionMixCrossSection)
				UNITY_DEFINE_INSTANCED_PROP(float4x4, _BoxRotationMatrix)
				UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST)
				UNITY_DEFINE_INSTANCED_PROP(float, _CrossMode)
			UNITY_INSTANCING_BUFFER_END(DmpURPEffectsCrossSectionMixCrossSection)


			float CheckCrossResult( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4x4 Inverse4x4(float4x4 input)
			{
				#define minor(a,b,c) determinant(float3x3(input.a, input.b, input.c))
				float4x4 cofactors = float4x4(
				minor( _22_23_24, _32_33_34, _42_43_44 ),
				-minor( _21_23_24, _31_33_34, _41_43_44 ),
				minor( _21_22_24, _31_32_34, _41_42_44 ),
				-minor( _21_22_23, _31_32_33, _41_42_43 ),
			
				-minor( _12_13_14, _32_33_34, _42_43_44 ),
				minor( _11_13_14, _31_33_34, _41_43_44 ),
				-minor( _11_12_14, _31_32_34, _41_42_44 ),
				minor( _11_12_13, _31_32_33, _41_42_43 ),
			
				minor( _12_13_14, _22_23_24, _42_43_44 ),
				-minor( _11_13_14, _21_23_24, _41_43_44 ),
				minor( _11_12_14, _21_22_24, _41_42_44 ),
				-minor( _11_12_13, _21_22_23, _41_42_43 ),
			
				-minor( _12_13_14, _22_23_24, _32_33_34 ),
				minor( _11_13_14, _21_23_24, _31_33_34 ),
				-minor( _11_12_14, _21_22_24, _31_32_34 ),
				minor( _11_12_13, _21_22_23, _31_32_33 ));
				#undef minor
				return transpose( cofactors ) / determinant( input );
			}
			
			float returnPosYZ0PosXZ0PosXY0115( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4 MultiBoxCrossProcess( float4 DefaultColor, float3 WorldPosition, int ArrayLength )
			{
				bool crossArea[24];
				for(int i=0;i<ArrayLength;i++)
				{
				crossArea[i] = 1;
				float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz;
				float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i];
				float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i];
				int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0;
				int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0;
				int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0;
				bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0;
				if(isCross && _ShowCrossArea)
				{
				return _CrossColor;
				}
				if(_CrossMode == 2)
				{
				clip( -(isCross));
				}
				}
				return DefaultColor;
			}
			
			float4 FinalCrossModeCheck188( int CurrentPassMode, float4 ThreeAAplane, float4 SingleBoxCross, float4 MultiBoxCross )
			{
				 if(CurrentPassMode == 0)
				{
				return ThreeAAplane;
				}
				 if(CurrentPassMode == 1)
				{
				return SingleBoxCross;
				}
				 if(CurrentPassMode == 2)
				{
				return MultiBoxCross;
				}
				return float4(1,1,1,1);
			}
			

			float3 _LightDirection;
#if ASE_SRP_VERSION >= 110000 
			float3 _LightPosition;
#endif
			VertexOutput VertexFunction( VertexInput v )
			{
				VertexOutput o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO( o );

				o.ase_texcoord2.xy = v.ase_texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord2.zw = 0;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif
				float3 normalWS = TransformObjectToWorldDir(v.ase_normal);

		#if ASE_SRP_VERSION >= 110000 
			#if _CASTING_PUNCTUAL_LIGHT_SHADOW
				float3 lightDirectionWS = normalize(_LightPosition - positionWS);
			#else
				float3 lightDirectionWS = _LightDirection;
			#endif
				float4 clipPos = TransformWorldToHClip(ApplyShadowBias(positionWS, normalWS, lightDirectionWS));
			#if UNITY_REVERSED_Z
				clipPos.z = min(clipPos.z, UNITY_NEAR_CLIP_VALUE);
			#else
				clipPos.z = max(clipPos.z, UNITY_NEAR_CLIP_VALUE);
			#endif
		#else
				float4 clipPos = TransformWorldToHClip(ApplyShadowBias(positionWS, normalWS, _LightDirection));
			#if UNITY_REVERSED_Z
				clipPos.z = min(clipPos.z, clipPos.w * UNITY_NEAR_CLIP_VALUE);
			#else
				clipPos.z = max(clipPos.z, clipPos.w * UNITY_NEAR_CLIP_VALUE);
			#endif
		#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = clipPos;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				o.clipPos = clipPos;
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_texcoord = v.ase_texcoord;
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_texcoord = patch[0].ase_texcoord * bary.x + patch[1].ase_texcoord * bary.y + patch[2].ase_texcoord * bary.z;
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif

			half4 frag(	VertexOutput IN 
						#ifdef ASE_DEPTH_WRITE_ON
						,out float outputDepth : ASE_SV_DEPTH
						#endif
						, FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID( IN );
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );
				
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				float _CrossMode_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_CrossMode);
				float CrossMode198 = _CrossMode_Instance;
				int CurrentPassMode188 = (int)CrossMode198;
				float4 _MainTex_ST_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_MainTex_ST);
				float2 uv_MainTex = IN.ase_texcoord2.xy * _MainTex_ST_Instance.xy + _MainTex_ST_Instance.zw;
				float4 AlbedoResult116 = ( tex2D( _MainTex, uv_MainTex ) * _MainColor );
				float4 CorssColor134 = _CrossColor;
				float4 switchResult81 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float dotResult46 = dot( float3(1,0,0) , ( WorldPosition - (_PosYZ).xyz ) );
				float PosYZ57 = ( dotResult46 + _PosYZ.w );
				float dotResult47 = dot( float3(0,1,0) , ( WorldPosition - (_PosXZ).xyz ) );
				float PosXZ57 = ( dotResult47 + _PosXZ.w );
				float dotResult48 = dot( float3(0,0,1) , ( WorldPosition - (_PosXY).xyz ) );
				float PosXY57 = ( dotResult48 + _PosXY.w );
				float localCheckCrossResult57 = CheckCrossResult( PosYZ57 , PosXZ57 , PosXY57 );
				clip( -( CrossMode198 == 0.0 ? localCheckCrossResult57 : 0.0 ) );
				float4 ThreeAAplaneResult111 = switchResult81;
				float4 ThreeAAplane188 = ThreeAAplaneResult111;
				float4 switchResult133 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4x4 _BoxRotationMatrix_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_BoxRotationMatrix);
				float4x4 invertVal91 = Inverse4x4( _BoxRotationMatrix_Instance );
				float3 break98 = ( mul( invertVal91, float4( ( WorldPosition - _Cubepos ) , 0.0 ) ).xyz + _Cubepos );
				float3 break100 = ( _Cubepos - _CubeExtent );
				float3 break99 = ( _Cubepos + _CubeExtent );
				float PosYZ115 = (( break98.x >= break100.x && break98.x <= break99.x ) ? 1.0 :  0.0 );
				float PosXZ115 = (( break98.y >= break100.y && break98.y <= break99.y ) ? 1.0 :  0.0 );
				float PosXY115 = (( break98.z >= break100.z && break98.z <= break99.z ) ? 1.0 :  0.0 );
				float localreturnPosYZ0PosXZ0PosXY0115 = returnPosYZ0PosXZ0PosXY0115( PosYZ115 , PosXZ115 , PosXY115 );
				clip( -( CrossMode198 == 1.0 ? localreturnPosYZ0PosXZ0PosXY0115 : 0.0 ) );
				float4 BoxCrossResult118 = switchResult133;
				float4 SingleBoxCross188 = BoxCrossResult118;
				float4 switchResult162 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4 DefaultColor156 = switchResult162;
				float3 WorldPosition156 = WorldPosition;
				int ArrayLength156 = (int)_CrossCubsArrayLength;
				float4 localMultiBoxCrossProcess156 = MultiBoxCrossProcess( DefaultColor156 , WorldPosition156 , ArrayLength156 );
				float4 MultiBoxCrossResult159 = localMultiBoxCrossProcess156;
				float4 MultiBoxCross188 = MultiBoxCrossResult159;
				float4 localFinalCrossModeCheck188 = FinalCrossModeCheck188( CurrentPassMode188 , ThreeAAplane188 , SingleBoxCross188 , MultiBoxCross188 );
				
				float Alpha = localFinalCrossModeCheck188.x;
				float AlphaClipThreshold = 0.5;
				float AlphaClipThresholdShadow = 0.5;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					#ifdef _ALPHATEST_SHADOW_ON
						clip(Alpha - AlphaClipThresholdShadow);
					#else
						clip(Alpha - AlphaClipThreshold);
					#endif
				#endif

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif
				#ifdef ASE_DEPTH_WRITE_ON
					outputDepth = DepthValue;
				#endif
				return 0;
			}

			ENDHLSL
		}

		
		Pass
		{
			
			Name "DepthOnly"
			Tags { "LightMode"="DepthOnly" }

			ZWrite On
			ColorMask 0
			AlphaToMask Off

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define ASE_SRP_VERSION 110000

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_DEPTHONLY

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"

			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma multi_compile_instancing


			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				float4 ase_texcoord2 : TEXCOORD2;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _MainColor;
			float4 _CrossColor;
			float4 _PosYZ;
			float4 _PosXZ;
			float4 _PosXY;
			float3 _Cubepos;
			float3 _CubeExtent;
			float _ShowCrossArea;
			float _CullMode;
			float _CrossCubsArrayLength;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			float4 CrossCubeExtentArray[24];
			float4x4 CrossCubeRotationMatrixArray[24];
			float4 CrossCubePosArray[24];
			sampler2D _MainTex;
			UNITY_INSTANCING_BUFFER_START(DmpURPEffectsCrossSectionMixCrossSection)
				UNITY_DEFINE_INSTANCED_PROP(float4x4, _BoxRotationMatrix)
				UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST)
				UNITY_DEFINE_INSTANCED_PROP(float, _CrossMode)
			UNITY_INSTANCING_BUFFER_END(DmpURPEffectsCrossSectionMixCrossSection)


			float CheckCrossResult( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4x4 Inverse4x4(float4x4 input)
			{
				#define minor(a,b,c) determinant(float3x3(input.a, input.b, input.c))
				float4x4 cofactors = float4x4(
				minor( _22_23_24, _32_33_34, _42_43_44 ),
				-minor( _21_23_24, _31_33_34, _41_43_44 ),
				minor( _21_22_24, _31_32_34, _41_42_44 ),
				-minor( _21_22_23, _31_32_33, _41_42_43 ),
			
				-minor( _12_13_14, _32_33_34, _42_43_44 ),
				minor( _11_13_14, _31_33_34, _41_43_44 ),
				-minor( _11_12_14, _31_32_34, _41_42_44 ),
				minor( _11_12_13, _31_32_33, _41_42_43 ),
			
				minor( _12_13_14, _22_23_24, _42_43_44 ),
				-minor( _11_13_14, _21_23_24, _41_43_44 ),
				minor( _11_12_14, _21_22_24, _41_42_44 ),
				-minor( _11_12_13, _21_22_23, _41_42_43 ),
			
				-minor( _12_13_14, _22_23_24, _32_33_34 ),
				minor( _11_13_14, _21_23_24, _31_33_34 ),
				-minor( _11_12_14, _21_22_24, _31_32_34 ),
				minor( _11_12_13, _21_22_23, _31_32_33 ));
				#undef minor
				return transpose( cofactors ) / determinant( input );
			}
			
			float returnPosYZ0PosXZ0PosXY0115( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4 MultiBoxCrossProcess( float4 DefaultColor, float3 WorldPosition, int ArrayLength )
			{
				bool crossArea[24];
				for(int i=0;i<ArrayLength;i++)
				{
				crossArea[i] = 1;
				float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz;
				float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i];
				float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i];
				int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0;
				int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0;
				int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0;
				bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0;
				if(isCross && _ShowCrossArea)
				{
				return _CrossColor;
				}
				if(_CrossMode == 2)
				{
				clip( -(isCross));
				}
				}
				return DefaultColor;
			}
			
			float4 FinalCrossModeCheck188( int CurrentPassMode, float4 ThreeAAplane, float4 SingleBoxCross, float4 MultiBoxCross )
			{
				 if(CurrentPassMode == 0)
				{
				return ThreeAAplane;
				}
				 if(CurrentPassMode == 1)
				{
				return SingleBoxCross;
				}
				 if(CurrentPassMode == 2)
				{
				return MultiBoxCross;
				}
				return float4(1,1,1,1);
			}
			

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				o.ase_texcoord2.xy = v.ase_texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord2.zw = 0;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;
				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float4 positionCS = TransformWorldToHClip( positionWS );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = positionCS;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				o.clipPos = positionCS;
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_texcoord = v.ase_texcoord;
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_texcoord = patch[0].ase_texcoord * bary.x + patch[1].ase_texcoord * bary.y + patch[2].ase_texcoord * bary.z;
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif
			half4 frag(	VertexOutput IN 
						#ifdef ASE_DEPTH_WRITE_ON
						,out float outputDepth : ASE_SV_DEPTH
						#endif
						, FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				float _CrossMode_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_CrossMode);
				float CrossMode198 = _CrossMode_Instance;
				int CurrentPassMode188 = (int)CrossMode198;
				float4 _MainTex_ST_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_MainTex_ST);
				float2 uv_MainTex = IN.ase_texcoord2.xy * _MainTex_ST_Instance.xy + _MainTex_ST_Instance.zw;
				float4 AlbedoResult116 = ( tex2D( _MainTex, uv_MainTex ) * _MainColor );
				float4 CorssColor134 = _CrossColor;
				float4 switchResult81 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float dotResult46 = dot( float3(1,0,0) , ( WorldPosition - (_PosYZ).xyz ) );
				float PosYZ57 = ( dotResult46 + _PosYZ.w );
				float dotResult47 = dot( float3(0,1,0) , ( WorldPosition - (_PosXZ).xyz ) );
				float PosXZ57 = ( dotResult47 + _PosXZ.w );
				float dotResult48 = dot( float3(0,0,1) , ( WorldPosition - (_PosXY).xyz ) );
				float PosXY57 = ( dotResult48 + _PosXY.w );
				float localCheckCrossResult57 = CheckCrossResult( PosYZ57 , PosXZ57 , PosXY57 );
				clip( -( CrossMode198 == 0.0 ? localCheckCrossResult57 : 0.0 ) );
				float4 ThreeAAplaneResult111 = switchResult81;
				float4 ThreeAAplane188 = ThreeAAplaneResult111;
				float4 switchResult133 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4x4 _BoxRotationMatrix_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_BoxRotationMatrix);
				float4x4 invertVal91 = Inverse4x4( _BoxRotationMatrix_Instance );
				float3 break98 = ( mul( invertVal91, float4( ( WorldPosition - _Cubepos ) , 0.0 ) ).xyz + _Cubepos );
				float3 break100 = ( _Cubepos - _CubeExtent );
				float3 break99 = ( _Cubepos + _CubeExtent );
				float PosYZ115 = (( break98.x >= break100.x && break98.x <= break99.x ) ? 1.0 :  0.0 );
				float PosXZ115 = (( break98.y >= break100.y && break98.y <= break99.y ) ? 1.0 :  0.0 );
				float PosXY115 = (( break98.z >= break100.z && break98.z <= break99.z ) ? 1.0 :  0.0 );
				float localreturnPosYZ0PosXZ0PosXY0115 = returnPosYZ0PosXZ0PosXY0115( PosYZ115 , PosXZ115 , PosXY115 );
				clip( -( CrossMode198 == 1.0 ? localreturnPosYZ0PosXZ0PosXY0115 : 0.0 ) );
				float4 BoxCrossResult118 = switchResult133;
				float4 SingleBoxCross188 = BoxCrossResult118;
				float4 switchResult162 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4 DefaultColor156 = switchResult162;
				float3 WorldPosition156 = WorldPosition;
				int ArrayLength156 = (int)_CrossCubsArrayLength;
				float4 localMultiBoxCrossProcess156 = MultiBoxCrossProcess( DefaultColor156 , WorldPosition156 , ArrayLength156 );
				float4 MultiBoxCrossResult159 = localMultiBoxCrossProcess156;
				float4 MultiBoxCross188 = MultiBoxCrossResult159;
				float4 localFinalCrossModeCheck188 = FinalCrossModeCheck188( CurrentPassMode188 , ThreeAAplane188 , SingleBoxCross188 , MultiBoxCross188 );
				
				float Alpha = localFinalCrossModeCheck188.x;
				float AlphaClipThreshold = 0.5;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif
				#ifdef ASE_DEPTH_WRITE_ON
				outputDepth = DepthValue;
				#endif

				return 0;
			}
			ENDHLSL
		}
		
		
		Pass
		{
			
			Name "Meta"
			Tags { "LightMode"="Meta" }

			Cull Off

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define ASE_SRP_VERSION 110000

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_META

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/MetaInput.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"

			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma multi_compile_instancing


			#pragma shader_feature _ _SMOOTHNESS_TEXTURE_ALBEDO_CHANNEL_A

			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 texcoord1 : TEXCOORD1;
				float4 texcoord2 : TEXCOORD2;
				float4 ase_texcoord : TEXCOORD0;
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				float4 ase_texcoord2 : TEXCOORD2;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _MainColor;
			float4 _CrossColor;
			float4 _PosYZ;
			float4 _PosXZ;
			float4 _PosXY;
			float3 _Cubepos;
			float3 _CubeExtent;
			float _ShowCrossArea;
			float _CullMode;
			float _CrossCubsArrayLength;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			float4 CrossCubeExtentArray[24];
			float4x4 CrossCubeRotationMatrixArray[24];
			float4 CrossCubePosArray[24];
			sampler2D _MainTex;
			UNITY_INSTANCING_BUFFER_START(DmpURPEffectsCrossSectionMixCrossSection)
				UNITY_DEFINE_INSTANCED_PROP(float4x4, _BoxRotationMatrix)
				UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST)
				UNITY_DEFINE_INSTANCED_PROP(float, _CrossMode)
			UNITY_INSTANCING_BUFFER_END(DmpURPEffectsCrossSectionMixCrossSection)


			float CheckCrossResult( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4x4 Inverse4x4(float4x4 input)
			{
				#define minor(a,b,c) determinant(float3x3(input.a, input.b, input.c))
				float4x4 cofactors = float4x4(
				minor( _22_23_24, _32_33_34, _42_43_44 ),
				-minor( _21_23_24, _31_33_34, _41_43_44 ),
				minor( _21_22_24, _31_32_34, _41_42_44 ),
				-minor( _21_22_23, _31_32_33, _41_42_43 ),
			
				-minor( _12_13_14, _32_33_34, _42_43_44 ),
				minor( _11_13_14, _31_33_34, _41_43_44 ),
				-minor( _11_12_14, _31_32_34, _41_42_44 ),
				minor( _11_12_13, _31_32_33, _41_42_43 ),
			
				minor( _12_13_14, _22_23_24, _42_43_44 ),
				-minor( _11_13_14, _21_23_24, _41_43_44 ),
				minor( _11_12_14, _21_22_24, _41_42_44 ),
				-minor( _11_12_13, _21_22_23, _41_42_43 ),
			
				-minor( _12_13_14, _22_23_24, _32_33_34 ),
				minor( _11_13_14, _21_23_24, _31_33_34 ),
				-minor( _11_12_14, _21_22_24, _31_32_34 ),
				minor( _11_12_13, _21_22_23, _31_32_33 ));
				#undef minor
				return transpose( cofactors ) / determinant( input );
			}
			
			float returnPosYZ0PosXZ0PosXY0115( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4 MultiBoxCrossProcess( float4 DefaultColor, float3 WorldPosition, int ArrayLength )
			{
				bool crossArea[24];
				for(int i=0;i<ArrayLength;i++)
				{
				crossArea[i] = 1;
				float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz;
				float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i];
				float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i];
				int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0;
				int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0;
				int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0;
				bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0;
				if(isCross && _ShowCrossArea)
				{
				return _CrossColor;
				}
				if(_CrossMode == 2)
				{
				clip( -(isCross));
				}
				}
				return DefaultColor;
			}
			
			float4 FinalCrossModeCheck188( int CurrentPassMode, float4 ThreeAAplane, float4 SingleBoxCross, float4 MultiBoxCross )
			{
				 if(CurrentPassMode == 0)
				{
				return ThreeAAplane;
				}
				 if(CurrentPassMode == 1)
				{
				return SingleBoxCross;
				}
				 if(CurrentPassMode == 2)
				{
				return MultiBoxCross;
				}
				return float4(1,1,1,1);
			}
			

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				o.ase_texcoord2.xy = v.ase_texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord2.zw = 0;
				
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif

				o.clipPos = MetaVertexPosition( v.vertex, v.texcoord1.xy, v.texcoord1.xy, unity_LightmapST, unity_DynamicLightmapST );
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = o.clipPos;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 texcoord1 : TEXCOORD1;
				float4 texcoord2 : TEXCOORD2;
				float4 ase_texcoord : TEXCOORD0;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.texcoord1 = v.texcoord1;
				o.texcoord2 = v.texcoord2;
				o.ase_texcoord = v.ase_texcoord;
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.texcoord1 = patch[0].texcoord1 * bary.x + patch[1].texcoord1 * bary.y + patch[2].texcoord1 * bary.z;
				o.texcoord2 = patch[0].texcoord2 * bary.x + patch[1].texcoord2 * bary.y + patch[2].texcoord2 * bary.z;
				o.ase_texcoord = patch[0].ase_texcoord * bary.x + patch[1].ase_texcoord * bary.y + patch[2].ase_texcoord * bary.z;
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			half4 frag(VertexOutput IN , FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				float _CrossMode_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_CrossMode);
				float CrossMode198 = _CrossMode_Instance;
				int CurrentPassMode188 = (int)CrossMode198;
				float4 _MainTex_ST_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_MainTex_ST);
				float2 uv_MainTex = IN.ase_texcoord2.xy * _MainTex_ST_Instance.xy + _MainTex_ST_Instance.zw;
				float4 AlbedoResult116 = ( tex2D( _MainTex, uv_MainTex ) * _MainColor );
				float4 CorssColor134 = _CrossColor;
				float4 switchResult81 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float dotResult46 = dot( float3(1,0,0) , ( WorldPosition - (_PosYZ).xyz ) );
				float PosYZ57 = ( dotResult46 + _PosYZ.w );
				float dotResult47 = dot( float3(0,1,0) , ( WorldPosition - (_PosXZ).xyz ) );
				float PosXZ57 = ( dotResult47 + _PosXZ.w );
				float dotResult48 = dot( float3(0,0,1) , ( WorldPosition - (_PosXY).xyz ) );
				float PosXY57 = ( dotResult48 + _PosXY.w );
				float localCheckCrossResult57 = CheckCrossResult( PosYZ57 , PosXZ57 , PosXY57 );
				clip( -( CrossMode198 == 0.0 ? localCheckCrossResult57 : 0.0 ) );
				float4 ThreeAAplaneResult111 = switchResult81;
				float4 ThreeAAplane188 = ThreeAAplaneResult111;
				float4 switchResult133 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4x4 _BoxRotationMatrix_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_BoxRotationMatrix);
				float4x4 invertVal91 = Inverse4x4( _BoxRotationMatrix_Instance );
				float3 break98 = ( mul( invertVal91, float4( ( WorldPosition - _Cubepos ) , 0.0 ) ).xyz + _Cubepos );
				float3 break100 = ( _Cubepos - _CubeExtent );
				float3 break99 = ( _Cubepos + _CubeExtent );
				float PosYZ115 = (( break98.x >= break100.x && break98.x <= break99.x ) ? 1.0 :  0.0 );
				float PosXZ115 = (( break98.y >= break100.y && break98.y <= break99.y ) ? 1.0 :  0.0 );
				float PosXY115 = (( break98.z >= break100.z && break98.z <= break99.z ) ? 1.0 :  0.0 );
				float localreturnPosYZ0PosXZ0PosXY0115 = returnPosYZ0PosXZ0PosXY0115( PosYZ115 , PosXZ115 , PosXY115 );
				clip( -( CrossMode198 == 1.0 ? localreturnPosYZ0PosXZ0PosXY0115 : 0.0 ) );
				float4 BoxCrossResult118 = switchResult133;
				float4 SingleBoxCross188 = BoxCrossResult118;
				float4 switchResult162 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4 DefaultColor156 = switchResult162;
				float3 WorldPosition156 = WorldPosition;
				int ArrayLength156 = (int)_CrossCubsArrayLength;
				float4 localMultiBoxCrossProcess156 = MultiBoxCrossProcess( DefaultColor156 , WorldPosition156 , ArrayLength156 );
				float4 MultiBoxCrossResult159 = localMultiBoxCrossProcess156;
				float4 MultiBoxCross188 = MultiBoxCrossResult159;
				float4 localFinalCrossModeCheck188 = FinalCrossModeCheck188( CurrentPassMode188 , ThreeAAplane188 , SingleBoxCross188 , MultiBoxCross188 );
				
				
				float3 Albedo = localFinalCrossModeCheck188.xyz;
				float3 Emission = 0;
				float Alpha = localFinalCrossModeCheck188.x;
				float AlphaClipThreshold = 0.5;

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				MetaInput metaInput = (MetaInput)0;
				metaInput.Albedo = Albedo;
				metaInput.Emission = Emission;
				
				return MetaFragment(metaInput);
			}
			ENDHLSL
		}

		
		Pass
		{
			
			Name "Universal2D"
			Tags { "LightMode"="Universal2D" }

			Blend One Zero, One Zero
			ZWrite On
			ZTest LEqual
			Offset 0 , 0
			ColorMask RGBA

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define ASE_SRP_VERSION 110000

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_2D

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			
			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma multi_compile_instancing


			#pragma shader_feature _ _SMOOTHNESS_TEXTURE_ALBEDO_CHANNEL_A

			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				float4 ase_texcoord2 : TEXCOORD2;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _MainColor;
			float4 _CrossColor;
			float4 _PosYZ;
			float4 _PosXZ;
			float4 _PosXY;
			float3 _Cubepos;
			float3 _CubeExtent;
			float _ShowCrossArea;
			float _CullMode;
			float _CrossCubsArrayLength;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			float4 CrossCubeExtentArray[24];
			float4x4 CrossCubeRotationMatrixArray[24];
			float4 CrossCubePosArray[24];
			sampler2D _MainTex;
			UNITY_INSTANCING_BUFFER_START(DmpURPEffectsCrossSectionMixCrossSection)
				UNITY_DEFINE_INSTANCED_PROP(float4x4, _BoxRotationMatrix)
				UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST)
				UNITY_DEFINE_INSTANCED_PROP(float, _CrossMode)
			UNITY_INSTANCING_BUFFER_END(DmpURPEffectsCrossSectionMixCrossSection)


			float CheckCrossResult( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4x4 Inverse4x4(float4x4 input)
			{
				#define minor(a,b,c) determinant(float3x3(input.a, input.b, input.c))
				float4x4 cofactors = float4x4(
				minor( _22_23_24, _32_33_34, _42_43_44 ),
				-minor( _21_23_24, _31_33_34, _41_43_44 ),
				minor( _21_22_24, _31_32_34, _41_42_44 ),
				-minor( _21_22_23, _31_32_33, _41_42_43 ),
			
				-minor( _12_13_14, _32_33_34, _42_43_44 ),
				minor( _11_13_14, _31_33_34, _41_43_44 ),
				-minor( _11_12_14, _31_32_34, _41_42_44 ),
				minor( _11_12_13, _31_32_33, _41_42_43 ),
			
				minor( _12_13_14, _22_23_24, _42_43_44 ),
				-minor( _11_13_14, _21_23_24, _41_43_44 ),
				minor( _11_12_14, _21_22_24, _41_42_44 ),
				-minor( _11_12_13, _21_22_23, _41_42_43 ),
			
				-minor( _12_13_14, _22_23_24, _32_33_34 ),
				minor( _11_13_14, _21_23_24, _31_33_34 ),
				-minor( _11_12_14, _21_22_24, _31_32_34 ),
				minor( _11_12_13, _21_22_23, _31_32_33 ));
				#undef minor
				return transpose( cofactors ) / determinant( input );
			}
			
			float returnPosYZ0PosXZ0PosXY0115( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4 MultiBoxCrossProcess( float4 DefaultColor, float3 WorldPosition, int ArrayLength )
			{
				bool crossArea[24];
				for(int i=0;i<ArrayLength;i++)
				{
				crossArea[i] = 1;
				float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz;
				float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i];
				float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i];
				int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0;
				int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0;
				int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0;
				bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0;
				if(isCross && _ShowCrossArea)
				{
				return _CrossColor;
				}
				if(_CrossMode == 2)
				{
				clip( -(isCross));
				}
				}
				return DefaultColor;
			}
			
			float4 FinalCrossModeCheck188( int CurrentPassMode, float4 ThreeAAplane, float4 SingleBoxCross, float4 MultiBoxCross )
			{
				 if(CurrentPassMode == 0)
				{
				return ThreeAAplane;
				}
				 if(CurrentPassMode == 1)
				{
				return SingleBoxCross;
				}
				 if(CurrentPassMode == 2)
				{
				return MultiBoxCross;
				}
				return float4(1,1,1,1);
			}
			

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID( v );
				UNITY_TRANSFER_INSTANCE_ID( v, o );
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO( o );

				o.ase_texcoord2.xy = v.ase_texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord2.zw = 0;
				
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float4 positionCS = TransformWorldToHClip( positionWS );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = positionCS;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif

				o.clipPos = positionCS;
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_texcoord = v.ase_texcoord;
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_texcoord = patch[0].ase_texcoord * bary.x + patch[1].ase_texcoord * bary.y + patch[2].ase_texcoord * bary.z;
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			half4 frag(VertexOutput IN , FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID( IN );
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				float _CrossMode_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_CrossMode);
				float CrossMode198 = _CrossMode_Instance;
				int CurrentPassMode188 = (int)CrossMode198;
				float4 _MainTex_ST_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_MainTex_ST);
				float2 uv_MainTex = IN.ase_texcoord2.xy * _MainTex_ST_Instance.xy + _MainTex_ST_Instance.zw;
				float4 AlbedoResult116 = ( tex2D( _MainTex, uv_MainTex ) * _MainColor );
				float4 CorssColor134 = _CrossColor;
				float4 switchResult81 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float dotResult46 = dot( float3(1,0,0) , ( WorldPosition - (_PosYZ).xyz ) );
				float PosYZ57 = ( dotResult46 + _PosYZ.w );
				float dotResult47 = dot( float3(0,1,0) , ( WorldPosition - (_PosXZ).xyz ) );
				float PosXZ57 = ( dotResult47 + _PosXZ.w );
				float dotResult48 = dot( float3(0,0,1) , ( WorldPosition - (_PosXY).xyz ) );
				float PosXY57 = ( dotResult48 + _PosXY.w );
				float localCheckCrossResult57 = CheckCrossResult( PosYZ57 , PosXZ57 , PosXY57 );
				clip( -( CrossMode198 == 0.0 ? localCheckCrossResult57 : 0.0 ) );
				float4 ThreeAAplaneResult111 = switchResult81;
				float4 ThreeAAplane188 = ThreeAAplaneResult111;
				float4 switchResult133 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4x4 _BoxRotationMatrix_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_BoxRotationMatrix);
				float4x4 invertVal91 = Inverse4x4( _BoxRotationMatrix_Instance );
				float3 break98 = ( mul( invertVal91, float4( ( WorldPosition - _Cubepos ) , 0.0 ) ).xyz + _Cubepos );
				float3 break100 = ( _Cubepos - _CubeExtent );
				float3 break99 = ( _Cubepos + _CubeExtent );
				float PosYZ115 = (( break98.x >= break100.x && break98.x <= break99.x ) ? 1.0 :  0.0 );
				float PosXZ115 = (( break98.y >= break100.y && break98.y <= break99.y ) ? 1.0 :  0.0 );
				float PosXY115 = (( break98.z >= break100.z && break98.z <= break99.z ) ? 1.0 :  0.0 );
				float localreturnPosYZ0PosXZ0PosXY0115 = returnPosYZ0PosXZ0PosXY0115( PosYZ115 , PosXZ115 , PosXY115 );
				clip( -( CrossMode198 == 1.0 ? localreturnPosYZ0PosXZ0PosXY0115 : 0.0 ) );
				float4 BoxCrossResult118 = switchResult133;
				float4 SingleBoxCross188 = BoxCrossResult118;
				float4 switchResult162 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4 DefaultColor156 = switchResult162;
				float3 WorldPosition156 = WorldPosition;
				int ArrayLength156 = (int)_CrossCubsArrayLength;
				float4 localMultiBoxCrossProcess156 = MultiBoxCrossProcess( DefaultColor156 , WorldPosition156 , ArrayLength156 );
				float4 MultiBoxCrossResult159 = localMultiBoxCrossProcess156;
				float4 MultiBoxCross188 = MultiBoxCrossResult159;
				float4 localFinalCrossModeCheck188 = FinalCrossModeCheck188( CurrentPassMode188 , ThreeAAplane188 , SingleBoxCross188 , MultiBoxCross188 );
				
				
				float3 Albedo = localFinalCrossModeCheck188.xyz;
				float Alpha = localFinalCrossModeCheck188.x;
				float AlphaClipThreshold = 0.5;

				half4 color = half4( Albedo, Alpha );

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				return color;
			}
			ENDHLSL
		}

		
		Pass
		{
			
			Name "DepthNormals"
			Tags { "LightMode"="DepthNormals" }

			ZWrite On
			Blend One Zero
            ZTest LEqual
            ZWrite On

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define ASE_SRP_VERSION 110000

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_DEPTHNORMALSONLY

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"

			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma multi_compile_instancing


			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				float3 worldNormal : TEXCOORD2;
				float4 ase_texcoord3 : TEXCOORD3;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _MainColor;
			float4 _CrossColor;
			float4 _PosYZ;
			float4 _PosXZ;
			float4 _PosXY;
			float3 _Cubepos;
			float3 _CubeExtent;
			float _ShowCrossArea;
			float _CullMode;
			float _CrossCubsArrayLength;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			float4 CrossCubeExtentArray[24];
			float4x4 CrossCubeRotationMatrixArray[24];
			float4 CrossCubePosArray[24];
			sampler2D _MainTex;
			UNITY_INSTANCING_BUFFER_START(DmpURPEffectsCrossSectionMixCrossSection)
				UNITY_DEFINE_INSTANCED_PROP(float4x4, _BoxRotationMatrix)
				UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST)
				UNITY_DEFINE_INSTANCED_PROP(float, _CrossMode)
			UNITY_INSTANCING_BUFFER_END(DmpURPEffectsCrossSectionMixCrossSection)


			float CheckCrossResult( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4x4 Inverse4x4(float4x4 input)
			{
				#define minor(a,b,c) determinant(float3x3(input.a, input.b, input.c))
				float4x4 cofactors = float4x4(
				minor( _22_23_24, _32_33_34, _42_43_44 ),
				-minor( _21_23_24, _31_33_34, _41_43_44 ),
				minor( _21_22_24, _31_32_34, _41_42_44 ),
				-minor( _21_22_23, _31_32_33, _41_42_43 ),
			
				-minor( _12_13_14, _32_33_34, _42_43_44 ),
				minor( _11_13_14, _31_33_34, _41_43_44 ),
				-minor( _11_12_14, _31_32_34, _41_42_44 ),
				minor( _11_12_13, _31_32_33, _41_42_43 ),
			
				minor( _12_13_14, _22_23_24, _42_43_44 ),
				-minor( _11_13_14, _21_23_24, _41_43_44 ),
				minor( _11_12_14, _21_22_24, _41_42_44 ),
				-minor( _11_12_13, _21_22_23, _41_42_43 ),
			
				-minor( _12_13_14, _22_23_24, _32_33_34 ),
				minor( _11_13_14, _21_23_24, _31_33_34 ),
				-minor( _11_12_14, _21_22_24, _31_32_34 ),
				minor( _11_12_13, _21_22_23, _31_32_33 ));
				#undef minor
				return transpose( cofactors ) / determinant( input );
			}
			
			float returnPosYZ0PosXZ0PosXY0115( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4 MultiBoxCrossProcess( float4 DefaultColor, float3 WorldPosition, int ArrayLength )
			{
				bool crossArea[24];
				for(int i=0;i<ArrayLength;i++)
				{
				crossArea[i] = 1;
				float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz;
				float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i];
				float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i];
				int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0;
				int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0;
				int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0;
				bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0;
				if(isCross && _ShowCrossArea)
				{
				return _CrossColor;
				}
				if(_CrossMode == 2)
				{
				clip( -(isCross));
				}
				}
				return DefaultColor;
			}
			
			float4 FinalCrossModeCheck188( int CurrentPassMode, float4 ThreeAAplane, float4 SingleBoxCross, float4 MultiBoxCross )
			{
				 if(CurrentPassMode == 0)
				{
				return ThreeAAplane;
				}
				 if(CurrentPassMode == 1)
				{
				return SingleBoxCross;
				}
				 if(CurrentPassMode == 2)
				{
				return MultiBoxCross;
				}
				return float4(1,1,1,1);
			}
			

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				o.ase_texcoord3.xy = v.ase_texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord3.zw = 0;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;
				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float3 normalWS = TransformObjectToWorldNormal( v.ase_normal );
				float4 positionCS = TransformWorldToHClip( positionWS );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif

				o.worldNormal = normalWS;

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = positionCS;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				o.clipPos = positionCS;
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_texcoord = v.ase_texcoord;
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_texcoord = patch[0].ase_texcoord * bary.x + patch[1].ase_texcoord * bary.y + patch[2].ase_texcoord * bary.z;
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif
			half4 frag(	VertexOutput IN 
						#ifdef ASE_DEPTH_WRITE_ON
						,out float outputDepth : ASE_SV_DEPTH
						#endif
						, FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				float _CrossMode_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_CrossMode);
				float CrossMode198 = _CrossMode_Instance;
				int CurrentPassMode188 = (int)CrossMode198;
				float4 _MainTex_ST_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_MainTex_ST);
				float2 uv_MainTex = IN.ase_texcoord3.xy * _MainTex_ST_Instance.xy + _MainTex_ST_Instance.zw;
				float4 AlbedoResult116 = ( tex2D( _MainTex, uv_MainTex ) * _MainColor );
				float4 CorssColor134 = _CrossColor;
				float4 switchResult81 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float dotResult46 = dot( float3(1,0,0) , ( WorldPosition - (_PosYZ).xyz ) );
				float PosYZ57 = ( dotResult46 + _PosYZ.w );
				float dotResult47 = dot( float3(0,1,0) , ( WorldPosition - (_PosXZ).xyz ) );
				float PosXZ57 = ( dotResult47 + _PosXZ.w );
				float dotResult48 = dot( float3(0,0,1) , ( WorldPosition - (_PosXY).xyz ) );
				float PosXY57 = ( dotResult48 + _PosXY.w );
				float localCheckCrossResult57 = CheckCrossResult( PosYZ57 , PosXZ57 , PosXY57 );
				clip( -( CrossMode198 == 0.0 ? localCheckCrossResult57 : 0.0 ) );
				float4 ThreeAAplaneResult111 = switchResult81;
				float4 ThreeAAplane188 = ThreeAAplaneResult111;
				float4 switchResult133 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4x4 _BoxRotationMatrix_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_BoxRotationMatrix);
				float4x4 invertVal91 = Inverse4x4( _BoxRotationMatrix_Instance );
				float3 break98 = ( mul( invertVal91, float4( ( WorldPosition - _Cubepos ) , 0.0 ) ).xyz + _Cubepos );
				float3 break100 = ( _Cubepos - _CubeExtent );
				float3 break99 = ( _Cubepos + _CubeExtent );
				float PosYZ115 = (( break98.x >= break100.x && break98.x <= break99.x ) ? 1.0 :  0.0 );
				float PosXZ115 = (( break98.y >= break100.y && break98.y <= break99.y ) ? 1.0 :  0.0 );
				float PosXY115 = (( break98.z >= break100.z && break98.z <= break99.z ) ? 1.0 :  0.0 );
				float localreturnPosYZ0PosXZ0PosXY0115 = returnPosYZ0PosXZ0PosXY0115( PosYZ115 , PosXZ115 , PosXY115 );
				clip( -( CrossMode198 == 1.0 ? localreturnPosYZ0PosXZ0PosXY0115 : 0.0 ) );
				float4 BoxCrossResult118 = switchResult133;
				float4 SingleBoxCross188 = BoxCrossResult118;
				float4 switchResult162 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4 DefaultColor156 = switchResult162;
				float3 WorldPosition156 = WorldPosition;
				int ArrayLength156 = (int)_CrossCubsArrayLength;
				float4 localMultiBoxCrossProcess156 = MultiBoxCrossProcess( DefaultColor156 , WorldPosition156 , ArrayLength156 );
				float4 MultiBoxCrossResult159 = localMultiBoxCrossProcess156;
				float4 MultiBoxCross188 = MultiBoxCrossResult159;
				float4 localFinalCrossModeCheck188 = FinalCrossModeCheck188( CurrentPassMode188 , ThreeAAplane188 , SingleBoxCross188 , MultiBoxCross188 );
				
				float Alpha = localFinalCrossModeCheck188.x;
				float AlphaClipThreshold = 0.5;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif
				
				#ifdef ASE_DEPTH_WRITE_ON
				outputDepth = DepthValue;
				#endif
				
				return float4(PackNormalOctRectEncode(TransformWorldToViewDir(IN.worldNormal, true)), 0.0, 0.0);
			}
			ENDHLSL
		}

		
		Pass
		{
			
			Name "GBuffer"
			Tags { "LightMode"="UniversalGBuffer" }
			
			Blend One Zero, One Zero
			ZWrite On
			ZTest LEqual
			Offset 0 , 0
			ColorMask RGBA
			

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define ASE_SRP_VERSION 110000

			
			#pragma multi_compile _ _MAIN_LIGHT_SHADOWS
			#pragma multi_compile _ _MAIN_LIGHT_SHADOWS_CASCADE
			#pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS
			#pragma multi_compile _ _ADDITIONAL_LIGHT_SHADOWS
			#pragma multi_compile _ _SHADOWS_SOFT
			#pragma multi_compile _ _MIXED_LIGHTING_SUBTRACTIVE
			#pragma multi_compile _ _GBUFFER_NORMALS_OCT
			
			#pragma multi_compile _ DIRLIGHTMAP_COMBINED
			#pragma multi_compile _ LIGHTMAP_ON

			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS SHADERPASS_GBUFFER

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/UnityGBuffer.hlsl"

			#if ASE_SRP_VERSION <= 70108
			#define REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR
			#endif

			#if defined(UNITY_INSTANCING_ENABLED) && defined(_TERRAIN_INSTANCED_PERPIXEL_NORMAL)
			    #define ENABLE_TERRAIN_PERPIXEL_NORMAL
			#endif

			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma multi_compile_instancing


			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_tangent : TANGENT;
				float4 texcoord1 : TEXCOORD1;
				float4 texcoord : TEXCOORD0;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				float4 lightmapUVOrVertexSH : TEXCOORD0;
				half4 fogFactorAndVertexLight : TEXCOORD1;
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
				float4 shadowCoord : TEXCOORD2;
				#endif
				float4 tSpace0 : TEXCOORD3;
				float4 tSpace1 : TEXCOORD4;
				float4 tSpace2 : TEXCOORD5;
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				float4 screenPos : TEXCOORD6;
				#endif
				float4 ase_texcoord7 : TEXCOORD7;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _MainColor;
			float4 _CrossColor;
			float4 _PosYZ;
			float4 _PosXZ;
			float4 _PosXY;
			float3 _Cubepos;
			float3 _CubeExtent;
			float _ShowCrossArea;
			float _CullMode;
			float _CrossCubsArrayLength;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			float4 CrossCubeExtentArray[24];
			float4x4 CrossCubeRotationMatrixArray[24];
			float4 CrossCubePosArray[24];
			sampler2D _MainTex;
			UNITY_INSTANCING_BUFFER_START(DmpURPEffectsCrossSectionMixCrossSection)
				UNITY_DEFINE_INSTANCED_PROP(float4x4, _BoxRotationMatrix)
				UNITY_DEFINE_INSTANCED_PROP(float4, _MainTex_ST)
				UNITY_DEFINE_INSTANCED_PROP(float, _CrossMode)
			UNITY_INSTANCING_BUFFER_END(DmpURPEffectsCrossSectionMixCrossSection)


			float CheckCrossResult( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4x4 Inverse4x4(float4x4 input)
			{
				#define minor(a,b,c) determinant(float3x3(input.a, input.b, input.c))
				float4x4 cofactors = float4x4(
				minor( _22_23_24, _32_33_34, _42_43_44 ),
				-minor( _21_23_24, _31_33_34, _41_43_44 ),
				minor( _21_22_24, _31_32_34, _41_42_44 ),
				-minor( _21_22_23, _31_32_33, _41_42_43 ),
			
				-minor( _12_13_14, _32_33_34, _42_43_44 ),
				minor( _11_13_14, _31_33_34, _41_43_44 ),
				-minor( _11_12_14, _31_32_34, _41_42_44 ),
				minor( _11_12_13, _31_32_33, _41_42_43 ),
			
				minor( _12_13_14, _22_23_24, _42_43_44 ),
				-minor( _11_13_14, _21_23_24, _41_43_44 ),
				minor( _11_12_14, _21_22_24, _41_42_44 ),
				-minor( _11_12_13, _21_22_23, _41_42_43 ),
			
				-minor( _12_13_14, _22_23_24, _32_33_34 ),
				minor( _11_13_14, _21_23_24, _31_33_34 ),
				-minor( _11_12_14, _21_22_24, _31_32_34 ),
				minor( _11_12_13, _21_22_23, _31_32_33 ));
				#undef minor
				return transpose( cofactors ) / determinant( input );
			}
			
			float returnPosYZ0PosXZ0PosXY0115( float PosYZ, float PosXZ, float PosXY )
			{
				return PosYZ > 0 && PosXZ > 0 && PosXY > 0;
			}
			
			float4 MultiBoxCrossProcess( float4 DefaultColor, float3 WorldPosition, int ArrayLength )
			{
				bool crossArea[24];
				for(int i=0;i<ArrayLength;i++)
				{
				crossArea[i] = 1;
				float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz;
				float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i];
				float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i];
				int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0;
				int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0;
				int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0;
				bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0;
				if(isCross && _ShowCrossArea)
				{
				return _CrossColor;
				}
				if(_CrossMode == 2)
				{
				clip( -(isCross));
				}
				}
				return DefaultColor;
			}
			
			float4 FinalCrossModeCheck188( int CurrentPassMode, float4 ThreeAAplane, float4 SingleBoxCross, float4 MultiBoxCross )
			{
				 if(CurrentPassMode == 0)
				{
				return ThreeAAplane;
				}
				 if(CurrentPassMode == 1)
				{
				return SingleBoxCross;
				}
				 if(CurrentPassMode == 2)
				{
				return MultiBoxCross;
				}
				return float4(1,1,1,1);
			}
			

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				o.ase_texcoord7.xy = v.texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord7.zw = 0;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif
				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float3 positionVS = TransformWorldToView( positionWS );
				float4 positionCS = TransformWorldToHClip( positionWS );

				VertexNormalInputs normalInput = GetVertexNormalInputs( v.ase_normal, v.ase_tangent );

				o.tSpace0 = float4( normalInput.normalWS, positionWS.x);
				o.tSpace1 = float4( normalInput.tangentWS, positionWS.y);
				o.tSpace2 = float4( normalInput.bitangentWS, positionWS.z);

				OUTPUT_LIGHTMAP_UV( v.texcoord1, unity_LightmapST, o.lightmapUVOrVertexSH.xy );
				OUTPUT_SH( normalInput.normalWS.xyz, o.lightmapUVOrVertexSH.xyz );

				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					o.lightmapUVOrVertexSH.zw = v.texcoord;
					o.lightmapUVOrVertexSH.xy = v.texcoord * unity_LightmapST.xy + unity_LightmapST.zw;
				#endif

				half3 vertexLight = VertexLighting( positionWS, normalInput.normalWS );
				#ifdef ASE_FOG
					half fogFactor = ComputeFogFactor( positionCS.z );
				#else
					half fogFactor = 0;
				#endif
				o.fogFactorAndVertexLight = half4(fogFactor, vertexLight);
				
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
				VertexPositionInputs vertexInput = (VertexPositionInputs)0;
				vertexInput.positionWS = positionWS;
				vertexInput.positionCS = positionCS;
				o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				
				o.clipPos = positionCS;
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				o.screenPos = ComputeScreenPos(positionCS);
				#endif
				return o;
			}
			
			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_tangent : TANGENT;
				float4 texcoord : TEXCOORD0;
				float4 texcoord1 : TEXCOORD1;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_tangent = v.ase_tangent;
				o.texcoord = v.texcoord;
				o.texcoord1 = v.texcoord1;
				
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_tangent = patch[0].ase_tangent * bary.x + patch[1].ase_tangent * bary.y + patch[2].ase_tangent * bary.z;
				o.texcoord = patch[0].texcoord * bary.x + patch[1].texcoord * bary.y + patch[2].texcoord * bary.z;
				o.texcoord1 = patch[0].texcoord1 * bary.x + patch[1].texcoord1 * bary.y + patch[2].texcoord1 * bary.z;
				
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif
			FragmentOutput frag ( VertexOutput IN 
								#ifdef ASE_DEPTH_WRITE_ON
								,out float outputDepth : ASE_SV_DEPTH
								#endif
								, FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC )
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(IN);

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif

				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					float2 sampleCoords = (IN.lightmapUVOrVertexSH.zw / _TerrainHeightmapRecipSize.zw + 0.5f) * _TerrainHeightmapRecipSize.xy;
					float3 WorldNormal = TransformObjectToWorldNormal(normalize(SAMPLE_TEXTURE2D(_TerrainNormalmapTexture, sampler_TerrainNormalmapTexture, sampleCoords).rgb * 2 - 1));
					float3 WorldTangent = -cross(GetObjectToWorldMatrix()._13_23_33, WorldNormal);
					float3 WorldBiTangent = cross(WorldNormal, -WorldTangent);
				#else
					float3 WorldNormal = normalize( IN.tSpace0.xyz );
					float3 WorldTangent = IN.tSpace1.xyz;
					float3 WorldBiTangent = IN.tSpace2.xyz;
				#endif
				float3 WorldPosition = float3(IN.tSpace0.w,IN.tSpace1.w,IN.tSpace2.w);
				float3 WorldViewDirection = _WorldSpaceCameraPos.xyz  - WorldPosition;
				float4 ShadowCoords = float4( 0, 0, 0, 0 );
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				float4 ScreenPos = IN.screenPos;
				#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
					ShadowCoords = IN.shadowCoord;
				#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
					ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
				#endif
	
				WorldViewDirection = SafeNormalize( WorldViewDirection );

				float _CrossMode_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_CrossMode);
				float CrossMode198 = _CrossMode_Instance;
				int CurrentPassMode188 = (int)CrossMode198;
				float4 _MainTex_ST_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_MainTex_ST);
				float2 uv_MainTex = IN.ase_texcoord7.xy * _MainTex_ST_Instance.xy + _MainTex_ST_Instance.zw;
				float4 AlbedoResult116 = ( tex2D( _MainTex, uv_MainTex ) * _MainColor );
				float4 CorssColor134 = _CrossColor;
				float4 switchResult81 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float dotResult46 = dot( float3(1,0,0) , ( WorldPosition - (_PosYZ).xyz ) );
				float PosYZ57 = ( dotResult46 + _PosYZ.w );
				float dotResult47 = dot( float3(0,1,0) , ( WorldPosition - (_PosXZ).xyz ) );
				float PosXZ57 = ( dotResult47 + _PosXZ.w );
				float dotResult48 = dot( float3(0,0,1) , ( WorldPosition - (_PosXY).xyz ) );
				float PosXY57 = ( dotResult48 + _PosXY.w );
				float localCheckCrossResult57 = CheckCrossResult( PosYZ57 , PosXZ57 , PosXY57 );
				clip( -( CrossMode198 == 0.0 ? localCheckCrossResult57 : 0.0 ) );
				float4 ThreeAAplaneResult111 = switchResult81;
				float4 ThreeAAplane188 = ThreeAAplaneResult111;
				float4 switchResult133 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4x4 _BoxRotationMatrix_Instance = UNITY_ACCESS_INSTANCED_PROP(DmpURPEffectsCrossSectionMixCrossSection,_BoxRotationMatrix);
				float4x4 invertVal91 = Inverse4x4( _BoxRotationMatrix_Instance );
				float3 break98 = ( mul( invertVal91, float4( ( WorldPosition - _Cubepos ) , 0.0 ) ).xyz + _Cubepos );
				float3 break100 = ( _Cubepos - _CubeExtent );
				float3 break99 = ( _Cubepos + _CubeExtent );
				float PosYZ115 = (( break98.x >= break100.x && break98.x <= break99.x ) ? 1.0 :  0.0 );
				float PosXZ115 = (( break98.y >= break100.y && break98.y <= break99.y ) ? 1.0 :  0.0 );
				float PosXY115 = (( break98.z >= break100.z && break98.z <= break99.z ) ? 1.0 :  0.0 );
				float localreturnPosYZ0PosXZ0PosXY0115 = returnPosYZ0PosXZ0PosXY0115( PosYZ115 , PosXZ115 , PosXY115 );
				clip( -( CrossMode198 == 1.0 ? localreturnPosYZ0PosXZ0PosXY0115 : 0.0 ) );
				float4 BoxCrossResult118 = switchResult133;
				float4 SingleBoxCross188 = BoxCrossResult118;
				float4 switchResult162 = (((ase_vface>0)?(AlbedoResult116):(CorssColor134)));
				float4 DefaultColor156 = switchResult162;
				float3 WorldPosition156 = WorldPosition;
				int ArrayLength156 = (int)_CrossCubsArrayLength;
				float4 localMultiBoxCrossProcess156 = MultiBoxCrossProcess( DefaultColor156 , WorldPosition156 , ArrayLength156 );
				float4 MultiBoxCrossResult159 = localMultiBoxCrossProcess156;
				float4 MultiBoxCross188 = MultiBoxCrossResult159;
				float4 localFinalCrossModeCheck188 = FinalCrossModeCheck188( CurrentPassMode188 , ThreeAAplane188 , SingleBoxCross188 , MultiBoxCross188 );
				
				float3 Albedo = localFinalCrossModeCheck188.xyz;
				float3 Normal = float3(0, 0, 1);
				float3 Emission = 0;
				float3 Specular = 0.5;
				float Metallic = 0;
				float Smoothness = 0.5;
				float Occlusion = 1;
				float Alpha = localFinalCrossModeCheck188.x;
				float AlphaClipThreshold = 0.5;
				float AlphaClipThresholdShadow = 0.5;
				float3 BakedGI = 0;
				float3 RefractionColor = 1;
				float RefractionIndex = 1;
				float3 Transmission = 1;
				float3 Translucency = 1;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				InputData inputData;
				inputData.positionWS = WorldPosition;
				inputData.viewDirectionWS = WorldViewDirection;
				inputData.shadowCoord = ShadowCoords;

				#ifdef _NORMALMAP
					#if _NORMAL_DROPOFF_TS
					inputData.normalWS = TransformTangentToWorld(Normal, half3x3( WorldTangent, WorldBiTangent, WorldNormal ));
					#elif _NORMAL_DROPOFF_OS
					inputData.normalWS = TransformObjectToWorldNormal(Normal);
					#elif _NORMAL_DROPOFF_WS
					inputData.normalWS = Normal;
					#endif
					inputData.normalWS = NormalizeNormalPerPixel(inputData.normalWS);
				#else
					inputData.normalWS = WorldNormal;
				#endif

				#ifdef ASE_FOG
					inputData.fogCoord = IN.fogFactorAndVertexLight.x;
				#endif

				inputData.vertexLighting = IN.fogFactorAndVertexLight.yzw;
				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					float3 SH = SampleSH(inputData.normalWS.xyz);
				#else
					float3 SH = IN.lightmapUVOrVertexSH.xyz;
				#endif

				inputData.bakedGI = SAMPLE_GI( IN.lightmapUVOrVertexSH.xy, SH, inputData.normalWS );
				#ifdef _ASE_BAKEDGI
					inputData.bakedGI = BakedGI;
				#endif

				BRDFData brdfData;
				InitializeBRDFData( Albedo, Metallic, Specular, Smoothness, Alpha, brdfData);
				half4 color;
				color.rgb = GlobalIllumination( brdfData, inputData.bakedGI, Occlusion, inputData.normalWS, inputData.viewDirectionWS);
				color.a = Alpha;

				#ifdef _TRANSMISSION_ASE
				{
					float shadow = _TransmissionShadow;
				
					Light mainLight = GetMainLight( inputData.shadowCoord );
					float3 mainAtten = mainLight.color * mainLight.distanceAttenuation;
					mainAtten = lerp( mainAtten, mainAtten * mainLight.shadowAttenuation, shadow );
					half3 mainTransmission = max(0 , -dot(inputData.normalWS, mainLight.direction)) * mainAtten * Transmission;
					color.rgb += Albedo * mainTransmission;
				
					#ifdef _ADDITIONAL_LIGHTS
						int transPixelLightCount = GetAdditionalLightsCount();
						for (int i = 0; i < transPixelLightCount; ++i)
						{
							Light light = GetAdditionalLight(i, inputData.positionWS);
							float3 atten = light.color * light.distanceAttenuation;
							atten = lerp( atten, atten * light.shadowAttenuation, shadow );
				
							half3 transmission = max(0 , -dot(inputData.normalWS, light.direction)) * atten * Transmission;
							color.rgb += Albedo * transmission;
						}
					#endif
				}
				#endif
				
				#ifdef _TRANSLUCENCY_ASE
				{
					float shadow = _TransShadow;
					float normal = _TransNormal;
					float scattering = _TransScattering;
					float direct = _TransDirect;
					float ambient = _TransAmbient;
					float strength = _TransStrength;
				
					Light mainLight = GetMainLight( inputData.shadowCoord );
					float3 mainAtten = mainLight.color * mainLight.distanceAttenuation;
					mainAtten = lerp( mainAtten, mainAtten * mainLight.shadowAttenuation, shadow );
				
					half3 mainLightDir = mainLight.direction + inputData.normalWS * normal;
					half mainVdotL = pow( saturate( dot( inputData.viewDirectionWS, -mainLightDir ) ), scattering );
					half3 mainTranslucency = mainAtten * ( mainVdotL * direct + inputData.bakedGI * ambient ) * Translucency;
					color.rgb += Albedo * mainTranslucency * strength;
				
					#ifdef _ADDITIONAL_LIGHTS
						int transPixelLightCount = GetAdditionalLightsCount();
						for (int i = 0; i < transPixelLightCount; ++i)
						{
							Light light = GetAdditionalLight(i, inputData.positionWS);
							float3 atten = light.color * light.distanceAttenuation;
							atten = lerp( atten, atten * light.shadowAttenuation, shadow );
				
							half3 lightDir = light.direction + inputData.normalWS * normal;
							half VdotL = pow( saturate( dot( inputData.viewDirectionWS, -lightDir ) ), scattering );
							half3 translucency = atten * ( VdotL * direct + inputData.bakedGI * ambient ) * Translucency;
							color.rgb += Albedo * translucency * strength;
						}
					#endif
				}
				#endif
				
				#ifdef _REFRACTION_ASE
					float4 projScreenPos = ScreenPos / ScreenPos.w;
					float3 refractionOffset = ( RefractionIndex - 1.0 ) * mul( UNITY_MATRIX_V, float4( WorldNormal, 0 ) ).xyz * ( 1.0 - dot( WorldNormal, WorldViewDirection ) );
					projScreenPos.xy += refractionOffset.xy;
					float3 refraction = SHADERGRAPH_SAMPLE_SCENE_COLOR( projScreenPos.xy ) * RefractionColor;
					color.rgb = lerp( refraction, color.rgb, color.a );
					color.a = 1;
				#endif
				
				#ifdef ASE_FINAL_COLOR_ALPHA_MULTIPLY
					color.rgb *= color.a;
				#endif
				
				#ifdef ASE_FOG
					#ifdef TERRAIN_SPLAT_ADDPASS
						color.rgb = MixFogColor(color.rgb, half3( 0, 0, 0 ), IN.fogFactorAndVertexLight.x );
					#else
						color.rgb = MixFog(color.rgb, IN.fogFactorAndVertexLight.x);
					#endif
				#endif
				
				#ifdef ASE_DEPTH_WRITE_ON
					outputDepth = DepthValue;
				#endif
				
				return BRDFDataToGbuffer(brdfData, inputData, Smoothness, Emission + color.rgb);
			}

			ENDHLSL
		}
		
	}
	
	CustomEditor "UnityEditor.ShaderGraph.PBRMasterGUI"
	Fallback "Hidden/InternalErrorShader"
	
}
/*ASEBEGIN
Version=18921
0;0;1536;795;3636.703;1335.325;1;True;True
Node;AmplifyShaderEditor.CommentaryNode;205;-2420.4,662.0404;Inherit;False;2311.892;852.7472;Comment;17;89;92;94;93;97;96;95;98;100;99;101;102;104;115;88;90;91;Single Box Cross;1,1,1,1;0;0
Node;AmplifyShaderEditor.Matrix4X4Node;90;-2163.143,712.0404;Float;False;InstancedProperty;_BoxRotationMatrix;BoxRotation Matrix;6;0;Create;True;0;0;0;False;0;False;1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;0;1;FLOAT4x4;0
Node;AmplifyShaderEditor.WorldPosInputsNode;88;-2370.4,884.6574;Float;False;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.CommentaryNode;87;-2411.918,-605.5342;Inherit;False;1598.515;1019.127;Comment;22;57;40;45;44;39;42;43;41;38;37;46;47;48;147;148;149;150;151;152;153;154;155;ThreeAAplaneCutting;1,1,1,1;0;0
Node;AmplifyShaderEditor.Vector3Node;89;-2366.183,1104.652;Float;False;Property;_Cubepos;Cubepos;7;0;Create;True;0;0;0;False;0;False;0,0,0;0,0,0;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.Vector4Node;148;-2056.552,-72.52789;Inherit;False;Property;_PosXZ;PosXZ;9;0;Create;True;0;0;0;False;0;False;0,0,0,0;0,0,0,0;0;5;FLOAT4;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.Vector4Node;151;-2047.307,217.4541;Inherit;False;Property;_PosXY;PosXY;10;0;Create;True;0;0;0;False;0;False;0,0,0,0;0,0,0,0;0;5;FLOAT4;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.SimpleSubtractOpNode;92;-1824.971,870.6091;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.Vector4Node;147;-2083.012,-369.5099;Inherit;False;Property;_PosYZ;PosYZ;11;0;Create;True;0;0;0;False;0;False;0,0,0,0;0,0,0,0;0;5;FLOAT4;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.InverseOpNode;91;-1803.738,782.976;Inherit;False;1;0;FLOAT4x4;0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;False;1;FLOAT4x4;0
Node;AmplifyShaderEditor.WorldPosInputsNode;40;-2361.918,-177.1337;Inherit;False;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.WireNode;45;-2077.983,119.3539;Inherit;False;1;0;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.ComponentMaskNode;150;-1866.356,-83.52786;Inherit;False;True;True;True;False;1;0;FLOAT4;0,0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.SimpleMultiplyOpNode;94;-1622.092,817.406;Inherit;False;2;2;0;FLOAT4x4;0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.ComponentMaskNode;152;-1859.307,255.188;Inherit;False;True;True;True;False;1;0;FLOAT4;0,0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.ComponentMaskNode;149;-1902.012,-368.5099;Inherit;False;True;True;True;False;1;0;FLOAT4;0,0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.WireNode;44;-2096.183,-379.8464;Inherit;False;1;0;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.Vector3Node;93;-1970.573,1261.901;Float;False;Property;_CubeExtent;CubeExtent;5;0;Create;True;0;0;0;False;0;False;0,0,0;0,0,0;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.SimpleSubtractOpNode;42;-1620.583,-145.1467;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.SimpleSubtractOpNode;43;-1601.783,176.9904;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.Vector3Node;39;-1621.418,17.20279;Inherit;False;Constant;_MatrixZ;Matrix Z;2;0;Create;True;0;0;0;False;0;False;0,0,1;0,0,0;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.SimpleAddOpNode;95;-1321.67,855.0283;Inherit;False;2;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.Vector3Node;38;-1630.869,-294.8921;Inherit;False;Constant;_MatrixY;Matrix Y;2;0;Create;True;0;0;0;False;0;False;0,1,0;0,0,0;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.SimpleSubtractOpNode;41;-1634.234,-420.7794;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.SimpleAddOpNode;97;-1319.176,1233.688;Inherit;False;2;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.Vector3Node;37;-1647.97,-575.4671;Inherit;False;Constant;_MatrixX;Matrix X;2;0;Create;True;0;0;0;False;0;False;1,0,0;0,0,0;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.SimpleSubtractOpNode;96;-1333.293,1055.417;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.SamplerNode;30;-2373.426,-1316.744;Inherit;True;Property;_MainTex;MainTex;3;0;Create;True;0;0;0;False;0;False;-1;None;None;True;0;False;white;Auto;False;Object;-1;Auto;Texture2D;8;0;SAMPLER2D;;False;1;FLOAT2;0,0;False;2;FLOAT;0;False;3;FLOAT2;0,0;False;4;FLOAT2;0,0;False;5;FLOAT;1;False;6;FLOAT;0;False;7;SAMPLERSTATE;;False;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.DotProductOpNode;48;-1418.483,119.7904;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT;0
Node;AmplifyShaderEditor.DotProductOpNode;46;-1470.094,-499.5704;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT;0
Node;AmplifyShaderEditor.DotProductOpNode;47;-1422.384,-205.2463;Inherit;False;2;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT;0
Node;AmplifyShaderEditor.ColorNode;32;-2286.789,-1106.506;Inherit;False;Property;_MainColor;MainColor;4;0;Create;True;0;0;0;False;0;False;1,1,1,0;1,1,1,0;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.BreakToComponentsNode;99;-1159.176,1217.688;Inherit;False;FLOAT3;1;0;FLOAT3;0,0,0;False;16;FLOAT;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4;FLOAT;5;FLOAT;6;FLOAT;7;FLOAT;8;FLOAT;9;FLOAT;10;FLOAT;11;FLOAT;12;FLOAT;13;FLOAT;14;FLOAT;15
Node;AmplifyShaderEditor.CommentaryNode;210;-766.5132,-525.1622;Inherit;False;1335.896;575.1622;Can be Improve By using function;13;208;3;7;6;0;5;4;2;204;81;85;86;135;;1,1,1,1;0;0
Node;AmplifyShaderEditor.BreakToComponentsNode;100;-1177.293,1049.724;Inherit;False;FLOAT3;1;0;FLOAT3;0,0,0;False;16;FLOAT;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4;FLOAT;5;FLOAT;6;FLOAT;7;FLOAT;8;FLOAT;9;FLOAT;10;FLOAT;11;FLOAT;12;FLOAT;13;FLOAT;14;FLOAT;15
Node;AmplifyShaderEditor.RangedFloatNode;189;-3135.292,-991.9209;Inherit;False;InstancedProperty;_CrossMode;CrossMode;1;2;[IntRange];[Enum];Create;True;0;3;ThreeAAplane;0;SingleBoxCross;1;MultiBoxCross;2;0;True;0;False;0;2;0;0;0;1;FLOAT;0
Node;AmplifyShaderEditor.BreakToComponentsNode;98;-1169.08,848.1652;Inherit;False;FLOAT3;1;0;FLOAT3;0,0,0;False;16;FLOAT;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4;FLOAT;5;FLOAT;6;FLOAT;7;FLOAT;8;FLOAT;9;FLOAT;10;FLOAT;11;FLOAT;12;FLOAT;13;FLOAT;14;FLOAT;15
Node;AmplifyShaderEditor.RegisterLocalVarNode;198;-2887.057,-998.3787;Inherit;False;CrossMode;-1;True;1;0;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleAddOpNode;153;-1279.874,284.1835;Inherit;False;2;2;0;FLOAT;0;False;1;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleAddOpNode;154;-1293.398,-205.6875;Inherit;False;2;2;0;FLOAT;0;False;1;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleAddOpNode;155;-1289.201,-437.5381;Inherit;False;2;2;0;FLOAT;0;False;1;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.ColorNode;79;-2281.321,-909.4615;Inherit;False;Property;_CrossColor;CrossColor;8;0;Create;True;0;0;0;True;0;False;0,0,0,0;0,0,0,0;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.CommentaryNode;206;-10.68596,984.3474;Inherit;False;435.5641;235.6;Block It When Not Use This Pass;2;201;200;;1,1,1,1;0;0
Node;AmplifyShaderEditor.TFHCCompareWithRange;104;-774.3297,898.3817;Inherit;False;5;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;1;False;4;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.CommentaryNode;208;-716.5132,-393.3003;Inherit;False;489.7625;298.2948;Block It When Not Use This Pass;2;190;199;;1,1,1,1;0;0
Node;AmplifyShaderEditor.TFHCCompareWithRange;101;-772.1207,1092.353;Inherit;False;5;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;1;False;4;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleMultiplyOpNode;33;-1972.904,-1174.078;Inherit;False;2;2;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.TFHCCompareWithRange;102;-770.9916,1310.788;Inherit;False;5;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;1;False;4;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.CustomExpressionNode;57;-1148.833,-228.1944;Inherit;False;return PosYZ > 0 && PosXZ > 0 && PosXY > 0@;1;Create;3;True;PosYZ;FLOAT;0;In;;Inherit;False;True;PosXZ;FLOAT;0;In;;Inherit;False;True;PosXY;FLOAT;0;In;;Inherit;False;Check Cross Result;False;False;0;;False;3;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.CustomExpressionNode;115;-461.5077,1086.703;Inherit;False;return PosYZ > 0 && PosXZ > 0 && PosXY > 0@;1;Create;3;True;PosYZ;FLOAT;0;In;;Inherit;False;True;PosXZ;FLOAT;0;In;;Inherit;False;True;PosXY;FLOAT;0;In;;Inherit;False;return PosYZ > 0 && PosXZ > 0 && PosXY > 0;True;False;0;;False;3;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.RegisterLocalVarNode;116;-1724.912,-1176.073;Inherit;False;AlbedoResult;-1;True;1;0;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.RegisterLocalVarNode;134;-2047.557,-911.1578;Inherit;False;CorssColor;-1;True;1;0;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;201;39.31403,1040.973;Inherit;False;198;CrossMode;1;0;OBJECT;;False;1;FLOAT;0
Node;AmplifyShaderEditor.GetLocalVarNode;199;-666.5132,-343.3003;Inherit;False;198;CrossMode;1;0;OBJECT;;False;1;FLOAT;0
Node;AmplifyShaderEditor.CommentaryNode;207;-2418.916,1766.134;Inherit;False;875.8464;886.1926;Comment;9;158;172;171;157;202;160;162;163;156;Multi Box Cross;1,1,1,1;0;0
Node;AmplifyShaderEditor.Compare;190;-406.3507,-280.6054;Inherit;False;0;4;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.GetLocalVarNode;163;-2368.916,1816.134;Inherit;False;116;AlbedoResult;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;117;544.2955,828.4518;Inherit;False;116;AlbedoResult;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;137;559.1255,920.9158;Inherit;False;134;CorssColor;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;204;-151.5528,-475.1622;Inherit;False;116;AlbedoResult;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;160;-2364.756,1893.543;Inherit;False;134;CorssColor;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;135;-154.9835,-383.8628;Inherit;False;134;CorssColor;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.Compare;200;245.278,1034.347;Inherit;False;0;4;0;FLOAT;0;False;1;FLOAT;1;False;2;FLOAT;0;False;3;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.SwitchByFaceNode;133;750.2769,889.4717;Inherit;False;2;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.WorldPosInputsNode;202;-2368.161,2001.527;Float;False;0;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.NegateNode;86;185.5263,-268.5753;Inherit;False;1;0;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.RangedFloatNode;171;-2368.161,2161.527;Inherit;False;Property;_CrossCubsArrayLength;CrossCubsArrayLength;12;0;Create;True;0;0;0;True;0;False;0;0;0;0;0;1;FLOAT;0
Node;AmplifyShaderEditor.NegateNode;113;823.2955,1044.193;Inherit;False;1;0;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.SwitchByFaceNode;81;123.3688,-460.018;Inherit;False;2;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.SwitchByFaceNode;162;-2139.325,1827.863;Inherit;False;2;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.ClipNode;85;354.583,-291.8079;Inherit;False;3;0;COLOR;0,0,0,0;False;1;FLOAT;0;False;2;FLOAT;0;False;1;COLOR;0
Node;AmplifyShaderEditor.CustomExpressionNode;156;-1801.869,1949.593;Inherit;False;$bool crossArea[24]@$for(int i=0@i<ArrayLength@i++)${$crossArea[i] = 1@$float3 BoxPos = mul(WorldPosition - CrossCubePosArray[i].xyz, CrossCubeRotationMatrixArray[i]) + CrossCubePosArray[i].xyz@$$float3 BoxExtentMin = CrossCubePosArray[i] - CrossCubeExtentArray[i]@$$float3 BoxExtentMax = CrossCubePosArray[i] + CrossCubeExtentArray[i]@$$int xCheck = BoxPos.x >= BoxExtentMin.x && BoxPos.x <= BoxExtentMax.x ? 1 : 0@$$int yCheck = BoxPos.y >= BoxExtentMin.y && BoxPos.y <= BoxExtentMax.y ? 1 : 0@$$int zCheck = BoxPos.z >= BoxExtentMin.z && BoxPos.z <= BoxExtentMax.z ? 1 : 0@$$bool isCross = xCheck > 0 && yCheck  > 0 && zCheck > 0@$$if(isCross && _ShowCrossArea)${$return _CrossColor@$}$$if(_CrossMode == 2)${$clip( -(isCross))@$}$}$$return DefaultColor@;4;Create;3;True;DefaultColor;FLOAT4;0,0,0,0;In;;Inherit;False;True;WorldPosition;FLOAT3;0,0,0;In;;Inherit;False;True;ArrayLength;INT;0;In;;Inherit;False;MultiBoxCrossProcess;False;False;0;;False;3;0;FLOAT4;0,0,0,0;False;1;FLOAT3;0,0,0;False;2;INT;0;False;1;FLOAT4;0
Node;AmplifyShaderEditor.ClipNode;114;1039.408,1016.484;Inherit;False;3;0;COLOR;0,0,0,0;False;1;FLOAT;0;False;2;FLOAT;0;False;1;COLOR;0
Node;AmplifyShaderEditor.RegisterLocalVarNode;111;733.6549,-274.7177;Inherit;False;ThreeAAplaneResult;-1;True;1;0;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.RegisterLocalVarNode;118;1250.807,1014.479;Inherit;False;BoxCrossResult;-1;True;1;0;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.RegisterLocalVarNode;159;-1392.163,1953.527;Inherit;False;MultiBoxCrossResult;-1;True;1;0;FLOAT4;0,0,0,0;False;1;FLOAT4;0
Node;AmplifyShaderEditor.GetLocalVarNode;119;1580.576,235.4994;Inherit;False;118;BoxCrossResult;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;203;1574.44,44.12283;Inherit;False;198;CrossMode;1;0;OBJECT;;False;1;FLOAT;0
Node;AmplifyShaderEditor.GetLocalVarNode;112;1582.358,122.0577;Inherit;False;111;ThreeAAplaneResult;1;0;OBJECT;;False;1;COLOR;0
Node;AmplifyShaderEditor.GetLocalVarNode;166;1583.324,337.8274;Inherit;False;159;MultiBoxCrossResult;1;0;OBJECT;;False;1;FLOAT4;0
Node;AmplifyShaderEditor.GlobalArrayNode;157;-2368.161,2273.527;Inherit;False;CrossCubePosArray;0;24;2;False;False;0;1;True;Object;-1;4;0;INT;0;False;2;INT;0;False;1;INT;0;False;3;INT;0;False;1;FLOAT4;0
Node;AmplifyShaderEditor.GlobalArrayNode;172;-2352.161,2513.527;Inherit;False;CrossCubeRotationMatrixArray;0;24;3;False;False;0;1;True;Object;-1;4;0;INT;0;False;2;INT;0;False;1;INT;0;False;3;INT;0;False;1;FLOAT4x4;0
Node;AmplifyShaderEditor.GlobalArrayNode;158;-2368.161,2385.527;Inherit;False;CrossCubeExtentArray;0;24;2;False;False;0;1;True;Object;-1;4;0;INT;0;False;2;INT;0;False;1;INT;0;False;3;INT;0;False;1;FLOAT4;0
Node;AmplifyShaderEditor.CustomExpressionNode;188;1888.745,66.4803;Inherit;False; if(CurrentPassMode == 0)${$return ThreeAAplane@$}$ if(CurrentPassMode == 1)${$return SingleBoxCross@$}$ if(CurrentPassMode == 2)${$return MultiBoxCross@$}$$return float4(1,1,1,1)@;4;Create;4;True;CurrentPassMode;INT;0;In;;Inherit;False;True;ThreeAAplane;FLOAT4;0,0,0,0;In;;Inherit;False;True;SingleBoxCross;FLOAT4;0,0,0,0;In;;Inherit;False;True;MultiBoxCross;FLOAT4;0,0,0,0;In;;Inherit;False;FinalCrossModeCheck;True;False;0;;False;4;0;INT;0;False;1;FLOAT4;0,0,0,0;False;2;FLOAT4;0,0,0,0;False;3;FLOAT4;0,0,0,0;False;1;FLOAT4;0
Node;AmplifyShaderEditor.RangedFloatNode;173;-3136.677,-1086.433;Inherit;False;Property;_ShowCrossArea;ShowCrossArea;2;1;[ToggleUI];Create;True;0;0;0;True;0;False;0;0;0;0;0;1;FLOAT;0
Node;AmplifyShaderEditor.RangedFloatNode;211;-3131.703,-906.325;Inherit;False;Property;_CullMode;CullMode;0;1;[Enum];Create;True;0;0;1;UnityEngine.Rendering.CullMode;True;0;False;0;0;0;0;0;1;FLOAT;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;5;0,0;Float;False;False;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;New Amplify Shader;94348b07e5e8bab40bd6c8a1e3df54cd;True;Universal2D;0;5;Universal2D;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;0;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;True;1;1;False;-1;0;False;-1;1;1;False;-1;0;False;-1;False;False;False;False;False;False;False;False;False;False;False;False;False;False;True;True;True;True;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;1;False;-1;True;3;False;-1;True;True;0;False;-1;0;False;-1;True;1;LightMode=Universal2D;False;False;0;Hidden/InternalErrorShader;0;0;Standard;0;False;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;4;0,0;Float;False;False;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;New Amplify Shader;94348b07e5e8bab40bd6c8a1e3df54cd;True;Meta;0;4;Meta;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;0;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;False;False;False;False;False;False;False;False;False;False;False;False;False;True;2;False;-1;False;False;False;False;False;False;False;False;False;False;False;False;False;False;True;1;LightMode=Meta;False;False;0;Hidden/InternalErrorShader;0;0;Standard;0;False;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;2;0,0;Float;False;False;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;New Amplify Shader;94348b07e5e8bab40bd6c8a1e3df54cd;True;ShadowCaster;0;2;ShadowCaster;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;0;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;False;False;True;False;False;False;False;0;False;-1;False;False;False;False;False;False;False;False;False;True;1;False;-1;True;3;False;-1;False;True;1;LightMode=ShadowCaster;False;False;0;Hidden/InternalErrorShader;0;0;Standard;0;False;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;6;0,0;Float;False;False;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;New Amplify Shader;94348b07e5e8bab40bd6c8a1e3df54cd;True;DepthNormals;0;6;DepthNormals;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;0;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;True;1;1;False;-1;0;False;-1;0;1;False;-1;0;False;-1;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;False;True;1;False;-1;True;3;False;-1;False;True;1;LightMode=DepthNormals;False;False;0;Hidden/InternalErrorShader;0;0;Standard;0;False;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;1;2268.442,54.39568;Half;False;True;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;Dmp/URP/Effects/CrossSection/MixCrossSection;94348b07e5e8bab40bd6c8a1e3df54cd;True;Forward;0;1;Forward;18;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;True;True;0;True;211;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;2;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;True;1;1;False;-1;0;False;-1;1;1;False;-1;0;False;-1;False;False;False;False;False;False;False;False;False;False;False;False;False;False;True;True;True;True;True;0;False;-1;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;True;1;False;-1;True;3;False;-1;True;True;0;False;-1;0;False;-1;True;1;LightMode=UniversalForward;False;False;0;Hidden/InternalErrorShader;0;0;Standard;38;Workflow;1;Surface;0;  Refraction Model;0;  Blend;0;Two Sided;1;Fragment Normal Space,InvertActionOnDeselection;0;Transmission;0;  Transmission Shadow;0.5,False,-1;Translucency;0;  Translucency Strength;1,False,-1;  Normal Distortion;0.5,False,-1;  Scattering;2,False,-1;  Direct;0.9,False,-1;  Ambient;0.1,False,-1;  Shadow;0.5,False,-1;Cast Shadows;1;  Use Shadow Threshold;0;Receive Shadows;1;GPU Instancing;1;LOD CrossFade;1;Built-in Fog;1;_FinalColorxAlpha;0;Meta Pass;1;Override Baked GI;0;Extra Pre Pass;0;DOTS Instancing;0;Tessellation;0;  Phong;0;  Strength;0.5,False,-1;  Type;0;  Tess;16,True,46;  Min;10,True,43;  Max;25,True,44;  Edge Length;16,False,-1;  Max Displacement;25,False,-1;Write Depth;0;  Early Z;0;Vertex Position,InvertActionOnDeselection;1;0;8;False;True;True;True;True;True;True;True;False;;False;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;3;0,0;Float;False;False;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;New Amplify Shader;94348b07e5e8bab40bd6c8a1e3df54cd;True;DepthOnly;0;3;DepthOnly;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;0;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;False;False;True;False;False;False;False;0;False;-1;False;False;False;False;False;False;False;False;False;True;1;False;-1;False;False;True;1;LightMode=DepthOnly;False;False;0;Hidden/InternalErrorShader;0;0;Standard;0;False;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;7;0,0;Float;False;False;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;New Amplify Shader;94348b07e5e8bab40bd6c8a1e3df54cd;True;GBuffer;0;7;GBuffer;0;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;0;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;True;1;1;False;-1;0;False;-1;1;1;False;-1;0;False;-1;False;False;False;False;False;False;False;False;False;False;False;False;False;False;True;True;True;True;True;0;False;-1;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;True;1;False;-1;True;3;False;-1;True;True;0;False;-1;0;False;-1;True;1;LightMode=UniversalGBuffer;False;False;0;Hidden/InternalErrorShader;0;0;Standard;0;False;0
Node;AmplifyShaderEditor.TemplateMultiPassMasterNode;0;0,0;Float;False;False;-1;2;UnityEditor.ShaderGraph.PBRMasterGUI;0;2;New Amplify Shader;94348b07e5e8bab40bd6c8a1e3df54cd;True;ExtraPrePass;0;0;ExtraPrePass;5;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;0;False;-1;False;False;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;False;False;False;True;3;RenderPipeline=UniversalPipeline;RenderType=Opaque=RenderType;Queue=Geometry=Queue=0;True;0;True;17;d3d9;d3d11;glcore;gles;gles3;metal;vulkan;xbox360;xboxone;xboxseries;ps4;playstation;psp2;n3ds;wiiu;switch;nomrt;0;False;True;1;1;False;-1;0;False;-1;0;1;False;-1;0;False;-1;False;False;False;False;False;False;False;False;False;False;False;False;True;0;False;-1;False;True;True;True;True;True;0;False;-1;False;False;False;False;False;False;False;True;False;255;False;-1;255;False;-1;255;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;7;False;-1;1;False;-1;1;False;-1;1;False;-1;False;True;1;False;-1;True;3;False;-1;True;True;0;False;-1;0;False;-1;True;0;False;False;0;Hidden/InternalErrorShader;0;0;Standard;0;False;0
WireConnection;92;0;88;0
WireConnection;92;1;89;0
WireConnection;91;0;90;0
WireConnection;45;0;40;0
WireConnection;150;0;148;0
WireConnection;94;0;91;0
WireConnection;94;1;92;0
WireConnection;152;0;151;0
WireConnection;149;0;147;0
WireConnection;44;0;40;0
WireConnection;42;0;40;0
WireConnection;42;1;150;0
WireConnection;43;0;45;0
WireConnection;43;1;152;0
WireConnection;95;0;94;0
WireConnection;95;1;89;0
WireConnection;41;0;44;0
WireConnection;41;1;149;0
WireConnection;97;0;89;0
WireConnection;97;1;93;0
WireConnection;96;0;89;0
WireConnection;96;1;93;0
WireConnection;48;0;39;0
WireConnection;48;1;43;0
WireConnection;46;0;37;0
WireConnection;46;1;41;0
WireConnection;47;0;38;0
WireConnection;47;1;42;0
WireConnection;99;0;97;0
WireConnection;100;0;96;0
WireConnection;98;0;95;0
WireConnection;198;0;189;0
WireConnection;153;0;48;0
WireConnection;153;1;151;4
WireConnection;154;0;47;0
WireConnection;154;1;148;4
WireConnection;155;0;46;0
WireConnection;155;1;147;4
WireConnection;104;0;98;0
WireConnection;104;1;100;0
WireConnection;104;2;99;0
WireConnection;101;0;98;1
WireConnection;101;1;100;1
WireConnection;101;2;99;1
WireConnection;33;0;30;0
WireConnection;33;1;32;0
WireConnection;102;0;98;2
WireConnection;102;1;100;2
WireConnection;102;2;99;2
WireConnection;57;0;155;0
WireConnection;57;1;154;0
WireConnection;57;2;153;0
WireConnection;115;0;104;0
WireConnection;115;1;101;0
WireConnection;115;2;102;0
WireConnection;116;0;33;0
WireConnection;134;0;79;0
WireConnection;190;0;199;0
WireConnection;190;2;57;0
WireConnection;200;0;201;0
WireConnection;200;2;115;0
WireConnection;133;0;117;0
WireConnection;133;1;137;0
WireConnection;86;0;190;0
WireConnection;113;0;200;0
WireConnection;81;0;204;0
WireConnection;81;1;135;0
WireConnection;162;0;163;0
WireConnection;162;1;160;0
WireConnection;85;0;81;0
WireConnection;85;1;86;0
WireConnection;156;0;162;0
WireConnection;156;1;202;0
WireConnection;156;2;171;0
WireConnection;114;0;133;0
WireConnection;114;1;113;0
WireConnection;111;0;85;0
WireConnection;118;0;114;0
WireConnection;159;0;156;0
WireConnection;188;0;203;0
WireConnection;188;1;112;0
WireConnection;188;2;119;0
WireConnection;188;3;166;0
WireConnection;1;0;188;0
WireConnection;1;6;188;0
ASEEND*/
//CHKSM=AC5073D5B14D4EA8A4FFC4F30E203715894005E4