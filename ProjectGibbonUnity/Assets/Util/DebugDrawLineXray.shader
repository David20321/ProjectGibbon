// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Receiver2/DebugDrawLineXray" {
	Properties{
		_TintColor("Tint Color", Color) = (0.5,0.5,0.5,0.5)
	}

	Category{
		Tags{ "Queue" = "Transparent +100" "IgnoreProjector" = "True" "RenderType" = "Transparent" }
		Blend SrcAlpha OneMinusSrcAlpha
		AlphaTest Greater .01
		ColorMask RGB
		Cull Off Lighting Off ZWrite Off Fog{ Color(0,0,0,0) }
		ZTest Always
		BindChannels{
			Bind "Color", color
			Bind "Vertex", vertex
		}

			// ---- Fragment program cards
		SubShader{
			Pass{

				CGPROGRAM
				#pragma vertex vert
				#pragma fragment frag

				#include "UnityCG.cginc"

				float4 _TintColor;

				struct appdata_t {
					float4 vertex : POSITION;
					float4 color : COLOR;
				};

				struct v2f {
					float4 vertex : POSITION;
					float4 color : COLOR;
				};

				v2f vert(appdata_t v)
				{
					v2f o;
					o.vertex = UnityObjectToClipPos(v.vertex);
					o.color = v.color;
					return o;
				}

				half4 frag(v2f i) : COLOR
				{
					return i.color * _TintColor;
				}
				ENDCG
			}
		}
	}
}
