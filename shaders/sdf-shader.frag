#version 450
#extension GL_GOOGLE_include_directive : require

#include "sdf-shader-body.glsl"

layout(location = 0) out vec4 outColor;

void main() {
  SdfRecordShared record = sdfRecordTable.records[recordIndex];

  vec2 ndc = clipPosition.xy / clipPosition.w;

  vec4 worldPosition = camera.inverseViewProj * vec4(ndc, 1.0, 1.0);

  worldPosition /= worldPosition.w;

  vec3 worldDirection = normalize(worldPosition.xyz - camera.position.xyz);

  vec3 localDirection =
    (record.inverseModel * vec4(worldDirection, 0.0)).xyz;

  vec3 localCameraPos =
    (record.inverseModel * vec4(camera.position.xyz, 1.0)).xyz;

  vec3 localHitPosition;
  float materialId;

  if (!march(record,
             localCameraPos,
             localDirection,
             localHitPosition,
             materialId)) {
    discard;
  }

  vec3 surfaceAlbedo = albedo(materialId);

  vec4 worldHitPos = record.model * vec4(localHitPosition, 1.0);

  vec4 clip = camera.viewProj * worldHitPos;

  float depth = clip.z / clip.w;

  if (depth <= 0.0 || isinf(depth) || isnan(depth)) {
    discard;
  }

  gl_FragDepth = depth;

  vec3 localNormal = estimateNormal(record, localHitPosition);

  if (dot(localNormal, localDirection) > 0.0) {
    localNormal = -localNormal;
  }

  vec3 worldNormal =
    normalize((record.model * vec4(localNormal, 0.0)).xyz);

  vec3 worldLightDir = light.direction.xyz;

  float diffuse = max(0.0, dot(worldNormal, worldLightDir));

  float departureBias = surfaceBias(record);

  vec3 shadowOrigin = worldHitPos.xyz + worldNormal * departureBias;

  float shadowed =
    diffuse > 0.0 ? lightReaching(shadowOrigin, worldLightDir) : 1.0;

  float ambientOcclusion = occlusion(record, localHitPosition, localNormal);

  vec3 ambient = hemisphere(worldNormal) * ambientOcclusion;

  vec3 color = surfaceAlbedo * (ambient + diffuse * shadowed);

  outColor = vec4(tonemap(color), 1.0);
}
