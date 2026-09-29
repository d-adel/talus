#version 450
#extension GL_GOOGLE_include_directive : require

#include "frame-globals.glsl"
#include "sdf-records.glsl"
#include "push-constants.glsl"

layout(location = 0) out vec4 clipPosition;
layout(location = 1) flat out uint recordIndex;

const uint CUBE_INDICES[36] = uint[](
  0,
  2,
  1,
  1,
  2,
  3,

  4,
  5,
  6,
  5,
  7,
  6,

  0,
  4,
  2,
  4,
  6,
  2,

  1,
  3,
  5,
  5,
  3,
  7,

  0,
  1,
  4,
  1,
  5,
  4,

  2,
  6,
  3,
  3,
  6,
  7);

void main() {
  recordIndex = push.recordIndex;
  SdfRecordShared record = sdfRecordTable.records[push.recordIndex];

  vec3 boxMin = record.localOrigin.xyz;

  vec3 span =
    record.voxelSize.xyz * vec3(record.resolutionVolumeIndex.xyz - 1u);

  vec3 boxMax = boxMin + span;

  vec3 center = 0.5 * (boxMin + boxMax);
  vec3 halfExtent = 0.5 * (boxMax - boxMin);

  boxMin = center - halfExtent;
  boxMax = center + halfExtent;

  uint corner = CUBE_INDICES[gl_VertexIndex];

  vec3 localPosition = vec3((corner & 1u) != 0u ? boxMax.x : boxMin.x,
                            (corner & 2u) != 0u ? boxMax.y : boxMin.y,
                            (corner & 4u) != 0u ? boxMax.z : boxMin.z);

  clipPosition = camera.viewProj * record.model * vec4(localPosition, 1.0);

  gl_Position = clipPosition;
}
