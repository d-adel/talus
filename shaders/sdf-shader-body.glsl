const int MAX_SDF_VOLUMES = TALUS_MAX_SDF_VOLUMES;
const uint BRICK_CELLS = TALUS_BRICK_CELLS;
const float bias = 0.01;

#include "frame-globals.glsl"
#include "scene-resources.glsl"
#include "sdf-records.glsl"

layout(std430, set = 2, binding = 3) readonly buffer FieldScene {
  Primitive primitives[];
} scene;

layout(set = 2, binding = 1)
  uniform texture3D distanceVolume[MAX_SDF_VOLUMES];

layout(set = 2, binding = 2)
  uniform utexture3D materialVolume[MAX_SDF_VOLUMES];

layout(std430, set = 2, binding = 6) readonly buffer BrickBounds {
  float values[];
} brickBounds;

struct RecordBoundsShared {
  vec4 minimum;
  vec4 maximum;
};

layout(std430, set = 2, binding = 7) readonly buffer RecordBoundsTable {
  RecordBoundsShared bounds[];
} recordBoundsTable;

layout(std430, set = 2, binding = 8) readonly buffer ShadowCells {
  uvec2 cells[];
} shadowCells;

layout(std430, set = 2, binding = 9) readonly buffer ShadowIndices {
  uint values[];
} shadowIndices;

layout(std140, set = 2, binding = 10) uniform ShadowGrid {
  vec4 axisU;
  vec4 axisV;
  vec4 cell;
} shadowGrid;

layout(location = 0) in vec4 clipPosition;
layout(location = 1) flat in uint recordIndex;


uint fieldPrimitiveOffset = 0u;
uint fieldPrimitiveCount = 0u;

#define FIELD_PRIMITIVE_AT(i) scene.primitives[fieldPrimitiveOffset + (i)]

#define FIELD_PRIMITIVE_COUNT fieldPrimitiveCount

#include "sdf-common.glsl"

float outsideGrid(SdfRecordShared record, vec3 p) {
  vec3 maxCorner = record.localOrigin.xyz
                   + record.voxelSize.xyz
                       * vec3(record.resolutionVolumeIndex.xyz - 1u);

  return length(p - clamp(p, record.localOrigin.xyz, maxCorner));
}

vec2 gridSample(SdfRecordShared record, vec3 p) {
  uint volumeIndex = record.resolutionVolumeIndex.w;

  vec3 lattice = (p - record.localOrigin.xyz) / record.voxelSize.xyz;

  vec3 uvw = (lattice + 0.5) / vec3(record.resolutionVolumeIndex.xyz);

  float distance = texture(sampler3D(distanceVolume[volumeIndex],
                                     samplers[TALUS_SAMPLER_LINEAR_CLAMP]),
                           uvw)
                     .r;

  uint material = texture(usampler3D(materialVolume[volumeIndex],
                                     samplers[TALUS_SAMPLER_NEAREST_CLAMP]),
                          uvw)
                    .r;

  return vec2(distance, float(material));
}

float brickBound(SdfRecordShared record, vec3 p) {
  uvec3 cells = record.resolutionVolumeIndex.xyz - 1u;
  uvec3 brickResolution = (cells + BRICK_CELLS - 1u) / BRICK_CELLS;

  vec3 lattice = (p - record.localOrigin.xyz) / record.voxelSize.xyz;

  uvec3 cell = min(uvec3(max(floor(lattice), vec3(0.0))), cells - 1u);
  uvec3 brick = min(cell / BRICK_CELLS, brickResolution - 1u);

  uint index =
    brick.x + brickResolution.x * (brick.y + brickResolution.y * brick.z);

  return brickBounds.values[uint(record.localOrigin.w) + 1u + index];
}

float brickExit(SdfRecordShared record, vec3 p, vec3 direction) {
  uvec3 cells = record.resolutionVolumeIndex.xyz - 1u;

  vec3 lattice = (p - record.localOrigin.xyz) / record.voxelSize.xyz;

  uvec3 cell = min(uvec3(max(floor(lattice), vec3(0.0))), cells - 1u);
  uvec3 base = (cell / BRICK_CELLS) * BRICK_CELLS;

  vec3 minimum =
    record.localOrigin.xyz + record.voxelSize.xyz * vec3(base);
  vec3 maximum = record.localOrigin.xyz
                 + record.voxelSize.xyz
                     * vec3(min(base + BRICK_CELLS, cells));

  float exit = params.maxDistance;

  for (int axis = 0; axis < 3; ++axis) {
    if (abs(direction[axis]) < 1e-8) {
      continue;
    }

    float wall = direction[axis] > 0.0 ? maximum[axis] : minimum[axis];

    exit = min(exit, (wall - p[axis]) / direction[axis]);
  }

  vec3 voxel = record.voxelSize.xyz;

  return max(exit, 0.5 * min(voxel.x, min(voxel.y, voxel.z)));
}

vec2 fieldDistance(SdfRecordShared record, vec3 p) {
  if (record.config.y == 0u) {
    fieldPrimitiveOffset = record.config.w;
    fieldPrimitiveCount = record.config.x;
    return sceneDistance(p);
  }

  float outside = outsideGrid(record, p);

  if (outside > 0.0) {
    return vec2(outside + record.voxelSize.w, 0.0);
  }

  return gridSample(record, p);
}

float fieldStep(SdfRecordShared record,
                vec3 p,
                vec3 direction,
                float value) {
  if (record.config.y == 0u) {
    return abs(value);
  }

  if (outsideGrid(record, p) > 0.0) {
    return abs(value);
  }

  float bound = brickBound(record, p);
  float exit = brickExit(record, p, direction);

  if (bound <= 0.0) {
    return exit;
  }

  return min(abs(value) / bound, exit);
}

float fieldShadowStep(SdfRecordShared record, vec3 p, float value) {
  if (record.config.y == 0u) {
    return abs(value);
  }

  if (outsideGrid(record, p) > 0.0) {
    return abs(value);
  }

  float bound = brickBounds.values[uint(record.localOrigin.w)];

  if (bound <= 0.0) {
    return abs(value);
  }

  return abs(value) / bound;
}

float gradientOffset(SdfRecordShared record) {
  if (record.config.y == 0u) {
    return params.gradientEpsilon;
  }

  vec3 voxel = record.voxelSize.xyz;

  return max(params.gradientEpsilon, max(voxel.x, max(voxel.y, voxel.z)));
}

float surfaceBias(SdfRecordShared record) {
  if (record.config.y == 0u) {
    return bias;
  }

  vec3 voxel = record.voxelSize.xyz;

  return max(bias, 2.0 * max(voxel.x, max(voxel.y, voxel.z)));
}

vec3 estimateNormal(SdfRecordShared record, vec3 p) {
  float h = gradientOffset(record);

  vec3 offsetX = vec3(h, 0.0, 0.0);
  vec3 offsetY = vec3(0.0, h, 0.0);
  vec3 offsetZ = vec3(0.0, 0.0, h);

  return normalize(vec3(fieldDistance(record, p + offsetX).x
                          - fieldDistance(record, p - offsetX).x,

                        fieldDistance(record, p + offsetY).x
                          - fieldDistance(record, p - offsetY).x,

                        fieldDistance(record, p + offsetZ).x
                          - fieldDistance(record, p - offsetZ).x));
}

bool march(SdfRecordShared record,
           vec3 origin,
           vec3 direction,
           out vec3 hitPosition,
           out float materialId) {
  float distanceTravelled = 0.0;

  float omega = params.omega;
  float previousRadius = 0.0;
  float stepLength = 0.0;
  float functionSign = 1.0;

  for (int i = 0; i < int(params.maxIterations); ++i) {
    vec3 p = origin + direction * distanceTravelled;

    vec2 field = fieldDistance(record, p);

    if (i == 0) {
      functionSign = field.x < 0.0 ? -1.0 : 1.0;
    }

    float signedRadius = functionSign * field.x;

    float radius = fieldStep(record, p, direction, field.x);

    bool relaxationFailed =
      omega > 1.0 && (radius + previousRadius) < stepLength;

    if (relaxationFailed) {
      stepLength -= omega * stepLength;
      omega = 1.0;
    } else {
      stepLength = (signedRadius < 0.0 ? -radius : radius) * omega;
    }

    previousRadius = radius;

    if (!relaxationFailed && abs(signedRadius) <= params.epsilon) {
      hitPosition = p;
      materialId = field.y;
      return true;
    }

    distanceTravelled += stepLength;

    if (distanceTravelled > params.maxDistance) {
      break;
    }
  }

  return false;
}

float lightReachingLocal(SdfRecordShared record,
                         vec3 origin,
                         vec3 direction) {
  float result = 1.0;
  float distanceTravelled = 0.0;

  for (int i = 0; i < int(params.maxIterations); ++i) {
    vec3 p = origin + direction * distanceTravelled;

    bool trueDistance = true;
    float distanceToSurface;
    float step;

    if (record.config.y == 0u) {
      distanceToSurface = fieldDistance(record, p).x;
      step = distanceToSurface;
    } else {
      float outside = outsideGrid(record, p);

      if (outside > 0.0) {
        distanceToSurface = outside + record.voxelSize.w;
        step = distanceToSurface;

        trueDistance = false;
      } else {
        distanceToSurface = fieldDistance(record, p).x;
        step = fieldShadowStep(record, p, distanceToSurface);
      }
    }

    if (trueDistance) {
      if (distanceToSurface <= params.epsilon) {
        return 0.0;
      }

      if (distanceTravelled > 0.0) {
        result =
          min(result,
              params.shadowSharpness * distanceToSurface / distanceTravelled);
      }
    }

    distanceTravelled += step;

    if (distanceTravelled > params.shadowMaxDistance || result < 0.01) {
      break;
    }
  }

  return clamp(result, 0.0, 1.0);
}

bool shadowRayReaches(SdfRecordShared record,
                      vec3 localOrigin,
                      vec3 localDirection,
                      float maxDistance) {
  vec3 minCorner = record.localOrigin.xyz;
  vec3 maxCorner = minCorner
                   + record.voxelSize.xyz
                       * vec3(record.resolutionVolumeIndex.xyz - 1u);

  vec3 inverseDirection = 1.0 / localDirection;

  vec3 firstPlane = (minCorner - localOrigin) * inverseDirection;
  vec3 secondPlane = (maxCorner - localOrigin) * inverseDirection;

  vec3 nearPlane = min(firstPlane, secondPlane);
  vec3 farPlane = max(firstPlane, secondPlane);

  float entry = max(max(nearPlane.x, nearPlane.y), nearPlane.z);
  float exitAt = min(min(farPlane.x, farPlane.y), farPlane.z);

  return exitAt >= max(entry, 0.0) && entry <= maxDistance;
}

bool worldBoundsReach(uint recordIndex,
                      vec3 worldOrigin,
                      vec3 inverseDirection,
                      float maxDistance) {
  RecordBoundsShared box = recordBoundsTable.bounds[recordIndex];

  vec3 firstPlane = (box.minimum.xyz - worldOrigin) * inverseDirection;
  vec3 secondPlane = (box.maximum.xyz - worldOrigin) * inverseDirection;

  vec3 nearPlane = min(firstPlane, secondPlane);
  vec3 farPlane = max(firstPlane, secondPlane);

  float entry = max(max(nearPlane.x, nearPlane.y), nearPlane.z);
  float exitAt = min(min(farPlane.x, farPlane.y), farPlane.z);

  return exitAt >= max(entry, 0.0) && entry <= maxDistance;
}

float lightReaching(vec3 worldOrigin, vec3 worldDirection) {
  float result = 1.0;

  vec3 inverseWorldDirection = 1.0 / worldDirection;

  uint first = 0u;
  uint last = 0u;

  if (shadowGrid.cell.z > 0.0) {
    float acrossU = dot(worldOrigin, shadowGrid.axisU.xyz) - shadowGrid.axisU.w;
    float acrossV = dot(worldOrigin, shadowGrid.axisV.xyz) - shadowGrid.axisV.w;

    float columnf = floor(acrossU * shadowGrid.cell.x);
    float rowf = floor(acrossV * shadowGrid.cell.y);

    if (columnf < 0.0 || rowf < 0.0 || columnf >= shadowGrid.cell.z
        || rowf >= shadowGrid.cell.w) {
      return 1.0;
    }

    uvec2 cell =
      shadowCells.cells[uint(rowf) * uint(shadowGrid.cell.z) + uint(columnf)];

    first = cell.x;
    last = cell.x + cell.y;
  }

  for (uint slot = first; slot < last; ++slot) {
    uint i = shadowIndices.values[slot];

    if (!worldBoundsReach(i,
                          worldOrigin,
                          inverseWorldDirection,
                          params.shadowMaxDistance)) {
      continue;
    }

    SdfRecordShared record = sdfRecordTable.records[i];

    vec3 localOrigin = (record.inverseModel * vec4(worldOrigin, 1.0)).xyz;

    vec3 localDirection = (record.inverseModel * vec4(worldDirection, 0.0)).xyz;

    if (!shadowRayReaches(record,
                          localOrigin,
                          localDirection,
                          params.shadowMaxDistance)) {
      continue;
    }

    float recordLight = lightReachingLocal(record, localOrigin, localDirection);

    result = min(result, recordLight);

    if (result <= 0.0) {
      break;
    }
  }

  return result;
}

vec3 hemisphere(vec3 normal) {
  if (light.shading.x <= 0.0) {
    return vec3(light.direction.w);
  }

  float up = clamp(normal.y, -1.0, 1.0);

  vec3 sky = mix(light.skyTop.rgb,
                 light.skyHorizon.rgb,
                 clamp(pow(1.0 - up, light.skyTop.w), 0.0, 1.0));

  vec3 ground = mix(light.groundBottom.rgb,
                    light.skyHorizon.rgb,
                    clamp(pow(1.0 + up, light.skyHorizon.w), 0.0, 1.0));

  return mix(ground, sky, step(0.0, up)) * light.shading.x;
}

float occlusion(SdfRecordShared record, vec3 point, vec3 normal) {
  if (light.shading.y <= 0.0) {
    return 1.0;
  }

  float occluded = 0.0;
  float weight = 1.0;

  for (int i = 1; i <= 5; ++i) {
    float reach = 0.03 * float(i) * float(i);

    float reached = fieldDistance(record, point + normal * reach).x;

    occluded += (reach - reached) * weight;
    weight *= 0.72;
  }

  return clamp(1.0 - light.shading.y * occluded, 0.0, 1.0);
}

vec3 tonemap(vec3 colour) {
  if (light.shading.z <= 0.0) {
    return colour;
  }

  vec3 x = colour * 0.6;

  return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14),
               vec3(0.0),
               vec3(1.0));
}

vec3 albedo(float materialId) {
  return materialTable.materials[uint(materialId + 0.5)].baseColor.rgb;
}
