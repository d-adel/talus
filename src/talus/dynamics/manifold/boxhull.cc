#include "boxhull.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace talus::dynamics {

namespace {

constexpr double LINEAR_SLOP = 0.005;
constexpr int NULL_INDEX = -1;
constexpr int MAX_CLIP_POINTS = 32;
constexpr uint8_t OWNER_A = 0u;
constexpr uint8_t OWNER_B = 1u;

struct HalfEdge {
  uint8_t next;
  uint8_t twin;
  uint8_t origin;
  uint8_t face;
};

constexpr std::array<HalfEdge, 24> EDGES{{
  {2, 1, 2, 0},    {17, 0, 1, 5},   {4, 3, 1, 0},    {20, 2, 5, 3},
  {6, 5, 5, 0},    {23, 4, 6, 4},   {0, 7, 6, 0},    {18, 6, 2, 2},
  {10, 9, 0, 1},   {21, 8, 3, 5},   {12, 11, 3, 1},  {16, 10, 7, 2},
  {14, 13, 7, 1},  {19, 12, 4, 4},  {8, 15, 4, 1},   {22, 14, 0, 3},
  {7, 17, 3, 2},   {9, 16, 2, 5},   {11, 19, 6, 2},  {5, 18, 7, 4},
  {15, 21, 1, 3},  {1, 20, 0, 5},   {3, 23, 4, 3},   {13, 22, 5, 4},
}};

constexpr std::array<uint8_t, 8> VERTEX_EDGE{8, 1, 0, 9, 13, 3, 5, 11};
constexpr std::array<uint8_t, 6> FACE_EDGE{0, 8, 16, 20, 19, 21};

struct Plane {
  glm::dvec3 normal{0.0};
  double offset = 0.0;
};

struct Hull {
  std::array<glm::dvec3, 8> points;
  std::array<Plane, 6> planes;
};

struct FeaturePair {
  uint8_t owner1 = 0u;
  uint8_t index1 = 0u;
  uint8_t owner2 = 0u;
  uint8_t index2 = 0u;
};

struct ClipVertex {
  glm::dvec3 position{0.0};
  double separation = 0.0;
  FeaturePair pair;
};

struct LocalPoint {
  glm::dvec3 point{0.0};
  double separation = 0.0;
  FeaturePair pair;
};

struct LocalManifold {
  glm::dvec3 normal{0.0};
  std::array<LocalPoint, 4> points{};
  int count = 0;
};

struct Frame {
  glm::dmat3 rotation{1.0};
  glm::dvec3 position{0.0};
};

enum class AxisType : uint8_t { FaceA, FaceB, Edge };

struct SeparatingAxis {
  glm::dvec3 normal{0.0};
  double separation = -std::numeric_limits<double>::infinity();
  int indexA = NULL_INDEX;
  int indexB = NULL_INDEX;
};

double planeSeparation(const Plane& plane, const glm::dvec3& point) {
  return glm::dot(plane.normal, point) - plane.offset;
}

Plane planeFrom(const glm::dvec3& normal, const glm::dvec3& point) {
  return {normal, glm::dot(normal, point)};
}

Hull makeHull(const glm::dvec3& half) {
  const double least = 0.2 * LINEAR_SLOP;
  const glm::dvec3 h = glm::max(glm::dvec3(least), half);
  Hull hull;

  hull.planes[0] = planeFrom(glm::dvec3(-1.0, 0.0, 0.0), -h);
  hull.planes[1] = planeFrom(glm::dvec3(1.0, 0.0, 0.0), h);
  hull.planes[2] = planeFrom(glm::dvec3(0.0, -1.0, 0.0), -h);
  hull.planes[3] = planeFrom(glm::dvec3(0.0, 1.0, 0.0), h);
  hull.planes[4] = planeFrom(glm::dvec3(0.0, 0.0, -1.0), -h);
  hull.planes[5] = planeFrom(glm::dvec3(0.0, 0.0, 1.0), h);
  hull.points[0] = glm::dvec3(h.x, h.y, h.z);
  hull.points[1] = glm::dvec3(-h.x, h.y, h.z);
  hull.points[2] = glm::dvec3(-h.x, -h.y, h.z);
  hull.points[3] = glm::dvec3(h.x, -h.y, h.z);
  hull.points[4] = glm::dvec3(h.x, h.y, -h.z);
  hull.points[5] = glm::dvec3(-h.x, h.y, -h.z);
  hull.points[6] = glm::dvec3(-h.x, -h.y, -h.z);
  hull.points[7] = glm::dvec3(h.x, -h.y, -h.z);

  return hull;
}

glm::dvec3 apply(const Frame& frame, const glm::dvec3& point) {
  return frame.rotation * point + frame.position;
}

Frame invert(const Frame& frame) {
  const glm::dmat3 back = glm::transpose(frame.rotation);

  return {back, -(back * frame.position)};
}

FeaturePair makePair(uint8_t owner1, int index1, uint8_t owner2, int index2) {
  return {owner1, static_cast<uint8_t>(index1), owner2, static_cast<uint8_t>(index2)};
}

FeaturePair flipPair(FeaturePair pair) {
  std::swap(pair.owner1, pair.owner2);
  pair.owner1 = static_cast<uint8_t>(1u - pair.owner1);
  pair.owner2 = static_cast<uint8_t>(1u - pair.owner2);
  std::swap(pair.index1, pair.index2);

  return pair;
}

uint32_t packFeature(const FeaturePair& pair) {
  return (static_cast<uint32_t>(pair.owner1) << 11u)
         | (static_cast<uint32_t>(pair.index1 & 31u) << 6u)
         | (static_cast<uint32_t>(pair.owner2) << 5u)
         | static_cast<uint32_t>(pair.index2 & 31u);
}

glm::dvec3 arbitraryPerp(const glm::dvec3& v) {
  if (v.x < -0.5 || 0.5 < v.x) {
    const double a = 0.67;
    const double b = -0.42;

    return glm::normalize(glm::dvec3(a * v.y + b * v.z, -a * v.x, -b * v.x));
  }

  if (v.y < -0.5 || 0.5 < v.y) {
    const double a = 0.67;
    const double c = -0.42;

    return glm::normalize(glm::dvec3(a * v.y, -a * v.x + c * v.z, -c * v.y));
  }

  const double b = 0.67;
  const double c = -0.42;

  return glm::normalize(glm::dvec3(b * v.z, c * v.z, -b * v.x - c * v.y));
}

double support(
  const Hull& hull,
  const glm::dvec3& direction,
  int& vertex
) {
  double best = -std::numeric_limits<double>::infinity();

  for (int index = 0; index != 8; ++index) {
    const double value = glm::dot(direction, hull.points[index]);

    if (value > best) {
      best = value;
      vertex = index;
    }
  }

  return best;
}

struct AxisQuery {
  SeparatingAxis faceA;
  SeparatingAxis faceB;
  SeparatingAxis edge;
  bool separated = false;
};

double boxSupport(const glm::dvec3& corner, const glm::dvec3& direction) {
  return std::abs(direction.x) * corner.x + std::abs(direction.y) * corner.y + std::abs(direction.z) * corner.z;
}

using Triple = std::array<double, 3>;

Triple tripleOf(const glm::dvec3& vector) {
  return {vector.x, vector.y, vector.z};
}

struct EdgeAxes {
  int faceU = 0;
  int faceV = 0;
  int axisU = 0;
  int axisV = 0;
  int axisW = 0;
  double signU = 0.0;
  double signV = 0.0;
  double signW = 0.0;
};

constexpr std::array<EdgeAxes, 12> EDGE_AXES = [] {
  std::array<EdgeAxes, 12> table{};

  for (int edge = 0; edge < 24; edge += 2) {
    EdgeAxes& axes = table[static_cast<size_t>(edge / 2)];
    const int faceU = EDGES[static_cast<size_t>(edge)].face;
    const int faceV = EDGES[EDGES[static_cast<size_t>(edge)].twin].face;
    std::array<double, 3> u{};
    std::array<double, 3> v{};

    u[static_cast<size_t>(faceU / 2)] = faceU % 2 == 1 ? 1.0 : -1.0;
    v[static_cast<size_t>(faceV / 2)] = faceV % 2 == 1 ? 1.0 : -1.0;

    const std::array<double, 3> w{v[1] * u[2] - u[1] * v[2], v[2] * u[0] - u[2] * v[0], v[0] * u[1] - u[0] * v[1]};

    axes.faceU = faceU;
    axes.faceV = faceV;
    axes.axisU = faceU / 2;
    axes.axisV = faceV / 2;
    axes.axisW = 3 - faceU / 2 - faceV / 2;
    axes.signU = u[static_cast<size_t>(axes.axisU)];
    axes.signV = v[static_cast<size_t>(axes.axisV)];
    axes.signW = w[static_cast<size_t>(axes.axisW)];
  }

  return table;
}();

constexpr std::array<std::array<int, 4>, 3> EDGE_OF = [] {
  std::array<std::array<int, 4>, 3> table{};

  for (int edge = 0; edge < 24; edge += 2) {
    const int first = EDGES[static_cast<size_t>(edge)].face;
    const int second = EDGES[EDGES[static_cast<size_t>(edge)].twin].face;
    const int low = first / 2 < second / 2 ? first : second;
    const int high = first / 2 < second / 2 ? second : first;

    table[static_cast<size_t>(3 - first / 2 - second / 2)][static_cast<size_t>((low % 2) * 2 + high % 2)] = edge;
  }

  return table;
}();

constexpr std::array<std::array<int, 2>, 3> OTHER_AXES{{{1, 2}, {0, 2}, {0, 1}}};

int edgeAround(int axis, const Triple& direction, double sign) {
  const std::array<int, 2>& other = OTHER_AXES[static_cast<size_t>(axis)];
  const int low = sign * direction[static_cast<size_t>(other[0])] > 0.0 ? 2 : 0;
  const int high = sign * direction[static_cast<size_t>(other[1])] > 0.0 ? 1 : 0;

  return EDGE_OF[static_cast<size_t>(axis)][static_cast<size_t>(low + high)];
}

Triple crossUnit(int axis, const Triple& vector) {
  Triple unit{};

  unit[static_cast<size_t>(axis)] = 1.0;

  return {unit[1] * vector[2] - vector[1] * unit[2], unit[2] * vector[0] - vector[2] * unit[0], unit[0] * vector[1] - vector[0] * unit[1]};
}

Triple crossByUnit(const Triple& vector, int axis) {
  Triple unit{};

  unit[static_cast<size_t>(axis)] = 1.0;

  return {vector[1] * unit[2] - unit[1] * vector[2], vector[2] * unit[0] - unit[2] * vector[0], vector[0] * unit[1] - unit[0] * vector[1]};
}

AxisQuery separatingAxis(
  const Hull& hullA,
  const Hull& hullB,
  const Frame& bInA,
  double speculative
) {
  AxisQuery query;
  const glm::dmat3 back = glm::transpose(bInA.rotation);
  const glm::dvec3& cornerA = hullA.points[0];
  const glm::dvec3& cornerB = hullB.points[0];
  std::array<glm::dvec3, 6> facesB{};

  for (int face = 0; face != 6; ++face) {
    facesB[static_cast<size_t>(face)] = -(bInA.rotation * hullB.planes[face].normal);
  }

  for (int face = 0; face != 6; ++face) {
    const Plane& plane = hullA.planes[face];
    const glm::dvec3 direction = -(back * plane.normal);
    const double offset = glm::dot(plane.normal, bInA.position) - plane.offset;
    const double separation = offset - boxSupport(cornerB, direction);

    if (separation > query.faceA.separation) {
      query.faceA = {plane.normal, separation, face, 0};

      if (separation > speculative) {
        query.separated = true;
        return query;
      }
    }
  }

  for (int face = 0; face != 6; ++face) {
    const Plane& plane = hullB.planes[face];
    const glm::dvec3& direction = facesB[static_cast<size_t>(face)];
    const double offset = glm::dot(direction, bInA.position) - plane.offset;
    const double separation = offset - boxSupport(cornerA, direction);

    if (separation > query.faceB.separation) {
      query.faceB = {direction, separation, 0, face};

      if (separation > speculative) {
        query.separated = true;
        return query;
      }
    }
  }

  if (query.faceA.indexA != NULL_INDEX) {
    const glm::dvec3 direction = -(back * hullA.planes[static_cast<size_t>(query.faceA.indexA)].normal);
    int vertex = 0;

    (void)support(hullB, direction, vertex);
    query.faceA.indexB = vertex;
  }

  if (query.faceB.indexB != NULL_INDEX) {
    int vertex = 0;

    (void)support(hullA, query.faceB.normal, vertex);
    query.faceB.indexA = vertex;
  }

  const double tolerance = LINEAR_SLOP * LINEAR_SLOP;
  std::array<Triple, 6> faces{};
  std::array<Triple, 12> dxc{};

  for (size_t face = 0; face != 6; ++face) {
    faces[face] = tripleOf(facesB[face]);
  }

  for (size_t slot = 0; slot != 12; ++slot) {
    const EdgeAxes& axes = EDGE_AXES[slot];

    dxc[slot] = tripleOf(glm::cross(facesB[static_cast<size_t>(axes.faceV)], facesB[static_cast<size_t>(axes.faceU)]));
  }

  std::array<int, 18> crossing{};
  int crossings = 0;

  for (int axisA = 0; axisA != 3; ++axisA) {
    const Triple row{bInA.rotation[0][axisA], bInA.rotation[1][axisA], bInA.rotation[2][axisA]};

    for (int axisB = 0; axisB != 3; ++axisB) {
      const Triple normal = crossUnit(axisA, tripleOf(bInA.rotation[axisB]));
      const Triple turned = crossByUnit(row, axisB);

      for (int side = 0; side != 2; ++side) {
        const double sign = side == 0 ? 1.0 : -1.0;
        const int edgeA = edgeAround(axisA, normal, sign);
        const int edgeB = edgeAround(axisB, turned, -sign);
        const EdgeAxes& a = EDGE_AXES[static_cast<size_t>(edgeA / 2)];
        const EdgeAxes& b = EDGE_AXES[static_cast<size_t>(edgeB / 2)];
        const Triple& c = faces[static_cast<size_t>(b.faceU)];
        const Triple& d = faces[static_cast<size_t>(b.faceV)];
        const Triple& dc = dxc[static_cast<size_t>(edgeB / 2)];
        const double cba = a.signW * c[static_cast<size_t>(a.axisW)];
        const double dba = a.signW * d[static_cast<size_t>(a.axisW)];
        const double adc = a.signU * dc[static_cast<size_t>(a.axisU)];
        const double bdc = a.signV * dc[static_cast<size_t>(a.axisV)];

        if (cba * dba < -tolerance && adc * bdc < -tolerance && cba * bdc > tolerance) {
          int at = crossings;
          const int key = edgeA * 24 + edgeB;

          while (at > 0 && crossing[static_cast<size_t>(at - 1)] > key) {
            crossing[static_cast<size_t>(at)] = crossing[static_cast<size_t>(at - 1)];
            at -= 1;
          }

          crossing[static_cast<size_t>(at)] = key;
          crossings += 1;
        }
      }
    }
  }

  std::array<glm::dvec3, 8> pointsB{};
  std::array<double, 12> lengthB{};

  for (int vertex = 0; vertex != 8; ++vertex) {
    pointsB[static_cast<size_t>(vertex)] = apply(bInA, hullB.points[vertex]);
  }

  lengthB.fill(-1.0);

  for (int index = 0; index != crossings; ++index) {
    const int edgeA = crossing[static_cast<size_t>(index)] / 24;
    const int edgeB = crossing[static_cast<size_t>(index)] % 24;
    const HalfEdge& halfA = EDGES[static_cast<size_t>(edgeA)];
    const HalfEdge& twinA = EDGES[halfA.twin];
    const HalfEdge& halfB = EDGES[static_cast<size_t>(edgeB)];
    const HalfEdge& twinB = EDGES[halfB.twin];
    const glm::dvec3 pA = hullA.points[halfA.origin];
    const glm::dvec3 eA = hullA.points[twinA.origin] - pA;
    const glm::dvec3 pB = pointsB[halfB.origin];
    const glm::dvec3 eB = pointsB[twinB.origin] - pB;
    double& edgeLength = lengthB[static_cast<size_t>(edgeB / 2)];

    if (edgeLength < 0.0) {
      edgeLength = glm::length(eB);
    }

    glm::dvec3 normal = glm::cross(eA, eB);
    const double length = glm::length(normal);

    if (!(length > 1.0e-6 * std::abs(eA[EDGE_AXES[static_cast<size_t>(edgeA / 2)].axisW]) * edgeLength)) {
      continue;
    }

    normal /= length;

    if (glm::dot(normal, pA) < 0.0) {
      normal = -normal;
    }

    const double separation = glm::dot(normal, pB - pA);

    if (separation > query.edge.separation) {
      query.edge = {normal, separation, edgeA, edgeB};

      if (separation > speculative) {
        query.separated = true;
        return query;
      }
    }
  }

  return query;
}

int incidentFace(const Hull& hull, const glm::dvec3& normal, int vertex) {
  int best = NULL_INDEX;
  double least = std::numeric_limits<double>::max();
  int edgeIndex = VERTEX_EDGE[static_cast<size_t>(vertex)];
  const int first = edgeIndex;
  const glm::dvec3 origin = hull.points[EDGES[edgeIndex].origin];

  do {
    const HalfEdge& edge = EDGES[edgeIndex];
    const HalfEdge& twin = EDGES[edge.twin];
    const glm::dvec3 axis = glm::normalize(hull.points[twin.origin] - origin);
    const double projection = std::abs(glm::dot(axis, normal));

    if (projection < least) {
      best = edgeIndex;
      least = projection;
    }

    edgeIndex = twin.next;
  } while (edgeIndex != first);

  const int face1 = EDGES[best].face;
  const int face2 = EDGES[EDGES[best].twin].face;

  return glm::dot(hull.planes[face1].normal, normal)
             < glm::dot(hull.planes[face2].normal, normal)
           ? face1
           : face2;
}

int buildPolygon(
  std::array<ClipVertex, MAX_CLIP_POINTS>& out,
  const Frame& frame,
  const Hull& hull,
  int face,
  const Plane& reference
) {
  int edgeIndex = FACE_EDGE[static_cast<size_t>(face)];
  const int first = edgeIndex;
  int count = 0;

  do {
    const HalfEdge& edge = EDGES[edgeIndex];
    const int nextIndex = edge.next;
    const HalfEdge& next = EDGES[nextIndex];
    ClipVertex vertex;

    vertex.position = apply(frame, hull.points[next.origin]);
    vertex.separation = planeSeparation(reference, vertex.position);
    vertex.pair = makePair(OWNER_B, edgeIndex, OWNER_B, nextIndex);
    out[static_cast<size_t>(count)] = vertex;
    count += 1;
    edgeIndex = nextIndex;
  } while (edgeIndex != first && count < MAX_CLIP_POINTS);

  return count;
}

int clipPolygon(
  std::array<ClipVertex, MAX_CLIP_POINTS>& out,
  const std::array<ClipVertex, MAX_CLIP_POINTS>& polygon,
  int count,
  const Plane& clip,
  int edge,
  const Plane& reference
) {
  ClipVertex vertex1 = polygon[static_cast<size_t>(count - 1)];
  double distance1 = planeSeparation(clip, vertex1.position);
  int outCount = 0;

  for (int index = 0; index < count && outCount + 2 <= MAX_CLIP_POINTS; ++index) {
    const ClipVertex vertex2 = polygon[static_cast<size_t>(index)];
    const double distance2 = planeSeparation(clip, vertex2.position);

    if (distance1 <= 0.0 && distance2 <= 0.0) {
      out[static_cast<size_t>(outCount++)] = vertex2;
    } else if (distance1 <= 0.0 && distance2 > 0.0) {
      const double fraction = distance1 / (distance1 - distance2);
      ClipVertex vertex;

      vertex.position =
        vertex1.position + fraction * (vertex2.position - vertex1.position);
      vertex.separation = planeSeparation(reference, vertex.position);
      vertex.pair = vertex2.pair;
      vertex.pair.owner2 = OWNER_A;
      vertex.pair.index2 = static_cast<uint8_t>(edge);
      out[static_cast<size_t>(outCount++)] = vertex;
    } else if (distance2 <= 0.0 && distance1 > 0.0) {
      const double fraction = distance1 / (distance1 - distance2);
      ClipVertex vertex;

      vertex.position =
        vertex1.position + fraction * (vertex2.position - vertex1.position);
      vertex.separation = planeSeparation(reference, vertex.position);
      vertex.pair = vertex1.pair;
      vertex.pair.owner1 = OWNER_A;
      vertex.pair.index1 = static_cast<uint8_t>(edge);
      out[static_cast<size_t>(outCount++)] = vertex;
      out[static_cast<size_t>(outCount++)] = vertex2;
    }

    vertex1 = vertex2;
    distance1 = distance2;
  }

  return outCount;
}

int reduceIndices(
  std::span<const glm::dvec3> position,
  std::span<const double> separation,
  const glm::dvec3& normal,
  double cutoff,
  double tolerance,
  std::array<int, 4>& chosen
) {
  const int total = static_cast<int>(position.size());

  if (total <= 4) {
    for (int index = 0; index != total; ++index) {
      chosen[static_cast<size_t>(index)] = index;
    }

    return total;
  }

  std::array<int, MAX_CLIP_POINTS> small{};
  std::vector<int> large(total > MAX_CLIP_POINTS ? static_cast<size_t>(total) : 0u);
  const std::span<int> order = total > MAX_CLIP_POINTS ? std::span<int>(large)
                                                       : std::span<int>(small).first(static_cast<size_t>(total));

  for (int index = 0; index != total; ++index) {
    order[static_cast<size_t>(index)] = index;
  }

  const auto at = [&](int index) {
    return position[static_cast<size_t>(order[static_cast<size_t>(index)])];
  };
  const auto depthOf = [&](int index) {
    return separation[static_cast<size_t>(order[static_cast<size_t>(index)])];
  };
  const auto take = [&](int index, int& count) {
    const int picked = order[static_cast<size_t>(index)];

    order[static_cast<size_t>(index)] = order[static_cast<size_t>(count - 1)];
    count -= 1;

    return picked;
  };
  const double bias = 0.95;
  const glm::dvec3 search = arbitraryPerp(normal);
  int count = total;
  int bestIndex = NULL_INDEX;
  double bestScore = -std::numeric_limits<double>::max();

  for (int index = 0; index != count; ++index) {
    if (depthOf(index) > cutoff) {
      continue;
    }

    const double score = -depthOf(index) + glm::dot(search, at(index));

    if (bias * score > bestScore) {
      bestIndex = index;
      bestScore = score;
    }
  }

  if (bestIndex == NULL_INDEX) {
    return 0;
  }

  chosen[0] = take(bestIndex, count);

  const glm::dvec3 a = position[static_cast<size_t>(chosen[0])];

  bestScore = 0.0;
  bestIndex = NULL_INDEX;

  for (int index = 0; index != count; ++index) {
    const glm::dvec3 offset = at(index) - a;
    const glm::dvec3 flat = offset - glm::dot(offset, normal) * normal;
    const double depth = std::max(0.0, -depthOf(index));
    const double score = glm::dot(flat, flat) + 4.0 * depth * depth;

    if (bias * score > bestScore) {
      bestScore = score;
      bestIndex = index;
    }
  }

  if (bestScore < tolerance) {
    return 1;
  }

  chosen[1] = take(bestIndex, count);

  const glm::dvec3 b = position[static_cast<size_t>(chosen[1])];
  const glm::dvec3 ba = b - a;
  double bestSignedArea = 0.0;

  bestScore = tolerance;
  bestIndex = NULL_INDEX;

  for (int index = 0; index != count; ++index) {
    const double signedArea = glm::dot(normal, glm::cross(ba, at(index) - a));
    const double score = std::abs(signedArea);

    if (bias * score >= bestScore) {
      bestScore = score;
      bestIndex = index;
      bestSignedArea = signedArea;
    }
  }

  if (bestIndex == NULL_INDEX) {
    return 2;
  }

  chosen[2] = take(bestIndex, count);

  const glm::dvec3 c = position[static_cast<size_t>(chosen[2])];
  const double sign = bestSignedArea < 0.0 ? -1.0 : 1.0;

  bestScore = tolerance;
  bestIndex = NULL_INDEX;

  for (int index = 0; index != count; ++index) {
    const glm::dvec3 p = at(index);
    const double u1 = sign * glm::dot(normal, glm::cross(p - a, ba));
    const double u2 = sign * glm::dot(normal, glm::cross(p - b, c - b));
    const double u3 = sign * glm::dot(normal, glm::cross(p - c, a - c));
    const double score = std::max(u1, std::max(u2, u3));

    if (bias * score > bestScore) {
      bestScore = score;
      bestIndex = index;
    }
  }

  if (bestIndex == NULL_INDEX) {
    return 3;
  }

  chosen[3] = take(bestIndex, count);

  return 4;
}

void reducePoints(
  LocalManifold& manifold,
  std::array<LocalPoint, MAX_CLIP_POINTS>& points,
  int count,
  double speculative
) {
  std::array<glm::dvec3, MAX_CLIP_POINTS> position{};
  std::array<double, MAX_CLIP_POINTS> separation{};

  for (int index = 0; index != count; ++index) {
    position[static_cast<size_t>(index)] = points[static_cast<size_t>(index)].point;
    separation[static_cast<size_t>(index)] =
      points[static_cast<size_t>(index)].separation;
  }

  std::array<int, 4> chosen{};
  const int kept = reduceIndices(
    std::span<const glm::dvec3>(position).first(static_cast<size_t>(count)),
    std::span<const double>(separation).first(static_cast<size_t>(count)),
    manifold.normal,
    speculative,
    speculative * speculative,
    chosen
  );

  for (int index = 0; index != kept; ++index) {
    manifold.points[static_cast<size_t>(index)] =
      points[static_cast<size_t>(chosen[static_cast<size_t>(index)])];
  }

  manifold.count = kept;
}

bool faceAContact(
  LocalManifold& manifold,
  double& clipSeparation,
  const Hull& hullA,
  const Hull& hullB,
  const Frame& bInA,
  const SeparatingAxis& query,
  double speculative
) {
  const int referenceFace = query.indexA;
  const Plane reference = hullA.planes[static_cast<size_t>(referenceFace)];
  const glm::dvec3 normalInB = glm::transpose(bInA.rotation) * reference.normal;
  const int incident = incidentFace(hullB, normalInB, query.indexB);
  std::array<ClipVertex, MAX_CLIP_POINTS> buffer1{};
  std::array<ClipVertex, MAX_CLIP_POINTS> buffer2{};
  int count = buildPolygon(buffer1, bInA, hullB, incident, reference);
  std::array<ClipVertex, MAX_CLIP_POINTS>* input = &buffer1;
  std::array<ClipVertex, MAX_CLIP_POINTS>* output = &buffer2;
  int edgeIndex = FACE_EDGE[static_cast<size_t>(referenceFace)];
  const int first = edgeIndex;

  do {
    const HalfEdge& edge = EDGES[edgeIndex];
    const int nextIndex = edge.next;
    const HalfEdge& next = EDGES[nextIndex];
    const glm::dvec3 vertex1 = hullA.points[edge.origin];
    const glm::dvec3 vertex2 = hullA.points[next.origin];
    const glm::dvec3 tangent = glm::normalize(vertex2 - vertex1);
    const glm::dvec3 binormal = glm::cross(tangent, reference.normal);

    count = clipPolygon(*output, *input, count, planeFrom(binormal, vertex1), edgeIndex, reference);
    std::swap(output, input);

    if (count < 3) {
      clipSeparation = 0.0;
      return false;
    }

    edgeIndex = nextIndex;
  } while (edgeIndex != first);

  std::array<LocalPoint, MAX_CLIP_POINTS> points{};
  double least = std::numeric_limits<double>::max();

  manifold.normal = reference.normal;

  for (int index = 0; index != count; ++index) {
    const ClipVertex& clipped = (*input)[static_cast<size_t>(index)];
    LocalPoint& point = points[static_cast<size_t>(index)];

    point.point = clipped.position - (0.5 * clipped.separation) * reference.normal;
    point.separation = clipped.separation;
    point.pair = clipped.pair;
    least = std::min(least, clipped.separation);
  }

  if (least >= speculative) {
    clipSeparation = 0.0;
    return false;
  }

  reducePoints(manifold, points, count, speculative);
  clipSeparation = least;

  return true;
}

bool faceBContact(
  LocalManifold& manifold,
  double& clipSeparation,
  const Hull& hullA,
  const Hull& hullB,
  const Frame& bInA,
  const SeparatingAxis& query,
  double speculative
) {
  const Frame aInB = invert(bInA);
  const SeparatingAxis flipped{-query.normal, query.separation, query.indexB, query.indexA};

  if (!faceAContact(manifold, clipSeparation, hullB, hullA, aInB, flipped, speculative)) {
    clipSeparation = 0.0;
    return false;
  }

  manifold.normal = -(bInA.rotation * manifold.normal);

  for (int index = 0; index != manifold.count; ++index) {
    LocalPoint& point = manifold.points[static_cast<size_t>(index)];

    point.point = apply(bInA, point.point);
    point.pair = flipPair(point.pair);
  }

  return true;
}

bool edgeContact(
  LocalManifold& manifold,
  const Hull& hullA,
  const Hull& hullB,
  const Frame& bInA,
  const SeparatingAxis& query
) {
  const HalfEdge& edgeA = EDGES[static_cast<size_t>(query.indexA)];
  const HalfEdge& twinA = EDGES[edgeA.twin];
  const glm::dvec3 pA = hullA.points[edgeA.origin];
  const glm::dvec3 eA = hullA.points[twinA.origin] - pA;
  const HalfEdge& edgeB = EDGES[static_cast<size_t>(query.indexB)];
  const HalfEdge& twinB = EDGES[edgeB.twin];
  const glm::dvec3 pB = apply(bInA, hullB.points[edgeB.origin]);
  const glm::dvec3 eB = apply(bInA, hullB.points[twinB.origin]) - pB;
  const double a11 = glm::dot(eA, eA);
  const double a12 = -glm::dot(eA, eB);
  const double a21 = glm::dot(eB, eA);
  const double a22 = -glm::dot(eB, eB);
  const glm::dvec3 w = pA - pB;
  const double b1 = -glm::dot(eA, w);
  const double b2 = -glm::dot(eB, w);
  const double det = a11 * a22 - a12 * a21;

  if (det * det < 1000.0 * std::numeric_limits<double>::min()) {
    return false;
  }

  const double s1 = (a22 * b1 - a12 * b2) / det;
  const double s2 = (a11 * b2 - a21 * b1) / det;

  if (!(0.0 <= s1 && s1 <= 1.0 && 0.0 <= s2 && s2 <= 1.0)) {
    return false;
  }

  const glm::dvec3 point1 = pA + s1 * eA;
  const glm::dvec3 point2 = pB + s2 * eB;

  manifold.normal = query.normal;
  manifold.count = 1;
  manifold.points[0].point = 0.5 * (point1 + point2);
  manifold.points[0].separation = glm::dot(query.normal, point2 - point1);
  manifold.points[0].pair = makePair(OWNER_A, query.indexA, OWNER_B, query.indexB);

  return true;
}

}

Manifold collideBoxHulls(const BoxPose& a, const BoxPose& b, double speculative) {
  Manifold result;
  const Hull hullA = makeHull(a.half);
  const Hull hullB = makeHull(b.half);
  const glm::dmat3 turnA = glm::mat3_cast(a.rotation);
  const glm::dmat3 turnB = glm::mat3_cast(b.rotation);
  const Frame bInA{glm::transpose(turnA) * turnB, glm::transpose(turnA) * (b.centre - a.centre)};
  const AxisQuery query = separatingAxis(hullA, hullB, bInA, speculative);

  if (query.separated) {
    return result;
  }

  LocalManifold manifold;
  double clipSeparation = 0.0;

  if (query.faceA.separation > query.faceB.separation) {
    faceAContact(manifold, clipSeparation, hullA, hullB, bInA, query.faceA, speculative);
  } else {
    faceBContact(manifold, clipSeparation, hullA, hullB, bInA, query.faceB, speculative);
  }

  if (query.edge.indexA != NULL_INDEX) {
    const double faceSeparation = std::max(query.faceA.separation, query.faceB.separation);

    if ((manifold.count == 0 && query.edge.separation > faceSeparation)
        || query.edge.separation > clipSeparation + LINEAR_SLOP) {
      LocalManifold edge;

      if (edgeContact(edge, hullA, hullB, bInA, query.edge)) {
        manifold = edge;
      }
    }
  }

  result.normal = -(turnA * manifold.normal);
  result.count = static_cast<uint32_t>(manifold.count);

  for (int index = 0; index != manifold.count; ++index) {
    const LocalPoint& point = manifold.points[static_cast<size_t>(index)];
    ManifoldPoint& out = result.points[static_cast<size_t>(index)];

    out.point = a.centre + turnA * point.point;
    out.separation = point.separation;
    out.feature = packFeature(point.pair);
  }

  return result;
}


uint32_t reduceManifold(
  std::span<const glm::dvec3> points,
  std::span<const double> separations,
  const glm::dvec3& normal,
  double cutoff,
  double tolerance,
  std::array<uint32_t, 4>& chosen
) {
  std::array<int, 4> picked{};
  const int kept =
    reduceIndices(points, separations, normal, cutoff, tolerance, picked);

  for (int index = 0; index != kept; ++index) {
    chosen[static_cast<size_t>(index)] =
      static_cast<uint32_t>(picked[static_cast<size_t>(index)]);
  }

  return static_cast<uint32_t>(kept);
}

}
