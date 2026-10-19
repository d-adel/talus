#include "continuous.h"

#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/compound/compound.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace talus::dynamics {

namespace {

constexpr double linearSlop = 0.005;
constexpr double safetyFactor = 0.5;
constexpr uint32_t maxSteps = 30u;
constexpr double turnMargin = 1.0 + 0x1.0p-40;

double fieldDistance(const field::SampledSdf& field, const glm::dvec3& point) {
  const glm::dvec3 low(field.origin);
  const glm::dvec3 high =
    low
    + glm::dvec3(field.voxelSize)
        * (glm::dvec3(field.resolution) - glm::dvec3(1.0));
  const glm::dvec3 inside = glm::clamp(point, low, high);
  const double away = glm::length(point - inside);
  const field::FieldProbeOf<double> probed =
    field::probe<double>(field, inside);
  const double slope = glm::length(probed.gradient);

  if (!std::isfinite(probed.distance) || !(slope >= 1.0e-6)) {
    return std::numeric_limits<double>::infinity();
  }

  return std::max(away, probed.distance / slope - away);
}

double smallestExtentOf(const RigidBody& body) {
  if (body.shape == RigidShape::Box) {
    return std::min(
      {body.halfExtents.x, body.halfExtents.y, body.halfExtents.z}
    );
  }

  if (body.shape == RigidShape::Compound) {
    return body.compound->thinnest;
  }

  glm::dvec3 extent(body.radius);

  for (const glm::dvec3& sample : body.samples) {
    extent = glm::max(extent, glm::abs(sample));
  }

  return std::min({extent.x, extent.y, extent.z});
}

RigidPose poseAt(const RigidPose& start, const RigidPose& end, double t) {
  const glm::dquat target =
    glm::dot(start.rotation, end.rotation) < 0.0 ? -end.rotation : end.rotation;

  return {
    glm::mix(start.position, end.position, t),
    glm::normalize((1.0 - t) * start.rotation + t * target)
  };
}

double distanceAt(
  const RigidBody& body,
  const RigidPose& pose,
  const field::SampledSdf& field
) {
  const glm::dmat3 turn = glm::mat3_cast(pose.rotation);
  double nearest = std::numeric_limits<double>::infinity();

  for (const glm::dvec3& sample : body.samples) {
    nearest = std::min(
      nearest,
      fieldDistance(field, pose.position + turn * sample)
    );
  }

  if (body.shape == RigidShape::Compound) {
    for (const CompoundChild& child : body.compound->children) {
      for (const glm::dvec3& sample : childSamples(child)) {
        nearest = std::min(
          nearest,
          fieldDistance(field, pose.position + turn * sample)
        );
      }
    }
  }

  if (body.radius > 0.0) {
    nearest =
      std::min(nearest, fieldDistance(field, pose.position) - body.radius);
  }

  return nearest;
}

template<typename Distance>
double advanceTo(
  const RigidPose& start,
  const RigidPose& end,
  double reach,
  double target,
  Distance&& distanceAtPose
) {
  const double tolerance = 0.25 * linearSlop;
  const double bound =
    glm::length(end.position - start.position)
    + turnBound(start.rotation, end.rotation) * reach;
  double distance = distanceAtPose(start);

  if (distance <= target + tolerance || !(bound > 0.0)) {
    return 1.0;
  }

  double t = 0.0;

  for (uint32_t step = 0u; step != maxSteps; ++step) {
    t += (distance - target) / bound;

    if (t >= 1.0) {
      return 1.0;
    }

    distance = distanceAtPose(poseAt(start, end, t));

    if (distance <= target + tolerance) {
      return t;
    }
  }

  return t;
}

struct Probe {
  std::vector<glm::dvec3> points;
  double radius = 0.0;
  Bounds local;
};

double boxDistance(const CompoundChild& child, const glm::dvec3& point) {
  const glm::dvec3 local = glm::conjugate(child.rotation) * (point - child.centre);
  const glm::dvec3 outside = glm::abs(local) - child.half;

  return glm::length(glm::max(outside, glm::dvec3(0.0)))
         + std::min(std::max({outside.x, outside.y, outside.z}), 0.0);
}

std::vector<Probe> probesOf(const RigidBody& body) {
  std::vector<Probe> probes;

  if (body.shape == RigidShape::Compound) {
    for (uint32_t index = 0u; index != body.compound->children.size(); ++index) {
      const std::array<glm::dvec3, childSampleCount> samples = childSamples(body.compound->children[index]);

      probes.push_back(Probe{
        std::vector<glm::dvec3>(samples.begin(), samples.end()),
        0.0,
        body.compound->childBounds[index]
      });
    }

    return probes;
  }

  Probe probe;

  probe.points = body.samples;
  probe.radius = body.radius;
  probe.local = Bounds{glm::dvec3(-body.radius), glm::dvec3(body.radius)};

  for (const glm::dvec3& sample : body.samples) {
    probe.local.low = glm::min(probe.local.low, sample);
    probe.local.high = glm::max(probe.local.high, sample);
  }

  probes.push_back(std::move(probe));

  return probes;
}

double probeDistance(const Probe& probe, const CompoundChild& target, const RigidPose& pose) {
  const glm::dmat3 turn = glm::mat3_cast(pose.rotation);
  double nearest = std::numeric_limits<double>::infinity();

  for (const glm::dvec3& point : probe.points) {
    nearest = std::min(nearest, boxDistance(target, pose.position + turn * point));
  }

  if (probe.radius > 0.0) {
    nearest = std::min(nearest, boxDistance(target, pose.position) - probe.radius);
  }

  return nearest;
}

double anchorFraction(
  const RigidBody& body,
  std::span<const Probe> probes,
  const RigidPose& from,
  const RigidPose& to,
  const RigidBody& anchor
) {
  const glm::dquat back = glm::conjugate(anchor.rotation);
  const RigidPose localFrom{back * (from.position - anchor.position), back * from.rotation};
  const RigidPose localTo{back * (to.position - anchor.position), back * to.rotation};
  const double reach = reachOf(body);
  const double turning = turnBound(localFrom.rotation, localTo.rotation) * reach;
  std::vector<std::pair<uint32_t, uint32_t>> pairs;
  std::vector<uint32_t> found;

  for (uint32_t probe = 0u; probe != probes.size(); ++probe) {
    const Bounds& local = probes[probe].local;
    const glm::dvec3 centre = 0.5 * (local.low + local.high);
    const glm::dvec3 half = 0.5 * (local.high - local.low);
    const Bounds swept = grown(
      unite(
        boxBoundsIn(localFrom.position + localFrom.rotation * centre, glm::mat3_cast(localFrom.rotation), half),
        boxBoundsIn(localTo.position + localTo.rotation * centre, glm::mat3_cast(localTo.rotation), half)
      ),
      turning + linearSlop
    );

    overlappingChildren(*anchor.compound, swept, found);

    for (const uint32_t child : found) {
      pairs.emplace_back(probe, child);
    }
  }

  if (pairs.empty()) {
    return 1.0;
  }

  return advanceTo(localFrom, localTo, reach, linearSlop, [&](const RigidPose& pose) {
    double nearest = std::numeric_limits<double>::infinity();

    for (const auto& [probe, child] : pairs) {
      nearest = std::min(nearest, probeDistance(probes[probe], anchor.compound->children[child], pose));
    }

    return nearest;
  });
}

}

double turnBound(const glm::dquat& from, const glm::dquat& to) {
  const glm::dquat relative = glm::conjugate(from) * to;
  const double sine =
    glm::length(glm::dvec3(relative.x, relative.y, relative.z));
  const double cosine = std::abs(relative.w);

  return (cosine > sine ? 2.0 * sine / cosine : std::numbers::pi) * turnMargin;
}

double timeOfImpact(
  const RigidBody& body,
  const RigidPose& start,
  const RigidPose& end,
  const field::SampledSdf& field,
  double target
) {
  return advanceTo(start, end, reachOf(body), target, [&](const RigidPose& pose) {
    return distanceAt(body, pose, field);
  });
}

std::vector<uint32_t> sweepAnchors(const BodySet& bodies, const Workers& workers) {
  std::vector<std::vector<uint32_t>> found(std::max(workers.count, 1u));

  parallelFor(workers, static_cast<uint32_t>(bodies.rigids.size()), 1024u, [&](uint32_t begin, uint32_t end, uint32_t worker) {
    for (uint32_t index = begin; index != end; ++index) {
      const RigidBody& body = bodies.rigids[index];

      if (body.shape == RigidShape::Compound && body.compound != nullptr && !(body.mass > 0.0)) {
        found[worker].push_back(index);
      }
    }
  });

  std::vector<uint32_t> anchors;

  for (const std::vector<uint32_t>& list : found) {
    anchors.insert(anchors.end(), list.begin(), list.end());
  }

  std::sort(anchors.begin(), anchors.end());

  return anchors;
}

void solveContinuousBody(
  BodySet& bodies,
  std::span<const RigidPose> start,
  uint32_t rigid,
  const IntegrationParameters& parameters,
  std::span<const uint32_t> anchors
) {
  {
    RigidBody& body = bodies.rigids[rigid];

    if (!(body.mass > 0.0)) {
      return;
    }

    const RigidPose& from = start[rigid];
    const RigidPose to{body.position, body.rotation};
    const double reach = reachOf(body);
    const double travel = glm::length(to.position - from.position);
    const double limit = safetyFactor * smallestExtentOf(body);
    const glm::dquat relative = glm::conjugate(from.rotation) * to.rotation;
    const double sine = glm::length(glm::dvec3(relative.x, relative.y, relative.z));

    if (travel + 2.0 * std::numbers::pi * sine * reach <= limit) {
      return;
    }

    const double motion = travel + turnBound(from.rotation, to.rotation) * reach;

    if (motion <= limit) {
      return;
    }

    const glm::dvec3 low =
      glm::min(from.position, to.position) - glm::dvec3(reach);
    const glm::dvec3 high =
      glm::max(from.position, to.position) + glm::dvec3(reach);
    double fraction = 1.0;

    for (uint32_t index = 0u; index != bodies.statics.size(); ++index) {
      const Bounds fixed = boundsOf(bodies, {Side::Static, index}, 0.0);

      if (glm::any(glm::lessThan(high, fixed.low))
          || glm::any(glm::greaterThan(low, fixed.high))) {
        continue;
      }

      const StaticField& placed = bodies.statics[index];
      const glm::dquat back = glm::conjugate(placed.rotation);
      const RigidPose localFrom{
        back * (from.position - placed.position),
        back * from.rotation
      };
      const RigidPose localTo{
        back * (to.position - placed.position),
        back * to.rotation
      };

      fraction = std::min(
        fraction,
        timeOfImpact(body, localFrom, localTo, *placed.field, linearSlop)
      );
    }

    if (!anchors.empty()) {
      const std::vector<Probe> probes = probesOf(body);

      for (const uint32_t index : anchors) {
        if (index == rigid) {
          continue;
        }

        const Bounds fixed = boundsOf(bodies, {Side::Rigid, index}, 0.0);

        if (glm::any(glm::lessThan(high, fixed.low))
            || glm::any(glm::greaterThan(low, fixed.high))) {
          continue;
        }

        fraction = std::min(fraction, anchorFraction(body, probes, from, to, bodies.rigids[index]));
      }
    }

    if (fraction < 1.0) {
      const RigidPose impact = poseAt(from, to, fraction);

      body.position = impact.position;
      body.rotation = impact.rotation;
      body.velocity -= (1.0 - fraction) * parameters.dt * parameters.gravity;
    }
  }
}

}
