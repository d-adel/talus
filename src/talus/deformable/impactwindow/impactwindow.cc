#include "impactwindow.ih"

#include <glm/gtc/packing.hpp>

#include <iterator>
#include <numeric>
#include <optional>

namespace talus::deformable {

namespace {

constexpr double PENALTY_SHARE = 0.05;
constexpr double REFERENCE_SHARE = 0.9;
constexpr double EXIT_PLASTIC_SHARE = 0.005;
constexpr double EXIT_INPUT_SHARE = 1.0e-6;
constexpr double DRIFT_FLOOR = 1.0e-9;

double waveSpeed(const talus::PhysicalMaterial& lane) {
  const double youngs = static_cast<double>(lane.youngsModulus);
  const double poissons = static_cast<double>(lane.poissonsRatio);
  const double density = static_cast<double>(lane.density);

  return std::sqrt(
    youngs * (1.0 - poissons)
    / (density * (1.0 + poissons) * (1.0 - 2.0 * poissons))
  );
}

struct MemberLaw {
  WindowLaw law;
  std::vector<double> mass;
  std::vector<uint32_t> nodeLane;
  std::vector<std::array<glm::vec3, 8>> rest;
  std::vector<ElasticParams> elastic;
  std::vector<PlasticParams> plastic;
  std::vector<talus::ElasticModel> model;
  std::vector<float> hysteresis;
  std::vector<double> seedShare;
};

MemberLaw memberLaw(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  double frameTime,
  const Settings& settings
) {
  MemberLaw made;

  made.law = windowLaw(mesh, materials, frameTime, settings);

  const size_t nodes = mesh.restPositions.size();

  made.mass.resize(nodes);

  for (size_t node = 0u; node != nodes; ++node) {
    made.mass[node] = static_cast<double>(mesh.nodeMass[node]);
  }

  made.nodeLane.assign(nodes, std::numeric_limits<uint32_t>::max());
  made.rest.resize(mesh.hexes.size());
  made.elastic.resize(mesh.hexes.size());
  made.plastic.resize(mesh.hexes.size());
  made.model.resize(mesh.hexes.size());
  made.hysteresis.resize(mesh.hexes.size());

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::PreparedHex& hex = mesh.hexes[element];
    const glm::vec3 corner = mesh.restPositions[hex.nodes[0]];
    const talus::PhysicalMaterial& lane =
      laneMaterial(materials, hex.material);

    for (size_t at = 0u; at != 8u; ++at) {
      made.rest[element][at] = mesh.restPositions[hex.nodes[at]] - corner;

      uint32_t& owner = made.nodeLane[hex.nodes[at]];

      if (owner == std::numeric_limits<uint32_t>::max()) {
        owner = hex.material;
      }
    }

    made.elastic[element] =
      ElasticParams{lane.youngsModulus, lane.poissonsRatio, lane.density};
    made.plastic[element] =
      PlasticParams{lane.yieldStress, lane.hardeningModulus};
    made.model[element] = lane.elasticModel;
    made.hysteresis[element] = lane.hysteresis;
  }

  for (uint32_t& owner : made.nodeLane) {
    if (owner == std::numeric_limits<uint32_t>::max()) {
      owner = 0u;
    }
  }

  std::vector<double> carried(nodes, 0.0);

  for (size_t seed = 0u; seed != mesh.surfaceSeeds.size(); ++seed) {
    const talus::PreparedHex& hex =
      mesh.hexes[mesh.surfaceSeedHexes[seed]];

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      carried[hex.nodes[corner]] +=
        static_cast<double>(mesh.surfaceSeedShapes[seed][corner]);
    }
  }

  made.seedShare.assign(mesh.surfaceSeeds.size(), 0.0);

  for (size_t seed = 0u; seed != mesh.surfaceSeeds.size(); ++seed) {
    const talus::PreparedHex& hex =
      mesh.hexes[mesh.surfaceSeedHexes[seed]];

    double share = std::numeric_limits<double>::infinity();

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const uint32_t node = hex.nodes[corner];

      if (mesh.surfaceSeedShapes[seed][corner] > 0.0f && carried[node] > 0.0) {
        share = std::min(share, made.mass[node] / carried[node]);
      }
    }

    made.seedShare[seed] = std::isfinite(share) ? share : 0.0;
  }

  return made;
}

struct Rigid {
  double mass = 0.0;
  glm::dvec3 centre{0.0};
  glm::dvec3 linear{0.0};
  glm::dvec3 angular{0.0};
  glm::dmat3 inertia{0.0};
};

Rigid rigidOf(std::span<const double> mass, const ImpactWindow& window) {
  const RigidMass body =
    rigidMass(mass, std::span<const glm::dvec3>(window.position));

  Rigid made;

  made.mass = body.mass;
  made.centre = body.centre;
  made.inertia = body.inertia;

  for (size_t node = 0u; node != mass.size(); ++node) {
    made.linear += mass[node] * window.velocity[node];
  }

  if (!(made.mass > 0.0)) {
    return made;
  }

  for (size_t node = 0u; node != mass.size(); ++node) {
    const glm::dvec3 arm = window.position[node] - made.centre;

    made.angular += mass[node] * glm::cross(arm, window.velocity[node]);
  }

  return made;
}

struct PartnerFrame {
  double mass = 0.0;
  glm::dmat3 inertia{0.0};
  glm::dvec3 startCentre{0.0};
  double base = 0.0;
  double now = 0.0;
  double gravityWork = 0.0;
  glm::dvec3 force{0.0};
  glm::dvec3 torque{0.0};
  glm::dvec3 drag{0.0};
  glm::dvec3 dragTorque{0.0};
  glm::dmat3 bodyInertia{0.0};
  glm::dmat3 bodyInverse{0.0};
  glm::dvec3 momentum{0.0};
};

struct PartnerSupport {
  double mass = 0.0;
  double stiffness = 0.0;
  std::vector<float> friction;
};

struct SupportLoad {
  size_t partner = 0u;
  glm::dvec3 force{0.0};
  glm::dvec3 reaction{0.0};
};

WindowSupportPair& supportPairOf(
  std::vector<WindowSupportPair>& pairs,
  uint32_t partner
) {
  const auto found = std::find_if(
    pairs.begin(),
    pairs.end(),
    [partner](const WindowSupportPair& pair) { return pair.partner == partner; }
  );

  if (found != pairs.end()) {
    return *found;
  }

  WindowSupportPair made;

  made.partner = partner;
  pairs.push_back(made);

  return pairs.back();
}

struct SupportStep {
  double penalty = 0.0;
  double carriedPenalty = 0.0;
  double correction = 0.0;
  double excessAbsorbed = 0.0;
};

constexpr double TRACE_FLOOR = 1.0e-6;
constexpr uint64_t TRACE_ID_MASK = 0xffffffffull;

uint64_t traceCode(WindowTraceKind kind, uint64_t id) {
  return (static_cast<uint64_t>(kind) << 32u) | (id & TRACE_ID_MASK);
}

WindowTracePartner traceDecode(uint64_t code) {
  WindowTracePartner decoded;

  decoded.kind = static_cast<WindowTraceKind>(code >> 32u);
  decoded.id = code & TRACE_ID_MASK;

  return decoded;
}

WindowTraceKind partnerTraceKind(const WindowPartner& partner) {
  if (partner.mesh != nullptr) {
    return partner.dynamic ? WindowTraceKind::StoredDynamic
                           : WindowTraceKind::Stored;
  }

  return partner.dynamic ? WindowTraceKind::Dynamic : WindowTraceKind::Static;
}

uint64_t partnerTraceId(const WindowPartner& partner) {
  return partner.mesh != nullptr ? (partner.key & TRACE_ID_MASK)
                                 : static_cast<uint64_t>(partner.body);
}

WindowTraceContact& traceEntry(
  std::vector<WindowTraceContact>& contacts,
  WindowTraceKind kind,
  uint64_t id
) {
  const auto found = std::find_if(
    contacts.begin(),
    contacts.end(),
    [kind, id](const WindowTraceContact& entry) {
      return entry.kind == kind && entry.id == id;
    }
  );

  if (found != contacts.end()) {
    return *found;
  }

  WindowTraceContact made;

  made.kind = kind;
  made.id = id;
  contacts.push_back(made);

  return contacts.back();
}

void traceContact(
  std::vector<WindowTraceContact>& contacts,
  WindowTraceKind kind,
  uint64_t id,
  double depth,
  double penalty
) {
  WindowTraceContact& entry = traceEntry(contacts, kind, id);

  ++entry.points;
  entry.deepest = std::max(entry.deepest, depth);
  entry.penalty += penalty;
}

void traceWork(
  std::vector<WindowTraceContact>& contacts,
  WindowTraceKind kind,
  uint64_t id,
  double work
) {
  if (work == 0.0) {
    return;
  }

  traceEntry(contacts, kind, id).work += work;
}

WindowTracePartner& tracePartner(
  std::vector<WindowTracePartner>& partners,
  WindowTraceKind kind,
  uint64_t id
) {
  const auto found = std::find_if(
    partners.begin(),
    partners.end(),
    [kind, id](const WindowTracePartner& entry) {
      return entry.kind == kind && entry.id == id;
    }
  );

  if (found != partners.end()) {
    return *found;
  }

  WindowTracePartner made;

  made.kind = kind;
  made.id = id;
  partners.push_back(made);

  return partners.back();
}

std::vector<WindowTracePartner> traceDifference(
  std::span<const uint64_t> from,
  std::span<const uint64_t> without
) {
  std::vector<uint64_t> kept;

  std::set_difference(
    from.begin(),
    from.end(),
    without.begin(),
    without.end(),
    std::back_inserter(kept)
  );

  std::vector<WindowTracePartner> decoded;

  decoded.reserve(kept.size());

  for (const uint64_t code : kept) {
    decoded.push_back(traceDecode(code));
  }

  return decoded;
}

double partnerKinetic(const WindowPartner& partner, const PartnerFrame& frame) {
  return 0.5 * frame.mass
           * glm::dot(partner.linearVelocity, partner.linearVelocity)
         + 0.5
             * glm::dot(
               partner.angularVelocity,
               frame.inertia * partner.angularVelocity
             );
}

double mixedKinetic(
  double mass,
  const glm::dmat3& inertia,
  const glm::dvec3& linearBefore,
  const glm::dvec3& angularBefore,
  const glm::dvec3& linearAfter,
  const glm::dvec3& angularAfter
) {
  return 0.5 * mass * glm::dot(linearBefore, linearAfter)
         + 0.5 * glm::dot(angularBefore, inertia * angularAfter);
}

struct ContactTransition {
  size_t member = 0u;
  uint32_t node = 0u;
  size_t partner = 0u;
  glm::dvec3 normal{0.0};
  glm::dvec3 arm{0.0};
  double energy = 0.0;
  uint32_t second = 0u;
  double along = 0.0;
  size_t target = std::numeric_limits<size_t>::max();
  std::array<uint32_t, 8> nodes{};
  std::array<double, 8> shape{};
  uint64_t from = 0u;
  size_t event = std::numeric_limits<size_t>::max();
};

struct SupportTransition {
  size_t partner = 0u;
  glm::dvec3 normal{0.0};
  glm::dvec3 arm{0.0};
  double energy = 0.0;
  size_t holder = std::numeric_limits<size_t>::max();
  glm::dvec3 lever{0.0};
};

double transitionImpulse(double quadratic, double linear, double constant) {
  if (constant == 0.0) {
    return 0.0;
  }

  const double discriminant = linear * linear - 4.0 * quadratic * constant;

  if (discriminant < 0.0) {
    return quadratic > 0.0 ? -linear / (2.0 * quadratic) : 0.0;
  }

  const double root = std::sqrt(discriminant);
  const double q = -0.5 * (linear + (linear < 0.0 ? -root : root));

  return q != 0.0 ? constant / q : 0.0;
}

glm::dvec3 unitOf(const glm::dvec3& gradient, const glm::dvec3& fallback) {
  const double length = glm::length(gradient);

  return length > 0.0 ? gradient / length : fallback;
}

ShapeAt partnerAt(const WindowPartner& partner) {
  return ShapeAt{glm::vec3(0.0f), glm::quat(partner.rotation)};
}

float fieldFriction(
  const WindowPartner& partner,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::vec3& relative
) {
  return laneMaterial(
           materials,
           talus::field::material(
             *partner.probe.field,
             fieldPointOf(partner.probe, partnerAt(partner), relative)
           )
  )
    .friction;
}

struct CarriedExcess {
  double held = 0.0;
  double released = 0.0;
};

CarriedExcess carriedExcess(
  bool first,
  double ended,
  double now,
  double before,
  double dt
) {
  if (first) {
    return CarriedExcess{std::max(0.0, ended - now), 0.0};
  }

  const double kept = std::max(0.0, before - WINDOW_ONSET_SPEED * dt);

  return CarriedExcess{kept, before - kept};
}

double releaseWork(double before, double now, double released, bool rectangle) {
  return rectangle ? now * released : 0.5 * (before + now) * released;
}

double rampForce(double stiffness, double width, double pressed) {
  if (!(pressed > 0.0)) {
    return 0.0;
  }

  return pressed <= width ? stiffness * pressed * pressed / (2.0 * width)
                          : stiffness * (pressed - 0.5 * width);
}

double heldOffset(
  ImpactWindow& window,
  bool opening,
  uint32_t node,
  uint32_t partner,
  uint32_t supported,
  double distance
) {
  if (opening) {
    if (!(distance < 0.0)) {
      return 0.0;
    }

    window.offsets.push_back(WindowOffset{node, partner, distance, supported});
    ++window.openingInside;

    return distance;
  }

  if (window.offsets.empty()) {
    return 0.0;
  }

  const auto kept = std::find_if(
    window.offsets.begin(),
    window.offsets.end(),
    [&](const WindowOffset& entry) {
      return entry.node == node && entry.partner == partner
             && entry.supported == supported;
    }
  );

  if (kept == window.offsets.end()) {
    return 0.0;
  }

  if (distance >= 0.0) {
    *kept = window.offsets.back();
    window.offsets.pop_back();

    return 0.0;
  }

  return kept->offset;
}

SupportStep supportPartners(
  WindowGroup& group,
  std::span<const PartnerSupport> supports,
  std::span<PartnerFrame> frames,
  std::span<std::vector<SupportLoad>> loads,
  std::span<const talus::PhysicalMaterial> materials,
  double dt,
  bool first,
  bool transitions,
  std::vector<SupportTransition>& changes,
  std::vector<WindowTraceContact>* traced
) {
  SupportStep step;

  for (size_t index = 0u; index != group.partners.size(); ++index) {
    WindowPartner& partner = group.partners[index];
    const PartnerSupport& support = supports[index];

    if (!partner.dynamic || support.friction.empty()) {
      continue;
    }

    PartnerFrame& frame = frames[index];
    const glm::dmat3 turn = glm::mat3_cast(partner.rotation);
    const size_t seeds = partner.seeds.size();

    std::erase_if(
      partner.supportTracks,
      [&group, seeds](const WindowContactTrack& track) {
        return track.depth.size() != seeds
               || std::none_of(
                 group.partners.begin(),
                 group.partners.end(),
                 [&track](const WindowPartner& other) {
                   return (!other.dynamic || other.massive)
                          && other.probe.field != nullptr
                          && other.body == track.body;
                 }
               );
      }
    );

    for (const WindowPartner& fixed : group.partners) {
      if (
        (fixed.dynamic && !(fixed.massive && !partner.massive))
        || fixed.probe.field == nullptr
      ) {
        continue;
      }

      const size_t holder =
        fixed.massive ? static_cast<size_t>(&fixed - group.partners.data())
                      : std::numeric_limits<size_t>::max();

      auto tracked = std::find_if(
        partner.supportTracks.begin(),
        partner.supportTracks.end(),
        [&fixed](const WindowContactTrack& track) {
          return track.body == fixed.body;
        }
      );
      const bool fresh = tracked == partner.supportTracks.end();

      if (fresh) {
        WindowContactTrack made;

        made.body = fixed.body;
        made.depth.assign(seeds, 0.0);
        made.push.assign(seeds, glm::dvec3(0.0));
        made.offset.assign(seeds, 0.0);
        made.endDepth.assign(seeds, 0.0);
        made.excess.assign(seeds, 0.0);
        partner.supportTracks.push_back(std::move(made));
        tracked = partner.supportTracks.end() - 1;
      }

      WindowContactTrack& track = *tracked;
      const bool absorbing = !group.skipPushAbsorption && !fresh
                             && track.endDepth.size() == seeds
                             && track.excess.size() == seeds;
      const bool opening = !group.skipSupportOpening && !group.members.empty();

      for (size_t seed = 0u; seed != seeds; ++seed) {
        const glm::dvec3 arm = turn * glm::dvec3(partner.seeds[seed]);
        const glm::dvec3 relative =
          partner.centerOfMass + arm - fixed.centerOfMass;
        const PreciseDistance probed =
          probeShapePrecise(fixed.probe, fixed.rotation, relative);

        if (opening && group.supportOffsetOnFirstWindow) {
          track.offset[seed] = heldOffset(
            *group.members.front().window,
            fresh,
            static_cast<uint32_t>(seed),
            fixed.body,
            partner.body,
            probed.distance
          );
        } else if (opening && fresh && probed.distance < 0.0) {
          track.offset[seed] = probed.distance;
          ++group.members.front().window->openingInside;
        } else if (opening && !fresh && !(probed.distance < 0.0)) {
          track.offset[seed] = 0.0;
        }

        const double offset = opening ? track.offset[seed] : 0.0;

        double absorbed = 0.0;
        double released = 0.0;

        if (absorbing) {
          const CarriedExcess kept = carriedExcess(
            first,
            track.endDepth[seed],
            probed.distance - offset,
            track.excess[seed],
            dt
          );

          absorbed = kept.held;
          released = kept.released;
          track.excess[seed] = absorbed;

          if (first) {
            step.excessAbsorbed = std::max(step.excessAbsorbed, absorbed);
          }
        }

        const double distance = probed.distance - offset + absorbed;
        const double previous = track.depth[seed];
        const glm::dvec3 previousPush = track.push[seed];
        const double pressedBefore = support.stiffness * -previous;
        const glm::dvec3 moving =
          fixed.massive
            ? partner.linearVelocity + glm::cross(partner.angularVelocity, arm)
                - fixed.linearVelocity
                - glm::cross(fixed.angularVelocity, relative)
            : partner.linearVelocity + glm::cross(partner.angularVelocity, arm);
        const bool tracking = transitions && !fresh;

        track.depth[seed] = std::min(distance, 0.0);
        track.push[seed] = glm::dvec3(0.0);

        if (tracking && previous < 0.0 && !(distance < 0.0)) {
          changes.push_back(
            SupportTransition{
              index,
              unitOf(previousPush, probed.normal),
              arm,
              0.5 * glm::dot(previousPush, dt * moving)
                - 0.5 * support.stiffness * previous * previous,
              holder,
              relative
            }
          );
        }

        if (!(distance < 0.0)) {
          step.correction += releaseWork(
            pressedBefore,
            0.0,
            released,
            group.rectangleReleaseBooking
          );

          continue;
        }

        const glm::dvec3 normal = unitOf(probed.gradient, probed.normal);
        const double pressed = support.stiffness * -distance;

        step.penalty += 0.5 * support.stiffness * distance * distance;
        step.carriedPenalty +=
          fresh ? 0.0 : 0.5 * support.stiffness * distance * distance;
        step.correction += releaseWork(
          pressedBefore,
          pressed,
          released,
          group.rectangleReleaseBooking
        );

        if (traced != nullptr) {
          traceContact(
            *traced,
            WindowTraceKind::Support,
            partner.body,
            -distance,
            0.5 * support.stiffness * distance * distance
          );
        }

        glm::dvec3 applied = pressed * probed.gradient;

        track.push[seed] = applied;

        if (tracking && !(previous < 0.0)) {
          changes.push_back(
            SupportTransition{
              index,
              normal,
              arm,
              0.5 * glm::dot(applied, dt * moving)
                + 0.5 * support.stiffness * distance * distance,
              holder,
              relative
            }
          );
        }
        const glm::dvec3 tangential =
          moving - glm::dot(moving, normal) * normal;
        const double slide = glm::length(tangential);

        if (slide > 0.0) {
          const double mu = static_cast<double>(std::sqrt(
            support.friction[seed]
            * fieldFriction(fixed, materials, glm::vec3(relative))
          ));
          const double impulse = std::min(
            mu * pressed * glm::length(probed.gradient) * dt,
            support.mass * slide
          );
          const glm::dvec3 drag = -(impulse / (dt * slide)) * tangential;

          applied += drag;
          frame.drag += drag;
          frame.dragTorque += glm::cross(arm, drag);

          if (fixed.massive) {
            frames[holder].drag -= drag;
            frames[holder].dragTorque -= glm::cross(relative, drag);
          }
        }

        frame.force += applied;
        frame.torque += glm::cross(arm, applied);

        if (fixed.massive) {
          frames[holder].force -= applied;
          frames[holder].torque -= glm::cross(relative, applied);
          loads[holder].push_back(
            SupportLoad{index, applied, -glm::cross(relative, applied)}
          );
        }
      }
    }
  }

  return step;
}

talus::PlasticPoint halfStored(const talus::PlasticPoint& point) {
  const glm::vec2 normal = glm::unpackHalf2x16(
    glm::packHalf2x16(glm::vec2(point.strain[0][0], point.strain[1][1]))
  );
  const glm::vec2 crossed = glm::unpackHalf2x16(
    glm::packHalf2x16(glm::vec2(point.strain[1][0], point.strain[2][0]))
  );
  const glm::vec2 last = glm::unpackHalf2x16(
    glm::packHalf2x16(glm::vec2(point.strain[2][1], point.equivalent))
  );

  talus::PlasticPoint stored;

  stored.strain = glm::mat3(0.0f);
  stored.strain[0][0] = normal.x;
  stored.strain[1][1] = normal.y;
  stored.strain[2][2] = -(normal.x + normal.y);
  stored.strain[1][0] = crossed.x;
  stored.strain[0][1] = crossed.x;
  stored.strain[2][0] = crossed.y;
  stored.strain[0][2] = crossed.y;
  stored.strain[2][1] = last.x;
  stored.strain[1][2] = last.x;
  stored.equivalent = last.y;

  return stored;
}

double internalForces(
  const talus::PreparedMesh& mesh,
  const MemberLaw& law,
  ImpactWindow& window,
  bool returnMapping,
  bool rotationDerivative,
  bool halfPlastic,
  double step,
  std::vector<glm::dvec3>& force,
  std::vector<glm::dvec3>& lossForce
) {
  std::fill(force.begin(), force.end(), glm::dvec3(0.0));
  std::fill(lossForce.begin(), lossForce.end(), glm::dvec3(0.0));

  double elastic = 0.0;

  std::array<glm::vec3, 8> deformed{};
  std::array<glm::vec3, 8> moving{};
  std::array<talus::PlasticPoint, 8> committed{};

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::PreparedHex& hex = mesh.hexes[element];
    const glm::dvec3 corner = window.position[hex.nodes[0]];

    for (size_t at = 0u; at != 8u; ++at) {
      deformed[at] = glm::vec3(window.position[hex.nodes[at]] - corner);
      committed[at] = window.plastic[8u * element + at];
    }

    if (law.model[element] == talus::ElasticModel::NeoHookean) {
      for (size_t at = 0u; at != 8u; ++at) {
        moving[at] = glm::vec3(window.velocity[hex.nodes[at]]);
      }

      const NeoHookeanHex cell = neoHookeanHex(
        deformed,
        law.rest[element],
        moving,
        hex,
        law.elastic[element],
        law.hysteresis[element],
        static_cast<float>(step)
      );

      for (size_t at = 0u; at != 8u; ++at) {
        force[hex.nodes[at]] += glm::dvec3(cell.force[at]);
        lossForce[hex.nodes[at]] += glm::dvec3(cell.lossForce[at]);
      }

      elastic += static_cast<double>(cell.energy);

      continue;
    }

    const HexPointTrial trial = hexPointTrial(
      deformed,
      law.rest[element],
      hex,
      law.elastic[element],
      law.plastic[element],
      committed,
      0.0f,
      ElementRotation::Polar,
      returnMapping,
      false,
      rotationDerivative
    );

    for (size_t at = 0u; at != 8u; ++at) {
      force[hex.nodes[at]] += glm::dvec3(trial.force[at]);

      if (returnMapping) {
        window.plastic[8u * element + at] =
          halfPlastic ? halfStored(trial.points[at]) : trial.points[at];
      }
    }

    elastic += static_cast<double>(trial.energy);
  }

  return elastic;
}

bool finiteVector(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y)
         && std::isfinite(value.z);
}

constexpr double BOTH_WAYS_SHARE = 0.5;
constexpr double SLIP_FLOOR = 1.0e-12;
constexpr size_t NO_INDEX = std::numeric_limits<size_t>::max();

constexpr double FLAT_SHARE = 1.0e-3;

struct PairProbe {
  size_t prober = 0u;
  size_t member = NO_INDEX;
  size_t partner = NO_INDEX;
  double share = 1.0;
};

glm::dvec3 tangentialOf(const glm::dvec3& value, const glm::dvec3& normal) {
  return value - glm::dot(value, normal) * normal;
}

double rampEnergy(double stiffness, double width, double pressed) {
  if (pressed <= width) {
    return stiffness * pressed * pressed * pressed / (6.0 * width);
  }

  return stiffness
         * (0.5 * pressed * pressed - 0.5 * width * pressed + width * width / 6.0);
}

bool beforeTrack(const WindowProbeTrack& left, const WindowProbeTrack& right) {
  return left.partner != right.partner ? left.partner < right.partner
                                       : left.point < right.point;
}

bool beforeReaction(const WindowReaction& left, const WindowReaction& right) {
  return left.from != right.from ? left.from < right.from
                                 : left.node < right.node;
}

const WindowProbeTrack* findTrack(
  std::span<const WindowProbeTrack> tracks,
  uint64_t partner,
  uint32_t point
) {
  WindowProbeTrack wanted;

  wanted.partner = partner;
  wanted.point = point;

  const auto found =
    std::lower_bound(tracks.begin(), tracks.end(), wanted, beforeTrack);

  return found != tracks.end() && found->partner == partner
             && found->point == point
           ? &*found
           : nullptr;
}

}

std::vector<ProbePoint> probePoints(
  const talus::PreparedMesh& mesh,
  bool nodesOnly
) {
  std::vector<ProbePoint> points;

  for (const uint32_t node : mesh.surfaceNodes) {
    points.push_back(ProbePoint{node, node, 0.0, 1.0});
  }

  if (nodesOnly) {
    return points;
  }

  struct Side {
    glm::uvec2 edge{0u};
    uint32_t triangle = 0u;
  };

  std::vector<Side> sides;

  sides.reserve(3u * mesh.surfaceTriangles.size());

  for (size_t face = 0u; face != mesh.surfaceTriangles.size(); ++face) {
    const glm::uvec3& corners = mesh.surfaceTriangles[face];

    for (int corner = 0; corner != 3; ++corner) {
      const uint32_t from = corners[corner];
      const uint32_t to = corners[(corner + 1) % 3];

      sides.push_back(
        Side{
          glm::uvec2(std::min(from, to), std::max(from, to)),
          static_cast<uint32_t>(face)
        }
      );
    }
  }

  std::sort(
    sides.begin(),
    sides.end(),
    [](const Side& left, const Side& right) {
      return left.edge.x != right.edge.x   ? left.edge.x < right.edge.x
             : left.edge.y != right.edge.y ? left.edge.y < right.edge.y
                                           : left.triangle < right.triangle;
    }
  );

  const auto restOf = [&mesh](uint32_t node) {
    return glm::dvec3(mesh.restPositions[node]);
  };

  for (size_t at = 0u; at != sides.size();) {
    size_t next = at + 1u;

    while (next != sides.size() && sides[next].edge == sides[at].edge) {
      ++next;
    }

    if (next - at == 2u) {
      const glm::uvec2 edge = sides[at].edge;
      const glm::uvec3& one = mesh.surfaceTriangles[sides[at].triangle];
      const glm::uvec3& two = mesh.surfaceTriangles[sides[at + 1u].triangle];
      const glm::dvec3 facing = glm::cross(
        restOf(one.y) - restOf(one.x),
        restOf(one.z) - restOf(one.x)
      );
      const double area = glm::length(facing);

      uint32_t opposite = two.x;

      for (int corner = 0; corner != 3; ++corner) {
        if (two[corner] != edge.x && two[corner] != edge.y) {
          opposite = two[corner];
        }
      }

      const double span = glm::length(restOf(edge.y) - restOf(edge.x));

      if (
        area > 0.0
        && glm::dot(facing / area, restOf(opposite) - restOf(edge.x))
             < -FLAT_SHARE * span
      ) {
        for (uint32_t step = 1u; step != WINDOW_EDGE_POINTS; ++step) {
          points.push_back(
            ProbePoint{
              edge.x,
              edge.y,
              static_cast<double>(step)
                / static_cast<double>(WINDOW_EDGE_POINTS),
              1.0 / static_cast<double>(WINDOW_EDGE_POINTS)
            }
          );
        }
      }
    }

    at = next;
  }

  return points;
}

namespace {

glm::dvec3 pointOf(const ProbePoint& spot, std::span<const glm::dvec3> values) {
  return (1.0 - spot.along) * values[spot.first]
         + spot.along * values[spot.second];
}

double pointReach(const ProbePoint& spot, std::span<const double> mass) {
  const double kept = 1.0 - spot.along;

  return kept * kept / mass[spot.first]
         + spot.along * spot.along / mass[spot.second];
}

void pushPoint(
  const ProbePoint& spot,
  const glm::dvec3& force,
  std::span<glm::dvec3> target
) {
  target[spot.first] += (1.0 - spot.along) * force;
  target[spot.second] += spot.along * force;
}

double rigidReach(
  double inverseMass,
  const glm::dmat3& inverse,
  const glm::dvec3& arm,
  const glm::dvec3& direction
) {
  const glm::dvec3 lever = glm::cross(arm, direction);

  return inverseMass + glm::dot(lever, inverse * lever);
}

void mergeReactions(std::vector<WindowReaction>& reactions) {
  std::sort(reactions.begin(), reactions.end(), beforeReaction);

  size_t kept = 0u;

  for (size_t at = 0u; at != reactions.size(); ++at) {
    if (
      kept != 0u && reactions[kept - 1u].from == reactions[at].from
      && reactions[kept - 1u].node == reactions[at].node
    ) {
      reactions[kept - 1u].force += reactions[at].force;
    } else {
      reactions[kept++] = reactions[at];
    }
  }

  reactions.resize(kept);
}

std::array<double, 8> cellWeights(
  const WindowGroup& group,
  const DeformedDistance& probed
) {
  return group.clampBandWeights
           ? hexShape(
               glm::clamp(probed.local, glm::dvec3(0.0), glm::dvec3(1.0))
             )
           : probed.weights;
}

void placeOnTarget(
  WindowProbeTrack& track,
  const WindowGroup& group,
  const DeformedDistance& probed,
  bool windowTarget,
  const glm::dvec3& placed,
  const WindowPartner* partner
) {
  if (windowTarget) {
    track.nodes = probed.nodes;
    track.shape = cellWeights(group, probed);
  } else {
    track.arm = placed - partner->centerOfMass;
  }
}

glm::dvec3 shapeVelocity(
  const std::array<uint32_t, 8>& nodes,
  const std::array<double, 8>& shape,
  std::span<const glm::dvec3> velocity
) {
  glm::dvec3 blended(0.0);

  for (size_t corner = 0u; corner != 8u; ++corner) {
    blended += shape[corner] * velocity[nodes[corner]];
  }

  return blended;
}

double shapeReach(
  const std::array<uint32_t, 8>& nodes,
  const std::array<double, 8>& shape,
  std::span<const double> mass
) {
  double reach = 0.0;

  for (size_t corner = 0u; corner != 8u; ++corner) {
    reach += shape[corner] * shape[corner] / mass[nodes[corner]];
  }

  return reach;
}

bool boxesApart(
  const glm::dvec3& lowA,
  const glm::dvec3& highA,
  const glm::dvec3& lowB,
  const glm::dvec3& highB
) {
  return glm::any(glm::lessThan(highA, lowB))
         || glm::any(glm::lessThan(highB, lowA));
}

struct ExitApproach {
  bool quiet = true;
  double largest = -std::numeric_limits<double>::infinity();
  uint32_t probes = 0u;
};

glm::dvec3 rigidSpin(const Rigid& rigid) {
  return glm::determinant(rigid.inertia) > 0.0
           ? glm::inverse(rigid.inertia) * rigid.angular
           : glm::dvec3(0.0);
}

ExitApproach approachQuiet(
  const WindowGroup& group,
  size_t at,
  std::span<const MemberLaw> laws,
  std::span<const ProbePoint> points,
  std::span<const std::optional<DeformedProbe>> memberProbes,
  std::span<const std::optional<DeformedProbe>> partnerProbes
) {
  ExitApproach result;

  const WindowMember& member = group.members[at];
  const ImpactWindow& window = *member.window;
  const Rigid rigid = rigidOf(laws[at].mass, window);

  if (!(rigid.mass > 0.0)) {
    return result;
  }

  const glm::dvec3 moving = rigid.linear / rigid.mass;
  const glm::dvec3 spin = rigidSpin(rigid);
  const double reach = static_cast<double>(member.mesh->voxelSize.x);
  const double trigger = static_cast<double>(member.trigger);

  const auto note = [&result, trigger](double approach) {
    result.largest = std::max(result.largest, approach);
    ++result.probes;
    result.quiet = result.quiet && !(approach > trigger);
  };

  for (size_t target = 0u; target != group.members.size(); ++target) {
    if (
      group.exitBlindToWindows || target == at || member.field == nullptr
      || group.members[target].field == nullptr
      || !memberProbes[target].has_value()
    ) {
      continue;
    }

    const ImpactWindow& other = *group.members[target].window;
    const Rigid otherRigid = rigidOf(laws[target].mass, other);
    const glm::dvec3 otherMoving = otherRigid.mass > 0.0
                                     ? otherRigid.linear / otherRigid.mass
                                     : glm::dvec3(0.0);
    const glm::dvec3 otherSpin = rigidSpin(otherRigid);

    for (const ProbePoint& spot : points) {
      const glm::dvec3 offset = pointOf(spot, window.position);
      const glm::dvec3 placed = window.origin + offset;
      const DeformedDistance probed =
        probeDeformed(*memberProbes[target], placed);

      if (!probed.found || probed.distance > reach) {
        continue;
      }

      const glm::dvec3 surface =
        otherMoving
        + glm::cross(otherSpin, placed - other.origin - otherRigid.centre);
      const glm::dvec3 rigidVelocity =
        moving + glm::cross(spin, offset - rigid.centre);

      note(glm::dot(probed.normal, surface - rigidVelocity));
    }
  }

  for (const uint32_t index : member.partners) {
    const WindowPartner& partner = group.partners[index];

    if (partner.probe.field == nullptr) {
      continue;
    }

    if (partner.mesh != nullptr) {
      if (!partnerProbes[index].has_value()) {
        continue;
      }

      const glm::dmat3 turn = glm::mat3_cast(partner.rotation);
      const glm::dmat3 back = glm::transpose(turn);
      const glm::dvec3 fieldOffset(partner.probe.fieldOffset);

      for (const ProbePoint& spot : points) {
        const glm::dvec3 offset = pointOf(spot, window.position);
        const glm::dvec3 relative =
          window.origin + offset - partner.centerOfMass;
        const DeformedDistance probed =
          probeDeformed(*partnerProbes[index], back * relative + fieldOffset);

        if (!probed.found || probed.distance > reach) {
          continue;
        }

        const glm::dvec3 surface =
          partner.linearVelocity
          + glm::cross(partner.angularVelocity, relative);
        const glm::dvec3 rigidVelocity =
          moving + glm::cross(spin, offset - rigid.centre);

        note(glm::dot(turn * probed.normal, surface - rigidVelocity));
      }

      continue;
    }

    for (size_t node = 0u; result.quiet && node != window.position.size();
         ++node) {
      const glm::dvec3 world = window.origin + window.position[node];
      const glm::dvec3 relative = world - partner.centerOfMass;
      const PreciseDistance probed =
        probeShapePrecise(partner.probe, partner.rotation, relative);

      if (probed.distance > reach) {
        continue;
      }

      const glm::dvec3 rigidVelocity =
        moving + glm::cross(spin, window.position[node] - rigid.centre);
      const glm::dvec3 surface =
        partner.linearVelocity + glm::cross(partner.angularVelocity, relative);
      const double approach = glm::dot(probed.normal, surface - rigidVelocity);

      if (approach > trigger) {
        result.quiet = false;
      }
    }
  }

  return result;
}
}

double supportStiffness(double mass, double pace, const Settings& settings) {
  return settings.windowPenaltyScale * PENALTY_SHARE * mass * pace * pace;
}

void endSupportDepths(WindowGroup& group) {
  for (WindowPartner& carried : group.partners) {
    const size_t seeds = carried.seeds.size();

    if (!carried.dynamic || seeds == 0u) {
      continue;
    }

    const glm::dmat3 carriedTurn = glm::mat3_cast(carried.rotation);

    for (WindowContactTrack& track : carried.supportTracks) {
      if (
        track.endDepth.size() != seeds || track.excess.size() != seeds
        || track.offset.size() != seeds
      ) {
        continue;
      }

      const auto fixed = std::find_if(
        group.partners.begin(),
        group.partners.end(),
        [&track](const WindowPartner& candidate) {
          return (!candidate.dynamic || candidate.massive)
                 && candidate.probe.field != nullptr
                 && candidate.body == track.body;
        }
      );

      if (fixed == group.partners.end()) {
        continue;
      }

      for (size_t seed = 0u; seed != seeds; ++seed) {
        const PreciseDistance ending = probeShapePrecise(
          fixed->probe,
          fixed->rotation,
          carried.centerOfMass + carriedTurn * glm::dvec3(carried.seeds[seed])
            - fixed->centerOfMass
        );
        const double offsetHeld =
          !group.skipSupportOpening && !group.members.empty()
            ? track.offset[seed]
            : 0.0;

        track.endDepth[seed] =
          std::min(ending.distance - offsetHeld + track.excess[seed], 0.0);
      }
    }
  }
}

WindowLaw windowLaw(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  double frameTime,
  const Settings& settings
) {
  WindowLaw law;

  for (const talus::PreparedHex& hex : mesh.hexes) {
    law.wave =
      std::max(law.wave, waveSpeed(laneMaterial(materials, hex.material)));
  }

  for (const talus::PreparedTet& tet : mesh.tets) {
    law.wave =
      std::max(law.wave, waveSpeed(laneMaterial(materials, tet.material)));
  }

  law.spacing = static_cast<double>(mesh.voxelSize.x);

  const double critical = law.spacing / law.wave;

  law.stepLimit = settings.windowStepShare * critical;
  law.referenceStep = REFERENCE_SHARE * critical;

  const double ratio = frameTime / law.stepLimit;

  law.substeps = std::isfinite(ratio) && ratio > 1.0
                   ? static_cast<uint32_t>(std::ceil(ratio))
                   : 1u;
  law.step = frameTime / static_cast<double>(law.substeps);

  const double pace = 2.0 / law.referenceStep;

  law.penalty.resize(mesh.nodeMass.size());

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    law.penalty[node] = settings.windowPenaltyScale * PENALTY_SHARE
                        * static_cast<double>(mesh.nodeMass[node]) * pace
                        * pace;
  }

  return law;
}

ImpactWindow openWindow(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& start,
  const glm::dvec3& centre
) {
  ImpactWindow window;

  const size_t nodes = mesh.restPositions.size();

  window.origin = centre;
  window.position.resize(nodes);
  window.velocity.resize(nodes);
  window.friction.assign(nodes, glm::dvec3(0.0));
  window.surface.assign(nodes, glm::dvec3(0.0));

  for (size_t node = 0u; node != nodes; ++node) {
    window.position[node] = glm::dvec3(start.position[node]) - centre;
    window.velocity[node] = node < start.velocity.size()
                              ? glm::dvec3(start.velocity[node])
                              : glm::dvec3(0.0);
  }

  const size_t points = 8u * mesh.hexes.size();

  if (start.plastic.size() == points) {
    window.plastic = start.plastic;
  } else {
    window.plastic.assign(points, talus::PlasticPoint{});
  }

  return window;
}

std::vector<WindowStep> stepWindows(
  WindowGroup& group,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::dvec3& gravity,
  double frameTime,
  const Settings& settings
) {
  const size_t count = group.members.size();
  const bool tracing = settings.reportWindowTrace;

  std::vector<WindowStep> steps(count);

  if (count == 0u) {
    return steps;
  }

  std::vector<MemberLaw> laws;
  laws.reserve(count);

  uint32_t substeps = 1u;

  for (const WindowMember& member : group.members) {
    laws.push_back(memberLaw(*member.mesh, materials, frameTime, settings));
    substeps = std::max(substeps, laws.back().law.substeps);
  }

  double gradientBound = 0.0;

  for (const WindowMember& member : group.members) {
    gradientBound = std::max(gradientBound, member.window->gradientBound);
  }

  const double shrinkMargin = group.shrinkWithoutMargin ? 0.0 : 1.0e-6;
  const double stepScale =
    !group.skipShrink && gradientBound > 1.0 + shrinkMargin
      ? std::sqrt(gradientBound)
      : 1.0;

  if (stepScale > 1.0) {
    substeps = static_cast<uint32_t>(
      std::ceil(static_cast<double>(substeps) * stepScale)
    );
  }

  const uint32_t shrunk = substeps;

  for (const WindowMember& member : group.members) {
    if (!group.skipStepHold && member.window->heldFrameTime == frameTime) {
      substeps = std::max(substeps, member.window->heldSubsteps);
    }
  }

  const double dt = frameTime / static_cast<double>(substeps);

  double referenceStep = std::numeric_limits<double>::infinity();

  for (const MemberLaw& held : laws) {
    referenceStep = std::min(referenceStep, held.law.referenceStep);
  }

  const double pace = 2.0 / referenceStep;

  std::vector<PartnerFrame> frames(group.partners.size());
  std::vector<PartnerSupport> supports(group.partners.size());
  std::vector<std::vector<SupportLoad>> loads(group.partners.size());

  for (size_t index = 0u; index != group.partners.size(); ++index) {
    WindowPartner& partner = group.partners[index];
    PartnerFrame& frame = frames[index];

    partner.deltaLinear = glm::dvec3(0.0);
    partner.deltaAngular = glm::dvec3(0.0);
    partner.touched = false;

    partner.supportPairs.clear();

    if (!partner.dynamic || !(partner.inverseMass > 0.0)) {
      continue;
    }

    frame.mass = 1.0 / partner.inverseMass;
    frame.inertia = glm::determinant(partner.inverseInertia) != 0.0
                      ? glm::inverse(partner.inverseInertia)
                      : glm::dmat3(0.0);

    const glm::dmat3 turn = glm::mat3_cast(partner.rotation);

    frame.bodyInverse = glm::transpose(turn) * partner.inverseInertia * turn;
    frame.bodyInertia = glm::transpose(turn) * frame.inertia * turn;
    frame.momentum = frame.inertia * partner.angularVelocity;
    frame.startCentre = partner.centerOfMass - dt * partner.linearVelocity;
    frame.base =
      mixedKinetic(
        frame.mass,
        frame.inertia,
        partner.linearVelocity
          - dt * (gravity + partner.inverseMass * partner.previousLoad.force),
        partner.angularVelocity
          - dt * (partner.inverseInertia * partner.previousLoad.torque),
        partner.linearVelocity,
        partner.angularVelocity
      )
      - frame.mass * glm::dot(gravity, frame.startCentre);
    frame.now = frame.base;

    if (partner.seeds.empty() || partner.probe.field == nullptr) {
      continue;
    }

    PartnerSupport& support = supports[index];

    support.mass = frame.mass / static_cast<double>(partner.seeds.size());
    support.stiffness = supportStiffness(support.mass, pace, settings);
    support.friction.reserve(partner.seeds.size());

    for (const glm::vec3& seed : partner.seeds) {
      support.friction.push_back(laneMaterial(
                                   materials,
                                   talus::field::material(
                                     *partner.probe.field,
                                     seed + partner.probe.fieldOffset
                                   )
      )
                                   .friction);
    }
  }

  std::vector<ImpactWindow> saved;
  saved.reserve(count);

  for (const WindowMember& member : group.members) {
    saved.push_back(*member.window);
  }

  const std::vector<WindowPartner> savedPartners = group.partners;

  struct LostProbe {
    uint64_t partner = 0u;
    uint32_t point = 0u;
    bool found = false;
    double distance = 0.0;
  };

  struct Scratch {
    std::vector<WindowReleaseEvent> releaseEvents;
    std::vector<LostProbe> lost;
    uint32_t pairUnfound = 0u;
    uint32_t pairReleases = 0u;
    double pairReleaseWork = 0.0;
    glm::dvec3 pairReleaseImpulse{0.0};
    std::vector<glm::dvec3> force;
    std::vector<glm::dvec3> loss;
    double hysteresis = 0.0;
    std::vector<glm::dvec3> friction;
    std::vector<glm::dvec3> surface;
    std::vector<uint8_t> contacting;
    std::vector<uint8_t> touched;
    double partnersCarried = 0.0;
    double partnerGravityCarried = 0.0;
    double pending = 0.0;
    double plasticStart = 0.0;
    glm::dvec3 angularStart{0.0};
    double penalty = 0.0;
    double elastic = 0.0;
    double gravity = 0.0;
    double kinetic = 0.0;
    bool contact = false;
    bool finite = true;
    std::vector<glm::dvec3> before;
    std::vector<WindowReaction> reactions;
    std::vector<WindowReaction> kicks;
    std::vector<WindowProbeTrack> tracks;
    std::vector<WindowProbeTrack> leaving;
    std::vector<uint64_t> touchedKeys;
    DeformedCounts counts;
    double probeStiffness = 0.0;
    uint32_t probeContacts = 0u;
    double gradientMax = 0.0;
    uint32_t engagedUnconverged = 0u;
    double tangential = 0.0;
    double tangentialStiffness = 0.0;
    uint32_t tangentialChecks = 0u;
    double dissipationLeast = std::numeric_limits<double>::infinity();
    double capRatio = 0.0;
    uint32_t stuckSubsteps = 0u;
    uint32_t reversals = 0u;
    double largestSlip = 0.0;
    uint32_t engagedNotFound = 0u;
    std::vector<WindowPairStep> pairSteps;
    double cappedSlip = 0.0;
    uint32_t cappedCount = 0u;
    std::vector<uint64_t> storedKeys;
    bool probing = false;
    bool priorProbing = false;
    std::vector<glm::dvec3> partnerForce;
    std::vector<glm::dvec3> partnerTorque;
    std::vector<glm::dvec3> priorForce;
    std::vector<glm::dvec3> priorTorque;
    double partnerWork = 0.0;
    double excessAbsorbed = 0.0;
    double correction = 0.0;
    double correctionWork = 0.0;
    double nodeCorrection = 0.0;
    double stepChange = 0.0;
    uint32_t stepChanges = 0u;
  };

  std::vector<Scratch> scratch(count);

  for (size_t at = 0u; at != count; ++at) {
    const WindowMember& member = group.members[at];
    ImpactWindow& window = *member.window;
    Scratch& held = scratch[at];

    const size_t nodes = window.position.size();

    held.force.assign(nodes, glm::dvec3(0.0));
    held.loss.assign(nodes, glm::dvec3(0.0));
    held.friction.assign(nodes, glm::dvec3(0.0));
    held.surface.assign(nodes, glm::dvec3(0.0));
    held.contacting.assign(nodes, 0u);
    held.touched.assign(group.partners.size(), 0u);
    held.before.assign(nodes, glm::dvec3(0.0));

    held.priorProbing = !window.reactions.empty();
    held.partnersCarried = window.ledger.partners;
    held.partnerGravityCarried = window.partnerGravity;
    held.plasticStart =
      window.started
        ? window.ledger.plastic
        : windowPlasticWork(*member.mesh, materials, window.plastic);
    held.angularStart = rigidOf(laws[at].mass, window).angular;
    held.partnerForce.assign(group.partners.size(), glm::dvec3(0.0));
    held.partnerTorque.assign(group.partners.size(), glm::dvec3(0.0));
    held.priorForce.assign(group.partners.size(), glm::dvec3(0.0));
    held.priorTorque.assign(group.partners.size(), glm::dvec3(0.0));

    for (const uint32_t index : member.partners) {
      for (const WindowPartnerLoad& carried : window.partnerLoads) {
        if (carried.body == group.partners[index].body) {
          held.priorForce[index] = carried.force;
          held.priorTorque[index] = carried.torque;
        }
      }
    }

    if (at != 0u) {
      continue;
    }

    for (size_t index = 0u; index != group.partners.size(); ++index) {
      const WindowPartner& partner = group.partners[index];

      if (
        !partner.dynamic
        || std::find(
             window.partnersSeen.begin(),
             window.partnersSeen.end(),
             partner.body
           ) != window.partnersSeen.end()
      ) {
        continue;
      }

      window.partnerEntries.resize(window.partnersSeen.size(), 0.0);
      window.partnersSeen.push_back(partner.body);
      window.partnerEntries.push_back(partnerKinetic(partner, frames[index]));
      window.partnerEntry += window.partnerEntries.back();
    }
  }

  std::vector<uint32_t> keys;
  keys.reserve(count);

  for (const WindowMember& member : group.members) {
    keys.push_back(member.key);
  }

  std::sort(keys.begin(), keys.end());

  const bool continued = std::all_of(
    group.members.begin(),
    group.members.end(),
    [&keys](const WindowMember& member) {
      return member.window->started && member.window->groupKeys == keys;
    }
  );
  double groupMinimum = continued ? group.members.front().window->groupMinimum
                                  : std::numeric_limits<double>::infinity();
  double groupRise = continued ? group.members.front().window->groupRise : 0.0;

  const size_t traceCount = tracing ? count : size_t{0};

  std::vector<std::vector<WindowTraceContact>> traceContacts(traceCount);
  std::vector<WindowLedger> traceBefore(traceCount);
  std::vector<uint32_t> traceOpenings(traceCount, 0u);
  std::vector<uint32_t> traceDropped(traceCount, 0u);
  std::vector<uint32_t> traceTransitions(traceCount, 0u);

  for (size_t tracedAt = 0u; tracedAt != traceCount; ++tracedAt) {
    const WindowMember& tracedMember = group.members[tracedAt];
    WindowTrace& opened = steps[tracedAt].trace;

    opened.groupChange = tracedMember.window->started && !continued;
    opened.creationStart = tracedMember.window->stepCreation;
    traceBefore[tracedAt] = tracedMember.window->ledger;

    for (const uint32_t tracedIndex : tracedMember.partners) {
      const WindowPartner& tracedPartner = group.partners[tracedIndex];

      tracePartner(
        opened.partners,
        partnerTraceKind(tracedPartner),
        partnerTraceId(tracedPartner)
      );
    }

    for (size_t tracedOther = 0u; tracedOther != count; ++tracedOther) {
      if (tracedOther != tracedAt) {
        tracePartner(
          opened.partners,
          WindowTraceKind::Window,
          group.members[tracedOther].key
        );
      }
    }
  }

  bool failed = false;
  double groupWork = 0.0;
  const bool groupSupport = !group.supportOnFirstWindow;
  double supportBaseline = 0.0;
  double supportPenalty = 0.0;
  double supportWork = 0.0;
  double supportDissipated = 0.0;
  double supportReceived = 0.0;
  double supportPending = 0.0;
  double supportTotal = 0.0;
  double supportMinimum = 0.0;
  double supportRise = 0.0;
  std::vector<std::array<double, 8>> supportSamples;
  std::vector<ContactTransition> transitions;
  std::vector<SupportTransition> supportChanges;

  std::vector<size_t> order(count);

  std::iota(order.begin(), order.end(), size_t{0});
  std::sort(order.begin(), order.end(), [&group](size_t left, size_t right) {
    return group.members[left].key < group.members[right].key;
  });

  std::vector<PairProbe> pairs;

  for (const size_t prober : order) {
    const WindowMember& probing = group.members[prober];

    for (const size_t target : order) {
      const WindowMember& probed = group.members[target];

      if (
        target == prober || probing.field == nullptr || probed.field == nullptr
      ) {
        continue;
      }

      if (
        group.oneWayProbing
        && !(probing.mesh->voxelSize.x < probed.mesh->voxelSize.x)
      ) {
        continue;
      }

      pairs.push_back(
        PairProbe{
          prober,
          target,
          NO_INDEX,
          group.oneWayProbing ? 1.0 : BOTH_WAYS_SHARE
        }
      );
    }

    std::vector<uint32_t> stored;

    for (const uint32_t index : probing.partners) {
      if (
        group.partners[index].mesh != nullptr
        && group.partners[index].probe.field != nullptr
      ) {
        stored.push_back(index);
      }
    }

    std::sort(
      stored.begin(),
      stored.end(),
      [&group](uint32_t left, uint32_t right) {
        return group.partners[left].key < group.partners[right].key;
      }
    );

    for (const uint32_t index : stored) {
      pairs.push_back(PairProbe{prober, NO_INDEX, index, 1.0});
    }
  }

  std::vector<std::optional<DeformedProbe>> memberProbes(count);
  std::vector<std::optional<DeformedProbe>> partnerProbes(
    group.partners.size()
  );

  std::vector<std::vector<ProbePoint>> memberPoints(count);

  for (const PairProbe& pair : pairs) {
    if (pair.member != NO_INDEX && !memberProbes[pair.member].has_value()) {
      const WindowMember& probed = group.members[pair.member];

      memberProbes[pair.member] = deformedProbe(*probed.mesh, *probed.field);
    }

    if (pair.partner != NO_INDEX && !partnerProbes[pair.partner].has_value()) {
      const WindowPartner& probed = group.partners[pair.partner];

      partnerProbes[pair.partner] =
        deformedProbe(*probed.mesh, *probed.probe.field);
      placeDeformedProbe(*partnerProbes[pair.partner], probed.shape);
    }

    if (memberPoints[pair.prober].empty()) {
      memberPoints[pair.prober] =
        probePoints(*group.members[pair.prober].mesh, group.nodeOnlyProbing);
    }
  }

  std::vector<glm::dvec3> lowOf(count);
  std::vector<glm::dvec3> highOf(count);
  std::vector<uint8_t> placedProbe(count, 0u);
  std::vector<glm::dvec3> worldScratch;

  const auto memberOfKey = [&group, count](uint64_t key) {
    for (size_t at = 0u; at != count; ++at) {
      if (windowPartnerKey(group.members[at].key) == key) {
        return at;
      }
    }

    return NO_INDEX;
  };

  const auto traceKindOfKey = [&group, &memberOfKey](uint64_t key) {
    if (memberOfKey(key) != NO_INDEX) {
      return WindowTraceKind::Window;
    }

    for (const WindowPartner& candidate : group.partners) {
      if (candidate.mesh != nullptr && candidate.key == key) {
        return partnerTraceKind(candidate);
      }
    }

    return WindowTraceKind::Stored;
  };

  for (uint32_t substep = 0u; substep != substeps && !failed; ++substep) {
    transitions.clear();
    supportChanges.clear();

    for (PartnerFrame& frame : frames) {
      frame.force = glm::dvec3(0.0);
      frame.torque = glm::dvec3(0.0);
      frame.drag = glm::dvec3(0.0);
      frame.dragTorque = glm::dvec3(0.0);
    }

    for (Scratch& held : scratch) {
      std::fill(
        held.partnerForce.begin(),
        held.partnerForce.end(),
        glm::dvec3(0.0)
      );
      std::fill(
        held.partnerTorque.begin(),
        held.partnerTorque.end(),
        glm::dvec3(0.0)
      );
    }

    for (size_t at = 0u; at != count; ++at) {
      const WindowMember& member = group.members[at];
      const talus::PreparedMesh& mesh = *member.mesh;
      const MemberLaw& law = laws[at];
      ImpactWindow& window = *member.window;
      Scratch& held = scratch[at];

      const size_t nodes = window.position.size();

      held.elastic = internalForces(
        mesh,
        law,
        window,
        true,
        settings.windowRotationDerivative,
        group.halfPlastic,
        dt,
        held.force,
        held.loss
      );

      held.gravity = 0.0;

      for (size_t node = 0u; node != nodes; ++node) {
        held.force[node] += law.mass[node] * gravity;
        held.gravity -=
          law.mass[node] * glm::dot(gravity, window.position[node]);
        held.friction[node] = glm::dvec3(0.0);
        held.surface[node] = glm::dvec3(0.0);
        held.contacting[node] = 0u;
      }

      held.penalty = 0.0;
      held.contact = false;
      held.nodeCorrection = 0.0;

      for (const uint32_t index : member.partners) {
        WindowPartner& partner = group.partners[index];
        PartnerFrame& frame = frames[index];

        if (partner.probe.field == nullptr || partner.mesh != nullptr) {
          continue;
        }

        auto tracked = std::find_if(
          window.contactTracks.begin(),
          window.contactTracks.end(),
          [&partner](const WindowContactTrack& track) {
            return track.body == partner.body;
          }
        );

        const bool nodeExcess = !group.skipPushAbsorption
                                && !group.skipExcessExtension
                                && partner.dynamic && window.started
                                && (substep != 0u
                                    || tracked != window.contactTracks.end());

        if (tracked == window.contactTracks.end()) {
          WindowContactTrack fresh;

          fresh.body = partner.body;
          fresh.depth.assign(nodes, 0.0);
          fresh.push.assign(nodes, glm::dvec3(0.0));
          window.contactTracks.push_back(std::move(fresh));
          tracked = window.contactTracks.end() - 1;
        }

        WindowContactTrack& track = *tracked;

        if (nodeExcess) {
          track.endDepth.resize(nodes, 0.0);
          track.excess.resize(nodes, 0.0);
        }

        for (size_t node = 0u; node != nodes; ++node) {
          if (
            !mesh.surfaceSeeds.empty() && node < mesh.restSurfaceDistance.size()
            && mesh.restSurfaceDistance[node] != 0.0f
          ) {
            continue;
          }

          const glm::dvec3 world = window.origin + window.position[node];
          const glm::dvec3 relative = world - partner.centerOfMass;
          const PreciseDistance probed =
            probeShapePrecise(partner.probe, partner.rotation, relative);
          const double distance = probed.distance;

          const size_t offsetsBefore = window.offsets.size();
          const double offset = heldOffset(
            window,
            !window.started,
            static_cast<uint32_t>(node),
            partner.body,
            NO_WINDOW_BODY,
            distance
          );

          if (tracing && window.offsets.size() > offsetsBefore) {
            ++traceOpenings[at];
          }

          if (tracing && window.offsets.size() < offsetsBefore) {
            ++traceDropped[at];
          }

          double nodeAbsorbed = 0.0;
          double nodeReleased = 0.0;

          if (nodeExcess) {
            const CarriedExcess kept = carriedExcess(
              substep == 0u,
              track.endDepth[node],
              distance - offset,
              track.excess[node],
              dt
            );

            nodeAbsorbed = kept.held;
            nodeReleased = kept.released;
            track.excess[node] = nodeAbsorbed;

            if (substep == 0u) {
              held.excessAbsorbed = std::max(held.excessAbsorbed, nodeAbsorbed);
            }
          }

          const double depth = distance - offset + nodeAbsorbed;
          const double previous = track.depth[node];
          const glm::dvec3 previousPush = track.push[node];
          const double pressedBefore = law.law.penalty[node] * -previous;
          const glm::dvec3 surface =
            partner.linearVelocity
            + glm::cross(partner.angularVelocity, relative);
          const glm::dvec3 moved = dt * (window.velocity[node] - surface);

          track.depth[node] = std::min(depth, 0.0);
          track.push[node] = glm::dvec3(0.0);

          const bool tracking =
            window.started && settings.windowContactTransitions;

          if (tracking && previous < 0.0 && !(depth < 0.0)) {
            transitions.push_back(
              ContactTransition{
                at,
                static_cast<uint32_t>(node),
                index,
                unitOf(previousPush, probed.normal),
                relative,
                0.5 * glm::dot(previousPush, moved)
                  - 0.5 * law.law.penalty[node] * previous * previous,
                static_cast<uint32_t>(node),
                0.0,
                std::numeric_limits<size_t>::max(),
                {},
                {},
                0u
              }
            );
          }

          if (!(depth < 0.0)) {
            held.nodeCorrection += releaseWork(
              pressedBefore,
              0.0,
              nodeReleased,
              group.rectangleReleaseBooking
            );

            continue;
          }

          held.contact = true;
          held.touched[index] = 1u;
          partner.touched = true;

          held.surface[node] = surface;
          held.contacting[node] = 1u;

          const glm::dvec3 normal = unitOf(probed.gradient, probed.normal);
          const double pressed = law.law.penalty[node] * -depth;

          held.penalty += 0.5 * law.law.penalty[node] * depth * depth;
          held.nodeCorrection += releaseWork(
            pressedBefore,
            pressed,
            nodeReleased,
            group.rectangleReleaseBooking
          );

          if (tracing) {
            traceContact(
              traceContacts[at],
              partnerTraceKind(partner),
              partnerTraceId(partner),
              -depth,
              0.5 * law.law.penalty[node] * depth * depth
            );
          }

          const glm::dvec3 push = pressed * probed.gradient;

          held.force[node] += push;
          track.push[node] = push;

          if (tracking && !(previous < 0.0)) {
            transitions.push_back(
              ContactTransition{
                at,
                static_cast<uint32_t>(node),
                index,
                normal,
                relative,
                0.5 * glm::dot(push, moved)
                  + 0.5 * law.law.penalty[node] * depth * depth,
                static_cast<uint32_t>(node),
                0.0,
                std::numeric_limits<size_t>::max(),
                {},
                {},
                0u
              }
            );
          }

          glm::dvec3 drag(0.0);

          if (window.started) {
            const glm::dvec3 trial =
              window.velocity[node] + dt * held.force[node] / law.mass[node];
            const glm::dvec3 slip = trial - surface;
            const glm::dvec3 tangential =
              slip - glm::dot(slip, normal) * normal;
            const double slide = glm::length(tangential);

            if (slide > 0.0) {
              const float laneFriction =
                laneMaterial(materials, law.nodeLane[node]).friction;
              const double mu = static_cast<double>(std::sqrt(
                laneFriction
                * fieldFriction(partner, materials, glm::vec3(relative))
              ));
              const double impulse =
                std::min(mu * glm::length(push) * dt, law.mass[node] * slide);

              drag = -(impulse / (dt * slide)) * tangential;
              held.friction[node] += drag;
            }
          }

          if (partner.dynamic) {
            const glm::dvec3 reaction = -(push + drag);

            frame.force += reaction;
            frame.torque += glm::cross(relative, reaction);
            held.partnerForce[index] += reaction;
            held.partnerTorque[index] += glm::cross(relative, reaction);
          }
        }

        for (size_t seed = 0u; seed != mesh.surfaceSeeds.size(); ++seed) {
          const talus::PreparedHex& hex =
            mesh.hexes[mesh.surfaceSeedHexes[seed]];
          const std::array<float, 8>& shape = mesh.surfaceSeedShapes[seed];

          glm::dvec3 world = window.origin;
          glm::dvec3 moving(0.0);
          double inverse = 0.0;

          for (uint32_t corner = 0u; corner != 8u; ++corner) {
            const uint32_t node = hex.nodes[corner];
            const double weight = static_cast<double>(shape[corner]);

            world += weight * window.position[node];
            moving += weight * window.velocity[node];
            inverse += weight * weight / law.mass[node];
          }

          const glm::dvec3 relative = world - partner.centerOfMass;
          const PreciseDistance probed =
            probeShapePrecise(partner.probe, partner.rotation, relative);
          const double depth = probed.distance;

          if (!(depth < 0.0) || !(inverse > 0.0)) {
            continue;
          }

          const double mass = 1.0 / inverse;
          const double stiffness = law.law.penalty[hex.nodes[0]]
                                   / law.mass[hex.nodes[0]]
                                   * law.seedShare[seed];
          const glm::dvec3 normal = unitOf(probed.gradient, probed.normal);
          const glm::dvec3 push = stiffness * -depth * probed.gradient;
          const glm::dvec3 surface =
            partner.linearVelocity
            + glm::cross(partner.angularVelocity, relative);

          held.penalty += 0.5 * stiffness * depth * depth;
          held.contact = true;
          held.touched[index] = 1u;
          partner.touched = true;

          glm::dvec3 drag(0.0);

          if (window.started) {
            const glm::dvec3 slip = moving + dt * push / mass - surface;
            const glm::dvec3 tangential =
              slip - glm::dot(slip, normal) * normal;
            const double slide = glm::length(tangential);

            if (slide > 0.0) {
              const float laneFriction =
                laneMaterial(materials, hex.material).friction;
              const double mu = static_cast<double>(std::sqrt(
                laneFriction
                * fieldFriction(partner, materials, glm::vec3(relative))
              ));
              const double impulse =
                std::min(mu * glm::length(push) * dt, mass * slide);

              drag = -(impulse / (dt * slide)) * tangential;
            }
          }

          for (uint32_t corner = 0u; corner != 8u; ++corner) {
            const uint32_t node = hex.nodes[corner];
            const double weight = static_cast<double>(shape[corner]);

            held.force[node] += weight * push;
            held.friction[node] += weight * drag;

            if (weight > 0.0) {
              held.surface[node] = surface;
              held.contacting[node] = 1u;
            }
          }

          if (partner.dynamic) {
            const glm::dvec3 reaction = -(push + drag);

            frame.force += reaction;
            frame.torque += glm::cross(relative, reaction);
            held.partnerForce[index] += reaction;
            held.partnerTorque[index] += glm::cross(relative, reaction);
          }
        }
      }

      std::erase_if(window.contactTracks, [&](const WindowContactTrack& track) {
        return std::none_of(
          member.partners.begin(),
          member.partners.end(),
          [&](uint32_t index) {
            return group.partners[index].body == track.body
                   && group.partners[index].probe.field != nullptr
                   && group.partners[index].mesh == nullptr;
          }
        );
      });
    }

    for (std::vector<SupportLoad>& partnerLoads : loads) {
      partnerLoads.clear();
    }

    const SupportStep supported = supportPartners(
      group,
      supports,
      frames,
      loads,
      materials,
      dt,
      substep == 0u,
      settings.windowContactTransitions,
      supportChanges,
      tracing ? &traceContacts.front() : nullptr
    );

    for (Scratch& held : scratch) {
      held.pending = 0.0;
    }

    supportPending = 0.0;

    if (substep == 0u) {
      supportBaseline = supported.carriedPenalty;
    }

    supportPenalty = supported.penalty;
    supportReceived += supported.correction;

    for (Scratch& held : scratch) {
      held.tracks.clear();
      held.lost.clear();
      held.leaving.clear();
      held.reactions.clear();
      held.kicks.clear();
      held.probeStiffness = 0.0;
      held.probeContacts = 0u;
      held.tangential = 0.0;
      held.tangentialStiffness = 0.0;
      held.probing = false;
      held.cappedSlip = 0.0;
      held.cappedCount = 0u;
      held.correction = held.nodeCorrection;
    }

    if (!scratch.empty()) {
      scratch.front().excessAbsorbed =
        std::max(scratch.front().excessAbsorbed, supported.excessAbsorbed);

      if (!groupSupport) {
        scratch.front().correction += supported.correction;
      }
    }

    if (!pairs.empty()) {
      std::fill(placedProbe.begin(), placedProbe.end(), uint8_t{0u});

      for (size_t at = 0u; at != count; ++at) {
        const ImpactWindow& window = *group.members[at].window;

        lowOf[at] = glm::dvec3(std::numeric_limits<double>::infinity());
        highOf[at] = glm::dvec3(-std::numeric_limits<double>::infinity());

        for (const glm::dvec3& at3 : window.position) {
          lowOf[at] = glm::min(lowOf[at], window.origin + at3);
          highOf[at] = glm::max(highOf[at], window.origin + at3);
        }
      }

      const double fieldRate =
        settings.windowPenaltyScale * PENALTY_SHARE * pace * pace;

      for (const PairProbe& pair : pairs) {
        const size_t prober = pair.prober;
        const WindowMember& member = group.members[prober];
        ImpactWindow& window = *member.window;
        Scratch& held = scratch[prober];
        const MemberLaw& law = laws[prober];
        const bool windowTarget = pair.member != NO_INDEX;
        const uint64_t proberKey = windowPartnerKey(member.key);

        const DeformedProbe* target = nullptr;
        WindowPartner* partner = nullptr;
        PartnerFrame* frame = nullptr;
        ImpactWindow* other = nullptr;
        Scratch* otherHeld = nullptr;
        const MemberLaw* otherLaw = nullptr;
        uint64_t key = 0u;
        glm::dmat3 turn(1.0);
        glm::dvec3 fieldOffset(0.0);

        if (windowTarget) {
          if (group.windowPassOncePerFrame && substep != 0u) {
            continue;
          }

          DeformedProbe& probe = *memberProbes[pair.member];

          if (
            boxesApart(
              lowOf[prober],
              highOf[prober],
              lowOf[pair.member] - glm::dvec3(probe.pad),
              highOf[pair.member] + glm::dvec3(probe.pad)
            )
          ) {
            continue;
          }

          other = group.members[pair.member].window;
          otherHeld = &scratch[pair.member];
          otherLaw = &laws[pair.member];

          if (placedProbe[pair.member] == 0u) {
            worldScratch.resize(other->position.size());

            for (size_t node = 0u; node != other->position.size(); ++node) {
              worldScratch[node] =
                group.gridFromRest
                  ? glm::dvec3(
                      group.members[pair.member].mesh->restPositions[node]
                    )
                  : other->origin + other->position[node];
            }

            placeDeformedProbe(probe, worldScratch);
            placedProbe[pair.member] = 1u;
          }

          target = &probe;
          key = windowPartnerKey(group.members[pair.member].key);
        } else {
          partner = &group.partners[pair.partner];
          frame = &frames[pair.partner];
          target = &*partnerProbes[pair.partner];
          key = partner->key;
          turn = glm::mat3_cast(partner->rotation);
          fieldOffset = glm::dvec3(partner->probe.fieldOffset);
        }

        const glm::dmat3 back = glm::transpose(turn);
        const std::vector<ProbePoint>& points = memberPoints[prober];
        const bool extended =
          !group.skipPushAbsorption && !group.skipExcessExtension
          && !windowTarget && partner->dynamic && window.started
          && std::binary_search(
            window.probedKeys.begin(),
            window.probedKeys.end(),
            key
          );

        WindowPairStep pairStep;

        pairStep.partner = key;

        bool pairEngaged = false;

        for (size_t sample = 0u; sample != points.size(); ++sample) {
          const ProbePoint& spot = points[sample];
          const uint32_t pointKey = static_cast<uint32_t>(sample);
          const glm::dvec3 placed =
            window.origin + pointOf(spot, window.position);
          const glm::dvec3 local =
            windowTarget
              ? placed
              : back * (placed - partner->centerOfMass) + fieldOffset;

          DeformedCounts queried;

          const DeformedDistance probed =
            probeDeformed(*target, local, &queried);

          held.counts.unconverged += queried.unconverged;
          held.counts.inverted += queried.inverted;

          if (!probed.found) {
            const WindowProbeTrack* lost =
              findTrack(window.probeTracks, key, pointKey);

            held.engagedNotFound +=
              lost != nullptr && lost->depth < 0.0 ? 1u : 0u;

            if (group.traceReleases && lost != nullptr) {
              held.lost.push_back(LostProbe{key, pointKey, false, 0.0});
            }

            continue;
          }

          if (!(probed.distance < 0.0)) {
            if (group.traceReleases) {
              held.lost.push_back(
                LostProbe{key, pointKey, true, probed.distance}
              );
            }

            const WindowProbeTrack* left =
              findTrack(window.probeTracks, key, pointKey);

            if (left != nullptr && left->depth < 0.0) {
              WindowProbeTrack cell;

              cell.partner = key;
              cell.point = pointKey;
              placeOnTarget(cell, group, probed, windowTarget, placed, partner);
              held.leaving.push_back(cell);
            }

            continue;
          }

          const WindowProbeTrack* previous =
            findTrack(window.probeTracks, key, pointKey);
          const double distance = probed.distance;

          double offset = 0.0;

          if (
            !window.started
            || (substep == 0u && previous == nullptr && !extended)
          ) {
            offset = distance;
            ++window.openingProbes;

            if (tracing) {
              ++traceOpenings[prober];
            }
            window.openingOffset = std::min(window.openingOffset, distance);
          } else if (previous != nullptr) {
            offset = previous->offset;
          }

          const bool excessTrack =
            !group.skipPushAbsorption && !windowTarget && partner->dynamic
            && (previous != nullptr || (extended && substep == 0u));

          double excess = 0.0;
          double excessReleased = 0.0;

          if (excessTrack) {
            const CarriedExcess kept = carriedExcess(
              substep == 0u,
              previous != nullptr ? previous->endDepth : 0.0,
              distance - offset,
              previous != nullptr ? previous->excess : 0.0,
              dt
            );

            excess = kept.held;
            excessReleased = kept.released;

            if (substep == 0u) {
              held.excessAbsorbed = std::max(held.excessAbsorbed, excess);
            }
          }

          const double depth = std::min(distance - offset + excess, 0.0);
          const double forceBefore = excessTrack && previous != nullptr
                                       ? rampForce(
                                           previous->stiffness,
                                           previous->width,
                                           -previous->depth
                                         )
                                       : 0.0;
          const glm::dvec3 gradient = turn * probed.gradient;
          const glm::dvec3 normal = turn * probed.normal;

          WindowProbeTrack track;

          track.partner = key;
          track.point = pointKey;
          track.depth = depth;
          track.offset = offset;
          track.first = spot.first;
          track.second = spot.second;
          track.along = spot.along;
          track.excess = excess;

          if (previous != nullptr) {
            track.stuck = previous->stuck;
            track.reversals = previous->reversals;
          }

          placeOnTarget(track, group, probed, windowTarget, placed, partner);

          if (!(depth < 0.0)) {
            if (excessTrack && substep != 0u) {
              held.correction += releaseWork(
                forceBefore,
                0.0,
                excessReleased,
                group.rectangleReleaseBooking
              );
            }

            held.tracks.push_back(track);

            continue;
          }

          const double inverseProber = pointReach(spot, law.mass);
          const bool movingPartner =
            !windowTarget && partner->dynamic && partner->inverseMass > 0.0;

          double inverseTarget = 0.0;

          if (windowTarget) {
            inverseTarget =
              shapeReach(track.nodes, track.shape, otherLaw->mass);
          } else if (movingPartner) {
            inverseTarget = rigidReach(
              partner->inverseMass,
              turn * frame->bodyInverse * back,
              track.arm,
              normal
            );
          }

          const bool engaged = previous != nullptr && previous->depth < 0.0;
          const double stiffness = engaged ? previous->stiffness
                                   : group.proberMassStiffness && windowTarget
                                     ? fieldRate * spot.length / inverseProber
                                     : fieldRate * pair.share * spot.length
                                         / (inverseProber + inverseTarget);

          double width = 0.0;

          if (engaged) {
            width = previous->width;
            track.approach = previous->approach;
          } else {
            glm::dvec3 approaching = pointOf(spot, window.velocity);

            if (windowTarget) {
              approaching -=
                shapeVelocity(track.nodes, track.shape, other->velocity);
            } else if (partner->dynamic) {
              approaching -= partner->linearVelocity
                             + glm::cross(partner->angularVelocity, track.arm);
            }

            const double approach =
              -glm::dot(unitOf(gradient, normal), approaching);

            width = group.zeroOnsetWidth
                      ? 0.0
                      : WINDOW_ONSET_STEPS * referenceStep
                          * std::max(approach, WINDOW_ONSET_SPEED);
            track.approach = approach;
          }

          const double pressed = -depth;

          double onsetForce = stiffness * (pressed - 0.5 * width);
          const double onsetEnergy = rampEnergy(stiffness, width, pressed);

          if (pressed <= width) {
            onsetForce = stiffness * pressed * pressed / (2.0 * width);
          }

          if (excessTrack && substep != 0u) {
            const double released = releaseWork(
              forceBefore,
              onsetForce,
              excessReleased,
              group.rectangleReleaseBooking
            );

            held.correction += released;

            if (tracing) {
              traceWork(
                traceContacts[prober],
                partnerTraceKind(*partner),
                partnerTraceId(*partner),
                -released
              );
            }
          }

          const glm::dvec3 push = onsetForce * gradient;

          track.stiffness = stiffness;
          track.width = width;
          track.push = push;

          for (WindowReleaseEvent& event : held.releaseEvents) {
            if (
              group.traceReleases && previous == nullptr
              && event.againSubstep < 0 && event.partner == key
              && event.point == pointKey
            ) {
              event.againSubstep = static_cast<int32_t>(substep);
              event.againDistance = distance;
              event.againOffset = offset;
              event.againForce = onsetForce;
              event.againStiffness = stiffness;
              event.againWidth = width;
            }
          }

          pairEngaged = true;
          pairStep.deepest = std::max(pairStep.deepest, pressed);
          pairStep.stiffness += stiffness;
          pairStep.load += glm::length(push);
          pairStep.width = std::max(pairStep.width, width);
          pairStep.approach = std::max(pairStep.approach, track.approach);

          held.penalty += onsetEnergy;

          if (group.penaltyOnBoth && windowTarget) {
            otherHeld->penalty += onsetEnergy;
          }

          if (tracing && windowTarget) {
            traceContact(
              traceContacts[prober],
              WindowTraceKind::Window,
              group.members[pair.member].key,
              pressed,
              onsetEnergy
            );
            traceContact(
              traceContacts[pair.member],
              WindowTraceKind::Window,
              member.key,
              pressed,
              group.penaltyOnBoth ? onsetEnergy : 0.0
            );
          } else if (tracing) {
            traceContact(
              traceContacts[prober],
              partnerTraceKind(*partner),
              partnerTraceId(*partner),
              pressed,
              onsetEnergy
            );
          }

          held.probeStiffness += stiffness;
          ++held.probeContacts;
          held.gradientMax = std::max(
            held.gradientMax,
            stiffness * (inverseProber + inverseTarget)
              * glm::dot(gradient, gradient) / fieldRate
          );
          held.engagedUnconverged += queried.unconverged != 0u ? 1u : 0u;
          held.contact = true;
          held.probing = true;
          if (
            std::find(held.touchedKeys.begin(), held.touchedKeys.end(), key)
            == held.touchedKeys.end()
          ) {
            held.touchedKeys.push_back(key);
          }

          glm::dvec3 tangentialForce(0.0);

          if (window.started) {
            const uint32_t lane =
              law.nodeLane[spot.along < 0.5 ? spot.first : spot.second];
            const float mine = laneMaterial(materials, lane).friction;
            const float theirs = windowTarget ? laneMaterial(
                                                  materials,
                                                  group.members[pair.member]
                                                    .mesh->hexes[probed.element]
                                                    .material
                                                )
                                                  .friction
                                              : laneMaterial(
                                                  materials,
                                                  talus::field::material(
                                                    *partner->probe.field,
                                                    glm::vec3(probed.rest)
                                                  )
                                                )
                                                  .friction;
            const double mu = static_cast<double>(std::sqrt(mine * theirs));
            const glm::dvec3 facing = unitOf(gradient, normal);

            glm::dvec3 partnerVelocity(0.0);

            if (windowTarget) {
              partnerVelocity =
                shapeVelocity(track.nodes, track.shape, other->velocity);
            } else if (partner->dynamic) {
              partnerVelocity =
                partner->linearVelocity
                + glm::cross(partner->angularVelocity, track.arm);
            }

            const double executedStep = substep == 0u ? window.step : dt;
            const double tangent =
              width > 0.0 ? stiffness * std::min(pressed / width, 1.0)
                          : stiffness;
            const glm::dvec3 kept =
              engaged ? std::sqrt(previous->tangentStiffness / tangent)
                          * previous->stretch
                      : glm::dvec3(0.0);
            const double lost = glm::dot(kept, facing);
            const glm::dvec3 projected = kept - lost * facing;
            const glm::dvec3 sliding = tangentialOf(
              pointOf(spot, window.velocity) - partnerVelocity,
              facing
            );
            const glm::dvec3 executed = executedStep * sliding;
            const glm::dvec3 trialStretch = projected + executed;
            const double trialLength = glm::length(trialStretch);
            const double cap = mu * glm::length(push);
            const double loss = 0.5 * tangent * lost * lost;
            const bool stuck = tangent * trialLength <= cap;

            glm::dvec3 stretch = trialStretch;
            double slipped = 0.0;

            if (!stuck && !group.skipReturnMap) {
              const double returnShare = cap / (tangent * trialLength);

              stretch = returnShare * trialStretch;
              slipped = tangent * trialLength * trialLength * returnShare
                        * (1.0 - returnShare);
            }

            const double dissipated =
              group.dissipationOnTotal
                ? tangent * glm::dot(stretch, executed) + loss
                : slipped + loss;

            tangentialForce = -tangent * stretch;

            if (group.reflectStuckSlip && stuck) {
              tangentialForce = -(2.0 * pair.share * spot.length
                                  / ((inverseProber + inverseTarget) * dt))
                                * sliding;
            }

            track.stretch = stretch;
            track.tangentStiffness = tangent;
            track.executed = executed;
            track.capped = !stuck;
            track.cappedBefore = engaged && previous->capped;

            const bool holding = engaged && stuck && !previous->capped;

            if (holding) {
              ++track.stuck;
              ++held.stuckSubsteps;
              track.reversed = glm::dot(executed, previous->executed) < 0.0
                               && glm::length(previous->executed)
                                    > SLIP_FLOOR * window.largestSlip;
            }

            if (!stuck) {
              held.cappedSlip += glm::length(trialStretch - stretch);
              ++held.cappedCount;
            }

            window.ledger.friction += dissipated;
            held.tangential += 0.5 * tangent * glm::dot(stretch, stretch);
            held.tangentialStiffness += tangent;
            ++held.tangentialChecks;
            held.dissipationLeast = std::min(held.dissipationLeast, dissipated);

            if (cap > 0.0) {
              held.capRatio =
                std::max(held.capRatio, glm::length(tangentialForce) / cap);
            }
          }

          const glm::dvec3 applied = push + tangentialForce;

          pushPoint(spot, applied, held.force);

          if (windowTarget) {
            otherHeld->contact = true;
            otherHeld->probing = true;
            if (
              std::find(
                otherHeld->touchedKeys.begin(),
                otherHeld->touchedKeys.end(),
                proberKey
              )
              == otherHeld->touchedKeys.end()
            ) {
              otherHeld->touchedKeys.push_back(proberKey);
            }

            size_t heaviest = 0u;

            for (size_t corner = 1u; group.lumpReaction && corner != 8u;
                 ++corner) {
              if (track.shape[corner] > track.shape[heaviest]) {
                heaviest = corner;
              }
            }

            for (size_t corner = 0u; !group.omitWindowReaction && corner != 8u;
                 ++corner) {
              const double weight = group.lumpReaction
                                      ? (corner == heaviest ? 1.0 : 0.0)
                                      : track.shape[corner];
              const glm::dvec3 reacted = -weight * applied;

              otherHeld->force[track.nodes[corner]] += reacted;
              otherHeld->reactions.push_back(
                WindowReaction{proberKey, track.nodes[corner], reacted}
              );
            }
          } else {
            held.touched[pair.partner] = 1u;
            partner->touched = true;

            if (
              std::find(held.storedKeys.begin(), held.storedKeys.end(), key)
              == held.storedKeys.end()
            ) {
              held.storedKeys.push_back(key);
            }

            if (movingPartner) {
              frame->force -= applied;
              frame->torque += glm::cross(track.arm, -applied);
              held.partnerForce[pair.partner] -= applied;
              held.partnerTorque[pair.partner] +=
                glm::cross(track.arm, -applied);
            }
          }

          held.tracks.push_back(track);
        }

        if (!pairEngaged) {
          continue;
        }

        const auto recorded = std::find_if(
          held.pairSteps.begin(),
          held.pairSteps.end(),
          [key](const WindowPairStep& entry) { return entry.partner == key; }
        );

        pairStep.loadMost = pairStep.load;

        if (recorded == held.pairSteps.end()) {
          held.pairSteps.push_back(pairStep);
        } else {
          if (pairStep.deepest > recorded->deepest) {
            recorded->deepest = pairStep.deepest;
            recorded->stiffness = pairStep.stiffness;
            recorded->load = pairStep.load;
          }

          recorded->loadMost = std::max(recorded->loadMost, pairStep.load);

          recorded->width = std::max(recorded->width, pairStep.width);
          recorded->approach = std::max(recorded->approach, pairStep.approach);
        }
      }

      for (const size_t prober : order) {
        Scratch& held = scratch[prober];

        std::sort(held.tracks.begin(), held.tracks.end(), beforeTrack);
        std::sort(held.leaving.begin(), held.leaving.end(), beforeTrack);
      }

      double slipLargest = 0.0;

      for (const Scratch& held : scratch) {
        for (const WindowProbeTrack& track : held.tracks) {
          if (track.depth < 0.0) {
            slipLargest = std::max(slipLargest, glm::length(track.executed));
          }
        }
      }

      for (size_t at = 0u; at != count; ++at) {
        ImpactWindow& window = *group.members[at].window;
        Scratch& held = scratch[at];

        for (WindowProbeTrack& track : held.tracks) {
          if (!track.reversed) {
            continue;
          }

          if (glm::length(track.executed) > SLIP_FLOOR * slipLargest) {
            ++track.reversals;
            ++held.reversals;
          } else {
            track.reversed = false;
          }
        }

        window.largestSlip = slipLargest;
        held.largestSlip = std::max(held.largestSlip, slipLargest);
      }
    }

    for (size_t at = 0u; substep == 0u && at != count; ++at) {
      ImpactWindow& window = *group.members[at].window;
      const Scratch& held = scratch[at];

      double received = 0.0;

      for (const WindowProbeTrack& prior : window.probeTracks) {
        const WindowProbeTrack* current =
          findTrack(held.tracks, prior.partner, prior.point);
        const bool engagedNow = current != nullptr && current->depth < 0.0;
        const WindowProbeTrack* ramp =
          engagedNow ? current : (prior.depth < 0.0 ? &prior : nullptr);

        if (ramp == nullptr) {
          continue;
        }

        const double energyNow =
          engagedNow ? rampEnergy(ramp->stiffness, ramp->width, -current->depth)
                     : 0.0;
        const double energyEnded =
          prior.endDepth < 0.0
            ? rampEnergy(ramp->stiffness, ramp->width, -prior.endDepth)
            : 0.0;

        received += energyNow - energyEnded;

        if (tracing) {
          traceWork(
            traceContacts[at],
            traceKindOfKey(prior.partner),
            prior.partner & TRACE_ID_MASK,
            energyEnded - energyNow
          );
        }
      }

      window.ledger.transfer -= received;
      window.receivedWork += received;
    }

    for (size_t at = 0u; at != count; ++at) {
      Scratch& held = scratch[at];

      if (held.correction == 0.0) {
        continue;
      }

      ImpactWindow& window = *group.members[at].window;

      window.ledger.transfer -= held.correction;
      window.receivedWork += held.correction;
      held.correctionWork += held.correction;
    }

    for (size_t at = 0u; at != count; ++at) {
      ImpactWindow& window = *group.members[at].window;
      const Scratch& held = scratch[at];

      for (const WindowProbeTrack& prior : window.probeTracks) {
        if (!(prior.depth < 0.0)) {
          continue;
        }

        const WindowProbeTrack* current =
          findTrack(held.tracks, prior.partner, prior.point);

        if (current == nullptr || !(current->depth < 0.0)) {
          window.ledger.friction += 0.5 * prior.tangentStiffness
                                    * glm::dot(prior.stretch, prior.stretch);
        }
      }
    }

    for (size_t at = 0u; at != count; ++at) {
      const WindowMember& member = group.members[at];
      const ImpactWindow& window = *member.window;
      const Scratch& held = scratch[at];

      if (
        group.skipPairRelease || !window.started
        || !settings.windowContactTransitions
      ) {
        continue;
      }

      for (const WindowProbeTrack& prior : window.probeTracks) {
        if (!(prior.depth < 0.0) || prior.point >= memberPoints[at].size()) {
          continue;
        }

        if (
          !group.doubleSubstepZeroBooking && substep == 0u
          && prior.endDepth < 0.0
        ) {
          continue;
        }

        const WindowProbeTrack* current =
          findTrack(held.tracks, prior.partner, prior.point);

        if (
          (current != nullptr && current->depth < 0.0)
          || (group.skipUnfoundRelease && current == nullptr)
        ) {
          continue;
        }

        const ProbePoint& spot = memberPoints[at][prior.point];
        const size_t target = memberOfKey(prior.partner);

        size_t rigid = NO_INDEX;

        for (size_t index = 0u; target == NO_INDEX && index != group.partners.size();
             ++index) {
          if (
            group.partners[index].mesh != nullptr
            && group.partners[index].key == prior.partner
          ) {
            rigid = index;
          }
        }

        if (target == NO_INDEX && rigid == NO_INDEX) {
          continue;
        }

        const WindowProbeTrack* cell =
          current != nullptr
            ? current
            : findTrack(held.leaving, prior.partner, prior.point);
        const WindowProbeTrack& reacting = cell != nullptr ? *cell : prior;

        glm::dvec3 surface(0.0);

        if (target != NO_INDEX) {
          surface = shapeVelocity(
            reacting.nodes,
            reacting.shape,
            group.members[target].window->velocity
          );
        } else if (group.partners[rigid].dynamic) {
          surface =
            group.partners[rigid].linearVelocity
            + glm::cross(group.partners[rigid].angularVelocity, reacting.arm);
        }

        const glm::dvec3 moved = dt * (pointOf(spot, window.velocity) - surface);

        if (target != NO_INDEX && current == nullptr) {
          ++scratch[at].pairUnfound;
        }

        size_t event = NO_INDEX;

        if (group.traceReleases && target != NO_INDEX) {
          WindowReleaseEvent made;

          made.substep = substep;
          made.point = prior.point;
          made.first = spot.first;
          made.second = spot.second;
          made.along = spot.along;
          made.partner = prior.partner;
          made.current = current != nullptr;
          made.priorDepth = prior.depth;
          made.priorOffset = prior.offset;
          made.priorStiffness = prior.stiffness;
          made.priorWidth = prior.width;
          made.priorPush = prior.push;
          made.energy =
            0.5 * glm::dot(prior.push, moved)
            - rampEnergy(prior.stiffness, prior.width, -prior.depth);
          made.nodes = prior.nodes;
          made.shape = prior.shape;

          for (const LostProbe& gone : held.lost) {
            if (gone.partner == prior.partner && gone.point == prior.point) {
              made.probed = true;
              made.found = gone.found;
              made.distance = gone.distance;
            }
          }

          if (current != nullptr) {
            made.probed = true;
            made.found = true;
            made.distance = current->depth;
          }

          event = scratch[at].releaseEvents.size();
          scratch[at].releaseEvents.push_back(made);
        }

        transitions.push_back(
          ContactTransition{
            at,
            spot.first,
            rigid,
            unitOf(prior.push, glm::dvec3(0.0)),
            reacting.arm,
            0.5 * glm::dot(prior.push, moved)
              - rampEnergy(prior.stiffness, prior.width, -prior.depth),
            spot.second,
            spot.along,
            target,
            reacting.nodes,
            reacting.shape,
            windowPartnerKey(member.key),
            event
          }
        );
      }
    }

    for (const ContactTransition& change : transitions) {
      const MemberLaw& law = laws[change.member];
      const ImpactWindow& window = *group.members[change.member].window;
      Scratch& held = scratch[change.member];
      const ProbePoint spot{change.node, change.second, change.along, 1.0};
      const bool rigidTarget = change.partner != NO_INDEX;
      const bool memberTarget = change.target != NO_INDEX;
      const glm::dvec3 before = pointOf(spot, window.velocity);
      const glm::dvec3 after =
        before
        + (1.0 - spot.along) * dt
            * (held.force[spot.first] + held.friction[spot.first])
            / law.mass[spot.first]
        + spot.along * dt
            * (held.force[spot.second] + held.friction[spot.second])
            / law.mass[spot.second];

      PartnerFrame* frame = rigidTarget ? &frames[change.partner] : nullptr;
      glm::dvec3 pointBefore(0.0);
      glm::dvec3 pointAfter(0.0);
      double reach = pointReach(spot, law.mass);
      bool moving = false;

      if (rigidTarget) {
        const WindowPartner& partner = group.partners[change.partner];

        pointBefore = partner.linearVelocity
                      + glm::cross(partner.angularVelocity, change.arm);
        pointAfter = pointBefore;
        moving = partner.dynamic && partner.inverseMass > 0.0;

        if (moving) {
          const glm::dmat3 turn = glm::mat3_cast(partner.rotation);
          const glm::dmat3 inverse =
            turn * frame->bodyInverse * glm::transpose(turn);
          const glm::dvec3 lever = glm::cross(change.arm, change.normal);

          pointAfter = partner.linearVelocity
                       + dt * (partner.inverseMass * frame->force + gravity)
                       + glm::cross(
                         inverse * (frame->momentum + dt * frame->torque),
                         change.arm
                       );
          reach += partner.inverseMass + glm::dot(lever, inverse * lever);
        }
      } else if (memberTarget) {
        const ImpactWindow& other = *group.members[change.target].window;
        const Scratch& otherHeld = scratch[change.target];
        const MemberLaw& otherLaw = laws[change.target];

        pointBefore = shapeVelocity(change.nodes, change.shape, other.velocity);
        pointAfter = pointBefore;

        for (size_t corner = 0u; corner != 8u; ++corner) {
          const uint32_t cornerNode = change.nodes[corner];

          pointAfter += change.shape[corner] * dt
                        * (otherHeld.force[cornerNode]
                           + otherHeld.friction[cornerNode])
                        / otherLaw.mass[cornerNode];
        }

        reach += shapeReach(change.nodes, change.shape, otherLaw.mass);
      }

      const double closing = dt * glm::dot(change.normal, before - pointBefore);
      const double opening = dt * glm::dot(change.normal, after - pointAfter);
      const double lambda = transitionImpulse(
        0.5 * dt * dt * reach,
        0.5 * (closing + opening),
        change.energy
      );
      const glm::dvec3 kick = lambda * change.normal;
      const uint64_t source =
        memberTarget ? windowPartnerKey(group.members[change.target].key)
                     : group.partners[change.partner].key;

      pushPoint(spot, kick, held.force);
      held.kicks.push_back(
        WindowReaction{source, spot.first, (1.0 - spot.along) * kick}
      );
      held.kicks.push_back(
        WindowReaction{source, spot.second, spot.along * kick}
      );
      held.pending += 0.5 * lambda * (opening + dt * dt * reach * lambda);
      held.contact = true;

      if (memberTarget) {
        ++held.pairReleases;
        held.pairReleaseWork +=
          0.5 * lambda * (opening + dt * dt * reach * lambda);
        held.pairReleaseImpulse += dt * kick;
      }

      if (memberTarget && change.event != NO_INDEX) {
        WindowReleaseEvent& event = held.releaseEvents[change.event];
        const ImpactWindow& other = *group.members[change.target].window;
        const Scratch& otherHeld = scratch[change.target];
        const MemberLaw& otherLaw = laws[change.target];
        const auto account = [&](
                               const ImpactWindow& bodyOf,
                               const Scratch& forcesOf,
                               const MemberLaw& massesOf,
                               uint32_t nodeAt,
                               const glm::dvec3& deltaV,
                               bool kicked
                             ) {
          const double nodeMass = massesOf.mass[nodeAt];
          const glm::dvec3 endV =
            bodyOf.velocity[nodeAt]
            + dt * (forcesOf.force[nodeAt] + forcesOf.friction[nodeAt])
                / nodeMass
            - (kicked ? deltaV : glm::dvec3(0.0));

          event.kineticChange += nodeMass * glm::dot(endV, deltaV)
                                 + 0.5 * nodeMass * glm::dot(deltaV, deltaV);
          event.momentumChange += nodeMass * deltaV;
          event.angularChange += glm::cross(
            bodyOf.origin + bodyOf.position[nodeAt],
            nodeMass * deltaV
          );
        };

        account(
          window,
          held,
          law,
          spot.first,
          dt * (1.0 - spot.along) * kick / law.mass[spot.first],
          true
        );

        if (spot.along > 0.0) {
          account(
            window,
            held,
            law,
            spot.second,
            dt * spot.along * kick / law.mass[spot.second],
            true
          );
        }

        glm::dvec3 matePoint(0.0);

        for (size_t corner = 0u; corner != 8u; ++corner) {
          const uint32_t cornerNode = change.nodes[corner];

          account(
            other,
            otherHeld,
            otherLaw,
            cornerNode,
            -dt * change.shape[corner] * kick / otherLaw.mass[cornerNode],
            false
          );
          matePoint +=
            change.shape[corner] * (other.origin + other.position[cornerNode]);
        }

        event.pointGap = glm::length(
          window.origin + pointOf(spot, window.position) - matePoint
        );
        event.closing = closing;
        event.opening = opening;
        event.reach = reach;
        event.lambda = lambda;
        event.kick = kick;
        event.booked = 0.5 * lambda * (opening + dt * dt * reach * lambda);
      }

      if (tracing) {
        ++traceTransitions[change.member];
        traceWork(
          traceContacts[change.member],
          rigidTarget ? partnerTraceKind(group.partners[change.partner])
                      : WindowTraceKind::Window,
          rigidTarget ? partnerTraceId(group.partners[change.partner])
                      : change.from,
          0.5 * lambda * (opening + dt * dt * reach * lambda)
        );
      }

      if (moving) {
        frame->force -= kick;
        frame->torque -= glm::cross(change.arm, kick);
        held.partnerForce[change.partner] -= kick;
        held.partnerTorque[change.partner] -= glm::cross(change.arm, kick);
      }

      if (memberTarget) {
        Scratch& otherHeld = scratch[change.target];

        for (size_t corner = 0u; corner != 8u; ++corner) {
          const glm::dvec3 reacted = -change.shape[corner] * kick;

          otherHeld.force[change.nodes[corner]] += reacted;
          otherHeld.reactions.push_back(
            WindowReaction{change.from, change.nodes[corner], reacted}
          );
        }
      }
    }

    for (const SupportTransition& change : supportChanges) {
      const WindowPartner& partner = group.partners[change.partner];
      PartnerFrame& frame = frames[change.partner];
      const glm::dmat3 turn = glm::mat3_cast(partner.rotation);
      const glm::dmat3 inverse =
        turn * frame.bodyInverse * glm::transpose(turn);
      const glm::dvec3 lever = glm::cross(change.arm, change.normal);
      const double reach =
        partner.inverseMass + glm::dot(lever, inverse * lever);
      const glm::dvec3 before =
        partner.linearVelocity
        + glm::cross(partner.angularVelocity, change.arm);
      const glm::dvec3 after =
        partner.linearVelocity
        + dt * (partner.inverseMass * frame.force + gravity)
        + glm::cross(
          inverse * (frame.momentum + dt * frame.torque),
          change.arm
        );
      double held = reach;
      glm::dvec3 carriedBefore(0.0);
      glm::dvec3 carriedAfter(0.0);

      if (change.holder != std::numeric_limits<size_t>::max()) {
        const WindowPartner& carrier = group.partners[change.holder];
        const PartnerFrame& carrierFrame = frames[change.holder];
        const glm::dmat3 carrierTurn = glm::mat3_cast(carrier.rotation);
        const glm::dmat3 carrierInverse =
          carrierTurn * carrierFrame.bodyInverse * glm::transpose(carrierTurn);
        const glm::dvec3 carrierLever = glm::cross(change.lever, change.normal);

        held += carrier.inverseMass
                + glm::dot(carrierLever, carrierInverse * carrierLever);
        carriedBefore = carrier.linearVelocity
                        + glm::cross(carrier.angularVelocity, change.lever);
        carriedAfter =
          carrier.linearVelocity
          + dt * (carrier.inverseMass * carrierFrame.force + gravity)
          + glm::cross(
            carrierInverse * (carrierFrame.momentum + dt * carrierFrame.torque),
            change.lever
          );
      }

      const double closing =
        dt * glm::dot(change.normal, before - carriedBefore);
      const double opening = dt * glm::dot(change.normal, after - carriedAfter);
      const double lambda = transitionImpulse(
        0.5 * dt * dt * held,
        0.5 * (closing + opening),
        change.energy
      );
      const glm::dvec3 kick = lambda * change.normal;
      const double booked = 0.5 * lambda * (opening + dt * dt * held * lambda);

      frame.force += kick;
      frame.torque += glm::cross(change.arm, kick);

      if (groupSupport) {
        supportPending += booked;
      } else {
        scratch.front().pending += booked;
      }

      if (tracing) {
        traceWork(
          traceContacts.front(),
          WindowTraceKind::Support,
          partner.body,
          booked
        );
      }

      if (change.holder != std::numeric_limits<size_t>::max()) {
        frames[change.holder].force -= kick;
        frames[change.holder].torque -= glm::cross(change.lever, kick);
        loads[change.holder].push_back(
          SupportLoad{change.partner, kick, -glm::cross(change.lever, kick)}
        );
      }
    }

    for (Scratch& held : scratch) {
      mergeReactions(held.reactions);
      mergeReactions(held.kicks);
    }

    for (size_t at = 0u; at != count; ++at) {
      const MemberLaw& law = laws[at];
      ImpactWindow& window = *group.members[at].window;
      Scratch& held = scratch[at];

      const size_t nodes = window.position.size();

      if (!window.started) {
        for (size_t node = 0u; node != nodes; ++node) {
          window.velocity[node] -= 0.5 * dt * held.force[node] / law.mass[node];
        }
      }

      held.kinetic = 0.0;

      const bool booking = held.probing || held.priorProbing;
      const double stepBefore = window.step;
      const bool changing =
        window.started && substep == 0u && stepBefore != dt;
      const bool recentre = changing && !group.skipRecentre;
      const double bookedStep = recentre ? stepBefore : dt;

      if (changing) {
        ++held.stepChanges;
      }

      for (size_t node = 0u; node != nodes; ++node) {
        const glm::dvec3 before = window.velocity[node];

        const glm::dvec3& sliding = held.contacting[node] != 0u
                                      ? held.surface[node]
                                      : window.surface[node];

        window.ledger.friction -=
          0.5 * bookedStep
          * (glm::dot(before - sliding, window.friction[node])
             + glm::dot(before - held.surface[node], held.friction[node]));

        if (booking) {
          held.before[node] = before;
        }

        if (recentre) {
          const glm::dvec3 accel =
            (held.force[node] + held.friction[node]) / law.mass[node];

          window.velocity[node] += 0.5 * (stepBefore + dt) * accel;
          held.hysteresis -= glm::dot(
            held.loss[node],
            dt * (window.velocity[node] - 0.5 * dt * accel)
          );
          held.kinetic += 0.5 * law.mass[node]
                          * glm::dot(
                            window.velocity[node] - dt * accel,
                            window.velocity[node]
                          );
          held.stepChange += 0.125 * law.mass[node]
                             * (stepBefore * stepBefore - dt * dt)
                             * glm::dot(accel, accel);

          continue;
        }

        const glm::dvec3 pushed =
          (held.force[node] + held.friction[node]) / law.mass[node];

        window.velocity[node] += dt * pushed;
        held.hysteresis -= glm::dot(
          held.loss[node],
          dt * (window.velocity[node] - 0.5 * dt * pushed)
        );

        held.kinetic +=
          0.5 * law.mass[node] * glm::dot(before, window.velocity[node]);
      }

      if (!booking) {
        continue;
      }

      const std::vector<WindowReaction>& previous = window.reactions;
      const std::vector<WindowReaction>& current = held.reactions;

      size_t early = 0u;
      size_t late = 0u;

      while (early != previous.size() || late != current.size()) {
        WindowReaction joined;

        if (
          late == current.size()
          || (early != previous.size() && beforeReaction(previous[early], current[late]))
        ) {
          joined = previous[early++];
        } else if (
          early == previous.size()
          || beforeReaction(current[late], previous[early])
        ) {
          joined = current[late++];
        } else {
          joined = previous[early++];
          joined.force += current[late].force;
          ++late;
        }

        const double work =
          0.5 * bookedStep * glm::dot(held.before[joined.node], joined.force);
        const size_t source = memberOfKey(joined.from);

        window.ledger.transfer -= work;
        window.receivedWork += work;

        if (source != NO_INDEX) {
          group.members[source].window->ledger.transfer += work;
          window.memberWork += work;
        }

        if (tracing) {
          traceWork(
            traceContacts[at],
            WindowTraceKind::Window,
            joined.from & TRACE_ID_MASK,
            -work
          );

          if (source != NO_INDEX) {
            traceWork(
              traceContacts[source],
              WindowTraceKind::Window,
              group.members[at].key,
              work
            );
          }
        }
      }
    }

    double supportDissipation = 0.0;

    for (size_t index = 0u; index != group.partners.size(); ++index) {
      WindowPartner& partner = group.partners[index];
      PartnerFrame& frame = frames[index];

      if (!partner.dynamic || !(partner.inverseMass > 0.0)) {
        continue;
      }

      const glm::dvec3 linearBefore = partner.linearVelocity;
      const glm::dvec3 angularBefore = partner.angularVelocity;

      supportDissipation -=
        0.5 * dt
        * (glm::dot(linearBefore, frame.drag + partner.previousDrag.force)
           + glm::dot(
             angularBefore,
             frame.dragTorque + partner.previousDrag.torque
           ));

      const double partnerTotal =
        0.5 * dt
        * (glm::dot(linearBefore, partner.previousLoad.force + frame.force)
           + glm::dot(angularBefore, partner.previousLoad.torque + frame.torque));

      double membersWork = 0.0;

      for (Scratch& held : scratch) {
        const double share =
          0.5 * dt
          * (glm::dot(linearBefore, held.priorForce[index] + held.partnerForce[index])
             + glm::dot(
               angularBefore,
               held.priorTorque[index] + held.partnerTorque[index]
             ));

        held.partnerWork += share;
        membersWork += share;

        if (tracing) {
          traceWork(
            traceContacts[static_cast<size_t>(&held - scratch.data())],
            partnerTraceKind(partner),
            partnerTraceId(partner),
            share
          );
        }
      }

      if (groupSupport) {
        supportWork += partnerTotal - membersWork;
      } else {
        scratch.front().partnerWork += partnerTotal - membersWork;
      }

      if (tracing) {
        traceWork(
          traceContacts.front(),
          WindowTraceKind::Support,
          partner.body,
          partnerTotal - membersWork
        );
      }

      groupWork += partnerTotal;

      const glm::dvec3 pushed = dt * partner.inverseMass * frame.force;
      const glm::dmat3 turn = glm::mat3_cast(partner.rotation);
      const glm::dmat3 inertia =
        turn * frame.bodyInertia * glm::transpose(turn);

      frame.momentum += dt * frame.torque;

      partner.linearVelocity += pushed + dt * gravity;
      partner.angularVelocity =
        turn * frame.bodyInverse * glm::transpose(turn) * frame.momentum;
      partner.deltaLinear += pushed;
      partner.deltaAngular += partner.angularVelocity - angularBefore;
      partner.previousLoad = PartnerLoad{frame.force, frame.torque};
      partner.previousDrag = PartnerLoad{frame.drag, frame.dragTorque};

      frame.now = mixedKinetic(
                    frame.mass,
                    inertia,
                    linearBefore,
                    angularBefore,
                    partner.linearVelocity,
                    partner.angularVelocity
                  )
                  - frame.mass * glm::dot(gravity, partner.centerOfMass);
      frame.gravityWork =
        frame.mass
        * glm::dot(gravity, partner.centerOfMass - frame.startCentre);
    }

    for (size_t index = 0u; index != group.partners.size(); ++index) {
      for (const SupportLoad& load : loads[index]) {
        WindowSupportPair& pair = supportPairOf(
          group.partners[index].supportPairs,
          group.partners[load.partner].body
        );

        pair.impulse -= dt * load.force;
        pair.angularImpulse += dt * load.reaction;
        pair.partnerImpulse += dt * load.force;
      }
    }

    for (Scratch& held : scratch) {
      held.priorForce.swap(held.partnerForce);
      held.priorTorque.swap(held.partnerTorque);
    }

    for (size_t at = 0u; at != count; ++at) {
      const WindowMember& member = group.members[at];
      ImpactWindow& window = *member.window;
      Scratch& held = scratch[at];

      const double partners = held.partnersCarried + held.partnerWork;
      double partnerGravity = held.partnerGravityCarried;

      for (size_t index = 0u; at == 0u && index != group.partners.size();
           ++index) {
        if (!group.partners[index].dynamic) {
          continue;
        }

        partnerGravity += frames[index].gravityWork;
      }

      WindowLedger& ledger = window.ledger;

      if (!groupSupport && at == 0u) {
        ledger.friction += supportDissipation;
      }

      const double plasticNow =
        windowPlasticWork(*member.mesh, materials, window.plastic);

      if (plasticNow > ledger.plastic) {
        window.cappedSlip = 0.0;
        window.cappedSubsteps = 0u;
      } else {
        window.cappedSlip += held.cappedSlip;
        window.cappedSubsteps += held.cappedCount;
      }

      ledger.kinetic = held.kinetic;
      ledger.gravity = held.gravity;
      ledger.elastic = held.elastic;
      ledger.plastic = plasticNow;
      ledger.hysteresis += held.hysteresis;
      held.hysteresis = 0.0;
      ledger.penalty =
        held.penalty + (!groupSupport && at == 0u ? supported.penalty : 0.0);
      ledger.tangential = held.tangential;
      ledger.partners = partners;
      ledger.pending = held.pending;
      ledger.total = ledger.kinetic + ledger.gravity + ledger.elastic
                     + ledger.plastic + ledger.hysteresis + ledger.penalty
                     + ledger.tangential
                     + ledger.friction + ledger.partners - ledger.injected
                     + held.pending + ledger.transfer;

      window.partnerGravity = partnerGravity;

      const double traceCreation = window.stepCreation;

      if (!window.started) {
        window.started = true;
        window.openingTotal = ledger.total;
        window.openingPenalty = ledger.penalty;
        window.openingKinetic = ledger.kinetic;
        window.openingElastic = ledger.elastic;
        window.openingGravity = ledger.gravity;
        window.minimumTotal = ledger.total;
        window.contactSinceMinimum = held.contact;
      } else {
        if (ledger.total < window.minimumTotal) {
          window.minimumTotal = ledger.total;
          window.contactSinceMinimum = held.contact;
        } else {
          window.contactSinceMinimum =
            window.contactSinceMinimum || held.contact;
        }

        const double rise = ledger.total - window.minimumTotal;

        if (rise > window.stepCreation) {
          window.stepCreation = rise;
          window.stepCreationContact = window.contactSinceMinimum;
        }
      }

      if (tracing) {
        WindowTrace& trace = steps[at].trace;
        std::vector<WindowTraceContact>& touching = traceContacts[at];
        std::vector<uint64_t> codes;

        for (const WindowTraceContact& entry : touching) {
          if (entry.points == 0u) {
            continue;
          }

          codes.push_back(traceCode(entry.kind, entry.id));

          WindowTracePartner& seen =
            tracePartner(trace.partners, entry.kind, entry.id);

          ++seen.touchSubsteps;
          seen.deepest = std::max(seen.deepest, entry.deepest);
        }

        std::sort(codes.begin(), codes.end());

        const double increase = window.stepCreation - traceCreation;

        if (increase >= TRACE_FLOOR) {
          WindowTraceJump jump;

          jump.substep = substep;
          jump.rise = window.stepCreation;
          jump.increase = increase;
          jump.ledger = ledger;
          jump.before = traceBefore[at];
          jump.contacts = touching;
          jump.entered = traceDifference(codes, window.tracedContacts);
          jump.left = traceDifference(window.tracedContacts, codes);
          jump.openings = traceOpenings[at];
          jump.dropped = traceDropped[at];
          jump.transitions = traceTransitions[at];
          jump.supportTransitions =
            at == 0u ? static_cast<uint32_t>(supportChanges.size()) : 0u;
          jump.correction = held.correction;
          jump.groupChange = trace.groupChange && substep == 0u;
          trace.jumps.push_back(std::move(jump));
        } else if (increase > 0.0) {
          trace.creep += increase;
        }

        window.tracedContacts = std::move(codes);
        traceBefore[at] = ledger;
        touching.clear();
        traceOpenings[at] = 0u;
        traceDropped[at] = 0u;
        traceTransitions[at] = 0u;
      }

      if (held.contact) {
        ++steps[at].contactSubsteps;

        if (window.firstContact < 0) {
          window.firstContact = window.substeps;
          window.lead = window.time;
        }
      }

      window.friction.swap(held.friction);
      window.surface.swap(held.surface);

      window.reactions.swap(held.reactions);
      window.kicks.swap(held.kicks);
      window.probeTracks.swap(held.tracks);
      window.receivedPeak = std::max(window.receivedPeak, window.receivedWork);
      window.memberPeak = std::max(window.memberPeak, window.memberWork);
      held.priorProbing = !window.reactions.empty();

      held.finite = std::isfinite(ledger.total);
    }

    supportDissipated += supportDissipation;
    supportTotal = supportPenalty + supportWork + supportDissipated
                   + supportPending - supportReceived;
    supportMinimum = substep == 0u ? std::min(supportBaseline, supportTotal)
                                   : std::min(supportMinimum, supportTotal);
    supportRise = std::max(supportRise, supportTotal - supportMinimum);

    if (group.recordSupportLedger) {
      supportSamples.push_back(
        std::array<double, 8>{
          static_cast<double>(substep),
          supportBaseline,
          supportPenalty,
          supportWork,
          supportDissipated,
          supportPending,
          supportReceived,
          supportTotal
        }
      );
    }

    double summed = 0.0;

    for (const WindowMember& member : group.members) {
      const ImpactWindow& window = *member.window;

      summed +=
        window.ledger.total - (window.openingTotal - window.openingPenalty);
    }

    groupMinimum = std::min(groupMinimum, summed);
    groupRise = std::max(groupRise, summed - groupMinimum);

    for (const Scratch& held : scratch) {
      failed = failed || !held.finite;
    }

    if (failed) {
      break;
    }

    for (size_t at = 0u; at != count; ++at) {
      ImpactWindow& window = *group.members[at].window;
      Scratch& held = scratch[at];

      for (size_t node = 0u; node != window.position.size(); ++node) {
        window.position[node] += dt * window.velocity[node];
        held.finite = held.finite && finiteVector(window.position[node]);
      }

      window.time += dt;
      ++window.substeps;
      failed = failed || !held.finite;
    }

    for (WindowPartner& partner : group.partners) {
      partner.centerOfMass += dt * partner.linearVelocity;

      const double spin = glm::length(partner.angularVelocity);

      if (spin > 0.0) {
        partner.rotation = glm::normalize(
          glm::angleAxis(spin * dt, partner.angularVelocity / spin)
          * partner.rotation
        );
      }
    }
  }

  if (failed) {
    for (size_t at = 0u; at != count; ++at) {
      ImpactWindow& window = *group.members[at].window;
      const bool broke = !scratch[at].finite;

      window = saved[at];

      if (broke) {
        window.diverged = true;
        window.closing = true;
      }

      steps[at] = WindowStep{};
      steps[at].restored = true;
      steps[at].ledger = window.ledger;
      steps[at].stepCreation = window.stepCreation;
      steps[at].stepCreationContact = window.stepCreationContact;
      steps[at].angularDrift = window.angularDrift;
      steps[at].groupRise = window.groupRise;
    }

    group.partners = savedPartners;

    return steps;
  }

  for (size_t index = 0u; index != group.partners.size(); ++index) {
    WindowPartner& partner = group.partners[index];

    if (!partner.dynamic || !(partner.inverseMass > 0.0)) {
      continue;
    }

    const glm::dmat3 turn = glm::mat3_cast(partner.rotation);

    partner.inverseInertia =
      turn * frames[index].bodyInverse * glm::transpose(turn);
  }

  for (WindowPartner& carrier : group.partners) {
    if (!carrier.massive) {
      continue;
    }

    carrier.touched = std::any_of(
      group.partners.begin(),
      group.partners.end(),
      [](const WindowPartner& pushed) {
        return pushed.dynamic && !pushed.massive && pushed.touched
               && !pushed.seeds.empty() && pushed.probe.field != nullptr;
      }
    );
  }

  for (size_t at = 0u; at != count; ++at) {
    if (!memberProbes[at].has_value()) {
      continue;
    }

    const ImpactWindow& ended = *group.members[at].window;

    worldScratch.resize(ended.position.size());

    for (size_t node = 0u; node != ended.position.size(); ++node) {
      worldScratch[node] = ended.origin + ended.position[node];
    }

    placeDeformedProbe(*memberProbes[at], worldScratch);
  }

  for (size_t at = 0u; at != count; ++at) {
    ImpactWindow& window = *group.members[at].window;
    const std::vector<ProbePoint>& points = memberPoints[at];

    for (WindowProbeTrack& track : window.probeTracks) {
      track.endDepth = 0.0;

      if (track.point >= points.size()) {
        continue;
      }

      const glm::dvec3 endPoint =
        window.origin + pointOf(points[track.point], window.position);
      const size_t targetAt = memberOfKey(track.partner);

      DeformedDistance ended;

      if (targetAt != NO_INDEX && memberProbes[targetAt].has_value()) {
        ended = probeDeformed(*memberProbes[targetAt], endPoint);
      } else {
        for (size_t index = 0u; index != group.partners.size(); ++index) {
          const WindowPartner& partner = group.partners[index];

          if (
            partner.key != track.partner || !partnerProbes[index].has_value()
          ) {
            continue;
          }

          const glm::dmat3 endTurn = glm::mat3_cast(partner.rotation);

          ended = probeDeformed(
            *partnerProbes[index],
            glm::transpose(endTurn) * (endPoint - partner.centerOfMass)
              + glm::dvec3(partner.probe.fieldOffset)
          );

          break;
        }
      }

      if (ended.found) {
        track.endDepth =
          std::min(ended.distance - track.offset + track.excess, 0.0);
      }
    }

    for (WindowContactTrack& track : window.contactTracks) {
      if (track.excess.size() != window.position.size()) {
        continue;
      }

      const auto owner = std::find_if(
        group.partners.begin(),
        group.partners.end(),
        [&track](const WindowPartner& candidate) {
          return candidate.body == track.body && candidate.mesh == nullptr
                 && candidate.probe.field != nullptr;
        }
      );

      if (owner == group.partners.end()) {
        continue;
      }

      for (size_t node = 0u; node != window.position.size(); ++node) {
        const PreciseDistance ending = probeShapePrecise(
          owner->probe,
          owner->rotation,
          window.origin + window.position[node] - owner->centerOfMass
        );

        double offsetHeld = 0.0;

        for (const WindowOffset& entry : window.offsets) {
          if (
            entry.node == node && entry.partner == track.body
            && entry.supported == NO_WINDOW_BODY
          ) {
            offsetHeld = entry.offset;

            break;
          }
        }

        track.endDepth[node] =
          std::min(ending.distance - offsetHeld + track.excess[node], 0.0);
      }
    }
  }

  endSupportDepths(group);

  double partnerChange = 0.0;

  for (size_t index = 0u; index != group.partners.size(); ++index) {
    if (group.partners[index].dynamic) {
      partnerChange += frames[index].now - frames[index].base;
    }
  }

  for (size_t at = 0u; at != count; ++at) {
    const WindowMember& member = group.members[at];
    ImpactWindow& window = *member.window;
    const Scratch& held = scratch[at];
    WindowStep& result = steps[at];

    window.partnerLoads.clear();

    for (const uint32_t index : member.partners) {
      const WindowPartner& partner = group.partners[index];

      if (partner.dynamic && partner.inverseMass > 0.0) {
        window.partnerLoads.push_back(
          WindowPartnerLoad{
            partner.body,
            held.priorForce[index],
            held.priorTorque[index]
          }
        );
      }
    }

    result.partnerChange = partnerChange;
    result.partnerWork = groupWork;
    window.step = dt;
    window.heldSubsteps = substeps;
    window.heldFrameTime = frameTime;
    result.stepChanges = held.stepChanges;
    result.stepChangeEnergy = held.stepChange;
    result.heldSubsteps = substeps - shrunk;
    ++window.frames;
    window.groupKeys = keys;
    window.probedKeys.clear();

    for (const PairProbe& pair : pairs) {
      if (pair.prober == at) {
        window.probedKeys.push_back(
          pair.member != NO_INDEX
            ? windowPartnerKey(group.members[pair.member].key)
            : group.partners[pair.partner].key
        );
      }
    }

    std::sort(window.probedKeys.begin(), window.probedKeys.end());
    window.probedKeys.erase(
      std::unique(window.probedKeys.begin(), window.probedKeys.end()),
      window.probedKeys.end()
    );
    window.groupMinimum = groupMinimum;
    window.groupRise = groupRise;
    result.groupRise = groupRise;

    if (at == 0u && groupSupport) {
      result.supportBaseline = supportBaseline;
      result.supportPenalty = supportPenalty;
      result.supportCreation = supportTotal - supportBaseline;
      result.supportRise = supportRise;
      result.supportReceived = supportReceived;
      result.supportWork = supportWork;
      result.supportSamples = std::move(supportSamples);
    }

    if (result.contactSubsteps == 0u) {
      const glm::dvec3 angular = rigidOf(laws[at].mass, window).angular;
      const double size = glm::length(held.angularStart);
      const double change = glm::length(angular - held.angularStart);

      window.angularDrift = std::max(
        window.angularDrift,
        size < DRIFT_FLOOR ? change : change / size
      );
      result.angularStart = held.angularStart;
      result.angularEnd = angular;
    }

    const double gravityWork =
      window.openingGravity - window.ledger.gravity + window.partnerGravity;

    result.substeps = substeps;
    result.plasticAdded = window.ledger.plastic - held.plasticStart;
    result.ledger = window.ledger;
    result.stepCreation = window.stepCreation;
    result.stepCreationContact = window.stepCreationContact;
    result.creation =
      window.ledger.total - (window.openingTotal - window.openingPenalty);
    result.input = window.openingKinetic + window.openingElastic
                   + window.partnerEntry + std::max(0.0, gravityWork)
                   + window.receivedPeak;
    result.angularDrift = window.angularDrift;

    for (const uint32_t index : member.partners) {
      result.dynamicPartners +=
        group.partners[index].dynamic && held.touched[index] != 0u ? 1u : 0u;

      if (held.touched[index] != 0u && group.partners[index].mesh == nullptr) {
        result.touchedKeys.push_back(group.partners[index].body);
      }
    }

    result.storedKeys = held.storedKeys;
    std::sort(result.storedKeys.begin(), result.storedKeys.end());

    for (const uint64_t touched : held.touchedKeys) {
      if (touched >= WINDOW_PARTNER_KEYS && memberOfKey(touched) != NO_INDEX) {
        result.touchedKeys.push_back(touched);
      }
    }

    std::sort(result.touchedKeys.begin(), result.touchedKeys.end());
    result.touchedKeys.erase(
      std::unique(result.touchedKeys.begin(), result.touchedKeys.end()),
      result.touchedKeys.end()
    );

    for (const uint64_t touched : result.touchedKeys) {
      result.windowPartners += touched >= WINDOW_PARTNER_KEYS ? 1u : 0u;
    }

    window.gradientBound = held.gradientMax;

    result.probeContacts = held.probeContacts;
    result.pairLeaves = held.pairReleases;
    result.pairUnfound = held.pairUnfound;
    result.releaseEvents = held.releaseEvents;
    result.pairReleases = held.pairReleases;
    result.pairReleaseWork = held.pairReleaseWork;
    result.pairReleaseImpulse = held.pairReleaseImpulse;
    result.probeStiffness = held.probeStiffness;
    result.transferInput = window.memberPeak;
    result.gradientBound = held.gradientMax;
    result.stepScale = stepScale;
    result.unconverged = held.counts.unconverged;
    result.inverted = held.counts.inverted;
    result.engagedUnconverged = held.engagedUnconverged;
    result.tangentialStiffness = held.tangentialStiffness;
    result.tangentialChecks = held.tangentialChecks;
    result.dissipationLeast =
      std::isfinite(held.dissipationLeast) ? held.dissipationLeast : 0.0;
    result.capRatio = held.capRatio;
    result.stuckSubsteps = held.stuckSubsteps;
    result.reversals = held.reversals;
    result.largestSlip = held.largestSlip;
    result.engagedNotFound = held.engagedNotFound;
    result.pairs = held.pairSteps;
    std::sort(
      result.pairs.begin(),
      result.pairs.end(),
      [](const WindowPairStep& left, const WindowPairStep& right) {
        return left.partner < right.partner;
      }
    );
    result.cappedSlip = window.cappedSlip;
    result.cappedSubsteps = window.cappedSubsteps;
    result.shrinkSubsteps = stepScale > 1.0 ? result.contactSubsteps : 0u;
    result.openingProbes = window.openingProbes;
    result.openingOffset = window.openingOffset;
    result.excessAbsorbed = held.excessAbsorbed;
    result.correctionWork = held.correctionWork;

    for (const WindowProbeTrack& track : window.probeTracks) {
      result.excessTracks += track.excess > 0.0 ? 1u : 0u;
    }

    if (member.body != NO_WINDOW_BODY) {
      for (size_t seen = 0u; seen != count; ++seen) {
        const ImpactWindow& other = *group.members[seen].window;

        for (size_t entry = 0u; seen != at && entry != other.partnersSeen.size()
                                && entry != other.partnerEntries.size();
             ++entry) {
          if (other.partnersSeen[entry] == member.body) {
            result.repeatedEntry += other.partnerEntries[entry];
          }
        }
      }
    }

    if (member.permanent || window.frames < EXIT_FRAMES) {
      continue;
    }

    const ExitApproach exitCheck = approachQuiet(
      group,
      at,
      laws,
      memberPoints[at],
      memberProbes,
      partnerProbes
    );

    result.exitApproach = exitCheck.probes != 0u ? exitCheck.largest : 0.0;
    result.exitProbes = exitCheck.probes;

    if (
      result.plasticAdded <= EXIT_PLASTIC_SHARE * window.ledger.plastic
                               + EXIT_INPUT_SHARE * result.input
      && exitCheck.quiet
    ) {
      window.closing = true;
    }
  }

  return steps;
}

WindowExit closeWindow(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const ImpactWindow& window,
  const Settings& settings
) {
  const MemberLaw law = memberLaw(mesh, materials, window.step, settings);

  ImpactWindow frozen = window;
  std::vector<glm::dvec3> force(window.position.size(), glm::dvec3(0.0));
  std::vector<glm::dvec3> loss(window.position.size(), glm::dvec3(0.0));

  static_cast<void>(internalForces(
    mesh,
    law,
    frozen,
    false,
    settings.windowRotationDerivative,
    false,
    0.0,
    force,
    loss
  ));

  WindowExit leaving;

  leaving.position.resize(window.position.size());
  leaving.velocity.resize(window.position.size());
  leaving.plastic = window.plastic;

  for (size_t node = 0u; node != window.position.size(); ++node) {
    leaving.position[node] = window.origin + window.position[node];
    leaving.velocity[node] =
      window.velocity[node] + 0.5 * window.step * force[node] / law.mass[node];
  }

  return leaving;
}

void editWindowVelocity(
  ImpactWindow& window,
  const talus::PreparedMesh& mesh,
  const glm::dvec3& change
) {
  double injected = 0.0;

  for (size_t node = 0u; node != window.velocity.size(); ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);
    const glm::dvec3 before = window.velocity[node];

    window.velocity[node] += change;
    injected +=
      0.5 * mass
      * (glm::dot(window.velocity[node], window.velocity[node]) - glm::dot(before, before));
  }

  window.ledger.injected += injected;
}

void mirrorWindow(
  const ImpactWindow& window,
  talus::DeformableState& state
) {
  const size_t nodes = window.position.size();

  state.position.resize(nodes);
  state.velocity.resize(nodes);
  state.acceleration.assign(nodes, glm::vec3(0.0f));

  for (size_t node = 0u; node != nodes; ++node) {
    state.position[node] = glm::vec3(window.origin + window.position[node]);
    state.velocity[node] = glm::vec3(window.velocity[node]);
  }

  state.plastic = window.plastic;
}

double windowPlasticWork(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  std::span<const talus::PlasticPoint> plastic
) {
  const size_t hexPoints = 8u * mesh.hexes.size();

  double work = 0.0;

  for (size_t point = 0u; point != plastic.size(); ++point) {
    const double strain = static_cast<double>(plastic[point].equivalent);

    if (!(strain > 0.0)) {
      continue;
    }

    double weight = 0.0;
    uint32_t lane = 0u;

    if (point < hexPoints) {
      weight = 0.125 * static_cast<double>(mesh.hexes[point / 8u].volume);
      lane = mesh.hexes[point / 8u].material;
    } else if (point - hexPoints < mesh.tets.size()) {
      weight = static_cast<double>(mesh.tets[point - hexPoints].volume);
      lane = mesh.tets[point - hexPoints].material;
    }

    const talus::PhysicalMaterial& held = laneMaterial(materials, lane);

    work += weight
            * (static_cast<double>(held.yieldStress) * strain
               + 0.5 * static_cast<double>(held.hardeningModulus) * strain
                   * strain);
  }

  return work;
}

}
