#include "impactwindow.ih"

namespace talus::deformable {

namespace {

constexpr double PENALTY_SHARE = 0.05;
constexpr double REFERENCE_SHARE = 0.9;
constexpr uint32_t EXIT_FRAMES = 2u;
constexpr double EXIT_PLASTIC_SHARE = 0.005;
constexpr double EXIT_INPUT_SHARE = 1.0e-6;
constexpr double DRIFT_FLOOR = 1.0e-9;

double waveSpeed(const objectmodel::PhysicalMaterial& lane) {
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
};

MemberLaw memberLaw(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
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

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const objectmodel::PreparedHex& hex = mesh.hexes[element];
    const glm::vec3 corner = mesh.restPositions[hex.nodes[0]];
    const objectmodel::PhysicalMaterial& lane =
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
  }

  for (uint32_t& owner : made.nodeLane) {
    if (owner == std::numeric_limits<uint32_t>::max()) {
      owner = 0u;
    }
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

struct SupportStep {
  double penalty = 0.0;
};

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
};

struct SupportTransition {
  size_t partner = 0u;
  glm::dvec3 normal{0.0};
  glm::dvec3 arm{0.0};
  double energy = 0.0;
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
  std::span<const objectmodel::PhysicalMaterial> materials,
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

SupportStep supportPartners(
  WindowGroup& group,
  std::span<const PartnerSupport> supports,
  std::span<PartnerFrame> frames,
  std::span<const objectmodel::PhysicalMaterial> materials,
  double dt,
  bool transitions,
  std::vector<SupportTransition>& changes
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
                   return !other.dynamic && other.probe.field != nullptr
                          && other.body == track.body;
                 }
               );
      }
    );

    for (const WindowPartner& fixed : group.partners) {
      if (fixed.dynamic || fixed.probe.field == nullptr) {
        continue;
      }

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
        partner.supportTracks.push_back(std::move(made));
        tracked = partner.supportTracks.end() - 1;
      }

      WindowContactTrack& track = *tracked;

      for (size_t seed = 0u; seed != seeds; ++seed) {
        const glm::dvec3 arm = turn * glm::dvec3(partner.seeds[seed]);
        const glm::dvec3 relative =
          partner.centerOfMass + arm - fixed.centerOfMass;
        const PreciseDistance probed =
          probeShapePrecise(fixed.probe, fixed.rotation, relative);
        const double distance = probed.distance;
        const double previous = track.depth[seed];
        const glm::dvec3 previousPush = track.push[seed];
        const glm::dvec3 moving =
          partner.linearVelocity + glm::cross(partner.angularVelocity, arm);
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
                - 0.5 * support.stiffness * previous * previous
            }
          );
        }

        if (!(distance < 0.0)) {
          continue;
        }

        const glm::dvec3 normal = unitOf(probed.gradient, probed.normal);
        const double pressed = support.stiffness * -distance;

        step.penalty += 0.5 * support.stiffness * distance * distance;

        glm::dvec3 applied = pressed * probed.gradient;

        track.push[seed] = applied;

        if (tracking && !(previous < 0.0)) {
          changes.push_back(
            SupportTransition{
              index,
              normal,
              arm,
              0.5 * glm::dot(applied, dt * moving)
                + 0.5 * support.stiffness * distance * distance
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
        }

        frame.force += applied;
        frame.torque += glm::cross(arm, applied);
      }
    }
  }

  return step;
}

double internalForces(
  const objectmodel::PreparedMesh& mesh,
  const MemberLaw& law,
  ImpactWindow& window,
  bool returnMapping,
  bool rotationDerivative,
  std::vector<glm::dvec3>& force
) {
  std::fill(force.begin(), force.end(), glm::dvec3(0.0));

  double elastic = 0.0;

  std::array<glm::vec3, 8> deformed{};
  std::array<objectmodel::PlasticPoint, 8> committed{};

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const objectmodel::PreparedHex& hex = mesh.hexes[element];
    const glm::dvec3 corner = window.position[hex.nodes[0]];

    for (size_t at = 0u; at != 8u; ++at) {
      deformed[at] = glm::vec3(window.position[hex.nodes[at]] - corner);
      committed[at] = window.plastic[8u * element + at];
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
        window.plastic[8u * element + at] = trial.points[at];
      }
    }

    elastic += static_cast<double>(trial.energy);
  }

  return elastic;
}

bool approachQuiet(
  const objectmodel::PreparedMesh& mesh,
  std::span<const double> mass,
  const ImpactWindow& window,
  const WindowGroup& group,
  const WindowMember& member
) {
  const Rigid rigid = rigidOf(mass, window);

  if (!(rigid.mass > 0.0)) {
    return true;
  }

  const glm::dvec3 moving = rigid.linear / rigid.mass;
  const glm::dvec3 spin = glm::determinant(rigid.inertia) > 0.0
                            ? glm::inverse(rigid.inertia) * rigid.angular
                            : glm::dvec3(0.0);
  const double reach = static_cast<double>(mesh.voxelSize.x);

  for (const uint32_t index : member.partners) {
    const WindowPartner& partner = group.partners[index];

    if (partner.probe.field == nullptr) {
      continue;
    }

    for (size_t node = 0u; node != window.position.size(); ++node) {
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

      if (approach > static_cast<double>(member.trigger)) {
        return false;
      }
    }
  }

  return true;
}

bool finiteVector(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y)
         && std::isfinite(value.z);
}

}

WindowLaw windowLaw(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  double frameTime,
  const Settings& settings
) {
  WindowLaw law;

  for (const objectmodel::PreparedHex& hex : mesh.hexes) {
    law.wave =
      std::max(law.wave, waveSpeed(laneMaterial(materials, hex.material)));
  }

  for (const objectmodel::PreparedTet& tet : mesh.tets) {
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
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& start,
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
    window.plastic.assign(points, objectmodel::PlasticPoint{});
  }

  return window;
}

std::vector<WindowStep> stepWindows(
  WindowGroup& group,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const glm::dvec3& gravity,
  double frameTime,
  const Settings& settings
) {
  const size_t count = group.members.size();

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

  const double dt = frameTime / static_cast<double>(substeps);

  double referenceStep = std::numeric_limits<double>::infinity();

  for (const MemberLaw& held : laws) {
    referenceStep = std::min(referenceStep, held.law.referenceStep);
  }

  const double pace = 2.0 / referenceStep;

  std::vector<PartnerFrame> frames(group.partners.size());
  std::vector<PartnerSupport> supports(group.partners.size());

  for (size_t index = 0u; index != group.partners.size(); ++index) {
    WindowPartner& partner = group.partners[index];
    PartnerFrame& frame = frames[index];

    partner.deltaLinear = glm::dvec3(0.0);
    partner.deltaAngular = glm::dvec3(0.0);
    partner.touched = false;

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
    support.stiffness =
      settings.windowPenaltyScale * PENALTY_SHARE * support.mass * pace * pace;
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

  struct Scratch {
    std::vector<glm::dvec3> force;
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
  };

  std::vector<Scratch> scratch(count);

  for (size_t at = 0u; at != count; ++at) {
    const WindowMember& member = group.members[at];
    ImpactWindow& window = *member.window;
    Scratch& held = scratch[at];

    const size_t nodes = window.position.size();

    held.force.assign(nodes, glm::dvec3(0.0));
    held.friction.assign(nodes, glm::dvec3(0.0));
    held.surface.assign(nodes, glm::dvec3(0.0));
    held.contacting.assign(nodes, 0u);
    held.touched.assign(group.partners.size(), 0u);
    held.partnersCarried = window.ledger.partners;
    held.partnerGravityCarried = window.partnerGravity;
    held.plasticStart =
      window.started
        ? window.ledger.plastic
        : windowPlasticWork(*member.mesh, materials, window.plastic);
    held.angularStart = rigidOf(laws[at].mass, window).angular;

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

      window.partnersSeen.push_back(partner.body);
      window.partnerEntry += partnerKinetic(partner, frames[index]);
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

  bool failed = false;
  std::vector<ContactTransition> transitions;
  std::vector<SupportTransition> supportChanges;

  for (uint32_t substep = 0u; substep != substeps && !failed; ++substep) {
    transitions.clear();
    supportChanges.clear();

    for (PartnerFrame& frame : frames) {
      frame.force = glm::dvec3(0.0);
      frame.torque = glm::dvec3(0.0);
      frame.drag = glm::dvec3(0.0);
      frame.dragTorque = glm::dvec3(0.0);
    }

    for (size_t at = 0u; at != count; ++at) {
      const WindowMember& member = group.members[at];
      const objectmodel::PreparedMesh& mesh = *member.mesh;
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
        held.force
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

      for (const uint32_t index : member.partners) {
        WindowPartner& partner = group.partners[index];
        PartnerFrame& frame = frames[index];

        if (partner.probe.field == nullptr) {
          continue;
        }

        auto tracked = std::find_if(
          window.contactTracks.begin(),
          window.contactTracks.end(),
          [&partner](const WindowContactTrack& track) {
            return track.body == partner.body;
          }
        );

        if (tracked == window.contactTracks.end()) {
          WindowContactTrack fresh;

          fresh.body = partner.body;
          fresh.depth.assign(nodes, 0.0);
          fresh.push.assign(nodes, glm::dvec3(0.0));
          window.contactTracks.push_back(std::move(fresh));
          tracked = window.contactTracks.end() - 1;
        }

        WindowContactTrack& track = *tracked;

        for (size_t node = 0u; node != nodes; ++node) {
          const glm::dvec3 world = window.origin + window.position[node];
          const glm::dvec3 relative = world - partner.centerOfMass;
          const PreciseDistance probed =
            probeShapePrecise(partner.probe, partner.rotation, relative);
          const double distance = probed.distance;

          double offset = 0.0;

          if (!window.started) {
            if (distance < 0.0) {
              window.offsets.push_back(
                WindowOffset{
                  static_cast<uint32_t>(node),
                  partner.body,
                  distance
                }
              );
              ++window.openingInside;
              offset = distance;
            }
          } else if (!window.offsets.empty()) {
            const auto kept = std::find_if(
              window.offsets.begin(),
              window.offsets.end(),
              [&](const WindowOffset& entry) {
                return entry.node == node && entry.partner == partner.body;
              }
            );

            if (kept != window.offsets.end()) {
              if (distance >= 0.0) {
                *kept = window.offsets.back();
                window.offsets.pop_back();
              } else {
                offset = kept->offset;
              }
            }
          }

          const double depth = distance - offset;
          const double previous = track.depth[node];
          const glm::dvec3 previousPush = track.push[node];
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
                  - 0.5 * law.law.penalty[node] * previous * previous
              }
            );
          }

          if (!(depth < 0.0)) {
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
                  + 0.5 * law.law.penalty[node] * depth * depth
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
          }
        }
      }

      std::erase_if(window.contactTracks, [&](const WindowContactTrack& track) {
        return std::none_of(
          member.partners.begin(),
          member.partners.end(),
          [&](uint32_t index) {
            return group.partners[index].body == track.body
                   && group.partners[index].probe.field != nullptr;
          }
        );
      });
    }

    const SupportStep supported = supportPartners(
      group,
      supports,
      frames,
      materials,
      dt,
      settings.windowContactTransitions,
      supportChanges
    );

    for (Scratch& held : scratch) {
      held.pending = 0.0;
    }

    for (const ContactTransition& change : transitions) {
      const MemberLaw& law = laws[change.member];
      const ImpactWindow& window = *group.members[change.member].window;
      Scratch& held = scratch[change.member];
      const WindowPartner& partner = group.partners[change.partner];
      PartnerFrame& frame = frames[change.partner];
      const double mass = law.mass[change.node];
      const glm::dvec3 before = window.velocity[change.node];
      const glm::dvec3 after =
        before
        + dt * (held.force[change.node] + held.friction[change.node]) / mass;
      const glm::dvec3 pointBefore =
        partner.linearVelocity
        + glm::cross(partner.angularVelocity, change.arm);
      const bool moving = partner.dynamic && partner.inverseMass > 0.0;

      glm::dvec3 pointAfter = pointBefore;
      double reach = 1.0 / mass;

      if (moving) {
        const glm::dmat3 turn = glm::mat3_cast(partner.rotation);
        const glm::dmat3 inverse =
          turn * frame.bodyInverse * glm::transpose(turn);
        const glm::dvec3 lever = glm::cross(change.arm, change.normal);

        pointAfter = partner.linearVelocity
                     + dt * (partner.inverseMass * frame.force + gravity)
                     + glm::cross(
                       inverse * (frame.momentum + dt * frame.torque),
                       change.arm
                     );
        reach += partner.inverseMass + glm::dot(lever, inverse * lever);
      }

      const double closing = dt * glm::dot(change.normal, before - pointBefore);
      const double opening = dt * glm::dot(change.normal, after - pointAfter);
      const double lambda = transitionImpulse(
        0.5 * dt * dt * reach,
        0.5 * (closing + opening),
        change.energy
      );
      const glm::dvec3 kick = lambda * change.normal;

      held.force[change.node] += kick;
      held.pending += 0.5 * lambda * (opening + dt * dt * reach * lambda);
      held.contact = true;

      if (moving) {
        frame.force -= kick;
        frame.torque -= glm::cross(change.arm, kick);
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
      const double closing = dt * glm::dot(change.normal, before);
      const double opening = dt * glm::dot(change.normal, after);
      const double lambda = transitionImpulse(
        0.5 * dt * dt * reach,
        0.5 * (closing + opening),
        change.energy
      );
      const glm::dvec3 kick = lambda * change.normal;

      frame.force += kick;
      frame.torque += glm::cross(change.arm, kick);
      scratch.front().pending +=
        0.5 * lambda * (opening + dt * dt * reach * lambda);
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

      for (size_t node = 0u; node != nodes; ++node) {
        const glm::dvec3 before = window.velocity[node];

        const glm::dvec3& sliding = held.contacting[node] != 0u
                                      ? held.surface[node]
                                      : window.surface[node];

        window.ledger.friction -=
          0.5 * dt
          * (glm::dot(before - sliding, window.friction[node])
             + glm::dot(before - held.surface[node], held.friction[node]));
        window.velocity[node] +=
          dt * (held.force[node] + held.friction[node]) / law.mass[node];
        held.kinetic +=
          0.5 * law.mass[node] * glm::dot(before, window.velocity[node]);
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

    for (size_t at = 0u; at != count; ++at) {
      const WindowMember& member = group.members[at];
      ImpactWindow& window = *member.window;
      Scratch& held = scratch[at];

      double partners = held.partnersCarried;
      double partnerGravity = held.partnerGravityCarried;

      for (size_t index = 0u; at == 0u && index != group.partners.size();
           ++index) {
        if (!group.partners[index].dynamic) {
          continue;
        }

        partners += frames[index].now - frames[index].base;
        partnerGravity += frames[index].gravityWork;
      }

      WindowLedger& ledger = window.ledger;

      if (at == 0u) {
        ledger.friction += supportDissipation;
      }

      ledger.kinetic = held.kinetic;
      ledger.gravity = held.gravity;
      ledger.elastic = held.elastic;
      ledger.plastic =
        windowPlasticWork(*member.mesh, materials, window.plastic);
      ledger.penalty = held.penalty + (at == 0u ? supported.penalty : 0.0);
      ledger.partners = partners;
      ledger.pending = held.pending;
      ledger.total = ledger.kinetic + ledger.gravity + ledger.elastic
                     + ledger.plastic + ledger.penalty + ledger.friction
                     + ledger.partners - ledger.injected + held.pending;

      window.partnerGravity = partnerGravity;

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

      if (held.contact) {
        ++steps[at].contactSubsteps;

        if (window.firstContact < 0) {
          window.firstContact = window.substeps;
          window.lead = window.time;
        }
      }

      window.friction.swap(held.friction);
      window.surface.swap(held.surface);

      held.finite = std::isfinite(ledger.total);
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

  for (size_t at = 0u; at != count; ++at) {
    const WindowMember& member = group.members[at];
    ImpactWindow& window = *member.window;
    const Scratch& held = scratch[at];
    WindowStep& result = steps[at];

    window.step = dt;
    ++window.frames;
    window.groupKeys = keys;
    window.groupMinimum = groupMinimum;
    window.groupRise = groupRise;
    result.groupRise = groupRise;

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
                   + window.partnerEntry + std::max(0.0, gravityWork);
    result.angularDrift = window.angularDrift;

    for (const uint32_t index : member.partners) {
      result.dynamicPartners +=
        group.partners[index].dynamic && held.touched[index] != 0u ? 1u : 0u;
    }

    if (
      !member.permanent && window.frames >= EXIT_FRAMES
      && result.plasticAdded <= EXIT_PLASTIC_SHARE * window.ledger.plastic
                                  + EXIT_INPUT_SHARE * result.input
      && approachQuiet(*member.mesh, laws[at].mass, window, group, member)
    ) {
      window.closing = true;
    }
  }

  return steps;
}

WindowExit closeWindow(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const ImpactWindow& window,
  const Settings& settings
) {
  const MemberLaw law = memberLaw(mesh, materials, window.step, settings);

  ImpactWindow frozen = window;
  std::vector<glm::dvec3> force(window.position.size(), glm::dvec3(0.0));

  static_cast<void>(internalForces(
    mesh,
    law,
    frozen,
    false,
    settings.windowRotationDerivative,
    force
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
  const objectmodel::PreparedMesh& mesh,
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
  objectmodel::DeformableState& state
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
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  std::span<const objectmodel::PlasticPoint> plastic
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

    const objectmodel::PhysicalMaterial& held = laneMaterial(materials, lane);

    work += weight
            * (static_cast<double>(held.yieldStress) * strain
               + 0.5 * static_cast<double>(held.hardeningModulus) * strain
                   * strain);
  }

  return work;
}

}
