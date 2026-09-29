#include "windowmonitor.h"

#include <talus/physics/physicsworld/physicsworld.h>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace windowmonitor {

namespace {

constexpr uint32_t JACOBI_SWEEPS = 64u;

double gravityOf() {
  return static_cast<double>(talus::physics::PhysicsWorld::GRAVITY);
}

double frameOf() {
  return static_cast<double>(talus::physics::PhysicsWorld::TIME_STEP);
}

glm::dvec3 principalMoments(glm::dmat3 inertia) {
  for (uint32_t sweep = 0u; sweep != JACOBI_SWEEPS; ++sweep) {
    const double off = std::abs(inertia[1][0]) + std::abs(inertia[2][0])
                       + std::abs(inertia[2][1]);

    if (off == 0.0) {
      break;
    }

    for (int row = 0; row != 2; ++row) {
      for (int column = row + 1; column != 3; ++column) {
        const double coupling = inertia[column][row];

        if (coupling == 0.0) {
          continue;
        }

        const double theta = 0.5
                             * std::atan2(
                               2.0 * coupling,
                               inertia[row][row] - inertia[column][column]
                             );
        const double cosine = std::cos(theta);
        const double sine = std::sin(theta);

        glm::dmat3 turn(1.0);

        turn[row][row] = cosine;
        turn[column][column] = cosine;
        turn[column][row] = -sine;
        turn[row][column] = sine;

        inertia = glm::transpose(turn) * inertia * turn;
      }
    }
  }

  return glm::dvec3(inertia[0][0], inertia[1][1], inertia[2][2]);
}

}

MonitoredBody monitoredBody(const talus::objectmodel::PreparedMesh& mesh) {
  MonitoredBody body;
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);

    body.mass += weight;
    centre += weight * glm::dvec3(mesh.restPositions[node]);
  }

  if (!(body.mass > 0.0)) {
    return body;
  }

  centre /= body.mass;

  glm::dmat3 inertia(0.0);

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - centre;

    inertia +=
      static_cast<double>(mesh.nodeMass[node])
      * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  const double trace = inertia[0][0] + inertia[1][1] + inertia[2][2];
  const glm::dvec3 moments = principalMoments(inertia);
  const double smallest = std::min({moments.x, moments.y, moments.z});
  const double largest = std::max({moments.x, moments.y, moments.z});

  body.gyration = std::sqrt(trace / (2.0 * body.mass));
  body.inertiaRatio = smallest > 0.0 ? largest / smallest : 1.0;

  return body;
}

double linearFloor(const MonitoredBody& body) {
  return body.mass * gravityOf() * frameOf();
}

double angularFloor(const MonitoredBody& body) {
  return linearFloor(body) * body.gyration;
}

double kineticFloor(const MonitoredBody& body) {
  const double speed = gravityOf() * frameOf();

  return 0.5 * body.mass * speed * speed;
}

double transitionBound(const MonitoredBody& body) {
  return std::max(1.0e-6, 2.0 * body.inertiaRatio * 0x1.0p-24);
}

double momentumApart(
  const glm::dvec3& before,
  const glm::dvec3& after,
  double floor
) {
  const double scale =
    std::max({glm::length(before), glm::length(after), floor});
  const double change = glm::length(after - before);

  return scale > 0.0 ? change / scale : change;
}

double restingAllowance(double restingMass, uint32_t frames) {
  return 2.0 * RESTING_DRIFT * restingMass * static_cast<double>(frames);
}

namespace {

void momentumLine(
  const talus::runtime::WindowRecord& record,
  uint32_t frame,
  const MonitoredBody& body,
  std::vector<std::string>& lines
) {
  const talus::deformable::WindowStep& step = record.step;

  if (step.contactSubsteps != 0u) {
    return;
  }

  const double drift =
    momentumApart(step.angularStart, step.angularEnd, angularFloor(body));

  if (drift > ANGULAR) {
    std::ostringstream line;

    line << "INVARIANT-VIOLATION momentum: window entity "
         << static_cast<uint32_t>(record.entity) << " frame " << frame
         << " angular " << drift;

    lines.push_back(line.str());
  }
}

}

std::vector<std::string> windowViolations(
  const talus::runtime::WindowRecord& record,
  uint32_t frame,
  double epsilon,
  const MonitoredBody& body,
  double restingMass
) {
  std::vector<std::string> lines;

  const talus::deformable::WindowStep& step = record.step;
  const uint32_t entity = static_cast<uint32_t>(record.entity);
  const double bound =
    epsilon * step.input + restingAllowance(restingMass, record.frame);

  if (step.stepCreation > bound || step.creation > bound) {
    const bool stepped = step.stepCreation >= step.creation;
    const bool contact =
      stepped ? step.stepCreationContact : step.contactSubsteps != 0u;

    std::ostringstream line;

    line << "INVARIANT-VIOLATION energy: window entity " << entity << " frame "
         << frame << " creation "
         << (stepped ? step.stepCreation : step.creation) << " input "
         << step.input << " contact " << (contact ? 1 : 0);

    lines.push_back(line.str());
  }

  momentumLine(record, frame, body, lines);

  return lines;
}

std::vector<std::string> frameViolations(
  std::span<const talus::runtime::WindowRecord> records,
  uint32_t frame,
  double epsilon,
  const std::function<MonitoredBody(talus::objectmodel::Entity)>& bodyOf,
  double restingMass
) {
  std::vector<std::string> lines;
  std::vector<uint32_t> judged;

  for (const talus::runtime::WindowRecord& first : records) {
    if (std::find(judged.begin(), judged.end(), first.group) != judged.end()) {
      continue;
    }

    judged.push_back(first.group);

    std::vector<const talus::runtime::WindowRecord*> members;

    for (const talus::runtime::WindowRecord& record : records) {
      if (record.group == first.group) {
        members.push_back(&record);
      }
    }

    if (members.size() == 1u) {
      for (std::string& line : windowViolations(
             first,
             frame,
             epsilon,
             bodyOf(first.entity),
             restingMass
           )) {
        lines.push_back(std::move(line));
      }

      continue;
    }

    double creation = 0.0;
    double rise = 0.0;
    double input = 0.0;
    uint32_t frames = 0u;
    bool contact = false;

    for (const talus::runtime::WindowRecord* member : members) {
      creation += member->step.creation;
      rise = std::max(rise, member->step.groupRise);
      input += member->step.input;
      frames = std::max(frames, member->frame);
      contact = contact || member->step.contactSubsteps != 0u;
    }

    const double bound =
      epsilon * input + restingAllowance(restingMass, frames);

    if (rise > bound || creation > bound) {
      std::ostringstream line;

      line << "INVARIANT-VIOLATION energy: window group";

      for (const talus::runtime::WindowRecord* member : members) {
        line << ' ' << static_cast<uint32_t>(member->entity);
      }

      line << " frame " << frame << " creation " << creation << " rise " << rise
           << " input " << input << " contact " << (contact ? 1 : 0);

      lines.push_back(line.str());
    }

    for (const talus::runtime::WindowRecord* member : members) {
      momentumLine(*member, frame, bodyOf(member->entity), lines);
    }
  }

  return lines;
}

std::vector<std::string> handbackViolations(
  const talus::runtime::PartnerHandback& handback,
  uint32_t frame,
  double epsilon
) {
  std::vector<std::string> lines;

  if (
    handback.forwardEnergy - handback.windowEnergy
    > HANDBACK * std::abs(handback.windowEnergy) + epsilon * handback.groupInput
  ) {
    std::ostringstream line;

    line << "INVARIANT-VIOLATION energy: handback partner " << handback.body
         << " frame " << frame << " before " << handback.windowEnergy
         << " after " << handback.forwardEnergy;

    lines.push_back(line.str());
  }

  return lines;
}

std::vector<std::string> transitionViolations(
  const talus::runtime::YieldTransition& transition,
  uint32_t frame,
  const MonitoredBody& body
) {
  std::vector<std::string> lines;

  const uint32_t entity = static_cast<uint32_t>(transition.entity);
  const double linear = momentumApart(
    transition.linearBefore,
    transition.linearAfter,
    linearFloor(body)
  );
  const double angular = momentumApart(
    transition.angularBefore,
    transition.angularAfter,
    angularFloor(body)
  );

  if (linear > MOMENTUM || angular > MOMENTUM) {
    std::ostringstream line;

    line << "INVARIANT-VIOLATION momentum: transition entity " << entity
         << " frame " << frame << " linear " << linear << " angular "
         << angular;

    lines.push_back(line.str());
  }

  if (
    transition.kineticAfter + transition.penaltyAfter - transition.kineticBefore
    > KINETIC * std::max(transition.kineticBefore, kineticFloor(body))
  ) {
    std::ostringstream line;

    line << "INVARIANT-VIOLATION energy: transition entity " << entity
         << " frame " << frame << " kinetic " << transition.kineticBefore << ' '
         << transition.kineticAfter << " penalty " << transition.penaltyAfter;

    lines.push_back(line.str());
  }

  return lines;
}

}
