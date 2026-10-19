#include "budget.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <glm/glm.hpp>

namespace talus::destruction {

namespace {

constexpr uint32_t trackRange = 512u;

}

double rigidCost(const StepCounts& counts, const CostModel& model) {
  return model.rigid[0] * counts.moving + model.rigid[1] * counts.awakeChunks + model.rigid[2] * counts.bodies;
}

double updateCost(const StepCounts& counts, const CostModel& model) {
  return model.update[0] * counts.visited + model.update[1] * counts.gathered + model.update[2] * counts.awakeManifolds
         + model.update[3] * counts.fractures + model.update[4] * counts.broken + model.update[5] * counts.bodies;
}

double stressCost(const StepCounts& counts, const CostModel& model) {
  return model.stress[0] * static_cast<double>(counts.factorWork) + model.stress[1] * static_cast<double>(counts.wideWork)
         + model.stress[2] * counts.solvedChunks + model.stress[3] * counts.updates;
}

StepCost predictCost(const StepCounts& counts, const CostModel& model) {
  StepCost cost;

  cost.rigid = rigidCost(counts, model);
  cost.update = updateCost(counts, model);
  cost.stress = stressCost(counts, model);
  cost.total = cost.rigid + cost.update + cost.stress;

  return cost;
}

void RestTracker::update(const dynamics::World& world, const dynamics::Workers& workers) {
  m_dt = world.parameters.dt;
  m_steps.resize(world.bodies.nextId, 0u);

  const auto count = static_cast<uint32_t>(world.bodies.rigids.size());

  dynamics::parallelFor(workers, count, trackRange, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t rigid = begin; rigid != end; ++rigid) {
      const dynamics::RigidBody& body = world.bodies.rigids[rigid];
      const double reach = body.compound != nullptr ? body.compound->reach : body.radius;
      const double speed = glm::length(body.velocity) + glm::length(body.angularVelocity) * reach;
      const bool slow = body.mass > 0.0 && !body.asleep && speed < restSpeed;

      m_steps[body.id] = slow ? m_steps[body.id] + 1u : 0u;
    }
  });
}

bool RestTracker::resting(uint32_t body) const noexcept {
  return body < m_steps.size() && (static_cast<double>(m_steps[body]) + 0.5) * m_dt >= restTime;
}

void RestTracker::reset(uint32_t body) noexcept {
  if (body < m_steps.size()) {
    m_steps[body] = 0u;
  }
}

uint32_t RestTracker::restSteps(uint32_t body) const noexcept {
  return body < m_steps.size() ? m_steps[body] : 0u;
}

}
