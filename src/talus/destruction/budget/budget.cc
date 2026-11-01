#include "budget.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <numeric>

namespace talus::destruction {

namespace {

constexpr uint32_t trackRange = 512u;

uint32_t nextSteps(const dynamics::RigidBody& body, uint32_t steps) {
  const double reach =
    body.compound != nullptr ? body.compound->reach : body.radius;
  const double speed =
    glm::length(body.velocity) + glm::length(body.angularVelocity) * reach;
  const bool slow = body.mass > 0.0 && !body.asleep && speed < restSpeed;

  return slow ? steps + 1u : 0u;
}
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
  m_visit.resize(world.bodies.rigids.size());
  std::iota(m_visit.begin(), m_visit.end(), 0u);
  track(world, workers, m_visit);
}

void RestTracker::updateAwake(
  const dynamics::World& world,
  const dynamics::Workers& workers
) {
  if (!m_tracked || m_layout != world.bodies.layout
      || (!m_counting.empty() && m_counting.back() >= world.bodies.rigids.size())
      || !dynamics::awakeSuperset(world, m_visit)) {
    update(world, workers);
    return;
  }

  const auto middle = static_cast<std::ptrdiff_t>(m_visit.size());

  m_visit.insert(m_visit.end(), m_counting.begin(), m_counting.end());
  std::inplace_merge(m_visit.begin(), m_visit.begin() + middle, m_visit.end());
  m_visit.erase(std::unique(m_visit.begin(), m_visit.end()), m_visit.end());
  track(world, workers, m_visit);
}

void RestTracker::track(
  const dynamics::World& world,
  const dynamics::Workers& workers,
  std::span<const uint32_t> rigids
) {
  m_tracked = false;
  m_dt = world.parameters.dt;
  m_steps.resize(world.bodies.nextId, 0u);

  dynamics::parallelFor(
    workers,
    static_cast<uint32_t>(rigids.size()),
    trackRange,
    [&](uint32_t begin, uint32_t end, uint32_t) {
      for (uint32_t entry = begin; entry != end; ++entry) {
        const dynamics::RigidBody& body = world.bodies.rigids[rigids[entry]];

        m_steps[body.id] = nextSteps(body, m_steps[body.id]);
      }
    }
  );

  m_counting.clear();

  for (const uint32_t rigid : rigids) {
    if (m_steps[world.bodies.rigids[rigid].id] != 0u) {
      m_counting.push_back(rigid);
    }
  }

  m_layout = world.bodies.layout;
  m_tracked = true;
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
