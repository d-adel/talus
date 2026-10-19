#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::dynamics {

enum class Owner : uint8_t { Softstep, Strainstep };

struct Island {
  std::vector<BodyRef> bodies;
  std::vector<uint32_t> contacts;
  bool external = false;
  bool asleep = false;
  Owner owner = Owner::Softstep;
  std::vector<uint32_t> pairs{};
};

[[nodiscard]] std::vector<Island> islandsOf(
  const BodySet& bodies,
  const ManifoldCache& cache,
  std::span<const Contact> contacts
);
void islandsOf(
  const BodySet& bodies,
  const ManifoldCache& cache,
  std::span<const Contact> contacts,
  std::vector<Island>& islands
);

inline constexpr uint32_t noIsland = 0xFFFFFFFFu;

class IslandStore {
public:
  void sync(const BodySet& bodies, const Workers& workers = Workers{});
  void link(uint32_t a, uint32_t b);
  void markRemoved(uint32_t id);
  void linkPairs(const BodySet& bodies, const ManifoldCache& cache);
  void linkContacts(const BodySet& bodies, std::span<const Contact> contacts);
  void build(
    BodySet& bodies,
    const ManifoldCache& cache,
    std::span<const uint32_t> changed,
    bool separated,
    std::span<const Contact> contacts,
    std::vector<Island>& islands
  );
  [[nodiscard]] std::vector<uint32_t> split(
    const BodySet& bodies,
    uint32_t island,
    const ManifoldCache& cache,
    std::span<const Contact> contacts
  );
  void requestSplit(uint32_t id);
  [[nodiscard]] uint32_t takeSplit();
  [[nodiscard]] uint32_t pendingSplit() const;

  [[nodiscard]] uint32_t splits() const;
  [[nodiscard]] uint32_t islandOf(uint32_t id) const;
  [[nodiscard]] std::span<const uint32_t> members(uint32_t island) const;
  [[nodiscard]] BodyRef refOf(uint32_t id) const;
  [[nodiscard]] uint32_t removed(uint32_t island) const;
  [[nodiscard]] bool visit(uint32_t island);

private:
  uint32_t create();

  std::vector<uint32_t> m_islandOf;
  std::vector<BodyRef> m_ref;
  std::vector<uint8_t> m_state;
  std::vector<uint32_t> m_present;
  std::vector<std::vector<uint32_t>> m_members;
  std::vector<uint32_t> m_removed;
  std::vector<uint32_t> m_free;
  std::vector<uint32_t> m_output;
  std::vector<uint32_t> m_outputStamp;
  std::vector<uint8_t> m_resting;
  std::vector<uint32_t> m_local;
  std::vector<uint32_t> m_localStamp;
  std::vector<uint32_t> m_parent;
  std::vector<uint32_t> m_visited;
  uint32_t m_stamp = 0u;
  uint32_t m_splits = 0u;
  uint32_t m_splitBody = noBody;
  uint32_t m_count = 0u;
  uint32_t m_bodies = 0u;
  bool m_dirty = true;
  bool m_full = true;
  bool m_builtContacts = false;
};

}
