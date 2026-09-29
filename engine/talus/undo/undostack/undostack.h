#pragma once

#include <talus/objectmodel/dynamiccomponents/dynamiccomponents.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace talus::undo {

struct FieldDiff {
  talus::objectmodel::Entity entity = talus::objectmodel::INVALID_ENTITY;
  std::string component;
  uint32_t field = 0u;
  talus::objectmodel::FieldKind kind = talus::objectmodel::FieldKind::Float;
  std::vector<std::byte> before;
  std::vector<std::byte> after;
};

class UndoStack {
public:
  explicit UndoStack(size_t depth);

  UndoStack(const UndoStack&) = delete;
  UndoStack& operator=(const UndoStack&) = delete;
  UndoStack(UndoStack&&) = delete;
  UndoStack& operator=(UndoStack&&) = delete;
  ~UndoStack() = default;

  [[nodiscard]]
  bool begin(const talus::objectmodel::World& world, std::string label);

  [[nodiscard]]
  bool beginFields(std::string label);

  void note(FieldDiff diff);

  [[nodiscard]]
  bool commit();

  [[nodiscard]]
  bool discard(talus::objectmodel::World& world);

  [[nodiscard]]
  bool open() const noexcept;

  [[nodiscard]]
  bool fieldsOpen() const noexcept;

  void record(const talus::objectmodel::World& world, std::string label);

  [[nodiscard]]
  bool undo(talus::objectmodel::World& world);

  [[nodiscard]]
  bool redo(talus::objectmodel::World& world);

  [[nodiscard]]
  bool undoIsFields() const noexcept;

  [[nodiscard]]
  bool redoIsFields() const noexcept;

  [[nodiscard]]
  size_t depth() const noexcept;

  void forgetSince(size_t depth);

  void clear() noexcept;

  [[nodiscard]]
  std::optional<std::string_view> undoLabel() const noexcept;

  [[nodiscard]]
  std::optional<std::string_view> redoLabel() const noexcept;

private:
  struct Step {
    std::unique_ptr<talus::objectmodel::World> world;
    std::vector<FieldDiff> fields;
    std::string label;

    [[nodiscard]] bool snapshot() const noexcept {
      return world != nullptr;
    }
  };

  [[nodiscard]]
  static Step capture(
    const talus::objectmodel::World& world,
    std::string label
  );

  static void restore(
    const talus::objectmodel::World& source,
    talus::objectmodel::World& destination
  );

  static void applyBefore(
    const std::vector<FieldDiff>& fields,
    talus::objectmodel::World& world
  );

  static void applyAfter(
    const std::vector<FieldDiff>& fields,
    talus::objectmodel::World& world
  );

  std::deque<Step> m_undo;
  std::vector<Step> m_redo;
  std::optional<Step> m_pending;
  size_t m_depth;
};

}
