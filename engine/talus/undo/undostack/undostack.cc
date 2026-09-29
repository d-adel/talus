#include "undostack.ih"

namespace talus::undo {

using talus::objectmodel::World;

UndoStack::UndoStack(size_t depth) : m_depth(depth) {}

UndoStack::Step UndoStack::capture(const World& world, std::string label) {
  Step step{std::make_unique<World>(), {}, std::move(label)};

  talus::objectmodel::instantiateWorld(world, *step.world);

  return step;
}

void UndoStack::restore(const World& source, World& destination) {
  destination.clear();

  talus::objectmodel::instantiateWorld(source, destination);
}

void UndoStack::applyBefore(
  const std::vector<FieldDiff>& fields,
  World& world
) {
  for (auto at = fields.rbegin(); at != fields.rend(); ++at) {
    static_cast<void>(talus::serialize::writeField(
      world,
      at->entity,
      at->component,
      at->field,
      at->kind,
      at->before
    ));
  }
}

void UndoStack::applyAfter(const std::vector<FieldDiff>& fields, World& world) {
  for (const FieldDiff& diff : fields) {
    static_cast<void>(talus::serialize::writeField(
      world,
      diff.entity,
      diff.component,
      diff.field,
      diff.kind,
      diff.after
    ));
  }
}

bool UndoStack::begin(const World& world, std::string label) {
  if (m_depth == 0 || m_pending) {
    return false;
  }

  m_pending = capture(world, std::move(label));

  return true;
}

bool UndoStack::beginFields(std::string label) {
  if (m_depth == 0 || m_pending) {
    return false;
  }

  m_pending = Step{nullptr, {}, std::move(label)};

  return true;
}

void UndoStack::note(FieldDiff diff) {
  if (!m_pending || m_pending->snapshot()) {
    return;
  }

  for (FieldDiff& held : m_pending->fields) {
    if (
      held.entity == diff.entity && held.component == diff.component
      && held.field == diff.field
    ) {
      held.after = std::move(diff.after);

      return;
    }
  }

  m_pending->fields.push_back(std::move(diff));
}

bool UndoStack::commit() {
  if (!m_pending) {
    return false;
  }

  if (!m_pending->snapshot() && m_pending->fields.empty()) {
    m_pending.reset();

    return false;
  }

  m_undo.push_back(std::move(*m_pending));
  m_pending.reset();

  m_redo.clear();

  while (m_undo.size() > m_depth) {
    m_undo.pop_front();
  }

  return true;
}

bool UndoStack::discard(World& world) {
  if (!m_pending) {
    return false;
  }

  if (m_pending->snapshot()) {
    restore(*m_pending->world, world);
  } else {
    applyBefore(m_pending->fields, world);
  }

  m_pending.reset();

  return true;
}

bool UndoStack::open() const noexcept {
  return m_pending.has_value();
}

bool UndoStack::fieldsOpen() const noexcept {
  return m_pending.has_value() && !m_pending->snapshot();
}

void UndoStack::record(const World& world, std::string label) {
  if (begin(world, std::move(label))) {
    static_cast<void>(commit());
  }
}

bool UndoStack::undo(World& world) {
  if (m_undo.empty()) {
    return false;
  }

  Step step = std::move(m_undo.back());
  m_undo.pop_back();

  if (step.snapshot()) {
    m_redo.push_back(capture(world, step.label));
    restore(*step.world, world);
  } else {
    applyBefore(step.fields, world);
    m_redo.push_back(std::move(step));
  }

  return true;
}

bool UndoStack::redo(World& world) {
  if (m_redo.empty()) {
    return false;
  }

  Step step = std::move(m_redo.back());
  m_redo.pop_back();

  if (step.snapshot()) {
    m_undo.push_back(capture(world, step.label));
    restore(*step.world, world);
  } else {
    applyAfter(step.fields, world);
    m_undo.push_back(std::move(step));
  }

  return true;
}

bool UndoStack::undoIsFields() const noexcept {
  return !m_undo.empty() && !m_undo.back().snapshot();
}

bool UndoStack::redoIsFields() const noexcept {
  return !m_redo.empty() && !m_redo.back().snapshot();
}

size_t UndoStack::depth() const noexcept {
  return m_undo.size();
}

void UndoStack::forgetSince(size_t depth) {
  while (m_undo.size() > depth) {
    m_undo.pop_back();
  }

  m_redo.clear();
  m_pending.reset();
}

void UndoStack::clear() noexcept {
  m_undo.clear();
  m_redo.clear();
  m_pending.reset();
}

std::optional<std::string_view> UndoStack::undoLabel() const noexcept {
  if (m_undo.empty()) {
    return std::nullopt;
  }

  return std::string_view{m_undo.back().label};
}

std::optional<std::string_view> UndoStack::redoLabel() const noexcept {
  if (m_redo.empty()) {
    return std::nullopt;
  }

  return std::string_view{m_redo.back().label};
}

}
