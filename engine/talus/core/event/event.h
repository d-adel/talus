#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace talus::core {

class EventDispatcher {
public:
  using SubscriptionId = std::size_t;

  static EventDispatcher& instance() {
    static EventDispatcher dispatcher;
    return dispatcher;
  }

  template<typename Event, typename Callback>
  SubscriptionId subscribe(Callback&& callback) {
    auto& handlers = getHandlers<Event>();

    const SubscriptionId id = m_nextSubscriptionId++;
    handlers.callbacks.emplace(id, std::forward<Callback>(callback));

    return id;
  }

  template<typename Event>
  void unsubscribe(SubscriptionId id) {
    const auto type = std::type_index(typeid(Event));
    const auto it = m_handlerLists.find(type);

    if (it == m_handlerLists.end()) {
      return;
    }

    auto& handlers = static_cast<HandlerList<Event>&>(*it->second);

    handlers.callbacks.erase(id);
  }

  template<typename Event>
  void dispatch(const Event& event) {
    const auto type = std::type_index(typeid(Event));
    const auto it = m_handlerLists.find(type);

    if (it == m_handlerLists.end()) {
      return;
    }

    auto& handlers = static_cast<HandlerList<Event>&>(*it->second);

    std::vector<std::function<void(const Event&)>> callbacks;
    callbacks.reserve(handlers.callbacks.size());

    for (const auto& [id, callback] : handlers.callbacks) {
      callbacks.push_back(callback);
    }

    for (const auto& callback : callbacks) {
      callback(event);
    }
  }

  EventDispatcher(const EventDispatcher&) = delete;
  EventDispatcher& operator=(const EventDispatcher&) = delete;
  EventDispatcher(EventDispatcher&&) = delete;
  EventDispatcher& operator=(EventDispatcher&&) = delete;

private:
  EventDispatcher() = default;

  struct IHandlerList {
    virtual ~IHandlerList() = default;
  };

  template<typename Event>
  struct HandlerList final : IHandlerList {
    std::unordered_map<SubscriptionId, std::function<void(const Event&)>>
      callbacks;
  };

  template<typename Event>
  HandlerList<Event>& getHandlers() {
    const auto type = std::type_index(typeid(Event));

    auto [it, inserted] = m_handlerLists.try_emplace(type);

    if (inserted) {
      it->second = std::make_unique<HandlerList<Event>>();
    }

    return static_cast<HandlerList<Event>&>(*it->second);
  }

  std::unordered_map<std::type_index, std::unique_ptr<IHandlerList>>
    m_handlerLists;

  SubscriptionId m_nextSubscriptionId = 0;
};

}
