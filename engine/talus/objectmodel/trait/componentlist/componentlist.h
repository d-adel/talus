#pragma once

#include <talus/objectmodel/trait/authored/authored.h>

namespace talus::objectmodel {

template<Authored... Ts>
struct ComponentList {
  template<typename Fn>
  static void each(Fn&& fn) {
    (fn.template operator()<Ts>(), ...);
  }
};

}
