#pragma once

#include <talus/serialize/glmmeta/glmmeta.h>

#include <talus/field/field.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/name/name.h>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>

template<>
struct glz::meta<talus::field::Primitive> {
  using T = talus::field::Primitive;

  static constexpr size_t MATRIX_VALUES = 16;

  static constexpr auto readInverse =
    [](T& primitive, const std::array<float, MATRIX_VALUES>& values) {
      std::copy(
        values.begin(),
        values.end(),
        glm::value_ptr(primitive.inverseModel)
      );
    };

  static constexpr auto writeInverse = [](const T& primitive) {
    std::array<float, MATRIX_VALUES> values{};

    std::copy_n(
      glm::value_ptr(primitive.inverseModel),
      values.size(),
      values.begin()
    );

    return values;
  };

  static constexpr auto value = glz::object(
    "inverseModel",
    glz::custom<readInverse, writeInverse>,
    "shape",
    &T::shape,
    "shapeConfig",
    &T::shapeConfig,
    "bounds",
    &T::bounds
  );
};

template<>
struct glz::meta<talus::objectmodel::VolumeElementType> {
  using T = talus::objectmodel::VolumeElementType;

  static constexpr auto value = glz::enumerate("hex", T::Hex, "tet", T::Tet);
};

template<>
struct glz::meta<talus::objectmodel::DeformableBody> {
  using T = talus::objectmodel::DeformableBody;

  static constexpr auto value = glz::object("element", &T::element);
};

template<>
struct glz::meta<talus::objectmodel::SdfGrid> {
  using T = talus::objectmodel::SdfGrid;

  static constexpr auto value = glz::object(
    "resolution",
    &T::resolution,
    "margin",
    &T::margin,
    "shadowCullMargin",
    &T::shadowCullMargin
  );
};

template<>
struct glz::meta<talus::objectmodel::Text> {
  using T = talus::objectmodel::Text;

  static constexpr auto read = [](T& text, const std::string& value) {
    text = T::of(value);
  };

  static constexpr auto write = [](const T& text) {
    return std::string(text.view());
  };

  static constexpr auto value = glz::custom<read, write>;
};

template<>
struct glz::meta<talus::objectmodel::Name> {
  using T = talus::objectmodel::Name;

  static constexpr auto value = glz::object("text", &T::text);
};
