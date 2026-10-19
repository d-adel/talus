#include "widecontact.h"

#include <emmintrin.h>

#include <cstddef>
#include <cstdint>

namespace talus::dynamics {

namespace {

constexpr uint32_t width = 4u;

using Float = __m128;

Float load(const float* source) {
  return _mm_load_ps(source);
}

Float loadMask(const uint32_t* source) {
  return _mm_castsi128_ps(_mm_load_si128(reinterpret_cast<const __m128i*>(source)));
}

void store(float* target, Float value) {
  _mm_store_ps(target, value);
}

Float splat(float value) {
  return _mm_set1_ps(value);
}

Float add(Float a, Float b) {
  return _mm_add_ps(a, b);
}

Float sub(Float a, Float b) {
  return _mm_sub_ps(a, b);
}

Float mul(Float a, Float b) {
  return _mm_mul_ps(a, b);
}

Float divide(Float a, Float b) {
  return _mm_div_ps(a, b);
}

Float root(Float a) {
  return _mm_sqrt_ps(a);
}

Float negate(Float a) {
  return _mm_xor_ps(a, _mm_set1_ps(-0.0f));
}

Float less(Float a, Float b) {
  return _mm_cmplt_ps(a, b);
}

Float lessEqual(Float a, Float b) {
  return _mm_cmple_ps(a, b);
}

Float absolute(Float a) {
  return _mm_andnot_ps(_mm_set1_ps(-0.0f), a);
}

Float notEqual(Float a, Float b) {
  return _mm_cmpneq_ps(a, b);
}

Float both(Float a, Float b) {
  return _mm_and_ps(a, b);
}

Float select(Float mask, Float whenTrue, Float whenFalse) {
  return _mm_or_ps(_mm_and_ps(mask, whenTrue), _mm_andnot_ps(mask, whenFalse));
}

uint32_t lanes(Float mask) {
  return static_cast<uint32_t>(_mm_movemask_ps(mask));
}

Float gather(float* const* rows, uint32_t offset) {
  return _mm_setr_ps(rows[0][offset], rows[1][offset], rows[2][offset], rows[3][offset]);
}

#include "widecontact.inl"

}

void prepareW4(WideContact<4u>& contact, const WideBodies& bodies, const WidePrepare& prepare) {
  prepareWide(contact, bodies, prepare);
}

void warmStartW4(WideContact<4u>& contact, const WideBodies& bodies) {
  warmStartWide(contact, bodies);
}

void pushW4(WideContact<4u>& contact, const WideBodies& bodies, float invH, float contactSpeed) {
  pushWide(contact, bodies, invH, contactSpeed);
}

void relaxW4(WideContact<4u>& contact, const WideBodies& bodies, float invH) {
  relaxWide(contact, bodies, invH);
}

void restituteW4(WideContact<4u>& contact, const WideBodies& bodies, float invH, float threshold) {
  restituteWide(contact, bodies, invH, threshold);
}

}
