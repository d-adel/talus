#include "widecontact.h"

#include <immintrin.h>

#include <cstddef>
#include <cstdint>

namespace talus::dynamics {

namespace {

constexpr uint32_t width = 8u;

using Float = __m256;

Float load(const float* source) {
  return _mm256_load_ps(source);
}

Float loadMask(const uint32_t* source) {
  return _mm256_castsi256_ps(_mm256_load_si256(reinterpret_cast<const __m256i*>(source)));
}

void store(float* target, Float value) {
  _mm256_store_ps(target, value);
}

Float splat(float value) {
  return _mm256_set1_ps(value);
}

Float add(Float a, Float b) {
  return _mm256_add_ps(a, b);
}

Float sub(Float a, Float b) {
  return _mm256_sub_ps(a, b);
}

Float mul(Float a, Float b) {
  return _mm256_mul_ps(a, b);
}

Float divide(Float a, Float b) {
  return _mm256_div_ps(a, b);
}

Float root(Float a) {
  return _mm256_sqrt_ps(a);
}

Float negate(Float a) {
  return _mm256_xor_ps(a, _mm256_set1_ps(-0.0f));
}

Float less(Float a, Float b) {
  return _mm256_cmp_ps(a, b, _CMP_LT_OQ);
}

Float lessEqual(Float a, Float b) {
  return _mm256_cmp_ps(a, b, _CMP_LE_OQ);
}

Float absolute(Float a) {
  return _mm256_andnot_ps(_mm256_set1_ps(-0.0f), a);
}

Float notEqual(Float a, Float b) {
  return _mm256_cmp_ps(a, b, _CMP_NEQ_UQ);
}

Float both(Float a, Float b) {
  return _mm256_and_ps(a, b);
}

Float select(Float mask, Float whenTrue, Float whenFalse) {
  return _mm256_blendv_ps(whenFalse, whenTrue, mask);
}

uint32_t lanes(Float mask) {
  return static_cast<uint32_t>(_mm256_movemask_ps(mask));
}

Float gather(float* const* rows, uint32_t offset) {
  return _mm256_setr_ps(
    rows[0][offset],
    rows[1][offset],
    rows[2][offset],
    rows[3][offset],
    rows[4][offset],
    rows[5][offset],
    rows[6][offset],
    rows[7][offset]
  );
}

#include "widecontact.inl"

}

void prepareW8(WideContact<8u>& contact, const WideBodies& bodies, const WidePrepare& prepare) {
  prepareWide(contact, bodies, prepare);
  _mm256_zeroupper();
}

void warmStartW8(WideContact<8u>& contact, const WideBodies& bodies) {
  warmStartWide(contact, bodies);
  _mm256_zeroupper();
}

void pushW8(WideContact<8u>& contact, const WideBodies& bodies, float invH, float contactSpeed) {
  pushWide(contact, bodies, invH, contactSpeed);
  _mm256_zeroupper();
}

void relaxW8(WideContact<8u>& contact, const WideBodies& bodies, float invH) {
  relaxWide(contact, bodies, invH);
  _mm256_zeroupper();
}

void restituteW8(WideContact<8u>& contact, const WideBodies& bodies, float invH, float threshold) {
  restituteWide(contact, bodies, invH, threshold);
  _mm256_zeroupper();
}

}
