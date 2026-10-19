#pragma once

#include <cstdint>

namespace talus::dynamics {

struct WideBodies {
  float* base = nullptr;
  uint32_t stride = 0u;
  uint32_t v = 0u;
  uint32_t w = 0u;
  uint32_t dp = 0u;
  uint32_t dq = 0u;
};

struct WidePrepare {
  float contactBiasRate = 0.0f;
  float contactMassScale = 0.0f;
  float contactImpulseScale = 0.0f;
  float staticBiasRate = 0.0f;
  float staticMassScale = 0.0f;
  float staticImpulseScale = 0.0f;
  float speculativeDistance = 0.0f;
  float minFrictionWeight = 0.0f;
  float singular = 0.0f;
};

template<uint32_t Width>
struct alignas(4u * Width) WidePoint {
  float anchorA[3][Width];
  float anchorB[3][Width];
  float baseSeparation[Width];
  float normalImpulse[Width];
  float totalNormalImpulse[Width];
  float restitutionImpulse[Width];
  float normalMass[Width];
  float relativeVelocity[Width];
  float leverArm[Width];
  uint32_t live[Width];
};

template<uint32_t Width>
struct alignas(4u * Width) WideContact {
  uint32_t a[Width];
  uint32_t b[Width];
  float invMassA[Width];
  float invMassB[Width];
  float invIA[3][3][Width];
  float invIB[3][3][Width];
  float normal[3][Width];
  float tangent1[3][Width];
  float tangent2[3][Width];
  float centerA[3][Width];
  float centerB[3][Width];
  float tangentMass[2][2][Width];
  float frictionImpulse[2][Width];
  float twistMass[Width];
  float twistImpulse[Width];
  float friction[Width];
  float restitution[Width];
  float biasRate[Width];
  float massScale[Width];
  float impulseScale[Width];
  WidePoint<Width> points[4];
};

void prepareW4(WideContact<4u>& contact, const WideBodies& bodies, const WidePrepare& prepare);
void warmStartW4(WideContact<4u>& contact, const WideBodies& bodies);
void pushW4(WideContact<4u>& contact, const WideBodies& bodies, float invH, float contactSpeed);
void relaxW4(WideContact<4u>& contact, const WideBodies& bodies, float invH);
void restituteW4(WideContact<4u>& contact, const WideBodies& bodies, float invH, float threshold);

void prepareW8(WideContact<8u>& contact, const WideBodies& bodies, const WidePrepare& prepare);
void warmStartW8(WideContact<8u>& contact, const WideBodies& bodies);
void pushW8(WideContact<8u>& contact, const WideBodies& bodies, float invH, float contactSpeed);
void relaxW8(WideContact<8u>& contact, const WideBodies& bodies, float invH);
void restituteW8(WideContact<8u>& contact, const WideBodies& bodies, float invH, float threshold);

}
