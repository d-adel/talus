struct Vec3 {
  Float x;
  Float y;
  Float z;
};

struct Mat3 {
  Float m[3][3];
};

struct Quat {
  Float x;
  Float y;
  Float z;
  Float w;
};

struct Rows {
  float* a[width];
  float* b[width];
};

Vec3 add3(const Vec3& a, const Vec3& b) {
  return {add(a.x, b.x), add(a.y, b.y), add(a.z, b.z)};
}

Vec3 sub3(const Vec3& a, const Vec3& b) {
  return {sub(a.x, b.x), sub(a.y, b.y), sub(a.z, b.z)};
}

Vec3 scale(Float s, const Vec3& v) {
  return {mul(s, v.x), mul(s, v.y), mul(s, v.z)};
}

Float dot3(const Vec3& a, const Vec3& b) {
  return add(add(mul(a.x, b.x), mul(a.y, b.y)), mul(a.z, b.z));
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
  return {
    sub(mul(a.y, b.z), mul(b.y, a.z)),
    sub(mul(a.z, b.x), mul(b.z, a.x)),
    sub(mul(a.x, b.y), mul(b.x, a.y))
  };
}

Vec3 transform(const Mat3& m, const Vec3& v) {
  return {
    add(add(mul(m.m[0][0], v.x), mul(m.m[1][0], v.y)), mul(m.m[2][0], v.z)),
    add(add(mul(m.m[0][1], v.x), mul(m.m[1][1], v.y)), mul(m.m[2][1], v.z)),
    add(add(mul(m.m[0][2], v.x), mul(m.m[1][2], v.y)), mul(m.m[2][2], v.z))
  };
}

Vec3 rotate(const Quat& q, const Vec3& v) {
  const Vec3 axis{q.x, q.y, q.z};
  const Vec3 uv = cross3(axis, v);
  const Vec3 uuv = cross3(axis, uv);
  const Float two = splat(2.0f);

  return {
    add(v.x, mul(add(mul(uv.x, q.w), uuv.x), two)),
    add(v.y, mul(add(mul(uv.y, q.w), uuv.y), two)),
    add(v.z, mul(add(mul(uv.z, q.w), uuv.z), two))
  };
}

Vec3 select3(Float mask, const Vec3& whenTrue, const Vec3& whenFalse) {
  return {
    select(mask, whenTrue.x, whenFalse.x),
    select(mask, whenTrue.y, whenFalse.y),
    select(mask, whenTrue.z, whenFalse.z)
  };
}

Float maximum(Float a, Float b) {
  return select(less(a, b), b, a);
}

Float minimum(Float a, Float b) {
  return select(less(b, a), b, a);
}

Float clamp(Float value, Float low, Float high) {
  return select(less(value, low), low, select(less(high, value), high, value));
}

Vec3 load3(const float (&source)[3][width]) {
  return {load(source[0]), load(source[1]), load(source[2])};
}

Mat3 load33(const float (&source)[3][3][width]) {
  Mat3 result{};

  for (uint32_t column = 0u; column != 3u; ++column) {
    for (uint32_t row = 0u; row != 3u; ++row) {
      result.m[column][row] = load(source[column][row]);
    }
  }

  return result;
}

Rows rowsOf(const WideContact<width>& contact, const WideBodies& bodies) {
  Rows rows{};

  for (uint32_t lane = 0u; lane != width; ++lane) {
    rows.a[lane] = bodies.base + static_cast<std::size_t>(contact.a[lane]) * bodies.stride;
    rows.b[lane] = bodies.base + static_cast<std::size_t>(contact.b[lane]) * bodies.stride;
  }

  return rows;
}

Vec3 gather3(float* const* rows, uint32_t offset) {
  return {gather(rows, offset), gather(rows, offset + 1u), gather(rows, offset + 2u)};
}

Quat gatherQuat(float* const* rows, uint32_t offset) {
  return {
    gather(rows, offset),
    gather(rows, offset + 1u),
    gather(rows, offset + 2u),
    gather(rows, offset + 3u)
  };
}

void scatter(
  float* const* rows,
  const WideBodies& bodies,
  Float write,
  const Vec3& v,
  const Vec3& w
) {
  const uint32_t mask = lanes(write);

  if (mask == 0u) {
    return;
  }

  alignas(4u * width) float values[6][width];

  store(values[0], v.x);
  store(values[1], v.y);
  store(values[2], v.z);
  store(values[3], w.x);
  store(values[4], w.y);
  store(values[5], w.z);

  for (uint32_t lane = 0u; lane != width; ++lane) {
    if (((mask >> lane) & 1u) == 0u) {
      continue;
    }

    float* row = rows[lane];

    row[bodies.v] = values[0][lane];
    row[bodies.v + 1u] = values[1][lane];
    row[bodies.v + 2u] = values[2][lane];
    row[bodies.w] = values[3][lane];
    row[bodies.w + 1u] = values[4][lane];
    row[bodies.w + 2u] = values[5][lane];
  }
}

void warmStartWide(WideContact<width>& contact, const WideBodies& bodies) {
  const Rows rows = rowsOf(contact, bodies);
  const Float zero = splat(0.0f);
  const Float mA = load(contact.invMassA);
  const Float mB = load(contact.invMassB);
  const Mat3 iA = load33(contact.invIA);
  const Mat3 iB = load33(contact.invIB);
  const Vec3 normal = load3(contact.normal);
  Float totalNormalImpulse = zero;
  Vec3 momentA{zero, zero, zero};
  Vec3 momentB{zero, zero, zero};

  for (uint32_t index = 0u; index != 4u; ++index) {
    const WidePoint<width>& point = contact.points[index];
    const Float live = loadMask(point.live);
    const Float impulse = load(point.normalImpulse);

    totalNormalImpulse = select(live, add(totalNormalImpulse, impulse), totalNormalImpulse);
    momentA = select3(live, add3(momentA, scale(impulse, load3(point.anchorA))), momentA);
    momentB = select3(live, add3(momentB, scale(impulse, load3(point.anchorB))), momentB);
  }

  const Vec3 frictionImpulse = add3(
    scale(load(contact.frictionImpulse[0]), load3(contact.tangent1)),
    scale(load(contact.frictionImpulse[1]), load3(contact.tangent2))
  );
  const Vec3 linearImpulse = add3(frictionImpulse, scale(totalNormalImpulse, normal));
  const Vec3 twistImpulse = scale(load(contact.twistImpulse), normal);
  const Vec3 angularA = add3(
    add3(cross3(momentA, normal), cross3(load3(contact.centerA), frictionImpulse)),
    twistImpulse
  );
  const Vec3 angularB = add3(
    add3(cross3(momentB, normal), cross3(load3(contact.centerB), frictionImpulse)),
    twistImpulse
  );
  const Vec3 vA = sub3(gather3(rows.a, bodies.v), scale(mA, linearImpulse));
  const Vec3 wA = sub3(gather3(rows.a, bodies.w), transform(iA, angularA));
  const Vec3 vB = add3(gather3(rows.b, bodies.v), scale(mB, linearImpulse));
  const Vec3 wB = add3(gather3(rows.b, bodies.w), transform(iB, angularB));

  scatter(rows.a, bodies, notEqual(mA, zero), vA, wA);
  scatter(rows.b, bodies, notEqual(mB, zero), vB, wB);
}

void pushWide(
  WideContact<width>& contact,
  const WideBodies& bodies,
  float invH,
  float contactSpeed
) {
  const Rows rows = rowsOf(contact, bodies);
  const Float zero = splat(0.0f);
  const Float one = splat(1.0f);
  const Float rate = splat(invH);
  const Float slowest = splat(-contactSpeed);
  const Float mA = load(contact.invMassA);
  const Float mB = load(contact.invMassB);
  const Mat3 iA = load33(contact.invIA);
  const Mat3 iB = load33(contact.invIB);
  Vec3 vA = gather3(rows.a, bodies.v);
  Vec3 wA = gather3(rows.a, bodies.w);
  Vec3 vB = gather3(rows.b, bodies.v);
  Vec3 wB = gather3(rows.b, bodies.w);
  const Vec3 dp = sub3(gather3(rows.b, bodies.dp), gather3(rows.a, bodies.dp));
  const Quat dqA = gatherQuat(rows.a, bodies.dq);
  const Quat dqB = gatherQuat(rows.b, bodies.dq);
  const Vec3 normal = load3(contact.normal);
  const Float massScale = load(contact.massScale);
  const Float impulseScale = load(contact.impulseScale);
  const Float softBias = mul(massScale, load(contact.biasRate));

  for (uint32_t index = 0u; index != 4u; ++index) {
    WidePoint<width>& point = contact.points[index];
    const Float live = loadMask(point.live);
    const Vec3 rA = load3(point.anchorA);
    const Vec3 rB = load3(point.anchorB);
    const Vec3 ds = add3(dp, sub3(rotate(dqB, rB), rotate(dqA, rA)));
    const Float s = add(dot3(ds, normal), load(point.baseSeparation));
    const Float apart = less(zero, s);
    const Float velocityBias =
      select(apart, mul(s, rate), maximum(mul(softBias, s), slowest));
    const Float pointMassScale = select(apart, one, massScale);
    const Float pointImpulseScale = select(apart, zero, impulseScale);
    const Vec3 vrA = add3(vA, cross3(wA, rA));
    const Vec3 vrB = add3(vB, cross3(wB, rB));
    const Float vn = dot3(sub3(vrB, vrA), normal);
    const Float impulse = load(point.normalImpulse);
    const Float delta = sub(
      mul(negate(load(point.normalMass)), add(mul(pointMassScale, vn), velocityBias)),
      mul(pointImpulseScale, impulse)
    );
    const Float newImpulse = maximum(add(impulse, delta), zero);
    const Vec3 P = scale(sub(newImpulse, impulse), normal);

    store(point.normalImpulse, select(live, newImpulse, impulse));
    vA = select3(live, sub3(vA, scale(mA, P)), vA);
    wA = select3(live, sub3(wA, transform(iA, cross3(rA, P))), wA);
    vB = select3(live, add3(vB, scale(mB, P)), vB);
    wB = select3(live, add3(wB, transform(iB, cross3(rB, P))), wB);
  }

  scatter(rows.a, bodies, notEqual(mA, zero), vA, wA);
  scatter(rows.b, bodies, notEqual(mB, zero), vB, wB);
}

void relaxWide(WideContact<width>& contact, const WideBodies& bodies, float invH) {
  const Rows rows = rowsOf(contact, bodies);
  const Float zero = splat(0.0f);
  const Float rate = splat(invH);
  const Float mA = load(contact.invMassA);
  const Float mB = load(contact.invMassB);
  const Mat3 iA = load33(contact.invIA);
  const Mat3 iB = load33(contact.invIB);
  Vec3 vA = gather3(rows.a, bodies.v);
  Vec3 wA = gather3(rows.a, bodies.w);
  Vec3 vB = gather3(rows.b, bodies.v);
  Vec3 wB = gather3(rows.b, bodies.w);
  const Vec3 dp = sub3(gather3(rows.b, bodies.dp), gather3(rows.a, bodies.dp));
  const Quat dqA = gatherQuat(rows.a, bodies.dq);
  const Quat dqB = gatherQuat(rows.b, bodies.dq);
  const Vec3 normal = load3(contact.normal);
  const Float friction = load(contact.friction);
  Float totalNormalImpulse = zero;
  Float totalTwistLimit = zero;

  for (uint32_t index = 0u; index != 4u; ++index) {
    WidePoint<width>& point = contact.points[index];
    const Float live = loadMask(point.live);
    const Vec3 rA = load3(point.anchorA);
    const Vec3 rB = load3(point.anchorB);
    const Vec3 ds = add3(dp, sub3(rotate(dqB, rB), rotate(dqA, rA)));
    const Float s = add(dot3(ds, normal), load(point.baseSeparation));
    const Float velocityBias = select(less(zero, s), mul(s, rate), zero);
    const Vec3 vrA = add3(vA, cross3(wA, rA));
    const Vec3 vrB = add3(vB, cross3(wB, rB));
    const Float vn = dot3(sub3(vrB, vrA), normal);
    const Float impulse = load(point.normalImpulse);
    const Float delta = mul(negate(load(point.normalMass)), add(vn, velocityBias));
    const Float newImpulse = maximum(add(impulse, delta), zero);
    const Float total = load(point.totalNormalImpulse);
    const Vec3 P = scale(sub(newImpulse, impulse), normal);

    store(point.normalImpulse, select(live, newImpulse, impulse));
    store(point.totalNormalImpulse, select(live, add(total, newImpulse), total));
    totalNormalImpulse = select(live, add(totalNormalImpulse, newImpulse), totalNormalImpulse);
    totalTwistLimit = select(
      live,
      add(totalTwistLimit, mul(load(point.leverArm), newImpulse)),
      totalTwistLimit
    );
    vA = select3(live, sub3(vA, scale(mA, P)), vA);
    wA = select3(live, sub3(wA, transform(iA, cross3(rA, P))), wA);
    vB = select3(live, add3(vB, scale(mB, P)), vB);
    wB = select3(live, add3(wB, transform(iB, cross3(rB, P))), wB);
  }

  {
    const Float twistSpeed = dot3(normal, sub3(wB, wA));
    const Float maxImpulse = mul(friction, totalTwistLimit);
    const Float oldImpulse = load(contact.twistImpulse);
    const Float newImpulse = clamp(
      sub(oldImpulse, mul(load(contact.twistMass), twistSpeed)),
      negate(maxImpulse),
      maxImpulse
    );
    const Vec3 twist = scale(sub(newImpulse, oldImpulse), normal);

    store(contact.twistImpulse, newImpulse);
    wA = sub3(wA, transform(iA, twist));
    wB = add3(wB, transform(iB, twist));
  }

  {
    const Vec3 rA = load3(contact.centerA);
    const Vec3 rB = load3(contact.centerB);
    const Vec3 tangent1 = load3(contact.tangent1);
    const Vec3 tangent2 = load3(contact.tangent2);
    const Vec3 vr = sub3(add3(vB, cross3(wB, rB)), add3(vA, cross3(wA, rA)));
    const Float vt1 = dot3(vr, tangent1);
    const Float vt2 = dot3(vr, tangent2);
    const Float deltaX = negate(add(
      mul(load(contact.tangentMass[0][0]), vt1),
      mul(load(contact.tangentMass[1][0]), vt2)
    ));
    const Float deltaY = negate(add(
      mul(load(contact.tangentMass[0][1]), vt1),
      mul(load(contact.tangentMass[1][1]), vt2)
    ));
    const Float oldX = load(contact.frictionImpulse[0]);
    const Float oldY = load(contact.frictionImpulse[1]);
    const Float maxImpulse = mul(friction, totalNormalImpulse);
    Float newX = add(oldX, deltaX);
    Float newY = add(oldY, deltaY);
    const Float lengthSquared = add(mul(newX, newX), mul(newY, newY));
    const Float clipped = less(mul(maxImpulse, maxImpulse), lengthSquared);
    const Float factor = divide(maxImpulse, root(lengthSquared));

    newX = select(clipped, mul(newX, factor), newX);
    newY = select(clipped, mul(newY, factor), newY);

    const Vec3 P = add3(scale(sub(newX, oldX), tangent1), scale(sub(newY, oldY), tangent2));

    store(contact.frictionImpulse[0], newX);
    store(contact.frictionImpulse[1], newY);
    vA = sub3(vA, scale(mA, P));
    wA = sub3(wA, transform(iA, cross3(rA, P)));
    vB = add3(vB, scale(mB, P));
    wB = add3(wB, transform(iB, cross3(rB, P)));
  }

  scatter(rows.a, bodies, notEqual(mA, zero), vA, wA);
  scatter(rows.b, bodies, notEqual(mB, zero), vB, wB);
}

void restituteWide(
  WideContact<width>& contact,
  const WideBodies& bodies,
  float invH,
  float threshold
) {
  const Float zero = splat(0.0f);
  const Float restitution = load(contact.restitution);
  const Float active = notEqual(restitution, zero);

  if (lanes(active) == 0u) {
    return;
  }

  const Rows rows = rowsOf(contact, bodies);
  const Float rate = splat(invH);
  const Float limit = splat(-threshold);
  const Float bouncing = less(zero, restitution);
  const Float mA = load(contact.invMassA);
  const Float mB = load(contact.invMassB);
  const Mat3 iA = load33(contact.invIA);
  const Mat3 iB = load33(contact.invIB);
  Vec3 vA = gather3(rows.a, bodies.v);
  Vec3 wA = gather3(rows.a, bodies.w);
  Vec3 vB = gather3(rows.b, bodies.v);
  Vec3 wB = gather3(rows.b, bodies.w);
  const Vec3 dp = sub3(gather3(rows.b, bodies.dp), gather3(rows.a, bodies.dp));
  const Quat dqA = gatherQuat(rows.a, bodies.dq);
  const Quat dqB = gatherQuat(rows.b, bodies.dq);
  const Vec3 normal = load3(contact.normal);

  for (uint32_t index = 0u; index != 4u; ++index) {
    WidePoint<width>& point = contact.points[index];
    const Float live = both(loadMask(point.live), active);
    const Float total = load(point.totalNormalImpulse);
    const Float restitutionImpulse = load(point.restitutionImpulse);
    const Float relativeVelocity = load(point.relativeVelocity);
    const Float compressionImpulse = sub(total, restitutionImpulse);
    const Float armed = both(
      both(bouncing, less(relativeVelocity, limit)),
      less(zero, compressionImpulse)
    );
    const Vec3 rA = load3(point.anchorA);
    const Vec3 rB = load3(point.anchorB);
    const Vec3 ds = add3(dp, sub3(rotate(dqB, rB), rotate(dqA, rA)));
    const Float s = add(dot3(ds, normal), load(point.baseSeparation));
    const Float velocityBias = select(
      armed,
      mul(restitution, relativeVelocity),
      select(less(zero, s), mul(s, rate), zero)
    );
    const Vec3 vrA = add3(vA, cross3(wA, rA));
    const Vec3 vrB = add3(vB, cross3(wB, rB));
    const Float vn = dot3(sub3(vrB, vrA), normal);
    const Float normalMass = load(point.normalMass);
    const Float oldImpulse = load(point.normalImpulse);
    const Float newImpulse = maximum(
      add(oldImpulse, mul(negate(normalMass), add(vn, velocityBias))),
      zero
    );
    const Float impulse = sub(newImpulse, oldImpulse);
    const Float approachImpulse = minimum(
      maximum(mul(negate(normalMass), vn), zero),
      maximum(impulse, zero)
    );
    const Float allowance = sub(
      mul(restitution, add(compressionImpulse, approachImpulse)),
      restitutionImpulse
    );
    const Float applied = select(
      armed,
      minimum(impulse, add(approachImpulse, maximum(allowance, zero))),
      impulse
    );
    const Vec3 P = scale(applied, normal);

    store(point.normalImpulse, select(live, add(oldImpulse, applied), oldImpulse));
    store(
      point.restitutionImpulse,
      select(live, add(restitutionImpulse, sub(applied, approachImpulse)), restitutionImpulse)
    );
    store(point.totalNormalImpulse, select(live, add(total, applied), total));
    vA = select3(live, sub3(vA, scale(mA, P)), vA);
    wA = select3(live, sub3(wA, transform(iA, cross3(rA, P))), wA);
    vB = select3(live, add3(vB, scale(mB, P)), vB);
    wB = select3(live, add3(wB, transform(iB, cross3(rB, P))), wB);
  }

  scatter(rows.a, bodies, both(notEqual(mA, zero), active), vA, wA);
  scatter(rows.b, bodies, both(notEqual(mB, zero), active), vB, wB);
}

void prepareWide(WideContact<width>& contact, const WideBodies& bodies, const WidePrepare& prepare) {
  const Rows rows = rowsOf(contact, bodies);
  const Float zero = splat(0.0f);
  const Float one = splat(1.0f);
  const Float mA = load(contact.invMassA);
  const Float mB = load(contact.invMassB);
  const Mat3 iA = load33(contact.invIA);
  const Mat3 iB = load33(contact.invIB);
  const Vec3 normal = load3(contact.normal);
  const Vec3 tangent1 = load3(contact.tangent1);
  const Vec3 tangent2 = load3(contact.tangent2);
  const Vec3 vA = gather3(rows.a, bodies.v);
  const Vec3 wA = gather3(rows.a, bodies.w);
  const Vec3 vB = gather3(rows.b, bodies.v);
  const Vec3 wB = gather3(rows.b, bodies.w);
  const Float moving = both(notEqual(mA, zero), notEqual(mB, zero));

  store(contact.biasRate, select(moving, splat(prepare.contactBiasRate), splat(prepare.staticBiasRate)));
  store(contact.massScale, select(moving, splat(prepare.contactMassScale), splat(prepare.staticMassScale)));
  store(
    contact.impulseScale,
    select(moving, splat(prepare.contactImpulseScale), splat(prepare.staticImpulseScale))
  );

  Vec3 centerA{zero, zero, zero};
  Vec3 centerB{zero, zero, zero};
  Float totalWeight = zero;

  for (uint32_t index = 0u; index != 4u; ++index) {
    WidePoint<width>& point = contact.points[index];
    const Float live = loadMask(point.live);
    const Vec3 rA = load3(point.anchorA);
    const Vec3 rB = load3(point.anchorB);
    const Float separation = load(point.baseSeparation);
    const Vec3 rnA = cross3(rA, normal);
    const Vec3 rnB = cross3(rB, normal);
    const Float kNormal = add(
      add(add(mA, mB), dot3(rnA, transform(iA, rnA))),
      dot3(rnB, transform(iB, rnB))
    );
    const Vec3 vrA = add3(vA, cross3(wA, rA));
    const Vec3 vrB = add3(vB, cross3(wB, rB));
    const Float weight = clamp(
      sub(splat(2.0f), divide(separation, splat(prepare.speculativeDistance))),
      splat(prepare.minFrictionWeight),
      one
    );

    store(point.baseSeparation, select(live, sub(separation, dot3(sub3(rB, rA), normal)), zero));
    store(point.normalMass, select(live, select(less(zero, kNormal), divide(one, kNormal), zero), zero));
    store(point.relativeVelocity, select(live, dot3(normal, sub3(vrB, vrA)), zero));
    centerA = select3(live, add3(centerA, scale(weight, rA)), centerA);
    centerB = select3(live, add3(centerB, scale(weight, rB)), centerB);
    totalWeight = select(live, add(totalWeight, weight), totalWeight);
  }

  centerA = {divide(centerA.x, totalWeight), divide(centerA.y, totalWeight), divide(centerA.z, totalWeight)};
  centerB = {divide(centerB.x, totalWeight), divide(centerB.y, totalWeight), divide(centerB.z, totalWeight)};

  for (uint32_t axis = 0u; axis != 3u; ++axis) {
    store(contact.centerA[axis], axis == 0u ? centerA.x : axis == 1u ? centerA.y : centerA.z);
    store(contact.centerB[axis], axis == 0u ? centerB.x : axis == 1u ? centerB.y : centerB.z);
  }

  for (uint32_t index = 0u; index != 4u; ++index) {
    WidePoint<width>& point = contact.points[index];
    const Float live = loadMask(point.live);
    const Vec3 offset = sub3(centerA, load3(point.anchorA));

    store(point.leverArm, select(live, root(dot3(offset, offset)), zero));
  }

  const Vec3 rtA1 = cross3(centerA, tangent1);
  const Vec3 rtA2 = cross3(centerA, tangent2);
  const Vec3 rtB1 = cross3(centerB, tangent1);
  const Vec3 rtB2 = cross3(centerB, tangent2);
  const Float k00 = add(
    add(add(mA, mB), dot3(rtA1, transform(iA, rtA1))),
    dot3(rtB1, transform(iB, rtB1))
  );
  const Float k11 = add(
    add(add(mA, mB), dot3(rtA2, transform(iA, rtA2))),
    dot3(rtB2, transform(iB, rtB2))
  );
  const Float k01 = add(dot3(rtA1, transform(iA, rtA2)), dot3(rtB1, transform(iB, rtB2)));
  const Float det = sub(mul(k00, k11), mul(k01, k01));
  const Float degenerate = lessEqual(absolute(det), splat(prepare.singular));
  const Float invDet = divide(one, det);

  store(contact.tangentMass[0][0], select(degenerate, zero, mul(invDet, k11)));
  store(contact.tangentMass[0][1], select(degenerate, zero, mul(negate(invDet), k01)));
  store(contact.tangentMass[1][0], select(degenerate, zero, mul(negate(invDet), k01)));
  store(contact.tangentMass[1][1], select(degenerate, zero, mul(invDet, k00)));

  Mat3 inertia{};

  for (uint32_t column = 0u; column != 3u; ++column) {
    for (uint32_t row = 0u; row != 3u; ++row) {
      inertia.m[column][row] = add(iA.m[column][row], iB.m[column][row]);
    }
  }

  const Float twist = dot3(normal, transform(inertia, normal));

  store(contact.twistMass, select(less(zero, twist), divide(one, twist), zero));
}
