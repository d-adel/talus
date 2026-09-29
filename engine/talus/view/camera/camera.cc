#include "camera.ih"

namespace talus::view {

namespace {

constexpr float PITCH_LIMIT = 89.0f;
constexpr float MIN_DISTANCE = 0.05f;
constexpr float MAX_DISTANCE = 5000.0f;
constexpr float PAN_RATE = 1.2f;
constexpr float ZOOM_RATE = 0.12f;

const glm::vec3 WORLD_UP(0.0f, 1.0f, 0.0f);

}

Camera::Camera() : m_position(glm::vec3(0, 0, 2)) {}

void Camera::place(const glm::vec3& position, float yaw, float pitch) {
  m_position = position;
  m_yaw = yaw;
  m_pitch = pitch;
  m_placed = true;
}

bool Camera::placed() const noexcept {
  return m_placed;
}

void Camera::reset() noexcept {
  m_position = glm::vec3(0.0f, 0.0f, 2.0f);
  m_yaw = 0.0f;
  m_pitch = 0.0f;
  m_distance = 10.0f;
  m_placed = false;
}

talus::objectmodel::Pose Camera::pose() const {
  glm::quat pitchRotation = glm::angleAxis(m_pitch, glm::vec3{1.f, 0.f, 0.f});
  glm::quat yawRotation = glm::angleAxis(m_yaw, glm::vec3{0.f, -1.f, 0.f});

  return {m_position, yawRotation * pitchRotation};
}

const talus::objectmodel::Lens& Camera::lens() const noexcept {
  return m_lens;
}

glm::vec3 Camera::forward() const noexcept {
  return pose().rotation * glm::vec3(0.0f, 0.0f, -1.0f);
}

glm::vec3 Camera::pivot() const noexcept {
  return m_position + forward() * m_distance;
}

glm::vec3 Camera::eye() const noexcept {
  return m_position;
}

void Camera::placeFrom(const talus::objectmodel::Pose& seat, float distance) {
  const glm::vec3 ahead = seat.rotation * glm::vec3(0.0f, 0.0f, -1.0f);

  m_distance = glm::clamp(distance, MIN_DISTANCE, MAX_DISTANCE);

  place(
    seat.position,
    std::atan2(ahead.x, -ahead.z),
    std::asin(glm::clamp(ahead.y, -1.0f, 1.0f))
  );
}

void Camera::frame(const glm::vec3& centre, float radius) {
  const float half = glm::radians(0.5f * m_lens.verticalFov);

  const float reach = glm::max(radius, 0.05f) / std::tan(half);

  m_distance = glm::clamp(reach + radius, MIN_DISTANCE, MAX_DISTANCE);

  if (!m_placed) {
    m_yaw = 0.0f;
    m_pitch = glm::radians(-20.0f);
  }

  place(centre - forward() * m_distance, m_yaw, m_pitch);
}

void Camera::orbitAround(const glm::vec3& around) {
  m_position = around - forward() * m_distance;
}

void Camera::orbit(float deltaYaw, float deltaPitch) {
  const glm::vec3 around = pivot();

  m_yaw += deltaYaw;
  m_pitch = glm::clamp(
    m_pitch + deltaPitch,
    glm::radians(-PITCH_LIMIT),
    glm::radians(PITCH_LIMIT)
  );

  orbitAround(around);
}

void Camera::pan(float deltaX, float deltaY) {
  const glm::vec3 ahead = forward();

  const glm::vec3 right = glm::normalize(glm::cross(ahead, WORLD_UP));
  const glm::vec3 up = glm::normalize(glm::cross(right, ahead));

  const float scale = PAN_RATE * m_distance;

  m_position += (-deltaX * right + deltaY * up) * scale;
}

void Camera::zoom(float delta) {
  const glm::vec3 around = pivot();

  m_distance = glm::clamp(
    m_distance * std::exp(-delta * ZOOM_RATE),
    MIN_DISTANCE,
    MAX_DISTANCE
  );

  orbitAround(around);
}

}
