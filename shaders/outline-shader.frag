#version 450

layout(set = 0, binding = 0) uniform texture2D selection;
layout(set = 0, binding = 1) uniform sampler point;
layout(set = 0, binding = 2) uniform texture2D scene;

layout(push_constant) uniform OutlinePush {
  uvec4 radius;
  vec4 colour;
  vec4 dimmed;
} outline;

layout(location = 0) out vec4 colour;

const float BIAS = 1e-4;

bool chosen(ivec2 at) {
  return texelFetch(sampler2D(selection, point), at, 0).r < 1.0;
}

void main() {
  ivec2 at = ivec2(gl_FragCoord.xy);
  ivec2 size = textureSize(sampler2D(selection, point), 0);

  if (chosen(at)) {
    discard;
  }

  int reach = int(outline.radius.x);

  bool found = false;
  bool visible = false;

  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      ivec2 probe = clamp(at + ivec2(dx, dy), ivec2(0), size - 1);

      if (!chosen(probe)) {
        continue;
      }

      found = true;

      float ahead = texelFetch(sampler2D(scene, point), probe, 0).r;
      float own = texelFetch(sampler2D(selection, point), probe, 0).r;

      if (ahead >= own - BIAS) {
        visible = true;
      }
    }
  }

  if (!found) {
    discard;
  }

  colour = visible ? outline.colour : outline.dimmed;
}
