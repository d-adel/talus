#version 450

layout(location = 0) out vec2 ndc;

void main() {
  vec2 corner = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);

  ndc = corner * 2.0 - 1.0;

  gl_Position = vec4(ndc, 0.0, 1.0);
}
