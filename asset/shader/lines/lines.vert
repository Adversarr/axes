#version 410
in vec3 position;
in vec4 color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec4 vColor;

void main() {
  // gl_Position = projection * view * model * vec4(position, 1.0);
  gl_Position = vec4(position, 1.0);
  vColor = color;
}

