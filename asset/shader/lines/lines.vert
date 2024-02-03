#version 410 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec4 vColor;

void main() {
  gl_Position = projection * view * model * vec4(position, 1.0);
  vColor = color;
  // vColor = gl_Position;
  // gl_Position = vec4(position, 1.0);
}

