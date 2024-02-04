#version 410 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color;

layout(location = 2) in vec3 instancePosition;
layout(location = 3) in vec4 instanceColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;


out vec4 vColor;

void main() {
  gl_Position = projection * view * model * vec4(position + instancePosition, 1.0);
  vColor = color + instanceColor;
}

