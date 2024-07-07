#version 410

// NOTE: Traditional
layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color;
layout(location = 2) in vec3 normal;

// NOTE: Instancing
layout(location = 3) in vec3 position_offset_;
layout(location = 4) in vec4 color_offset_;
layout(location = 5) in vec3 scale_;

out vec3 fragPos;
out vec3 fragNormal;
flat out vec3 flatFragNormal;
flat out vec4 flatFragColor;
out vec4 fragColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  fragNormal = mat3(transpose(inverse(model))) * normal;
  flatFragNormal = fragNormal;
  vec3 real_position;
  real_position.x = position.x * scale_.x;
  real_position.y = position.y * scale_.y;
  real_position.z = position.z * scale_.z;

  fragPos = vec3(model * vec4(position_offset_ + real_position, 1.0));
  vec4 raw_color = color_offset_ + color;
  flatFragColor = raw_color;
  fragColor = raw_color;
  gl_Position = projection * view * vec4(fragPos, 1.0);
}
