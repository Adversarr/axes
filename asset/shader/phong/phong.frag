#version 410

in vec3 fragNormal;
in vec3 fragPos;
flat in vec4 flatFragColor;
in vec4 fragColor;
uniform int enableFlat;

out vec4 color;

void main() {
  if (enableFlat == 1) {
    color = flatFragColor;
  } else {
    color = fragColor;
  }
}

