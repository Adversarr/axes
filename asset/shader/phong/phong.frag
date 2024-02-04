#version 410

in vec3 fragPos;
in vec3 fragNormal;
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

