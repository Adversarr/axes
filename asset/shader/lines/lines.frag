#version 410 core
in vec4 vColor;
out vec4 fColor;

uniform int dim_far_away_from_center;

void main() {
  vec4 color = vColor;

  if (dim_far_away_from_center > 0) {
    float z = gl_FragCoord.z / gl_FragCoord.w;
    float depth = 1.0 - z;
    color.a *= depth;
  }

  fColor = color;
}
