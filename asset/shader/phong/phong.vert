#version 410

// NOTE: Traditional
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec4 color;

// NOTE: Instancing
layout(location = 3) in vec3 position_offset_;
layout(location = 4) in vec4 color_offset_;

out vec3 fragNormal;
out vec3 fragPos;
flat out vec4 flatFragColor;
out vec4 fragColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// NOTE: Lighting
uniform int enableLighting;
uniform int enableFlat;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform float ambientStrength;

uniform int enableWireframe;
uniform vec4 wireframeColor;

void main() {
  fragNormal = mat3(transpose(inverse(model))) * normal;
  fragPos = vec3(model * vec4(position_offset_ + position, 1.0));
  vec4 raw_color = color_offset_ + color;
  if (enableWireframe > 0) {
    raw_color = wireframeColor;
  } else if (enableLighting == 1) {
    vec3 norm = normalize(fragNormal);
    vec3 lightDir = normalize(lightPos - fragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * raw_color.xyz;
    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = spec * vec3(1.0);
    vec3 ambient = ambientStrength * raw_color.xyz;
    vec3 result = (ambient + diffuse + specular);
    raw_color = vec4(result, raw_color.w);
  }
  if (enableFlat == 1) {
    flatFragColor = raw_color;
  } else {
    fragColor = raw_color;
  }
  gl_Position = projection * view * vec4(fragPos, 1.0);
}
