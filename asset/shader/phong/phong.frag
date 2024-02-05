#version 410

in vec3 fragPos;
in vec3 fragNormal;
flat in vec4 flatFragColor;
flat in vec3 flatFragNormal;
in vec4 fragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;

// x => Amb
// y => Diffuse
// z => Specular
// w => Flat
uniform vec4 lightCoefficient;

out vec4 color;


vec3 compute_lighting(vec3 normal, vec3 base_color) {
  vec3 lightDir = normalize(lightPos - fragPos);
  float diff = max(dot(normal, lightDir), 0.0);
  vec3 diffuse = diff * base_color;
  vec3 viewDir = normalize(viewPos - fragPos);
  vec3 reflectDir = reflect(-lightDir, normal);
  float spec = pow(max(dot(viewDir, reflectDir), 0.0), 64.0);
  vec3 specular = spec * base_color;
  vec3 ambient = base_color;
  return (lightCoefficient.x * ambient + lightCoefficient.y * diffuse + lightCoefficient.z * specular);
}

void main() {
  vec3 normal = normalize(fragNormal);
  vec3 flat_normal = normalize(flatFragNormal);
  vec4 color_lighted = vec4(compute_lighting(normal, fragColor.xyz), 1.0);
  vec4 color_flat = vec4(compute_lighting(flat_normal, flatFragColor.xyz), 1.0);
  color = color_flat * lightCoefficient.w + color_lighted * (1.0 - lightCoefficient.w);
}

