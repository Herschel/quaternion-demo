in vec4 v_world_position;
in vec4 v_color;
in vec4 v_normal;
out varying vec4 f_color;

uniform vec3 u_light_position;

void main() {
    vec3 light = normalize(u_light_position - v_world_position.xyz);
    float brightness = clamp(dot(light, normalize(v_normal.xyz)), 0.01, 1.0);
    f_color = brightness * v_color;
}
