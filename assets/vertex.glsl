#version 140

in vec3 position;
in vec4 color;
in vec3 normal;
out vec4 v_color;
out vec4 v_normal;
out vec4 v_world_position;
uniform mat4 u_view;
uniform mat4 u_model;
uniform mat4 u_projection;

void main() {
    v_color = color;
    v_world_position = u_model * vec4(position, 1.0);
    v_normal = u_model * vec4(normal, 0.0);
    gl_Position = u_projection * u_view * v_world_position;
}