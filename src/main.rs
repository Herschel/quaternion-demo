#[macro_use]
extern crate conrod;
#[macro_use]
extern crate glium;
extern crate nalgebra;

mod quaternion;

use conrod::{widget, color, Colorable, Positionable, Labelable, Sizeable, Widget};
use glium::{Program, Surface, IndexBuffer, VertexBuffer};
use quaternion::Quaternion;
use std::error::Error;
use nalgebra::base::Vector3;
use nalgebra::core::Matrix4;

#[derive(Copy, Clone)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 4],
    normal: [f32; 3],
}
implement_vertex!(Vertex, position, color, normal);

#[derive(Copy, Clone, Debug)]
struct Transform {
    position: Vector3<f32>,
    rotation: Quaternion,
    scale: Vector3<f32>,
}

impl Transform {
    fn new() -> Transform {
        Transform {
            position: [0.0, 0.0, 0.0].into(),
            rotation: Quaternion::identity(),
            scale: [1.0, 1.0, 1.0].into(),
        }
    }

    fn to_matrix(&self) -> Matrix4<f32> {
        let mut pos = Matrix4::identity();
        pos[(0,3)] = self.position[0];
        pos[(1,3)] = self.position[1];
        pos[(2,3)] = self.position[2];
        
        let rot = self.rotation.into_matrix();

        let mut scale = Matrix4::identity();
        scale[(0,0)] = self.scale[0];
        scale[(1,1)] = self.scale[1];
        scale[(2,2)] = self.scale[2];

        pos * rot * scale
    }
}

struct Model {
    vertex_buffer: VertexBuffer<Vertex>,
    index_buffer: IndexBuffer<u16>,
    transform: Transform,
}

struct Camera {
    transform: Transform,
    projection: Matrix4<f32>,
}

impl Camera {
    fn new() -> Self {
        Camera {
            transform: Transform::new(),
            projection: Matrix4::new_perspective(16.0 / 9.0, 3.14 / 4.0, 1.0, 1000.0),
        }
    }
}
fn main() -> Result<(), Box<Error>> {
    const WIDTH: u32 = 1280;
    const HEIGHT: u32 = 720;

    // Build the window.
    let mut events_loop = glium::glutin::EventsLoop::new();
    let window = glium::glutin::WindowBuilder::new()
        .with_title("Quaternion Demo")
        .with_dimensions(WIDTH, HEIGHT);
    let context = glium::glutin::ContextBuilder::new()
        .with_vsync(true)
        .with_multisampling(4);
    let display = glium::Display::new(window, context, &events_loop).unwrap();

    // construct our `Ui`.
    let mut ui = conrod::UiBuilder::new([WIDTH as f64, HEIGHT as f64]).build();

    // Generate the widget identifiers.
    widget_ids!(struct Ids {
        canvas,

        euler_angles_button,
        axis_angle_button,

        yaw, pitch, roll,
        axis_x, axis_y, axis_z,
        axis_angle,

        add_rotation,
        clear_rotations,
        animate_rotations
    });
    let ids = Ids::new(ui.widget_id_generator());

    // Add a `Font` to the `Ui`'s `font::Map` from file.
    const FONT_PATH: &'static str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/assets/NotoSans-Regular.ttf");
    ui.fonts.insert_from_file(FONT_PATH).unwrap();

    // A type used for converting `conrod::render::Primitives` into `Command`s that can be used
    // for drawing to the glium `Surface`.
    let mut renderer = conrod::backend::glium::Renderer::new(&display).unwrap();

    // The image map describing each of our widget->image mappings (in our case, none).
    let image_map = conrod::image::Map::<glium::texture::Texture2d>::new();

    let program = create_shader_program(&display)?;
    let mut camera = Camera::new();
    camera.transform.position[2] = 5.0;
    let mut model = create_axes_model(&display)?;

    let mut quaternion_list: Vec<Quaternion> = vec![Quaternion::identity()];
    let mut euler_angles: [f32; 3] = [0.0; 3];
    let mut axis: Vector3<f32> = [1.0, 0.0, 0.0].into();
    let mut axis_angle: f32 = 0.0;

    let mut euler_angles_mode = true;

    let mut animating = false;
    let mut animate_index = 0;
    let mut animate_timer = 0.0;
    
    let mut events = Vec::new();

    'render: loop {
        events.clear();

        // Get all the new events since the last frame.
        events_loop.poll_events(|event| { events.push(event); });

        // Process the events.
        for event in events.drain(..) {

            // Break from the loop upon `Escape` or closed window.
            match event.clone() {
                glium::glutin::Event::WindowEvent { event, .. } => {
                    match event {
                        glium::glutin::WindowEvent::Closed |
                        glium::glutin::WindowEvent::KeyboardInput {
                            input: glium::glutin::KeyboardInput {
                                virtual_keycode: Some(glium::glutin::VirtualKeyCode::Escape),
                                ..
                            },
                            ..
                        } => break 'render,
                        _ => (),
                    }
                }
                _ => (),
            };

            // Use the `winit` backend feature to convert the winit event to a conrod input.
            let input = match conrod::backend::winit::convert_event(event, &display) {
                None => continue,
                Some(input) => input,
            };

            // Handle the input with the `Ui`.
            ui.handle_event(input);

            // Set the widgets.
            let ui = &mut ui.set_widgets();

            widget::Canvas::new()
                .color(color::DARK_GRAY)
                .align_top()
                .align_left()
                .w(300.0)
                .h(ui.win_h)
                .set(ids.canvas, ui);

            const PAD: f64 = 10.0;

            use widget::Slider;

            if widget::Button::new()
                .label("Euler Angles")
                .label_color(if euler_angles_mode { color::RED } else { color::BLACK })
                .top_left_with_margin_on(ids.canvas, PAD)
                .w(120.0)
                .h(30.0)
                .padded_w_of(ids.canvas, PAD)
                .set(ids.euler_angles_button, ui)
                .was_clicked()
            {
                euler_angles_mode = true;
            }

            if widget::Button::new()
                .label("Axis Angle")
                .label_color(if !euler_angles_mode { color::RED } else { color::BLACK })
                .h(30.0)
                .set(ids.axis_angle_button, ui)
                .was_clicked()
            {
                euler_angles_mode = false;
            }

            if euler_angles_mode {
                for value in Slider::new(euler_angles[0].to_degrees(), 0.0, 360.0)
                    .label("Yaw")
                    .label_color(color::RED)
                    .padded_w_of(ids.canvas, PAD)
                    .h(30.0)
                    .set(ids.yaw, ui)
                {
                    euler_angles[0] = value.to_radians();
                    let cur_quaternion = quaternion_list.last_mut().unwrap();
                    *cur_quaternion = Quaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                }

                for value in Slider::new(euler_angles[1].to_degrees(), 0.0, 360.0)
                    .label("Pitch")
                    .label_color(color::RED)
                    .padded_w_of(ids.canvas, PAD)
                    .h(30.0)
                    .set(ids.pitch, ui)
                {
                    euler_angles[1] = value.to_radians();
                    let cur_quaternion = quaternion_list.last_mut().unwrap();
                    *cur_quaternion = Quaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                }

                for value in Slider::new(euler_angles[2].to_degrees(), 0.0, 360.0)
                    .label("Roll")
                    .label_color(color::RED)

                    .padded_w_of(ids.canvas, PAD)
                    .h(30.0)
                    .set(ids.roll, ui)
                {
                    euler_angles[2] = value.to_radians();
                    let cur_quaternion = quaternion_list.last_mut().unwrap();
                    *cur_quaternion = Quaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                }
            } else {
                for value in Slider::new(axis[0], 0.0, 1.0)
                    .label("Axis X")
                    .label_color(color::RED)

                    .padded_w_of(ids.canvas, PAD)
                    .h(30.0)
                    .set(ids.axis_x, ui)
                {
                    axis[0] = value;
                    let n = axis.clone().normalize();
                    let cur_quaternion = quaternion_list.last_mut().unwrap();
                    *cur_quaternion = Quaternion::from_axis_angle(n[0], n[1], n[2], axis_angle);
                }

                for value in Slider::new(axis[1], 0.0, 1.0)
                    .label("Axis Y")
                    .label_color(color::RED)

                    .padded_w_of(ids.canvas, PAD)
                    .h(30.0)
                    .set(ids.axis_y, ui)
                {
                    axis[1] = value;
                    let n = axis.clone().normalize();
                    let cur_quaternion = quaternion_list.last_mut().unwrap();
                    *cur_quaternion = Quaternion::from_axis_angle(n[0], n[1], n[2], axis_angle);
                }

                for value in Slider::new(axis[2], 0.0, 1.0)
                    .label("Axis Z")
                    .label_color(color::RED)

                    .padded_w_of(ids.canvas, PAD)
                    .h(30.0)
                    .set(ids.axis_z, ui)
                {
                    axis[2] = value;
                    let n = axis.clone().normalize();
                    let cur_quaternion = quaternion_list.last_mut().unwrap();
                    *cur_quaternion = Quaternion::from_axis_angle(n[0], n[1], n[2], axis_angle);
                }

                for value in Slider::new(axis_angle.to_degrees(), 0.0, 360.0)
                    .label("Angle")
                    .label_color(color::RED)

                    .padded_w_of(ids.canvas, PAD)
                    .h(30.0)
                    .set(ids.axis_angle, ui)
                {
                    axis_angle = value.to_radians();
                    let n = axis.clone().normalize();
                    let cur_quaternion = quaternion_list.last_mut().unwrap();
                    *cur_quaternion = Quaternion::from_axis_angle(n[0], n[1], n[2], axis_angle);
                }
            }

            if widget::Button::new()
                .label("Add Rotation")
                .set(ids.add_rotation, ui)
                .was_clicked()
            {
                quaternion_list.push(Quaternion::identity());
            }

            if widget::Button::new()
                .label("Clear Rotations")
                .set(ids.clear_rotations, ui)
                .was_clicked()
            {
                quaternion_list = vec![Quaternion::identity()];
            }

            if widget::Button::new()
                .label("Animate Rotations")
                .set(ids.animate_rotations, ui)
                .was_clicked()
            {
                animating = true;
                animate_index = 0;
                animate_timer = 0.0;
            }
        }

        // Draw the `Ui` if it has changed.
        let primitives = ui.draw();
        {
            renderer.fill(&display, primitives, &image_map);
            let mut target = display.draw();
            target.clear_color_and_depth((0.0, 0.0, 0.0, 1.0), 1.0);

            // Draw the model.
            let mut rotation = Quaternion::identity();

            if animating {
                for q in quaternion_list.iter().take(animate_index) {
                    rotation *= *q;
                }

                rotation = rotation.slerp(rotation*quaternion_list[animate_index], animate_timer);

                animate_timer += 1.0 / 60.0;
                if animate_timer >= 1.0 {
                    animate_index += 1;
                    animate_timer -= 1.0;
                    if animate_index >= quaternion_list.len() {
                        animating = false;
                    }
                }
            } else {
                for q in &quaternion_list {
                    rotation *= *q;
                }
            }

            model.transform.rotation = rotation;
            render_model(&model, &program, &camera, &mut target)?;

            renderer.draw(&display, &mut target, &image_map)?;
            target.finish()?;
        }

        std::thread::sleep(std::time::Duration::from_millis(16));
    }

    Ok(())
}

fn create_shader_program(display: &glium::Display) -> Result<Program, Box<Error>> {
    use std::fs::File;
    use std::io::Read;

    let vertex_src = {
        let mut file = File::open(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/vertex.glsl"))?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;
        contents
    };

    let fragment_src = {
        let mut file = File::open(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/fragment.glsl"))?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;
        contents
    };

    let program = Program::from_source(display, &vertex_src, &fragment_src, None)?;
    Ok(program)
}

fn create_axes_model(display: &glium::Display) -> Result<Model, Box<Error>> {
    const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
    const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
    const BLUE: [f32; 4] = [0.0, 0.0, 1.0, 1.0];
    const GRAY: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

    const RIGHT: [f32; 3] = [1.0, 0.0, 0.0];
    const LEFT: [f32; 3] = [-1.0, 0.0, 0.0];
    const UP: [f32; 3] = [0.0, 1.0, 0.0];
    const DOWN: [f32; 3] = [0.0, -1.0, 0.0];
    const FRONT: [f32; 3] = [0.0, 0.0, 1.0];
    const BACK: [f32; 3] = [0.0, 0.0, -1.0];

    let vertices = vec![
        // -y
        Vertex { position: [-0.5, -0.5, -0.5], color: GRAY, normal: DOWN },
        Vertex { position: [-0.5, -0.5, 0.5], color: GRAY, normal: DOWN },
        Vertex { position: [0.5, -0.5, 0.5], color: GRAY, normal: DOWN },
        Vertex { position: [0.5, -0.5, -0.5], color: GRAY, normal: DOWN },

        // +y
        Vertex { position: [-0.5, 0.5, -0.5], color: GRAY, normal: UP },
        Vertex { position: [-0.5, 0.5, 0.5], color: GRAY, normal: UP },
        Vertex { position: [0.5, 0.5, 0.5], color: GRAY, normal: UP },
        Vertex { position: [0.5, 0.5, -0.5], color: GRAY, normal: UP },

        // -z
        Vertex { position: [-0.5, -0.5, -0.5], color: GRAY, normal: BACK },
        Vertex { position: [-0.5, 0.5, -0.5], color: GRAY, normal: BACK },
        Vertex { position: [0.5, 0.5, -0.5], color: GRAY, normal: BACK },
        Vertex { position: [0.5, -0.5, -0.5], color: GRAY, normal: BACK },

        // +z
        Vertex { position: [-0.5, -0.5, 0.5], color: GRAY, normal: FRONT },
        Vertex { position: [-0.5, 0.5, 0.5], color: GRAY, normal: FRONT },
        Vertex { position: [0.5, 0.5, 0.5], color: GRAY, normal: FRONT },
        Vertex { position: [0.5, -0.5, 0.5], color: GRAY, normal: FRONT },

        // -x
        Vertex { position: [-0.5, -0.5, -0.5], color: GRAY, normal: LEFT },
        Vertex { position: [-0.5, -0.5, 0.5], color: GRAY, normal: LEFT },
        Vertex { position: [-0.5, 0.5, 0.5], color: GRAY, normal: LEFT },
        Vertex { position: [-0.5, 0.5, -0.5], color: GRAY, normal: LEFT },
        
        // +x
        Vertex { position: [0.5, -0.5, -0.5], color: GRAY, normal: RIGHT },
        Vertex { position: [0.5, -0.5, 0.5], color: GRAY, normal: RIGHT },
        Vertex { position: [0.5, 0.5, 0.5], color: GRAY, normal: RIGHT },
        Vertex { position: [0.5, 0.5, -0.5], color: GRAY, normal: RIGHT },
    ];
    let indices = vec![
        // y
        0, 1, 2, 0, 2, 3,
        4, 5, 6, 4, 6, 7,

        // z
        8, 9, 10, 8, 10, 11,
        12, 13, 14, 12, 14, 15,

        // x
        16, 17, 18, 16, 18, 19,
        20, 21, 22, 20, 22, 23,
    ];
    let model = Model {
        vertex_buffer: VertexBuffer::new(display, &vertices)?,
        index_buffer: IndexBuffer::new(display, glium::index::PrimitiveType::TrianglesList, &indices)?,
        transform: Transform::new(),
    };

    Ok(model)
}

fn render_model(model: &Model, program: &glium::Program, camera: &Camera, target: &mut glium::Frame) -> Result<(), Box<Error>> {
    let view_matrix: [[f32; 4]; 4] = camera.transform.to_matrix().try_inverse().unwrap().into();
    let projection_matrix: [[f32; 4]; 4] = camera.projection.into();
    let model_matrix: [[f32; 4]; 4] = model.transform.to_matrix().into();
    let light_pos: [f32; 3] = [2.0, 2.0, 2.0];

    use glium::draw_parameters::BackfaceCullingMode;
    target.draw(&model.vertex_buffer, &model.index_buffer, &program,
            &uniform! {
                u_model: model_matrix,
                u_view: view_matrix,
                u_projection: projection_matrix,
                u_light_position: light_pos,
            },
            &glium::DrawParameters {
                depth: glium::Depth {
                    test: glium::DepthTest::IfLess,
                    write: true,
                    .. Default::default()
                },
                backface_culling: BackfaceCullingMode::CullingDisabled,
                .. Default::default()
            })?;

    Ok(())
}
