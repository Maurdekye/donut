use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use camera::Camera;
use crossterm::{
    ExecutableCommand, cursor,
    terminal::{Clear, ClearType},
};
use glam::{Mat3, Vec3, mat3, vec3};
use objects::torus;
use raymarch::RaymarchOptions;

mod raymarch {
    use glam::{Vec2Swizzles, Vec3, vec2};

    #[derive(Clone, Copy, Debug)]
    pub struct Ray {
        pub origin: Vec3,
        pub direction: Vec3,
    }

    impl Ray {
        pub fn at(&self, t: f32) -> Vec3 {
            self.origin + self.direction * t
        }
    }

    pub struct RaymarchOptions {
        pub max_iterations: usize,
        pub far_clip: f32,
        pub epsilon: f32,
        pub normal_epsilon: f32,
    }

    impl Default for RaymarchOptions {
        fn default() -> Self {
            Self {
                max_iterations: 100,
                far_clip: 1e3,
                epsilon: 1e-4,
                normal_epsilon: 1e-3,
            }
        }
    }

    #[allow(dead_code)]
    pub enum RaymarchResult {
        MissedScene {
            iterations: usize,
            nearest_distance: f32,
        },
        ReachedMaxIterations {
            nearest_distance: f32,
        },
        HitScene {
            iterations: usize,
            depth: f32,
            normal: Vec3,
        },
    }

    pub fn raymarch(
        ray: Ray,
        scene: impl Fn(Vec3) -> f32,
        options: &RaymarchOptions,
    ) -> RaymarchResult {
        use RaymarchResult::*;
        let mut depth = 0.0;
        let mut nearest_distance = options.far_clip;
        for i in 0..options.max_iterations {
            let scene_pos = ray.at(depth);
            let scene_distance = (scene)(scene_pos);
            nearest_distance = nearest_distance.min(scene_distance);
            if depth > options.far_clip {
                return MissedScene {
                    iterations: i,
                    nearest_distance,
                };
            } else if scene_distance < options.epsilon {
                let offset_vec = vec2(options.normal_epsilon, 0.0);
                let x = (scene)(scene_pos + offset_vec.xyy());
                let y = (scene)(scene_pos + offset_vec.yxy());
                let z = (scene)(scene_pos + offset_vec.yyx());
                let normal = Vec3 { x, y, z }.normalize();
                return HitScene {
                    iterations: i,
                    depth,
                    normal,
                };
            } else {
                depth += scene_distance;
            }
        }
        ReachedMaxIterations { nearest_distance }
    }
}

mod objects {
    use glam::{Vec3, vec2};

    pub fn torus(
        pos: Vec3,
        origin: Vec3,
        normal: Vec3,
        major_radius: f32,
        minor_radius: f32,
    ) -> f32 {
        let p = pos - origin;
        let q = vec2(
            (p - normal * p.dot(normal)).length() - major_radius,
            p.dot(normal),
        )
        .length();
        q - minor_radius
    }
}

mod camera {
    use glam::{Mat3, Vec3};

    use crate::raymarch::{Ray, RaymarchOptions, RaymarchResult, raymarch};

    pub struct Camera<const W: usize, const H: usize> {
        pub origin: Vec3,
        pub basis: Mat3,
    }

    #[derive(Debug)]
    pub struct RenderResult<const W: usize, const H: usize> {
        pub depth: [[f32; W]; H],
        pub proximity: [[f32; W]; H],
        pub normals: [[Vec3; W]; H],
    }

    impl<const W: usize, const H: usize> RenderResult<W, H> {
        pub fn new() -> RenderResult<W, H> {
            RenderResult {
                depth: [[0.0; W]; H],
                proximity: [[0.0; W]; H],
                normals: [[Vec3::ZERO; W]; H],
            }
        }
    }

    impl<const W: usize, const H: usize> Camera<W, H> {
        pub fn render(
            &self,
            scene: impl Fn(Vec3) -> f32,
            raymarch_options: &RaymarchOptions,
        ) -> RenderResult<W, H> {
            let mut render_result = RenderResult::new();
            let z = 1.0;
            for y_pixel in 0..H {
                let y = (y_pixel as f32) / (H as f32) - 0.5;
                for x_pixel in 0..W {
                    let x = (x_pixel as f32) / (W as f32) - 0.5;
                    let direction = Vec3 { x, y, z }.normalize();
                    let direction = self.basis * direction;
                    let origin = self.origin;
                    let ray = Ray { direction, origin };
                    let res_depth = &mut render_result.depth[y_pixel][x_pixel];
                    let res_proximity = &mut render_result.proximity[y_pixel][x_pixel];
                    let res_normals = &mut render_result.normals[y_pixel][x_pixel];
                    match raymarch(ray, &scene, raymarch_options) {
                        RaymarchResult::MissedScene {
                            nearest_distance, ..
                        } => {
                            *res_depth = f32::INFINITY;
                            *res_proximity = nearest_distance;
                        }
                        RaymarchResult::ReachedMaxIterations { nearest_distance } => {
                            *res_depth = f32::NAN;
                            *res_proximity = nearest_distance;
                        }
                        RaymarchResult::HitScene { depth, normal, .. } => {
                            *res_depth = depth;
                            *res_normals = normal;
                        }
                    }
                }
            }
            render_result
        }
    }
}

fn main() {
    const WIDTH: usize = 60;
    const HEIGHT: usize = 30;

    let camera: Camera<WIDTH, HEIGHT> = Camera {
        origin: vec3(0.0, 0.0, 0.0),
        basis: mat3(Vec3::X, Vec3::Y, Vec3::Z),
    };
    let start = Instant::now();
    let options = RaymarchOptions::default();

    std::io::stdout().execute(Clear(ClearType::All)).unwrap();
    loop {
        std::io::stdout().execute(cursor::MoveTo(0, 0)).unwrap();
        let time = Instant::now().duration_since(start).as_secs_f32();
        let scene = |pos| {
            let normal = vec3(0.0, 0.0, 1.0);
            let normal = Mat3::from_axis_angle(vec3(0.0, 1.0, 0.0), time / 3.0) * normal;
            let normal = Mat3::from_axis_angle(vec3(1.0, 0.0, 0.0), time / 10.2467) * normal;
            let normal =
                Mat3::from_axis_angle(vec3(0.0, 0.0, 1.0), (time / 13.2251f32).sin() * 6.0)
                    * normal;
            torus(pos, vec3(0.0, 0.0, 10.0), normal, 3.0, 1.0)
        };
        let result = camera.render(scene, &options);
        for (depth_row, normals_row) in result.depth.into_iter().zip(result.normals) {
            let line: String = depth_row
                .into_iter()
                .zip(normals_row)
                .map(|(depth, normal)| {
                    let mut brightness = 0.0;
                    if depth.is_finite() {
                        brightness = (normal.dot(vec3(0.0, -1.0, 0.0)) + 1.0) / 2.0;
                    }
                    match brightness {
                        0.1..=0.25 => '.',
                        0.25..=0.5 => '-',
                        0.5..=0.75 => '+',
                        0.75.. => '#',
                        _ => ' ',
                    }
                })
                .collect();
            println!("{line}");
        }
        sleep(Duration::from_secs_f32(1.0 / 60.0));
    }
}
