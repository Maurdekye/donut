mod raymarch {
    use glam::{Vec2, Vec2Swizzles, Vec3, vec2};

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
    }

    impl Default for RaymarchOptions {
        fn default() -> Self {
            Self {
                max_iterations: 100,
                far_clip: 1e10,
                epsilon: 1e-10,
            }
        }
    }

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
                let offset_vec = vec2(options.epsilon, 0.0);
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
    use glam::Vec3;

    pub fn torus(
        pos: Vec3,
        origin: Vec3,
        normal: Vec3,
        major_radius: f32,
        minor_radius: f32,
    ) -> f32 {
        let plane_dist = (pos - origin).project_onto(normal);
        let plane_point = pos - normal * plane_dist;
        let major_circle = (plane_point - origin).normalize() * major_radius;
        (pos - major_circle).length() - minor_radius
    }
}

mod camera {
    use glam::{Mat3, Vec3};

    use crate::raymarch::{Ray, RaymarchOptions, RaymarchResult, raymarch};

    pub struct Camera<const W: usize, const H: usize> {
        origin: Vec3,
        basis: Mat3,
    }

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
    println!("Hello, world!");
}
