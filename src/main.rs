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
        pub calc_normal: bool,
        pub epsilon: f32,
    }

    impl Default for RaymarchOptions {
        fn default() -> Self {
            Self {
                max_iterations: 100,
                far_clip: 1e10,
                calc_normal: false,
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
            normal: Option<Vec3>,
        },
    }

    pub fn raymarch(
        ray: Ray,
        scene: impl Fn(Vec3) -> f32,
        options: RaymarchOptions,
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
                let normal = options.calc_normal.then(|| {
                    let offset_vec = vec2(options.epsilon, 0.0);
                    let x = (scene)(scene_pos + offset_vec.xyy());
                    let y = (scene)(scene_pos + offset_vec.yxy());
                    let z = (scene)(scene_pos + offset_vec.yyx());
                    Vec3 { x, y, z }.normalize()
                });
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

fn main() {
    println!("Hello, world!");
}
