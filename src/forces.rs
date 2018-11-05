#[allow(unused)]use nphysics2d::math::Velocity;
use nphysics2d::object::BodyHandle;
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::object::BodySet;
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::algebra::Force2;
#[allow(unused)]use nalgebra::{Isometry2, Point2, Vector2};

use std::f32;
use nphysics2d::object::BodyPartMut;

extern crate rand;
use std::option::Option;

//pub struct RadialForce {
//    parts: Vec<BodyHandle>, // Body parts affected by the force generator.
//    center: Point2<f32>,
//}
//
//impl RadialForce {
//    // Creates a new radial force generator.
//    pub fn new(center: Point2<f32>, parts: Vec<BodyHandle>) -> Self {
//        RadialForce {
//            parts,
//            center,
//        }
//    }
//
//    /// Add a body part to be affected by this force generator.
//    pub fn add_body_part(&mut self, body: BodyHandle) {
//        self.parts.push(body)
//    }
//}
//
//impl ForceGenerator<f32> for RadialForce {
//    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
//        for handle in &self.parts {
//            // Generate the force only if the body has not been removed from the world.
//            if bodies.contains(*handle) {
//                let mut part = bodies.body_part_mut(*handle);
//
//                // The `.as_ref()` retrieves a `BodyPart` from the `BodyPartMut`.
//                let delta_pos = part.as_ref().center_of_mass() - self.center;
//
//                // We set the force such that it is equal to ten times the distance
//                // between the body part and self.center.
//                let force = Force2::linear(delta_pos / 2.0);
//
//                // Apply the force.
//                part.apply_force(&force);
//            }
//        }
//
//        // If `false` is returned, the physis world will remove
//        // this force generator after this call.
//        true
//    }
//}

pub struct RoamingForce {
    time: f32,
    part: BodyHandle
}

impl RoamingForce {
    pub fn new(part: BodyHandle) -> Self {
        let phase = rand::random::<f32>();
        RoamingForce { time: phase, part: part }
    }
}

impl ForceGenerator<f32> for RoamingForce {
    fn apply(&mut self, params: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        if bodies.contains(self.part) {
            self.time += params.dt;
            let mut part = bodies.body_part_mut(self.part);
            let center = part.as_ref().center_of_mass();

            let scale = 10.0;
            let t = self.time * 60.0;
            let x = scale * t.sin();
            let y = scale * t.cos();

            //println!("dt: {:?} x: {:?} y: {:?}", self.time, x, y);

            let force = Force2::linear(Vector2::new(x, y));

            // Apply the force.
            part.apply_force(&force);
        }
        true
    }
}

pub struct Surroundings {
    bodies: Vec<BodyHandle>,
}

impl Surroundings {
}

pub struct SurroundingsProvider {}
impl SurroundingsProvider {
    // TODO: updatable surroundings
    pub fn new() -> Self { SurroundingsProvider {} }
    pub fn closest(&self, center: Point2<f32>) -> Option<&BodyHandle> {
        //self.bodies.first()
        None
    }

}

pub struct ChasingForce {
    part: BodyHandle,
    observer: SurroundingsProvider
}

const PREDATOR_SPEED: f32 = 5.0;

impl ChasingForce {

    pub fn new(part: BodyHandle, provider: SurroundingsProvider) -> Self {
        ChasingForce { part: part , observer: provider}
    }


    pub fn move_random(&mut self, mut my_body: BodyPartMut<f32>, center: Point2<f32>) {
        let x = rand::random::<f32>() * PREDATOR_SPEED;
        let y = rand::random::<f32>() * PREDATOR_SPEED;
        let force = Force2::linear(Vector2::new(x, y));
        my_body.apply_force(&force)
    }

    pub fn move_towards(&mut self, my_body: BodyPartMut<f32>, center: Point2<f32>, part: BodyHandle) {

    }

}

impl ForceGenerator<f32> for ChasingForce {
    fn apply(&mut self, params: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        if bodies.contains(self.part) {
            let mut part = bodies.body_part_mut(self.part);
            let center = part.as_ref().center_of_mass();

            match self.observer.closest(center) {
                Some(&otherBody) => {
                    self.move_towards(part, center, otherBody);
                }
                None => {
                    self.move_random(part, center);
                }
            }

//            let scale = 10.0;
//            let t = self.time * 60.0;
//            let x = scale * t.sin();
//            let y = scale * t.cos();
//
//            //println!("dt: {:?} x: {:?} y: {:?}", self.time, x, y);
//
//            let force = Force2::linear(Vector2::new(x, y));
//
//            // Apply the force.
//            part.apply_force(&force);
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}