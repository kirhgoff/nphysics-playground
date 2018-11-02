#[allow(unused)]use nphysics2d::math::Velocity;
use nphysics2d::object::BodyHandle;
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::object::BodySet;
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::algebra::Force2;
#[allow(unused)]use nalgebra::{Isometry2, Point2, Vector2};


pub struct RadialForce {
    parts: Vec<BodyHandle>, // Body parts affected by the force generator.
    center: Point2<f32>,
}

impl RadialForce {
    // Creates a new radial force generator.
    pub fn new(center: Point2<f32>, parts: Vec<BodyHandle>) -> Self {
        RadialForce {
            parts,
            center,
        }
    }

    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyHandle) {
        self.parts.push(body)
    }
}

impl ForceGenerator<f32> for RadialForce {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        for handle in &self.parts {
            // Generate the force only if the body has not been removed from the world.
            if bodies.contains(*handle) {
                let mut part = bodies.body_part_mut(*handle);

                // The `.as_ref()` retrieves a `BodyPart` from the `BodyPartMut`.
                let delta_pos = part.as_ref().center_of_mass() - self.center;

                // We set the force such that it is equal to ten times the distance
                // between the body part and self.center.
                let force = Force2::linear(delta_pos / 2.0);

                // Apply the force.
                part.apply_force(&force);
            }
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}