#[allow(unused)]
extern crate nalgebra;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;


#[allow(unused)]use nalgebra::{Isometry2, Point2, Vector2};
#[allow(unused)]use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
#[allow(unused)]use nphysics2d::joint::RevoluteJoint;
#[allow(unused)]use nphysics2d::math::{Inertia, Velocity};
#[allow(unused)]use nphysics2d::object::{BodyHandle, BodyStatus, Material};
#[allow(unused)]use nphysics2d::volumetric::Volumetric;
#[allow(unused)]use nphysics2d::world::World;
#[allow(unused)]use nphysics_testbed2d::Testbed;

#[allow(unused)]use nphysics2d::algebra::Force2;
#[allow(unused)]use nphysics2d::algebra::Velocity2;

mod forces;
use self::forces::RadialForce;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(-Vector2::y() * ground_rady, nalgebra::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create the boxes
     */
    let num = 10;
    let radx = 0.1;
    let rady = 0.1;
    let shiftx = radx * 2.0;
    let shifty = rady * 2.0;
    let centerx = shiftx * (num as f32) / 2.0;
    let centery = shifty / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(
        radx - COLLIDER_MARGIN,
        rady - COLLIDER_MARGIN,
    )));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0..num {
            if i % 2 == 0 || j % 2 == 0 { continue; };

            let x = i as f32 * shiftx - centerx;
            let y = j as f32 * shifty + centery;

            /*
             * Create the rigid body.
             */
            let pos = Isometry2::new(Vector2::new(x, y), 0.0);
            let mut handle = world.add_rigid_body(pos, inertia, center_of_mass);

            {
//                let rb = world.rigid_body_mut(handle).unwrap();


//                let linear = Vector2::new(-5.0 + 1 as f32 * i as f32, -5.0 + 1 as f32 * j as f32);
//                let angular = 20.0;
//                rb.set_velocity(Velocity2::new(linear, angular));

            }
            let force = RadialForce::new(Point2::new(x, y), vec![handle]);
            world.add_force_generator(force);

            /*
             * Create the collider.
             */
            world.add_collider(
                COLLIDER_MARGIN,
                geom.clone(),
                handle,
                Isometry2::identity(),
                Material::default(),
            );
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(0.0, -1.0), 240.0);
    testbed.hide_performance_counters();
    testbed.run();
}
