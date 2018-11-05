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
use forces::RoamingForce;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    let mut testbed = Testbed::new_empty();

    let mut world = World::new();
    //world.set_gravity(Vector2::new(0.0, -9.8));
    world.set_gravity(Vector2::new(0.0, 0.0));

    create_ground(&mut world);

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
            if i % 3 != 0 || j % 3 != 0 { continue; };

            let x = i as f32 * shiftx - centerx;
            let y = j as f32 * shifty + centery;
            let angle = x * 0.5;

            let handle = create_body(&mut world, inertia, center_of_mass, geom.clone(), x, y, angle);

            world.add_force_generator(RoamingForce::new(handle));

            // Sensor
            let radius = 0.2;
            let sensor_geom = ShapeHandle::new(Ball::new(radius * 5.0));
            world.add_sensor(sensor_geom, handle, Isometry2::identity());
//            testbed.set_body_color(&world, handle, Point3::new(0.5, 1.0, 1.0));

            // Callback that will be executed on the main loop to handle proximities.
            testbed.add_callback(move |world, graphics, _| {
                println!("--------- Callback -----------------");
                let w = &world;
                for prox in w.proximity_events() {
                    println!("Contact: {:?}", prox);
                }
            });
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::new(0.0, -1.0), 120.0);
    testbed.hide_performance_counters();
    testbed.run();
}

fn create_body(world: &mut World<f32>, inertia: Inertia<f32>, center_of_mass: Point2<f32>, geom: ShapeHandle<f32>, x: f32, y: f32, angle: f32) -> BodyHandle {
    let pos = Isometry2::new(Vector2::new(x, y), angle);
    let mut handle = world.add_rigid_body(pos, inertia, center_of_mass);
    {
        let rb = world.rigid_body_mut(handle).unwrap();
        rb.set_status(BodyStatus::Dynamic);
    }

    world.add_collider(
        COLLIDER_MARGIN,
        geom,
        handle,
        Isometry2::identity(),
        Material::default(),
    );

    handle
}

fn create_ground(world: &mut World<f32>) {
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
}
