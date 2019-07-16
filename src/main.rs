extern crate kiss3d;
extern crate nalgebra as na;

use na::{Point3, Vector3, distance_squared, UnitQuaternion};
use nalgebra::geometry::Translation;
use rand::prelude::*;


use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};

mod metaball;
use metaball::*;

//region Rendering

const WIDTH: u32 = 400;
const HEIGHT: u32 = 400;

pub fn main() {
    let mut rng = thread_rng();

    let constr = MetaballShape {
        points: (0..20).map(|_| Metaball {
            pos: Point3::new(rng.gen_range(-2.0,2.0), 
                             rng.gen_range(-2.0,2.0), 
                             rng.gen_range(-2.0,2.0))
            }).collect()
    };

    let mut window = Window::new("Kiss3d: cube");
    
    for cube in constr.cubify(0.25) {

        let he = 2.0 * cube.half_extents();

        let mut c = window.add_cube(he[0] as f32, he[1] as f32, he[2] as f32);

        let center = cube.center().coords;

        c.prepend_to_local_translation(
            &Translation::from(Vector3::new(center[0] as f32, 
                                center[1] as f32, center[2] as f32)));
    }

    window.set_light(Light::StickToCamera);

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);

    while window.render() {
        // Nothing.
    }
}

//endregion