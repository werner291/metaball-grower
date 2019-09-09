extern crate kiss3d;
extern crate nalgebra as na;

use na::{Point3, Vector3};
use rand::prelude::*;


use kiss3d::light::Light;
use kiss3d::window::Window;

mod metaball;
use metaball::*;

mod meta_iso;
use meta_iso::*;

//region Rendering

pub fn main() {
    let mut rng = thread_rng();

    let constr = MetaballShape {
        points: (0..500).map(|_| Metaball {
            pos: Point3::new(rng.gen_range(-5.0,5.0), 
                             rng.gen_range(-5.0,5.0), 
                             rng.gen_range(-5.0,5.0))
            }).collect()
    };

    let mut window = Window::new("Kiss3d: cube");

    window.add_trimesh(
        metaballs_to_trimesh(&constr, 0.5), 
        Vector3::new(1.0,1.0,1.0)
    );

    window.set_light(Light::StickToCamera);

    while window.render() {
        // Nothing.
    }
}

//endregion