extern crate kiss3d;
extern crate nalgebra as na;

use na::{Point3, Vector3};
use rand::prelude::*;
use ncollide3d::procedural::TriMesh;


use kiss3d::light::Light;
use kiss3d::window::Window;

mod metaball;
use metaball::*;

//region Rendering

pub fn main() {
    let mut rng = thread_rng();

    let constr = MetaballShape {
        points: (0..20).map(|_| Metaball {
            pos: Point3::new(rng.gen_range(-5.0,5.0), 
                             rng.gen_range(-5.0,5.0), 
                             rng.gen_range(-5.0,5.0))
            }).collect()
    };

    let mut window = Window::new("Kiss3d: cube");

    window.add_trimesh(
        TriMesh::new(constr.triangulate(0.5).iter()
                           .flat_map(|t| t.vertices())
                           .map(|v| Point3::new(v[0] as f32, v[1] as f32, v[2] as f32))
                           .collect(), 
                     None, None, None), 
        Vector3::new(1.0,1.0,1.0));

    // for cube in constr.cubify(0.1) {

    //     let he = 2.0 * cube.half_extents();

    //     let mut c = window.add_cube(he[0] as f32, he[1] as f32, he[2] as f32);

    //     let center = cube.center().coords;

    //     c.prepend_to_local_translation(
    //         &Translation::from(
    //             Vector3::new(center[0] as f32, 
    //                          center[1] as f32, 
    //                          center[2] as f32)));
    // }

    window.set_light(Light::StickToCamera);

    while window.render() {
        // Nothing.
    }
}

//endregion