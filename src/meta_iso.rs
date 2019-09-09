
use nalgebra::{Point3, Vector3, distance_squared};
use ncollide3d::bounding_volume::AABB;
use ncollide3d::shape::Triangle;
use ncollide3d::procedural::TriMesh;

use crate::metaball::*;    

pub fn metaballs_to_trimesh(metas : &MetaballShape, min_size: f64) -> TriMesh<f32> {

    let tris = triangulate(metas, min_size);

    let verts_64 : Vec<Point3<f64>> = 
        tris.iter()
            .flat_map(|t| vec![*t.a(), *t.b(), *t.c()].into_iter())
            .collect();

    let normals_64 : Vec<Vector3<f64>> = verts_64.iter().map(|pt| metas.gradient(pt).normalize()).collect();

    TriMesh::new(
        verts_64.iter().map(|v| Point3::new(v[0] as f32, v[1] as f32, v[2] as f32)).collect(), 
        None,//Some(normals_64.iter().map(|v| Vector3::new(v[0] as f32, v[1] as f32, v[2] as f32)).collect()),
        None, None)

}

pub fn triangulate(metas : &MetaballShape, min_size: f64) -> Vec<Triangle<f64>> {

    triangulate_bounded(metas,
        &AABB::new(
            Point3::new(metas.extent_min(0), metas.extent_min(1), metas.extent_min(2)),
            Point3::new(metas.extent_max(0), metas.extent_max(1), metas.extent_max(2))),
        min_size)

}

fn triangulate_bounded(metas : &MetaballShape, bounds: &AABB<f64>, min_size: f64) -> Vec<Triangle<f64>> {

    let mins = bounds.mins();
    let maxs = bounds.maxs();
    let mids = bounds.center();

    if (maxs[0] - mins[0]).max(maxs[1] - mins[1]).max(maxs[2] - mins[2]) > min_size {

        (0..8).flat_map(move |i| {

            let octant = AABB::new(
                Point3::new(if i & 0b100 == 0 {mins[0]} else {mids[0]},
                            if i & 0b010 == 0 {mins[1]} else {mids[1]}, 
                            if i & 0b001 == 0 {mins[2]} else {mids[2]}), 

                Point3::new(if i & 0b100 == 0 {mids[0]} else {maxs[0]},
                            if i & 0b010 == 0 {mids[1]} else {maxs[1]}, 
                            if i & 0b001 == 0 {mids[2]} else {maxs[2]}) );

            triangulate_bounded(metas, &octant,min_size)

        }).collect()

    } else if metas.influence(&bounds.center()) >= THRESHOLD {

        let c = bounds.center();
        let he = bounds.half_extents();
        
        let cube_verts = 
            [[[c + Vector3::new(-he[0],-he[1],-he[2]),
                c + Vector3::new(-he[0],-he[1], he[2])],
                [c + Vector3::new(-he[0], he[1],-he[2]),
                c + Vector3::new(-he[0], he[1], he[2])]],
                [[c + Vector3::new( he[0],-he[1],-he[2]),
                c + Vector3::new( he[0],-he[1], he[2])],
                [c + Vector3::new( he[0], he[1],-he[2]),
                c + Vector3::new( he[0], he[1], he[2])]]];

        vec![
            Triangle::new(cube_verts[0][0][0], cube_verts[0][0][1], cube_verts[0][1][0]),
            Triangle::new(cube_verts[0][1][0], cube_verts[0][0][1], cube_verts[0][1][1]),

            Triangle::new(cube_verts[1][0][0], cube_verts[1][1][0], cube_verts[1][0][1]),
            Triangle::new(cube_verts[1][1][0], cube_verts[1][1][1], cube_verts[1][0][1]),

            Triangle::new(cube_verts[0][0][0], cube_verts[1][0][0], cube_verts[0][0][1]),
            Triangle::new(cube_verts[1][0][0], cube_verts[1][0][1], cube_verts[0][0][1]),

            Triangle::new(cube_verts[0][1][0], cube_verts[0][1][1], cube_verts[1][1][0]),
            Triangle::new(cube_verts[1][1][0], cube_verts[0][1][1], cube_verts[1][1][1]),

            Triangle::new(cube_verts[0][0][0], cube_verts[0][1][0], cube_verts[1][0][0]),
            Triangle::new(cube_verts[1][0][0], cube_verts[0][1][0], cube_verts[1][1][0]),
            
            Triangle::new(cube_verts[0][0][1], cube_verts[1][0][1], cube_verts[0][1][1]),
            Triangle::new(cube_verts[1][0][1], cube_verts[1][1][1], cube_verts[0][1][1]),
        ].into_iter().filter(|tri| {
            let n = tri.scaled_normal().normalize(); // Unwrap is safe due to constant input
            let sample_pos = c + n * n.dot(&he).abs() * 2.0;
            metas.influence(&sample_pos) < THRESHOLD
        })
        .map(|t| Triangle::new(metas.project_onto(*t.a()),metas.project_onto(*t.b()),metas.project_onto(*t.c())))
        .collect()
    } else {
        vec![]
    }
}