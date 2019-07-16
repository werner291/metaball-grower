use nalgebra::{Point3, Vector3, distance_squared};
use ncollide3d::query::{RayCast, Ray};
use ncollide3d::bounding_volume::{AABB, BoundingSphere};
use ncollide3d::math::Isometry;
use nalgebra::RealField;
use std::iter::{once, empty};

#[derive(Debug)]
pub struct Metaball {
    pub pos: Point3<f64>
}

const THRESHOLD : f64 = 0.5;

impl Metaball {
    
    #[inline]
    pub fn influence(&self, pt: &Point3<f64>) -> f64 {
        let r_2 = distance_squared(&self.pos, pt);

        if r_2 > 1.0 {
            0.0
        } else {
            (1.0 - r_2) * (1.0 - r_2)
        }
    }

     #[inline]
    pub fn gradient(&self, pt: &Point3<f64>) -> Vector3<f64> {
        (pt - self.pos).normalize()
    }

}

pub struct MetaballShape {
    pub points: Vec<Metaball>
}

impl MetaballShape {

    pub fn influence(&self, pt: &Point3<f64>) -> f64 {
        self.points.iter().map(|p| p.influence(pt)).sum()
    }

    pub fn gradient(&self, pt: &Point3<f64>) -> Vector3<f64> {
        self.points.iter().fold(Vector3::new(0.0,0.0,0.0), |g, p| g + p.gradient(pt) * p.influence(pt)).normalize()
    }

    pub fn cubify(&self, min_size: f64) -> Vec<AABB<f64>> {

        self.cubify_bounded(
            &AABB::new(
                Point3::new(self.points.iter().map(|pt| pt.pos[0]).min_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) - 1.0,
                            self.points.iter().map(|pt| pt.pos[1]).min_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) - 1.0,
                            self.points.iter().map(|pt| pt.pos[2]).min_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) - 1.0),
                Point3::new(self.points.iter().map(|pt| pt.pos[0]).max_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) + 1.0,
                            self.points.iter().map(|pt| pt.pos[1]).max_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) + 1.0,
                            self.points.iter().map(|pt| pt.pos[2]).max_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) + 1.0)),
            min_size)
    }

    fn cubify_bounded(&self, bounds: &AABB<f64>, min_size: f64) -> Vec<AABB<f64>> {

        let mins = bounds.mins();
        let maxs = bounds.maxs();
        let mids = bounds.center();

        if distance_squared(mins, maxs) > min_size * min_size {

            (0..8).flat_map(move |i| {

                let octant = AABB::new(
                    Point3::new(if i & 0b100 == 0 {mins[0]} else {mids[0]},
                                if i & 0b010 == 0 {mins[1]} else {mids[1]}, 
                                if i & 0b001 == 0 {mins[2]} else {mids[2]}), 

                    Point3::new(if i & 0b100 == 0 {mids[0]} else {maxs[0]},
                                if i & 0b010 == 0 {mids[1]} else {maxs[1]}, 
                                if i & 0b001 == 0 {mids[2]} else {maxs[2]}));

                self.cubify_bounded(&octant,min_size)

            }).collect()

        } else if self.influence(&bounds.center()) >= THRESHOLD {
            vec![bounds.clone()]
        } else {
            vec![]
        }
    }
}

// fn meshify<N:RealField, F:Fn(Point3<N>)->N, G:Fn(Point3<N>)->Vector3<N>>(f:F,n:N,c1: Point3<N>, c2: Point3<N>, min_size: N) -> TriMesh {

    

// }