use nalgebra::{Point3, Vector3, distance_squared};
use ncollide3d::bounding_volume::AABB;
use ncollide3d::shape::Triangle;

#[derive(Debug)]
pub struct Metaball {
    pub pos: Point3<f64>
}

pub const THRESHOLD : f64 = 0.5;

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
        let r = pt - self.pos;

        if r.dot(&r) > 1.0 {
            Vector3::new(0.0,0.0,0.0)
        } else {
            - 4.0 * r * (1.0 - r.dot(&r))
        }
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

    pub fn extent_min(&self, dim: usize) -> f64 {
        self.points.iter().map(|pt| pt.pos[dim])
            .min_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) - 1.0
    }

    pub fn extent_max(&self, dim: usize) -> f64 {
        self.points.iter().map(|pt| pt.pos[dim])
            .max_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) + 1.0
    }

    pub fn bounding_box(&self) -> AABB<f64> {
        AABB::new(
                Point3::new(self.extent_min(0), self.extent_min(1), self.extent_min(2)),
                Point3::new(self.extent_max(0), self.extent_max(1), self.extent_max(2)))
    }

    pub fn project_onto(&self, mut pt: Point3<f64>) -> Point3<f64> {
        
        while self.gradient(&pt).norm_squared() == 0.0 {

            let grav_dir : Vector3<f64> = self.points.iter().map(|mb| {
                let delta : Vector3<f64> = pt - mb.pos;
                let d_2 : f64 = delta.dot(&delta);
                delta.normalize() / d_2
            }).sum();

            pt -= grav_dir.normalize() / 5.0;
        }

        while (self.influence(&pt) - THRESHOLD).abs() > 0.01 {
            pt -= (self.influence(&pt) - THRESHOLD) * self.gradient(&pt) / 5.0;
        }

        pt
    }
}

