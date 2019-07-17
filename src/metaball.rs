use nalgebra::{Point3, Vector3, distance_squared};
use ncollide3d::bounding_volume::AABB;
use ncollide3d::shape::Triangle;

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

    fn extent_min(&self, dim: usize) -> f64 {
        self.points.iter().map(|pt| pt.pos[dim])
            .min_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) - 1.0
    }

     fn extent_max(&self, dim: usize) -> f64 {
        self.points.iter().map(|pt| pt.pos[dim])
            .max_by(|a,b| a.partial_cmp(b).unwrap()).unwrap_or(0.0) + 1.0
    }

    pub fn triangulate(&self, min_size: f64) -> Vec<Triangle<f64>> {

        self.triangulate_bounded(
            &AABB::new(
                Point3::new(self.extent_min(0), self.extent_min(1), self.extent_min(2)),
                Point3::new(self.extent_max(0), self.extent_max(1), self.extent_max(2))),
            min_size)

    }

    fn triangulate_bounded(&self, bounds: &AABB<f64>, min_size: f64) -> Vec<Triangle<f64>> {

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

                self.triangulate_bounded(&octant,min_size)

            }).collect()

        } else if self.influence(&bounds.center()) >= THRESHOLD {

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
                self.influence(&sample_pos) < THRESHOLD
            })
            .map(|t| Triangle::new(self.project_onto(*t.a()),self.project_onto(*t.b()),self.project_onto(*t.c())))
            .collect()
        } else {
            vec![]
        }
    }

    fn project_onto(&self, mut pt: Point3<f64>) -> Point3<f64> {
        
        // while self.gradient(&pt).norm_squared() == 0.0 {

        //     let grav_dir : Vector3<f64> = self.points.iter().map(|mb| {
        //         let delta : Vector3<f64> = pt - mb.pos;
        //         let d_2 : f64 = delta.dot(&delta);
        //         delta.normalize() / d_2
        //     }).sum();

        //     pt -= grav_dir.normalize() / 5.0;
        // }

        while (self.influence(&pt) - THRESHOLD).abs() > 0.01 {
            pt -= (self.influence(&pt) - THRESHOLD) * self.gradient(&pt) / 5.0;
        }

        pt
    }
}

