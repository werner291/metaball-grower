use nalgebra::{Point3, Vector3, distance_squared};
use rand::prelude::*;
use sdl2::event::Event;
use sdl2::gfx::primitives::DrawRenderer;
use sdl2::keyboard::Keycode;
use std::time::Duration;
use ncollide3d::query::{RayCast, Ray};
use ncollide3d::bounding_volume::BoundingSphere;
use ncollide3d::math::Isometry;

//region Metaballs

#[derive(Debug)]
struct Metaball {
    pos: Point3<f64>
}

impl Metaball {
    
    #[inline]
    fn influence(&self, pt: &Point3<f64>) -> f64 {
        let r_2 = distance_squared(&self.pos, pt);

        if r_2 > 1.0 {
            0.0
        } else {
            (1.0 - r_2) * (1.0 - r_2)
        }
    }

     #[inline]
    fn gradient(&self, pt: &Point3<f64>) -> Vector3<f64> {
        (pt - self.pos).normalize()
    }
}

struct MetaballShape {
    points: Vec<Metaball>
}

impl MetaballShape {

    fn influence(&self, pt: &Point3<f64>) -> f64 {
        self.points.iter().map(|p| p.influence(pt)).sum()
    }

    fn gradient(&self, pt: &Point3<f64>) -> Vector3<f64> {
        self.points.iter().fold(Vector3::new(0.0,0.0,0.0), |g, p| g + p.gradient(pt) * p.influence(pt)).normalize()
    }

    pub fn raycast(&self, ray: Ray<f64>) -> Option<Point3<f64>> {
        // let ball = Ball::new(1.0);

        let mut intersecting_balls : Vec<(f64,&Metaball)> = self.points.iter().filter_map(|pt| {
            BoundingSphere::new(pt.pos, 1.0).toi_with_ray(&Isometry::identity(), &ray, true).map(|toi| (toi, pt))
        }).collect();

        intersecting_balls.sort_by(|(a,_),(b,_)| a.partial_cmp(&b).expect("No NaNs expected."));

        if intersecting_balls.is_empty() {
            return None;
        }

        let mut from = 0;
        let mut to = 1;
        let mut t = intersecting_balls[0].0;

        while from < intersecting_balls.len() {

            while to < intersecting_balls.len() && t > intersecting_balls[to-1].0 {
                to += 1;
            }

            t += 0.05;

            let ray_pt = ray.origin + t * ray.dir;
            
            let value : f64 = intersecting_balls[from..to].iter().map(|(_,p)| p.influence(&ray_pt)).sum();

            if value > 0.5 {

                return Some(ray_pt);
            }

            while from < intersecting_balls.len() && t > intersecting_balls[from].0 + 1.0 {
                from += 1;
            }
        }
        
        None
    }
}

//endregion

//region Rendering

const WIDTH: u32 = 400;
const HEIGHT: u32 = 400;

fn view_ray(x: f64, y: f64) -> Ray<f64> {
    let aspect_ratio = HEIGHT as f64 / WIDTH as f64;

    Ray::new(
        Point3::new(0.0, 0.0, -5.0),
        Vector3::new(
            x / WIDTH as f64 - 0.5,
            -(y / WIDTH as f64 - aspect_ratio / 2.0),
            1.0,
        ).normalize(),
    )
}

pub fn main() {
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem
        .window("rust-sdl2 demo", WIDTH, HEIGHT)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut rng = rand::thread_rng();

    'running: loop {

        let constr = MetaballShape {
            points: (0..20).map(|_| Metaball {
                pos: Point3::new(rng.gen_range(-2.0,2.0), 
                                 rng.gen_range(-2.0,2.0), 
                                 rng.gen_range(-2.0,2.0))
                }).collect()
        };

        let skip = 1;

        canvas.set_draw_color((0,0,0,255));
        canvas.fill_rect(canvas.viewport()).expect("Failed to clear image.");

        // Drawing
        for x in (0..canvas.viewport().width()).step_by(skip) {
            for y in (0..canvas.viewport().height()).step_by(skip) {
                let ray = view_ray(x as f64, y as f64);

                if let Some(pt) = constr.raycast(ray) {
                
                    let gradient = constr.gradient(&pt);
                    let light_dir = Vector3::new(0.0,1.0,0.0);

                    let light_intensity = gradient.dot(&light_dir).max(0.0) * 0.8 + 0.2;

                    let color = ((light_intensity * 255.0) as u8,
                                 (light_intensity * 255.0) as u8,
                                 (light_intensity * 255.0) as u8,
                                 255);

                    for xx in x..x + skip as u32 {
                        for yy in y..y + skip as u32 {
                            canvas.pixel(xx as i16, yy as i16, color).unwrap();
                        }
                    }
                }

            }

            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit { .. }
                    | Event::KeyDown {
                        keycode: Some(Keycode::Escape),
                        ..
                    } => {
                        break 'running;
                    }
                    _ => {}
                }
            }

            canvas.present();
        }

        ::std::thread::sleep(Duration::from_millis(2000));
    };
}

//endregion