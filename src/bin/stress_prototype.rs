extern crate sdl2; 
extern crate nalgebra as na;

use na::{Point2, Vector2};

use sdl2::pixels::Color;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::rect::{Rect, Point};
use std::time::Duration;

pub fn main() {

    let nodes : Vec<Point2<f32>> = (0..10).map(|i| Point2::new(400f32, 525f32 - (i as f32) * 50f32)).collect();
    let nodes_sdl : Vec<Point> = nodes.iter().map(|pt| Point::new(pt[0] as i32, pt[1] as i32)).collect();
    let edges : Vec<(usize, usize)> = (0..9).flat_map(|i| vec![(i, i+1), (i+1, i)]).collect();

    let mut edge_pressure : Vec<f32> = edges.iter().map(|_| 0.0).collect();

    let gravity = Vector2::new(0f32, 1f32);

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
 
    let window = video_subsystem.window("rust-sdl2 demo", 800, 600)
        .position_centered()
        .build()
        .unwrap();
 
    let mut canvas = window.into_canvas().build().unwrap();
 
    canvas.set_draw_color(Color::RGB(0, 255, 255));
    canvas.clear();
    canvas.present();
    let mut event_pump = sdl_context.event_pump().unwrap();
    
    'running: loop {

        let mut resultants : Vec<Vector2<f32>> = nodes.iter().enumerate()
            .map(|(i, pos)| {
                gravity
            }).collect();

        for ((i,j),p) in edges.iter().zip(edge_pressure.iter()) {
            let delta = nodes[*i] - nodes[*j];

            resultants[*i] -= delta.normalize() * *p;
        }

        edge_pressure

        //region Drawing

        canvas.set_draw_color(Color::RGB(200,200,200));
        canvas.clear();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                _ => {}
            }
        }

        canvas.set_draw_color(Color::RGB(255,0,0));
        for &(i, j) in edges.iter() {
            canvas.draw_line(nodes_sdl[i], nodes_sdl[j]).expect("Drawing edge should succeed.")
        }

        

        for (pt, r_n) in nodes_sdl.iter().zip(resultants.iter().map(|r|r.norm())) {
            canvas.set_draw_color(Color::RGB(0,0,0));
            canvas.fill_rect(Rect::new(pt.x() - 5, pt.y() - 5, 10, 10)).expect("Point cannot be drawn.");

            canvas.set_draw_color(Color::RGB((200.0 * r_n).max(255.0) as u8,
                                             (255.0 - (200.0 * r_n).max(255.0)) as u8,
                                             0));

            canvas.fill_rect(Rect::new(pt.x() - 3, pt.y() - 3, 7, 7)).expect("Point cannot be drawn.");
        }

        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));

        //endregion
    }
}