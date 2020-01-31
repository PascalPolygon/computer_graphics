#[allow(unused_variables)]
extern crate image;
extern crate nalgebra;
extern crate ndarray;

use image::GenericImageView;
use ndarray::arr2;

#[derive(Debug)]
struct Point {
    x: i32,
    y: i32,
}

#[derive(Debug)]
struct Triangle {
    p_0: Point,
    p_1: Point,
    p_2: Point,
}

fn main() {
    println!("Hello, world!");
    let img = image::open("/home/pascal/computer_graphics/santa-fung-afro-007.jpg").unwrap();
    println!("dimensions {:?}", img.dimensions());

    //Hard coded src triangle coordinates
    let src_tri = Triangle {
        p_0: Point { x: 200, y: 200 },
        p_1: Point { x: 1400, y: 200 },
        p_2: Point { x: 1000, y: 1000 },
    };

    let dst_tri = Triangle {
        p_0: Point { x: 200, y: 800 },
        p_1: Point { x: 1200, y: 400 },
        p_2: Point { x: 900, y: 1000 },
    };

    println!(
        "Source triangle: {:#?}\n Destination triangle: {:#?}",
        src_tri, dst_tri
    );

    let _x = arr2(&[
        [src_tri.p_0.x, src_tri.p_0.y],
        [src_tri.p_1.x, src_tri.p_1.y],
        [src_tri.p_2.x, src_tri.p_2.y],
    ]);

    let _x_prime = arr2(&[
        [dst_tri.p_0.x, dst_tri.p_0.y],
        [dst_tri.p_1.x, dst_tri.p_1.y],
        [dst_tri.p_2.x, dst_tri.p_2.y],
    ]);

    println!("Source triangle matrix: {:#?}", _x);
}
