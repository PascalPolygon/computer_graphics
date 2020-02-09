#[allow(unused_variables)]
extern crate image;
extern crate nalgebra as na;
extern crate ndarray;
#[macro_use]
extern crate rulinalg;

use image::GenericImageView;
// use na::{inverse, Matrix6x6};
use ndarray::{arr2, Array2};
use rulinalg::matrix::decomposition::PartialPivLu;
use rulinalg::matrix::Matrix;
use rulinalg::utils;

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

    // let mut _a = Array2::<i32>::zeros((6, 6));
    let mut _a: Matrix<i32>;

    println!("Source triangle matrix: {:#?}", _x);

    _a = estimate_affine(&_x, &_x_prime);

    println!("M matrix: {:#?}", _a);
}

fn estimate_affine(_x: &Array2<i32>, _x_prime: &Array2<i32>) -> Matrix<i32> {
    let _m = matrix![_x[[0, 0]] as f64, _x[[0, 1]] as f64, 1.0, 0.0, 0.0, 0.0;
                         0.0, 0.0, 0.0, _x[[0, 0]] as f64, _x[[0, 1]] as f64, 1.0;
                         _x[[1, 0]] as f64, _x[[1, 1]] as f64, 1.0, 0.0, 0.0, 0.0;
                         0.0, 0.0, 0.0, _x[[1, 0]] as f64, _x[[1, 1]] as f64, 1.0;
                         _x[[2, 0]] as f64, _x[[2, 1]] as f64, 1.0, 0.0, 0.0, 0.0;
                         0.0, 0.0, 0.0, _x[[2, 0]] as f64, _x[[2, 1]] as f64, 1.0];
    let mut _m_alg = Matrix::new(
        6,
        6,
        vec![
            _x[[0, 0]],
            _x[[0, 1]],
            1,
            0,
            0,
            0,
            0,
            0,
            0,
            _x[[0, 0]],
            _x[[0, 1]],
            1,
            _x[[1, 0]],
            _x[[1, 1]],
            1,
            0,
            0,
            0,
            0,
            0,
            0,
            _x[[1, 0]],
            _x[[1, 1]],
            1,
            _x[[2, 0]],
            _x[[2, 1]],
            1,
            0,
            0,
            0,
            0,
            0,
            0,
            _x[[2, 0]],
            _x[[2, 1]],
            1,
        ],
    );
    let _m_lu = PartialPivLu::decompose(_m).expect("Matrix is invertible.");
    //compute inverse
    let mut _m_inv = _m_lu.inverse();
    println!("M iverse: {:#?}", _m_inv);
    // let _b = arr2(&[
    //     [_x_prime[[0, 0]]],
    //     [],
    //     [_x_prime[[1, 0]]],
    //     [_x_prime[[1, 1]]],
    //     [_x_prime[[2, 0]]],
    //     [_x_prime[[2, 1]]],
    // ]);
    let _b = matrix![_x_prime[[0, 0]];
     _x_prime[[0, 1]];
     _x_prime[[1, 0]];
     _x_prime[[1, 1]];
     _x_prime[[2, 0]];
     _x_prime[[2, 1]];
    ];
    //compute transformation matrix
    let mut _a = utils::dot(&_m_inv, &_b);
    // let mut a = _m_inv * b;
    println!("b matrix: {:#?}", _b);
    _m_alg
}
