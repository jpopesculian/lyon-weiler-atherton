use lyon::math::point;
use lyon::path::{FillRule, Polygon};
use lyon_weiler_atherton::{clip, SelectionRule};
use std::fs::File;
use xml_utils::{Color, FillOptions, Svg, SvgPath, ViewBox};

fn main() {
    let left = Polygon {
        points: &[
            point(-100., 100.),
            point(100., 100.),
            point(100., -100.),
            point(-100., -100.),
        ],
        closed: true,
    };

    let right = Polygon {
        points: &[
            point(-50., 50.),
            point(150., 50.),
            point(150., -150.),
            point(-50., -150.),
        ],
        closed: true,
    };

    let out = clip(
        left.path_events(),
        right.path_events(),
        SelectionRule::Intersection,
        FillRule::NonZero,
        0.,
    )
    .pop();

    let view_box = ViewBox::from_wh(400., 400.);
    let left_svg = SvgPath {
        path: left.path_events().collect(),
        fill: Some(FillOptions {
            color: Color::Rgb(255, 0, 0),
        }),
        ..Default::default()
    };
    let right_svg = SvgPath {
        path: right.path_events().collect(),
        fill: Some(FillOptions {
            color: Color::Rgb(0, 255, 0),
        }),
        ..Default::default()
    };
    let mut svgs = vec![left_svg, right_svg];

    for path in out {
        svgs.push(SvgPath {
            path,
            fill: Some(FillOptions {
                color: Color::Rgb(0, 0, 255),
            }),
            ..Default::default()
        });
    }

    let svg = Svg {
        view_box,
        children: svgs,
    };

    let mut out = File::create("target/squares_intersection.svg").unwrap();
    svg.write(&mut out, None).unwrap();
}
