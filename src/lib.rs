use lyon::algorithms::hit_test::hit_test_path;
use lyon::geom::arrayvec::ArrayVec;
use lyon::geom::{BezierSegment, CubicBezierSegment, LineSegment, QuadraticBezierSegment};
use lyon::path::{FillRule, PathEvent};
use rayon::prelude::*;

trait BezierSegmentExt<S> {
    fn to_linear(self) -> Option<LineSegment<S>>;
    fn to_quadratic(self) -> Option<QuadraticBezierSegment<S>>;
    fn to_cubic(self) -> Option<CubicBezierSegment<S>>;
}
impl<S> BezierSegmentExt<S> for BezierSegment<S> {
    fn to_linear(self) -> Option<LineSegment<S>> {
        match self {
            BezierSegment::Linear(inner) => Some(inner),
            _ => None,
        }
    }
    fn to_quadratic(self) -> Option<QuadraticBezierSegment<S>> {
        match self {
            BezierSegment::Quadratic(inner) => Some(inner),
            _ => None,
        }
    }
    fn to_cubic(self) -> Option<CubicBezierSegment<S>> {
        match self {
            BezierSegment::Cubic(inner) => Some(inner),
            _ => None,
        }
    }
}

fn bezier_segment(event: PathEvent) -> Option<BezierSegment<f32>> {
    match event {
        PathEvent::Begin { .. } => None,
        PathEvent::Line { from, to } => Some(LineSegment { from, to }.into()),
        PathEvent::End {
            last,
            first,
            close: true,
        } => Some(
            LineSegment {
                from: last,
                to: first,
            }
            .into(),
        ),
        PathEvent::End { close: false, .. } => None,
        PathEvent::Quadratic { from, ctrl, to } => {
            Some(QuadraticBezierSegment { from, to, ctrl }.into())
        }
        PathEvent::Cubic {
            from,
            ctrl1,
            ctrl2,
            to,
        } => Some(
            CubicBezierSegment {
                from,
                to,
                ctrl1,
                ctrl2,
            }
            .into(),
        ),
    }
}

fn intersections_t(
    left: &BezierSegment<f32>,
    right: &BezierSegment<f32>,
) -> ArrayVec<[(f32, f32); 9]> {
    let mut out = ArrayVec::new();
    match left {
        BezierSegment::Linear(left) => match right {
            BezierSegment::Linear(right) => {
                if let Some(intersection) = left.intersection_t(right) {
                    out.push(intersection);
                }
            }
            BezierSegment::Quadratic(right) => {
                for (tr, tl) in right.line_segment_intersections_t(&left) {
                    out.push((tl, tr))
                }
            }
            BezierSegment::Cubic(right) => {
                for (tr, tl) in right.line_segment_intersections_t(&left) {
                    out.push((tl, tr))
                }
            }
        },
        BezierSegment::Quadratic(left) => match right {
            BezierSegment::Linear(right) => {
                out.try_extend_from_slice(&left.line_segment_intersections_t(&right))
                    .unwrap();
            }
            BezierSegment::Quadratic(right) => {
                out.try_extend_from_slice(&left.to_cubic().quadratic_intersections_t(&right))
                    .unwrap();
            }
            BezierSegment::Cubic(right) => {
                out.try_extend_from_slice(&left.to_cubic().cubic_intersections_t(&right))
                    .unwrap();
            }
        },
        BezierSegment::Cubic(left) => match right {
            BezierSegment::Linear(right) => {
                out.try_extend_from_slice(&left.line_segment_intersections_t(&right))
                    .unwrap();
            }
            BezierSegment::Quadratic(right) => {
                out.try_extend_from_slice(&left.quadratic_intersections_t(&right))
                    .unwrap();
            }
            BezierSegment::Cubic(right) => {
                out.try_extend_from_slice(&left.cubic_intersections_t(&right))
                    .unwrap();
            }
        },
    }
    out
}

struct IndexedIntersectionT {
    left: (usize, f32),
    right: (usize, f32),
}

fn path_intersections_t(left: &[PathEvent], right: &[PathEvent]) -> Vec<IndexedIntersectionT> {
    left.into_par_iter()
        .enumerate()
        .flat_map(|(left_index, left_event)| {
            right
                .into_par_iter()
                .enumerate()
                .map(move |(right_index, right_event)| {
                    ((left_index, left_event), (right_index, right_event))
                })
        })
        .filter_map(|((left_index, left_event), (right_index, right_event))| {
            bezier_segment(*left_event).and_then(|left| {
                bezier_segment(*right_event).map(|right| ((left_index, left), (right_index, right)))
            })
        })
        .flat_map(|((left_index, left), (right_index, right))| {
            intersections_t(&left, &right)
                .as_slice()
                .to_vec()
                .into_par_iter()
                .map(move |(tl, tr)| IndexedIntersectionT {
                    left: (left_index, tl),
                    right: (right_index, tr),
                })
        })
        .collect()
}

fn insert_intersection(events: &mut Vec<PathEvent>, index: usize, t: f32) {
    let event = events[index];
    let segment = bezier_segment(event).expect("PathEvent should be a valid BezierSegment");
    let (left, right) = segment.split(t);
    match event {
        PathEvent::Begin { .. } => panic!("PathEvent::Begin not expected"),
        PathEvent::Line { .. } => {
            let left = left
                .to_linear()
                .expect("PathEvent::Line expects BezierSegment::Linear");
            let right = right
                .to_linear()
                .expect("PathEvent::Line expects BezierSegment::Linear");
            events[index] = PathEvent::Line {
                from: left.from,
                to: left.to,
            };
            events.insert(
                index + 1,
                PathEvent::Line {
                    from: right.from,
                    to: right.to,
                },
            )
        }
        PathEvent::End { close: true, .. } => {
            let left = left
                .to_linear()
                .expect("PathEvent::End expects BezierSegment::Linear");
            let right = right
                .to_linear()
                .expect("PathEvent::End expects BezierSegment::Linear");
            events[index] = PathEvent::Line {
                from: left.from,
                to: left.to,
            };
            events.insert(
                index + 1,
                PathEvent::End {
                    last: right.from,
                    first: right.to,
                    close: true,
                },
            )
        }
        PathEvent::End { close: false, .. } => panic!("PathEvent::End must be closed"),
        PathEvent::Quadratic { .. } => {
            let left = left
                .to_quadratic()
                .expect("PathEvent::End expects BezierSegment::Linear");
            let right = right
                .to_quadratic()
                .expect("PathEvent::End expects BezierSegment::Linear");
            events[index] = PathEvent::Quadratic {
                from: left.from,
                ctrl: left.ctrl,
                to: left.to,
            };
            events.insert(
                index + 1,
                PathEvent::Quadratic {
                    from: right.from,
                    ctrl: right.ctrl,
                    to: right.to,
                },
            )
        }
        PathEvent::Cubic { .. } => {
            let left = left
                .to_cubic()
                .expect("PathEvent::End expects BezierSegment::Linear");
            let right = right
                .to_cubic()
                .expect("PathEvent::End expects BezierSegment::Linear");
            events[index] = PathEvent::Cubic {
                from: left.from,
                ctrl1: right.ctrl1,
                ctrl2: right.ctrl2,
                to: left.to,
            };
            events.insert(
                index + 1,
                PathEvent::Cubic {
                    from: right.from,
                    ctrl1: right.ctrl1,
                    ctrl2: right.ctrl2,
                    to: right.to,
                },
            )
        }
    }
}

fn insert_intersections<I>(path: &mut Vec<PathEvent>, insertions: I) -> Vec<usize>
where
    I: Iterator<Item = (usize, f32)>,
{
    let insertions: Vec<(usize, f32)> = insertions.collect();
    let mut sorted: Vec<(usize, f32)> = insertions.clone();
    sorted.sort_by(|(i1, f1), (i2, f2)| match i1.cmp(i2) {
        std::cmp::Ordering::Equal => f1.partial_cmp(f2).unwrap(),
        c => c,
    });
    let mut inserted_indices = vec![0; insertions.len()];
    let mut offset = 0;
    for (index, t) in sorted {
        insert_intersection(path, index + offset, t);
        let og_index = insertions.iter().position(|i| i == &(index, t)).unwrap();
        inserted_indices[og_index] = index + offset;
        offset += 1;
    }
    inserted_indices
}

fn update_intersections(
    left: &mut Vec<PathEvent>,
    right: &mut Vec<PathEvent>,
) -> Vec<(usize, usize)> {
    let intersections = path_intersections_t(left, right);
    let left_inserted = insert_intersections(left, intersections.iter().map(|i| i.left));
    let right_inserted = insert_intersections(right, intersections.iter().map(|i| i.right));
    left_inserted
        .into_iter()
        .zip(right_inserted.into_iter())
        .collect()
}

#[derive(Copy, Clone)]
enum IntersectionLabel {
    InsideToOutside,
    OutsideToInside,
}

fn label_intersections(
    left: &[PathEvent],
    right: &[PathEvent],
    intersections: &[(usize, usize)],
    fill_rule: FillRule,
    tolerance: f32,
) -> Vec<IntersectionLabel> {
    let right_intersections = intersections.iter().map(|(_, i)| *i).collect::<Vec<_>>();
    let mut inside = match &right[0] {
        PathEvent::Begin { at } => hit_test_path(at, left.iter().cloned(), fill_rule, tolerance),
        _ => panic!("path should start with PathEvent::Begin"),
    };
    let mut intersection_labels = Vec::with_capacity(right_intersections.len());
    for (index, _) in right.iter().enumerate() {
        if right_intersections.contains(&index) {
            intersection_labels.push(if inside {
                IntersectionLabel::InsideToOutside
            } else {
                IntersectionLabel::OutsideToInside
            });
            inside = !inside;
        }
    }
    intersection_labels
}

#[derive(Copy, Clone)]
pub enum SelectionRule {
    Intersection,
}

fn select_path_events(
    left: &[PathEvent],
    right: &[PathEvent],
    intersections: &[(usize, usize)],
    intersection_labels: &[IntersectionLabel],
    selection_rule: SelectionRule,
) -> Vec<PathEvent> {
    let starting_intersection = intersections[0];
    let mut is_cur_left = true;
    let mut cur_path_index = starting_intersection.0;
    let mut out = Vec::with_capacity(right.len());

    let starting_point = match &left[cur_path_index] {
        PathEvent::Line { to, .. }
        | PathEvent::Quadratic { to, .. }
        | PathEvent::Cubic { to, .. } => *to,
        PathEvent::Begin { .. } => {
            unreachable!("an intersection cannot occur at a PathEvent::Begin")
        }
        PathEvent::End {
            first, close: true, ..
        } => *first,
        PathEvent::End { close: false, .. } => {
            unreachable!("an intersection cannot occur at a PathEvent::End that does not close")
        }
    };
    out.push(PathEvent::Begin { at: starting_point });

    loop {
        if let Some(index) = intersections
            .iter()
            .map(|(l, r)| if is_cur_left { l } else { r })
            .enumerate()
            .find_map(|(index, intersection)| {
                if intersection == &cur_path_index {
                    Some(index)
                } else {
                    None
                }
            })
        {
            let label = intersection_labels[index];
            let intersection = intersections[index];
            match (label, selection_rule) {
                (IntersectionLabel::InsideToOutside, SelectionRule::Intersection) => {
                    is_cur_left = true;
                    cur_path_index = intersection.0;
                }
                (IntersectionLabel::OutsideToInside, SelectionRule::Intersection) => {
                    is_cur_left = false;
                    cur_path_index = intersection.1;
                }
            }
        }

        cur_path_index += 1;
        if (is_cur_left && cur_path_index >= left.len())
            || (!is_cur_left && cur_path_index >= right.len())
        {
            // skip PathEvent::Begin event
            cur_path_index = 1;
        }

        let (next_event, mut is_end) = if is_cur_left {
            (
                left[cur_path_index],
                cur_path_index == starting_intersection.0,
            )
        } else {
            (
                right[cur_path_index],
                cur_path_index == starting_intersection.1,
            )
        };

        match next_event {
            PathEvent::Begin { .. } => {
                panic!("should skip first PathEvent::Begin while select path events")
            }
            PathEvent::End {
                last, close: false, ..
            } => {
                out.push(PathEvent::End {
                    last,
                    first: starting_point,
                    close: false,
                });
                is_end = true;
            }
            PathEvent::End {
                last,
                first,
                close: true,
            } => {
                if is_end {
                    out.push(PathEvent::End {
                        last,
                        first: starting_point,
                        close: true,
                    });
                } else {
                    out.push(PathEvent::Line {
                        from: last,
                        to: first,
                    });
                }
            }
            PathEvent::Line { from, to } => {
                if is_end {
                    out.push(PathEvent::End {
                        last: from,
                        first: starting_point,
                        close: true,
                    });
                } else {
                    out.push(PathEvent::Line { from, to });
                }
            }
            PathEvent::Quadratic { from, to, ctrl } => {
                out.push(PathEvent::Quadratic { from, to, ctrl });
                if is_end {
                    out.push(PathEvent::End {
                        last: to,
                        first: starting_point,
                        close: true,
                    });
                }
            }
            PathEvent::Cubic {
                from,
                to,
                ctrl1,
                ctrl2,
            } => {
                out.push(PathEvent::Cubic {
                    from,
                    to,
                    ctrl1,
                    ctrl2,
                });
                if is_end {
                    out.push(PathEvent::End {
                        last: to,
                        first: starting_point,
                        close: true,
                    });
                }
            }
        };

        if is_end {
            break;
        }
    }

    out
}

pub fn clip<LeftIter, RightIter>(
    left: LeftIter,
    right: RightIter,
    selection_rule: SelectionRule,
    fill_rule: FillRule,
    tolerance: f32,
) -> Vec<PathEvent>
where
    LeftIter: IntoIterator<Item = PathEvent>,
    RightIter: IntoIterator<Item = PathEvent>,
{
    let mut left = left.into_iter().collect();
    let mut right = right.into_iter().collect();
    let intersections = update_intersections(&mut left, &mut right);
    let intersection_labels =
        label_intersections(&left, &right, &intersections, fill_rule, tolerance);
    select_path_events(
        &left,
        &right,
        &intersections,
        &intersection_labels,
        selection_rule,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intersect_squares() {
        use lyon::geom::point;
        use lyon::path::polygon::Polygon;
        let left = Polygon {
            points: &[
                point(-10., 10.),
                point(10., 10.),
                point(10., -10.),
                point(-10., -10.),
            ],
            closed: true,
        };

        let right = Polygon {
            points: &[
                point(-5., 5.),
                point(15., 5.),
                point(15., -15.),
                point(-5., -15.),
            ],
            closed: true,
        };

        let out = clip(
            left.path_events(),
            right.path_events(),
            SelectionRule::Intersection,
            FillRule::NonZero,
            0.,
        );

        let expected_out = Polygon {
            points: &[
                point(10., 5.),
                point(10., -10.),
                point(-5., -10.),
                point(-5., 5.),
            ],
            closed: true,
        };

        assert_eq!(out, expected_out.path_events().collect::<Vec<_>>());
    }
}
