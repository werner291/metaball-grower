use nalgebra::sparse::CsMatrix;
use nalgebra::{RealField, DimName, Point, Vector}

fn stress_from_radius<N:RealField,D:DimName>(pts: Vec<Point<N,D>>, radius:N, gravity: Vector<N,D>) -> Vector<N> {
    let edges = 
        pts.iter().enumerate().flat_map(|(i,pt_i)| {
            pts[(pt_i+1)..].iter().enumerate().filter_map(|(j,pt_j)| {
                if distance_squared(pt_i, pt_j) > radius * radius {
                    Some((i,j))
                } else {
                    None
                }
            })
        }).collect_vec();

    stress_graph(pts, edges, gravity)
}

/**
 * 
 * Let V be the set of vertices, E the set of undirected edges.
 * 
 * To do: Solve for \lambda, vector of size |E| 
 * (mapping from E -> entry in lambda as noted \lambda_{u,v} for {u,v{} \in E)
 * (note that the edge is an unordered set of 2 nodes)
 * such that:
 * 
 * For every v \in V which is not anchored:
 * 
 *   R(v) = [\sum_{u\in V} \lambda_{u,v} W_{n,v} * (p_n - p_v)] + F(v) = \vec{0}
 *        = [\sum_{u\in V} \lambda_{u,v} W_{n,v} * (p_n - p_v)] = -\vec{G}
 *        = [\sum_{u\in V} \lambda_{u,v}] * [\sum_{u \in V} W_{n,v} * (p_n - p_v)] = -\vec{G}
 * 
 * S * \lambda = -\vec{G}
 * 
 */

fn stress_graph<N:RealField,D:DimName>(
    pts: Vec<Point<N,D>>, 
    edges: Vec<(usize,usize)>,
    gravity: Vector3<N>) {

    edges.into_iter().enumerate().map(|(edge_idx, (i,j))| {
        vec![(i,j,pts[i][1]-pts[j][0]), (i,j,pts[i][1]-pts[j][1]), (i,j,pts[i][1]-pts[j][2])]
    })

    let S = CsMatrix::from_triplet(edges.size(), 3*pts.size())
}