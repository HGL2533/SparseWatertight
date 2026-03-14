#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <cmath>

#include <Eigen/Core>
#include <igl/point_mesh_squared_distance.h>
#include <igl/per_face_normals.h>
//#include <igl/flood_fill.h>
#include <igl/adjacency_list.h>
#include <igl/remove_unreferenced.h>
#include <igl/barycenter.h>
#include <igl/doublearea.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>
//#include <igl/cotmatrix.h>
//#include <igl/massmatrix.h>
//#include <igl/invert_diag.h>
#include <igl/aabb.h>
#include <igl/fast_winding_number.h>
#include <igl/embree/reorient_facets_raycast.h>
#include <igl/dual_contouring.h>
#include <igl/marching_cubes.h>
#include <igl/marching_tets.h>
#include <omp.h>


struct Vector3iHash {
    std::size_t operator()(const Eigen::RowVector3i& k) const {
        using std::size_t;
        using std::hash;
        size_t h1 = hash<int>()(k(0));
        size_t h2 = hash<int>()(k(1));
        size_t h3 = hash<int>()(k(2));
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct GridNode {
    double sdf;
    Eigen::RowVector3d normal;
    Eigen::RowVector3d position;
};

class SparseSDFGrid {
public:
    // 存储所有活跃的角点数据，Key 是 Grid 索引 (i,j,k)
    std::unordered_map<Eigen::RowVector3i, GridNode, Vector3iHash> sparse_nodes;

    // 存储所有活跃的体素 (Cell) 索引
    std::vector<Eigen::RowVector3i> active_cells;

    double cell_size;
    int _resolution;
    Eigen::RowVector3d grid_origin;

public:
    // 构造函数：输入 Mesh 和 分辨率
    // resolution: 长轴方向大约划分多少个格子
    SparseSDFGrid(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, int resolution) {
        build(V, F, resolution);
    }


    void compute_sdf_values_floodfill(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    void compute_sdf_values_windingnumber(const Eigen::MatrixXd& V, Eigen::MatrixXi& F, double threshold = 0.5);

    void optimize_grid(const Eigen::MatrixXd& V_target, const Eigen::MatrixXi& F_target, int iters, double lr);
    void bilateral_mesh_denoise(Eigen::MatrixXd& V, Eigen::MatrixXi& F,
        double SigmaNormal = 0.2,
        int NormalIterNum = 10,
        int VertexIterNum = 10);

    void extract_mesh_dc(Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out);
    void extract_mesh_mc(Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out, double isovalue = 0.0);
    void extract_mesh_tets(Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out, double isovalue = 0.0);
    
    void export_grid_wireframe(const std::string& filename);

private:
    void build(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, int resolution);
    

    inline bool triBoxOverlap(
        const Eigen::RowVector3d& box_center,
        const Eigen::RowVector3d& box_half,
        const Eigen::RowVector3d& v0,
        const Eigen::RowVector3d& v1,
        const Eigen::RowVector3d& v2);

};