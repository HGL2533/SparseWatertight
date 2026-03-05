#include "sparseGrid.hpp"

// 来自 Akenine-Möller, 2001
// tri: v0, v1, v2
// box: center + halfsize
inline bool SparseSDFGrid::triBoxOverlap(
    const Eigen::RowVector3d& box_center,
    const Eigen::RowVector3d& box_half,
    const Eigen::RowVector3d& v0,
    const Eigen::RowVector3d& v1,
    const Eigen::RowVector3d& v2)
{
    // Move triangle into box coordinate
    Eigen::RowVector3d tv0 = v0 - box_center;
    Eigen::RowVector3d tv1 = v1 - box_center;
    Eigen::RowVector3d tv2 = v2 - box_center;

    Eigen::RowVector3d e0 = tv1 - tv0;
    Eigen::RowVector3d e1 = tv2 - tv1;
    Eigen::RowVector3d e2 = tv0 - tv2;

    auto axisTest = [&](double a, double b,
        double fa, double fb,
        double v0a, double v0b,
        double v1a, double v1b,
        double v2a, double v2b,
        double box_a, double box_b)
        {
            double p0 = a * v0a - b * v0b;
            double p1 = a * v1a - b * v1b;
            double p2 = a * v2a - b * v2b;

            double min_p = std::min({ p0, p1, p2 });
            double max_p = std::max({ p0, p1, p2 });

            double rad = fa * box_a + fb * box_b;
            return !(min_p > rad || max_p < -rad);
        };

    // Edge 0
    {
        double fex = std::abs(e0.x());
        double fey = std::abs(e0.y());
        double fez = std::abs(e0.z());

        if (!axisTest(e0.z(), e0.y(), fez, fey,
            tv0.y(), tv0.z(), tv1.y(), tv1.z(), tv2.y(), tv2.z(),
            box_half.y(), box_half.z())) return false;

        if (!axisTest(e0.z(), e0.x(), fez, fex,
            tv0.x(), tv0.z(), tv1.x(), tv1.z(), tv2.x(), tv2.z(),
            box_half.x(), box_half.z())) return false;

        if (!axisTest(e0.y(), e0.x(), fey, fex,
            tv0.x(), tv0.y(), tv1.x(), tv1.y(), tv2.x(), tv2.y(),
            box_half.x(), box_half.y())) return false;
    }

    // Edge 1
    {
        double fex = std::abs(e1.x());
        double fey = std::abs(e1.y());
        double fez = std::abs(e1.z());

        if (!axisTest(e1.z(), e1.y(), fez, fey,
            tv0.y(), tv0.z(), tv1.y(), tv1.z(), tv2.y(), tv2.z(),
            box_half.y(), box_half.z())) return false;

        if (!axisTest(e1.z(), e1.x(), fez, fex,
            tv0.x(), tv0.z(), tv1.x(), tv1.z(), tv2.x(), tv2.z(),
            box_half.x(), box_half.z())) return false;

        if (!axisTest(e1.y(), e1.x(), fey, fex,
            tv0.x(), tv0.y(), tv1.x(), tv1.y(), tv2.x(), tv2.y(),
            box_half.x(), box_half.y())) return false;
    }

    // Edge 2
    {
        double fex = std::abs(e2.x());
        double fey = std::abs(e2.y());
        double fez = std::abs(e2.z());

        if (!axisTest(e2.z(), e2.y(), fez, fey,
            tv0.y(), tv0.z(), tv1.y(), tv1.z(), tv2.y(), tv2.z(),
            box_half.y(), box_half.z())) return false;

        if (!axisTest(e2.z(), e2.x(), fez, fex,
            tv0.x(), tv0.z(), tv1.x(), tv1.z(), tv2.x(), tv2.z(),
            box_half.x(), box_half.z())) return false;

        if (!axisTest(e2.y(), e2.x(), fey, fex,
            tv0.x(), tv0.y(), tv1.x(), tv1.y(), tv2.x(), tv2.y(),
            box_half.x(), box_half.y())) return false;
    }

    // AABB overlap test
    for (int i = 0; i < 3; ++i) {
        double min_v = std::min({ tv0[i], tv1[i], tv2[i] });
        double max_v = std::max({ tv0[i], tv1[i], tv2[i] });
        if (min_v > box_half[i] || max_v < -box_half[i])
            return false;
    }

    // Plane-box overlap
    Eigen::RowVector3d normal = e0.cross(e1);
    double d = -normal.dot(tv0);

    double r =
        box_half.x() * std::abs(normal.x()) +
        box_half.y() * std::abs(normal.y()) +
        box_half.z() * std::abs(normal.z());

    double s = normal.dot(Eigen::RowVector3d::Zero()) + d;
    if (std::abs(s) > r) return false;

    return true;
}



void SparseSDFGrid::build(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, int resolution) {
    active_cells.clear();
    sparse_nodes.clear();
    this->_resolution = resolution;

    Eigen::RowVector3d min_point = V.colwise().minCoeff();
    Eigen::RowVector3d max_point = V.colwise().maxCoeff();
    Eigen::RowVector3d extent = max_point - min_point;

    double cell_size = extent.maxCoeff() / double(_resolution);
    this->cell_size = cell_size;

    double origin_padding = 2.0 * cell_size;
    grid_origin = min_point - Eigen::RowVector3d::Constant(origin_padding);

    // ==========================================================
    // 扩大基础判定盒，替代昂贵的后处理膨胀
    // 原来是 0.5 * cell_size (刚好贴合体素)，现在设为 1.0 * cell_size。
    // 这相当于让相交判定自带了 0.5 个 cell_size 的容错，
    // 刚好覆盖了后续 eps = 0.5 * cell_size 的等值面偏移。
    // ==========================================================
    const int GRID_PADDING = 2; // 稍微调大一点保证循环边界安全
    Eigen::RowVector3d box_half = Eigen::RowVector3d::Constant(1.0 * cell_size);

    // 直接用 unordered_set 去重收集，得到最终的 active_cells
    std::unordered_set<Eigen::RowVector3i, Vector3iHash> tight_cell_set;

    for (int f = 0; f < F.rows(); ++f) {
        Eigen::RowVector3d v0 = V.row(F(f, 0));
        Eigen::RowVector3d v1 = V.row(F(f, 1));
        Eigen::RowVector3d v2 = V.row(F(f, 2));

        Eigen::RowVector3d tri_min = v0.cwiseMin(v1).cwiseMin(v2);
        Eigen::RowVector3d tri_max = v0.cwiseMax(v1).cwiseMax(v2);

        Eigen::RowVector3i idx_min = ((tri_min - grid_origin) / cell_size).array().floor().cast<int>();
        Eigen::RowVector3i idx_max = ((tri_max - grid_origin) / cell_size).array().ceil().cast<int>();

        idx_min.array() -= GRID_PADDING;
        idx_max.array() += GRID_PADDING;

        for (int i = idx_min.x(); i <= idx_max.x(); ++i) {
            for (int j = idx_min.y(); j <= idx_max.y(); ++j) {
                for (int k = idx_min.z(); k <= idx_max.z(); ++k) {
                    Eigen::RowVector3d box_center = grid_origin + (Eigen::RowVector3d(i, j, k) + Eigen::RowVector3d(0.5, 0.5, 0.5)) * cell_size;
                    // 使用放大后的 box_half 进行测试
                    if (triBoxOverlap(box_center, box_half, v0, v1, v2)) {
                        tight_cell_set.insert(Eigen::RowVector3i(i, j, k));
                    }
                }
            }
        }
    }

    active_cells.assign(tight_cell_set.begin(), tight_cell_set.end());

    // 收集角点节点 (保持不变)
    for (const auto& cell_idx : active_cells) {
        for (int dx = 0; dx <= 1; ++dx) {
            for (int dy = 0; dy <= 1; ++dy) {
                for (int dz = 0; dz <= 1; ++dz) {
                    Eigen::RowVector3i node_idx = cell_idx + Eigen::RowVector3i(dx, dy, dz);
                    if (sparse_nodes.find(node_idx) == sparse_nodes.end()) {
                        GridNode node;
                        node.position = grid_origin + node_idx.cast<double>() * cell_size;
                        sparse_nodes[node_idx] = node;
                    }
                }
            }
        }
    }
}



void SparseSDFGrid::compute_sdf_values_floodfill(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    Eigen::MatrixXd P(sparse_nodes.size(), 3);
    std::vector<Eigen::RowVector3i> map_keys;
    map_keys.reserve(sparse_nodes.size());

    int row = 0;
    for (const auto& kv : sparse_nodes) {
        P.row(row) = kv.second.position;
        map_keys.push_back(kv.first);
        row++;
    }

    Eigen::VectorXd sqD;
    Eigen::VectorXi I;
    Eigen::MatrixXd C;
    igl::point_mesh_squared_distance(P, V, F, sqD, I, C);

    Eigen::RowVector3i max_key = Eigen::RowVector3i::Constant(0);
    for (const auto& key : map_keys) {
        max_key = max_key.cwiseMax(key);
    }

    Eigen::Vector3i res = (max_key + Eigen::RowVector3i(2, 2, 2)).cast<int>();
    int total_nodes = res.x() * res.y() * res.z();
    Eigen::VectorXf S(total_nodes);
    S.setConstant(std::numeric_limits<float>::quiet_NaN());

    // 构建墙壁
    double eps = 0.5 * cell_size + 1e-6;
    double eps_sq = eps * eps;
    for (int i = 0; i < map_keys.size(); ++i) {
        Eigen::RowVector3i key = map_keys[i];
        int idx = key.x() + res.x() * (key.y() + res.y() * key.z());
        if (sqD(i) <= eps_sq) {
            S(idx) = 0.0f;
        }
    }

    // BFS 泛洪寻找外部
    std::queue<Eigen::RowVector3i> q;
    q.push(Eigen::RowVector3i(0, 0, 0));
    S(0) = 1.0f; // 外部

    int dx[] = { 1, -1, 0, 0, 0, 0 };
    int dy[] = { 0, 0, 1, -1, 0, 0 };
    int dz[] = { 0, 0, 0, 0, 1, -1 };

    while (!q.empty()) {
        Eigen::RowVector3i curr = q.front();
        q.pop();

        for (int d = 0; d < 6; ++d) {
            Eigen::RowVector3i nbr = curr + Eigen::RowVector3i(dx[d], dy[d], dz[d]);
            if (nbr.x() >= 0 && nbr.x() < res.x() &&
                nbr.y() >= 0 && nbr.y() < res.y() &&
                nbr.z() >= 0 && nbr.z() < res.z()) {
                int nbr_idx = nbr.x() + res.x() * (nbr.y() + res.y() * nbr.z());
                if (std::isnan(S(nbr_idx))) {
                    S(nbr_idx) = 1.0f;
                    q.push(nbr);
                }
            }
        }
    }

    // 绝对二值化赋值
    for (int i = 0; i < map_keys.size(); ++i) {
        GridNode& node = sparse_nodes[map_keys[i]];
        Eigen::RowVector3i key = map_keys[i];
        int idx = key.x() + res.x() * (key.y() + res.y() * key.z());

        if (S(idx) == 1.0f) {
            node.sdf = 1.0; // 外部为正
        }
        else {
            node.sdf = -1.0; 
        }
    }
}
void SparseSDFGrid::compute_sdf_values_windingnumber(const Eigen::MatrixXd& V, Eigen::MatrixXi& F, double threshold) {
    if (sparse_nodes.empty()) return;

    // winding number计算依赖于mesh是已定向的, 进行重定向操作
    Eigen::VectorXi C;
    igl::embree::reorient_facets_raycast(V, F, F, C);
    std::cout << "[reorient]   over!" << std::endl;

    // 准备采样点坐标
    int num_nodes = sparse_nodes.size();
    Eigen::MatrixXd P(num_nodes, 3);
    std::vector<Eigen::Vector3i> map_keys;
    map_keys.reserve(num_nodes);

    int row = 0;
    for (const auto& kv : sparse_nodes) {
        Eigen::Vector3i k_col = kv.first.transpose();
        map_keys.push_back(k_col);
        P.row(row) = kv.second.position;
        row++;
    }

    Eigen::VectorXd W;
    igl::fast_winding_number(V, F, P, W);
    // W >= 0.5 认为在内部 (-)，否则在外部 (+)
    for (int i = 0; i < num_nodes; ++i) {
        Eigen::RowVector3i row_key = map_keys[i].transpose();
        GridNode& node = sparse_nodes[row_key];
        if (W(i) >= threshold) {
            node.sdf = -1.0; // 内部
        }
        else {
            node.sdf = 1.0;  // 外部
        }
    }
}




void SparseSDFGrid::bilateral_mesh_denoise(Eigen::MatrixXd& V, Eigen::MatrixXi& F,
    double SigmaNormal,
    int NormalIterNum,
    int VertexIterNum)
{
    int num_faces = F.rows();
    int num_verts = V.rows();

    // 预计算点面邻接关系 (Vertex-Face Adjacency)
    std::vector<std::vector<int>> VF, VFi;
    igl::vertex_triangle_adjacency(num_verts, F, VF, VFi);

    Eigen::VectorXd Area;
    igl::doublearea(V, F, Area);
    Area *= 0.5;

    Eigen::MatrixXd FaceCenter;
    igl::barycenter(V, F, FaceCenter);

    // ==========================================================
    // 质量优化：构建 1-Ring 面邻域
    // ==========================================================
    std::vector<std::vector<int>> FF_1ring(num_faces);
    double SigmaCenter = 0.0;
    int edge_count = 0;

    for (int i = 0; i < num_faces; ++i) {
        std::unordered_set<int> neighbors;
        // 遍历当前面的 3 个顶点
        for (int j = 0; j < 3; ++j) {
            int v_idx = F(i, j);
            // 将包含这些顶点的所有面加入邻域
            for (int f_idx : VF[v_idx]) {
                if (f_idx != i) {
                    neighbors.insert(f_idx);
                }
            }
        }
        FF_1ring[i].assign(neighbors.begin(), neighbors.end());

        // 计算 SigmaCenter 基础值
        for (int nei_f : FF_1ring[i]) {
            SigmaCenter += (FaceCenter.row(i) - FaceCenter.row(nei_f)).norm();
            edge_count++;
        }
    }
    SigmaCenter /= (double)edge_count;


    std::vector<std::vector<double>> Precomputed_Ws_Area(num_faces);
    for (int i = 0; i < num_faces; ++i) {
        Precomputed_Ws_Area[i].resize(FF_1ring[i].size());
        for (size_t k = 0; k < FF_1ring[i].size(); ++k) {
            int nei_f = FF_1ring[i][k];
            double delta_center = (FaceCenter.row(i) - FaceCenter.row(nei_f)).norm();
            Precomputed_Ws_Area[i][k] = Area(nei_f) * std::exp(-(delta_center * delta_center) / (2 * SigmaCenter * SigmaCenter));
        }
    }

    Eigen::MatrixXd NewNormal;
    igl::per_face_normals(V, F, NewNormal);


    for (int iter = 0; iter < NormalIterNum; ++iter) {
        Eigen::MatrixXd TempNormal = NewNormal;

        // 多线程并发过滤法线
#pragma omp parallel for num_threads(12)
        for (int i = 0; i < num_faces; ++i) {
            Eigen::RowVector3d N_sum = Eigen::RowVector3d::Zero();
            double Kp = 0.0;

            // 遍历 1-Ring 邻域
            for (size_t k = 0; k < FF_1ring[i].size(); ++k) {
                int nei_f = FF_1ring[i][k];
                double delta_normal = (TempNormal.row(i) - TempNormal.row(nei_f)).norm();

                // 只有法向权重需要实时计算
                double Wr = std::exp(-(delta_normal * delta_normal) / (2 * SigmaNormal * SigmaNormal));

                // 直接读取预计算好的空间权重
                double weight = Precomputed_Ws_Area[i][k] * Wr;
                N_sum += weight * TempNormal.row(nei_f);
                Kp += weight;
            }
            if (Kp > 1e-8) {
                NewNormal.row(i) = (N_sum / Kp).normalized();
            }
            else {
                NewNormal.row(i) = TempNormal.row(i);
            }
        }
    }

    // ==========================================================
    // 根据新法线更新顶点位置
    // ==========================================================
    for (int iter = 0; iter < VertexIterNum; ++iter) {
        Eigen::MatrixXd V_new = V;

        // 顶点变了，面中心必须重新算
        igl::barycenter(V, F, FaceCenter);

        // 多线程并发更新顶点
#pragma omp parallel for num_threads(12)
        for (int i = 0; i < num_verts; ++i) {
            Eigen::RowVector3d delta_xi = Eigen::RowVector3d::Zero();
            int num_adj_faces = VF[i].size();

            if (num_adj_faces > 0) {
                for (int f_idx : VF[i]) {
                    Eigen::RowVector3d cj = FaceCenter.row(f_idx);
                    Eigen::RowVector3d nj = NewNormal.row(f_idx);

                    double dist = (cj - V.row(i)).dot(nj);
                    delta_xi += nj * dist;
                }
                V_new.row(i) = V.row(i) + delta_xi / (double)num_adj_faces;
            }
        }
        V = V_new;
    }
}



// 记录提取的 Mesh 顶点是由哪两个体素角点插值而来的
struct VertexGridBinding {
    int idA = -1;
    int idB = -1;
    bool valid = false;
};

void SparseSDFGrid::optimize_grid(const Eigen::MatrixXd& V_target, const Eigen::MatrixXi& F_target, int iters, double lr) {
    // 提取固定拓扑的初始二值化SDF的网格
    Eigen::MatrixXd V_ext;
    Eigen::MatrixXi F_ext;
    extract_mesh_mc(V_ext, F_ext, 0.0);
    // ==========================================================
    // 内存展平 (Flatten Hashmap)
    // ==========================================================
    int num_nodes = sparse_nodes.size();
    std::unordered_map<Eigen::RowVector3i, int, Vector3iHash> key_to_id;
    std::vector<Eigen::RowVector3i> id_to_key(num_nodes);
    std::vector<Eigen::RowVector3d*> node_pos_ptrs(num_nodes);

    int current_id = 0;
    for (auto& kv : sparse_nodes) {
        key_to_id[kv.first] = current_id;
        id_to_key[current_id] = kv.first;
        node_pos_ptrs[current_id] = &(kv.second.position);
        current_id++;
    }

    std::vector<VertexGridBinding> bindings(V_ext.rows());
    
#pragma omp parallel for num_threads(12)
    for (int i = 0; i < V_ext.rows(); ++i) {
        Eigen::RowVector3d v = V_ext.row(i);
        Eigen::RowVector3d float_idx = (v - grid_origin) / cell_size;

        Eigen::RowVector3i nodeA, nodeB;
        for (int d = 0; d < 3; ++d) {
            double frac = float_idx(d) - std::floor(float_idx(d));
            if (std::abs(frac - 0.5) < 0.1) {
                nodeA(d) = std::floor(float_idx(d));
                nodeB(d) = std::ceil(float_idx(d));
            }
            else {
                nodeA(d) = std::round(float_idx(d));
                nodeB(d) = std::round(float_idx(d));
            }
        }

        if (key_to_id.count(nodeA) && key_to_id.count(nodeB)) {
            bindings[i].idA = key_to_id[nodeA];
            bindings[i].idB = key_to_id[nodeB];
            bindings[i].valid = true;
        }
    }

    std::vector<std::vector<int>> A;
    igl::adjacency_list(F_ext, A);
    igl::AABB<Eigen::MatrixXd, 3> tree_target;
    tree_target.init(V_target, F_target);

    double lambda_laplacian = 0.6;
    int num_ext_verts = V_ext.rows();

    for (int iter = 0; iter < iters; ++iter) {
        Eigen::MatrixXd V_current = V_ext;

#pragma omp parallel for num_threads(12)
        for (int i = 0; i < num_ext_verts; ++i) {
            if (bindings[i].valid) {
                Eigen::RowVector3d pA = *(node_pos_ptrs[bindings[i].idA]);
                Eigen::RowVector3d pB = *(node_pos_ptrs[bindings[i].idB]);
                V_current.row(i) = 0.5 * (pA + pB);
            }
        }

        Eigen::VectorXd sqD;
        Eigen::VectorXi I;
        Eigen::MatrixXd C_nearest;
        tree_target.squared_distance(V_target, F_target, V_current, sqD, I, C_nearest);

        std::vector<Eigen::RowVector3d> grad_accum(num_nodes, Eigen::RowVector3d::Zero());
        std::vector<int> grad_count(num_nodes, 0);

        for (int i = 0; i < num_ext_verts; ++i) {
            if (!bindings[i].valid) continue;

            //// 1. 单纯的方向投影，但是单层结构会产生紧贴的double mesh
            //Eigen::RowVector3d dir_udf = C_nearest.row(i) - V_current.row(i);
            
            
            // 处理带有明显*单层开放面*的结果时可以使用，进行双层化处理
            Eigen::RowVector3d diff = C_nearest.row(i) - V_current.row(i);
            double dist = diff.norm();

            // 设定一个微小的目标厚度 (比如 cell_size 的 10% 到 20%)
            // 对于双层网格，最终总厚度将是 2 * margin
            double margin = 0.1 * cell_size;

            Eigen::RowVector3d dir_udf = Eigen::RowVector3d::Zero();
            if (dist > 1e-8) {
                // 如果 dist < margin，算出来是负的，向外推开,防止重合
                dir_udf = diff * ((dist - margin) / dist);
            }

            Eigen::RowVector3d dir_laplacian = Eigen::RowVector3d::Zero();
            if (!A[i].empty()) {
                Eigen::RowVector3d centroid = Eigen::RowVector3d::Zero();
                for (int neighbor_idx : A[i]) {
                    centroid += V_current.row(neighbor_idx);
                }
                centroid /= double(A[i].size());
                dir_laplacian = centroid - V_current.row(i);
            }

            Eigen::RowVector3d total_direction = dir_udf + lambda_laplacian * dir_laplacian;

            int idA = bindings[i].idA;
            int idB = bindings[i].idB;

            grad_accum[idA] += total_direction;
            grad_count[idA] += 1;
            grad_accum[idB] += total_direction;
            grad_count[idB] += 1;
        }

#pragma omp parallel for num_threads(12)
        for (int k = 0; k < num_nodes; ++k) {
            if (grad_count[k] > 0) {
                Eigen::RowVector3d raw_grad = grad_accum[k] / double(grad_count[k]);
                // 直接在 3D 空间中移动体素角点
                *(node_pos_ptrs[k]) += lr * raw_grad;
            }
        }

        std::cout << "Iter " << iter << " | Mean UDF + Laplacian Loss: " << std::fixed << std::setprecision(8) << std::sqrt(sqD.mean()) << std::endl;
    }
}


void SparseSDFGrid::extract_mesh(Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out) {
    // =========================================================
    // 扁平化数据 (Map -> Matrix) & 构建边 (GI)
    // =========================================================
    int num_nodes = sparse_nodes.size();
    Eigen::VectorXd Gf(num_nodes);
    Eigen::MatrixXd GV(num_nodes, 3);

    // 辅助映射：Grid Index -> Linear Index (0..N-1)
    std::unordered_map<Eigen::RowVector3i, int, Vector3iHash> grid_to_linear;

    int idx = 0;
    for (const auto& kv : sparse_nodes) {
        grid_to_linear[kv.first] = idx;
        Gf(idx) = kv.second.sdf;
        GV.row(idx) = kv.second.position;
        idx++;
    }

    // 构建稀疏网格的边 (GI) - 仅连接存在的节点
    std::vector<Eigen::RowVector2i> edges;
    edges.reserve(num_nodes * 3);

    for (const auto& kv : sparse_nodes) {
        int u = grid_to_linear[kv.first];
        Eigen::RowVector3i k = kv.first;

        // 检查 +X, +Y, +Z 邻居
        Eigen::RowVector3i nbrs[3] = {
            {k(0) + 1, k(1), k(2)},
            {k(0), k(1) + 1, k(2)},
            {k(0), k(1), k(2) + 1}
        };

        for (int i = 0; i < 3; ++i) {
            auto it = grid_to_linear.find(nbrs[i]);
            if (it != grid_to_linear.end()) {
                edges.push_back(Eigen::RowVector2i(u, it->second));
            }
        }
    }

    Eigen::MatrixXi GI(edges.size(), 2);
    for (size_t i = 0; i < edges.size(); ++i) GI.row(i) = edges[i];

    // =========================================================
    // 定义插值函数 (不再查 Mesh，而是查 Grid)
    // =========================================================

    // 辅助 lambda: 三线性插值
    // p: 世界坐标
    // return_sdf: true 返回 sdf, false 返回 normal 的某个分量(需外部组合)
// 辅助 lambda: 三线性插值
    auto interpolate_grid = [&](const Eigen::Matrix<double, 1, 3>& p) -> std::pair<double, Eigen::RowVector3d> {
        // [修复点 1]: 不要使用 Eigen::Array3d (它是3x1)，直接用 RowVector3d (1x3)
        // p 是 1x3, grid_origin 是 1x3, 结果保持 1x3
        Eigen::RowVector3d local_pos = (p - grid_origin) / cell_size;

        //找到左下角索引 (base index)
        Eigen::RowVector3i base_idx;
        base_idx(0) = std::floor(local_pos(0));
        base_idx(1) = std::floor(local_pos(1));
        base_idx(2) = std::floor(local_pos(2));

        // 修复: 现在 local_pos 和 base_idx 都是 1x3，可以直接相减
        Eigen::RowVector3d t = local_pos - base_idx.cast<double>();

        //获取 8 个邻居的值
        double v[8];
        Eigen::RowVector3d n[8];

        for (int z = 0; z < 2; ++z) {
            for (int y = 0; y < 2; ++y) {
                for (int x = 0; x < 2; ++x) {
                    int index = x + 2 * y + 4 * z;
                    Eigen::RowVector3i neighbor_key = base_idx + Eigen::RowVector3i(x, y, z);

                    auto it = sparse_nodes.find(neighbor_key);
                    if (it != sparse_nodes.end()) {
                        v[index] = it->second.sdf;
                        n[index] = it->second.normal;
                    }
                    else {
                        v[index] = 10.0;
                        n[index] = Eigen::RowVector3d::Zero();
                    }
                }
            }
        }

        // 三线性插值 SDF (标量插值)
        double c00 = v[0] * (1.0 - t(0)) + v[1] * t(0);
        double c10 = v[2] * (1.0 - t(0)) + v[3] * t(0);
        double c01 = v[4] * (1.0 - t(0)) + v[5] * t(0);
        double c11 = v[6] * (1.0 - t(0)) + v[7] * t(0);
        
        double c0 = c00 * (1.0 - t(1)) + c10 * t(1);
        double c1 = c01 * (1.0 - t(1)) + c11 * t(1);
        
        double val = c0 * (1.0 - t(2)) + c1 * t(2);

        // 三线性插值 Normal (向量插值)
        Eigen::RowVector3d n00 = n[0] * (1.0 - t(0)) + n[1] * t(0);
        Eigen::RowVector3d n10 = n[2] * (1.0 - t(0)) + n[3] * t(0);
        Eigen::RowVector3d n01 = n[4] * (1.0 - t(0)) + n[5] * t(0);
        Eigen::RowVector3d n11 = n[6] * (1.0 - t(0)) + n[7] * t(0);

        Eigen::RowVector3d n_y0 = n00 * (1.0 - t(1)) + n10 * t(1);
        Eigen::RowVector3d n_y1 = n01 * (1.0 - t(1)) + n11 * t(1);

        Eigen::RowVector3d norm = n_y0 * (1.0 - t(2)) + n_y1 * t(2);

        return { val, norm };
        };


    auto f_func = [&](const Eigen::Matrix<double, 1, 3>& p) -> double {
        return interpolate_grid(p).first;
        };


    auto f_grad_func = [&](const Eigen::Matrix<double, 1, 3>& p) -> Eigen::Matrix<double, 1, 3> {
        Eigen::RowVector3d grad = interpolate_grid(p).second;
        if (grad.squaredNorm() > 1e-10) {
            grad.normalize();
        }
        return grad;
        };

    Eigen::RowVector3d step(cell_size, cell_size, cell_size);

    igl::dual_contouring(
        f_func,
        f_grad_func,
        step,
        Gf,
        GV,
        GI,
        true,  // constrained: 防止顶点飞出 Grid
        true,  // triangles
        true,  // root_finding: 使用插值函数寻找精确零点
        V_out,
        F_out
    );
}

void SparseSDFGrid::extract_mesh_mc(Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out, double isovalue)
{
    // =========================================================
    // 扁平化数据 (Map -> Linear Arrays S & GV)
    // =========================================================
    int num_nodes = sparse_nodes.size();

    // S: SDF 值, GV: 顶点坐标
    Eigen::VectorXd S(num_nodes);
    Eigen::MatrixXd GV(num_nodes, 3);

    // 辅助映射：Grid Index (i,j,k) -> Linear Index (0..N-1)
    // 用于构建 GI 矩阵时查找索引
    std::unordered_map<Eigen::RowVector3i, int, Vector3iHash> grid_to_linear;
    grid_to_linear.reserve(num_nodes);

    int idx = 0;
    for (const auto& kv : sparse_nodes) {
        grid_to_linear[kv.first] = idx;
        S(idx) = kv.second.sdf;
        GV.row(idx) = kv.second.position;
        idx++;
    }

    // 定义 8 个角点的相对偏移量
    const int offset_x[8] = { 0, 1, 1, 0, 0, 1, 1, 0 };
    const int offset_y[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
    const int offset_z[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };

    std::vector<Eigen::Matrix<int, 1, 8>> cells;
    cells.reserve(active_cells.size());

    // 遍历所有活跃的 Cell
    // active_cells 是我们在 build 阶段收集的
    for (const auto& base_idx : active_cells) {
        Eigen::Matrix<int, 1, 8> cell_indices;
        bool is_valid_cell = true;

        // 检查该 Cell 的 8 个角点是否都存在于 sparse_nodes 中
        for (int k = 0; k < 8; ++k) {
            Eigen::RowVector3i corner_key = base_idx + Eigen::RowVector3i(offset_x[k], offset_y[k], offset_z[k]);

            auto it = grid_to_linear.find(corner_key);
            if (it != grid_to_linear.end()) {
                cell_indices(k) = it->second;
            }
            else {
                // 如果某个角点缺失（可能在 padding 之外），则放弃该 Cell
                // 因为 MC 需要完整的立方体数据
                is_valid_cell = false;
                break;
            }
        }

        if (is_valid_cell) {
            cells.push_back(cell_indices);
        }
    }

    // 转换为 Eigen 矩阵
    Eigen::MatrixXi GI(cells.size(), 8);
    for (size_t i = 0; i < cells.size(); ++i) {
        GI.row(i) = cells[i];
    }

    std::cout << "MC Input: " << S.rows() << " vertices, " << GI.rows() << " cells." << "sdf max val: " << S.maxCoeff() << "sdf min val: " << S.minCoeff() << std::endl;

    // =========================================================
    // 调用 Sparse Marching Cubes
    // =========================================================
    if (GI.rows() > 0) {
        igl::marching_cubes(S, GV, GI, isovalue, V_out, F_out);
        F_out.col(0).swap(F_out.col(2));
    }
    else {
        std::cerr << "Warning: No valid cells found for Marching Cubes!" << std::endl;
    }
    std::cout << "MC extract over" << std::endl;
}

// Marching Tetrahedra 提取网格
void SparseSDFGrid::extract_mesh_tets(Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out, double isovalue)
{
    // =========================================================
    // 1. 扁平化数据 (Map -> TV & S)
    // =========================================================
    int num_nodes = sparse_nodes.size();

    // TV: 四面体网格的顶点 (Tet Vertices) -> 即 Grid Corner
    // S:  顶点的标量值 (Scalars) -> 即 SDF
    Eigen::MatrixXd TV(num_nodes, 3);
    Eigen::VectorXd S(num_nodes);

    // 辅助映射：Grid Index (i,j,k) -> Linear Index
    std::unordered_map<Eigen::RowVector3i, int, Vector3iHash> grid_to_linear;
    grid_to_linear.reserve(num_nodes);

    int idx = 0;
    for (const auto& kv : sparse_nodes) {
        grid_to_linear[kv.first] = idx;
        TV.row(idx) = kv.second.position;
        S(idx) = kv.second.sdf;
        idx++;
    }

    // =========================================================
    // 2. 将体素剖分为四面体 (Voxel -> Tets)
    // =========================================================

    // 我们采用标准的将一个立方体剖分成 6 个四面体的方法 (Kuhn triangulation)
    // 这种方法不增加额外的中心点，且保证相邻立方体的对角线一致（如果索引顺序一致）。

    // 定义 Grid Corner 的相对偏移 (与 MC 一致)
    // 0:(0,0,0), 1:(1,0,0), 2:(1,1,0), 3:(0,1,0) (Bottom Z=0)
    // 4:(0,0,1), 5:(1,0,1), 6:(1,1,1), 7:(0,1,1) (Top Z=1)
    const int offset_x[8] = { 0, 1, 1, 0, 0, 1, 1, 0 };
    const int offset_y[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
    const int offset_z[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };

    // 6个四面体的顶点索引组合 (基于 0-7)
    // 这里的顺序是经过精心设计的，以共享 0-6 对角线
    const int tets_lut[6][4] = {
        {0, 1, 2, 6},
        {0, 2, 3, 6},
        {0, 1, 5, 6},
        {0, 4, 5, 6},
        {0, 3, 7, 6},
        {0, 4, 7, 6}
    };

    std::vector<Eigen::RowVector4i> all_tets;
    // 预估大小：每个 active cell 生成 6 个 tet
    all_tets.reserve(active_cells.size() * 6);

    // 遍历所有 Active Cells
    for (const auto& base_idx : active_cells) {
        int corner_indices[8];
        bool is_valid_cell = true;

        // 1. 获取当前 Cell 的 8 个角点的全局线性索引
        for (int k = 0; k < 8; ++k) {
            Eigen::RowVector3i corner_key = base_idx + Eigen::RowVector3i(offset_x[k], offset_y[k], offset_z[k]);

            auto it = grid_to_linear.find(corner_key);
            if (it != grid_to_linear.end()) {
                corner_indices[k] = it->second;
            }
            else {
                // 如果某个角点缺失（在 Grid 边界外），无法构成完整 Cell
                is_valid_cell = false;
                break;
            }
        }

        // 2. 如果 Cell 完整，将其切分为 6 个四面体并加入列表
        if (is_valid_cell) {
            for (int t = 0; t < 6; ++t) {
                Eigen::RowVector4i tet;
                tet(0) = corner_indices[tets_lut[t][0]];
                tet(1) = corner_indices[tets_lut[t][1]];
                tet(2) = corner_indices[tets_lut[t][2]];
                tet(3) = corner_indices[tets_lut[t][3]];
                all_tets.push_back(tet);
            }
        }
    }

    // 构建 TT (Tetrahedra Indices) 矩阵
    Eigen::MatrixXi TT(all_tets.size(), 4);
    for (size_t i = 0; i < all_tets.size(); ++i) {
        TT.row(i) = all_tets[i];
    }

    std::cout << "MT Input: " << TV.rows() << " vertices, " << TT.rows() << " tets." << std::endl;

    // =========================================================
    // 3. 调用 Marching Tets
    // =========================================================
    if (TT.rows() > 0) {
        // 使用最简单的重载版本，只需要 V, F 输出
        igl::marching_tets(TV, TT, S, isovalue, V_out, F_out);
    }
    else {
        std::cerr << "Warning: No valid tets found for Marching Tets!" << std::endl;
    }
}



//// 导出稀疏网格的线框 (Wireframe) 到 OBJ 文件
//    // 可视化每一个 active cell 为一个立方体框
//void SparseSDFGrid::export_grid_wireframe(const std::string& filename) 
//{
//    std::ofstream out(filename);
//    if (!out.is_open()) {
//        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
//        return;
//    }
//
//    std::cout << "Exporting grid wireframe to " << filename << "..." << std::endl;
//
//    // OBJ 索引从 1 开始
//    int v_offset = 1;
//
//    // 遍历所有活跃的 Cell
//    for (const auto& idx : active_cells) {
//        // 计算当前 Cell 最小角（左下后）的世界坐标
//        Eigen::RowVector3d base_pos = grid_origin + idx.cast<double>() * cell_size;
//        double s = cell_size;
//
//        // 1. 写入当前 Cell 的 8 个顶点
//        // 为了简单起见，我们不共享顶点（文件会稍大，但逻辑最简单，且 debug 够用了）
//        // 顺序：底面 0-3，顶面 4-7
//
//        // Bottom face (z)
//        out << "v " << base_pos(0) << " " << base_pos(1) << " " << base_pos(2) << "\n"; // 0
//        out << "v " << base_pos(0) + s << " " << base_pos(1) << " " << base_pos(2) << "\n"; // 1
//        out << "v " << base_pos(0) + s << " " << base_pos(1) + s << " " << base_pos(2) << "\n"; // 2
//        out << "v " << base_pos(0) << " " << base_pos(1) + s << " " << base_pos(2) << "\n"; // 3
//
//        // Top face (z + s)
//        out << "v " << base_pos(0) << " " << base_pos(1) << " " << base_pos(2) + s << "\n"; // 4
//        out << "v " << base_pos(0) + s << " " << base_pos(1) << " " << base_pos(2) + s << "\n"; // 5
//        out << "v " << base_pos(0) + s << " " << base_pos(1) + s << " " << base_pos(2) + s << "\n"; // 6
//        out << "v " << base_pos(0) << " " << base_pos(1) + s << " " << base_pos(2) + s << "\n"; // 7
//
//        // 2. 写入 12 条线段 (Lines)
//        // 格式: l index1 index2
//
//        // 底面框
//        out << "l " << v_offset + 0 << " " << v_offset + 1 << "\n";
//        out << "l " << v_offset + 1 << " " << v_offset + 2 << "\n";
//        out << "l " << v_offset + 2 << " " << v_offset + 3 << "\n";
//        out << "l " << v_offset + 3 << " " << v_offset + 0 << "\n";
//
//        // 顶面框
//        out << "l " << v_offset + 4 << " " << v_offset + 5 << "\n";
//        out << "l " << v_offset + 5 << " " << v_offset + 6 << "\n";
//        out << "l " << v_offset + 6 << " " << v_offset + 7 << "\n";
//        out << "l " << v_offset + 7 << " " << v_offset + 4 << "\n";
//
//        // 连接底面和顶面的柱子
//        out << "l " << v_offset + 0 << " " << v_offset + 4 << "\n";
//        out << "l " << v_offset + 1 << " " << v_offset + 5 << "\n";
//        out << "l " << v_offset + 2 << " " << v_offset + 6 << "\n";
//        out << "l " << v_offset + 3 << " " << v_offset + 7 << "\n";
//
//        // 更新索引偏移，准备下一个 Cell
//        v_offset += 8;
//    }
//
//    out.close();
//    std::cout << "Done. Exported " << active_cells.size() << " cells." << std::endl;
//}
void SparseSDFGrid::export_grid_wireframe(const std::string& filename)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    std::cout << "Exporting grid wireframe (deduplicated) to " << filename << "..." << std::endl;

    std::unordered_map<Eigen::RowVector3i, int, Vector3iHash> node_to_vid;
    int next_vid = 1;

    auto emit_vertex = [&](const Eigen::RowVector3i& node_idx) -> int {
        // 1. 检查这个点是否已经输出过（去重，用于 OBJ 文件压缩体积）
        auto it = node_to_vid.find(node_idx);
        if (it != node_to_vid.end()) {
            return it->second;
        }

        // 2. [关键修改] 从 sparse_nodes 中查找真实的坐标！
        // 而不是用公式重新计算
        Eigen::RowVector3d pos;

        auto it_node = sparse_nodes.find(node_idx);
        if (it_node != sparse_nodes.end()) {
            // 找到了！使用存储在里面的坐标（可能是变形后的）
            pos = it_node->second.position;
        }
        else {
            // 理论上 Active Cell 的角点都应该在 sparse_nodes 里
            // 但为了防止 crash，如果没找到，做一个 fallback
            pos = grid_origin + node_idx.cast<double>() * cell_size;
        }

        out << "v " << pos(0) << " " << pos(1) << " " << pos(2) << "\n";

        // 记录 ID，下次遇到同个 index 直接返回
        node_to_vid[node_idx] = next_vid;
        return next_vid++;
     };

    for (const auto& cell : active_cells) {

        Eigen::RowVector3i v[8] = {
            cell + Eigen::RowVector3i(0,0,0),
            cell + Eigen::RowVector3i(1,0,0),
            cell + Eigen::RowVector3i(1,1,0),
            cell + Eigen::RowVector3i(0,1,0),
            cell + Eigen::RowVector3i(0,0,1),
            cell + Eigen::RowVector3i(1,0,1),
            cell + Eigen::RowVector3i(1,1,1),
            cell + Eigen::RowVector3i(0,1,1)
        };

        int vid[8];
        for (int i = 0; i < 8; ++i) {
            vid[i] = emit_vertex(v[i]);
        }

        // bottom
        out << "l " << vid[0] << " " << vid[1] << "\n";
        out << "l " << vid[1] << " " << vid[2] << "\n";
        out << "l " << vid[2] << " " << vid[3] << "\n";
        out << "l " << vid[3] << " " << vid[0] << "\n";

        // top
        out << "l " << vid[4] << " " << vid[5] << "\n";
        out << "l " << vid[5] << " " << vid[6] << "\n";
        out << "l " << vid[6] << " " << vid[7] << "\n";
        out << "l " << vid[7] << " " << vid[4] << "\n";

        // vertical
        out << "l " << vid[0] << " " << vid[4] << "\n";
        out << "l " << vid[1] << " " << vid[5] << "\n";
        out << "l " << vid[2] << " " << vid[6] << "\n";
        out << "l " << vid[3] << " " << vid[7] << "\n";
    }

    out.close();
    std::cout << "Done. Unique nodes: " << node_to_vid.size()
        << ", cells: " << active_cells.size() << std::endl;
}