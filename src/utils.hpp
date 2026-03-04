//#include <unordered_set>
//#include <iostream>
//#include <Eigen/Core>
//#include <igl/edge_topology.h>
//#include <igl/boundary_facets.h>
//#include <igl/is_edge_manifold.h>
//#include <igl/is_vertex_manifold.h>
//#include <igl/copyleft/cgal/SelfIntersectMesh.h>
//
//int CountSelfIntersections(
//    const Eigen::MatrixXd& V,
//    const Eigen::MatrixXi& F)
//{
//    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
//
//    // libigl 参数
//    igl::copyleft::cgal::RemeshSelfIntersectionsParam params;
//    params.detect_only = true;  // 关键：只检测，不remesh
//
//    // 这些输出我们不关心，但构造器需要
//    Eigen::MatrixXd VV;
//    Eigen::MatrixXi FF;
//    Eigen::MatrixXi IF;
//    Eigen::VectorXi J;
//    Eigen::VectorXi IM;
//
//    // 构造 SelfIntersectMesh 实例
//    igl::copyleft::cgal::SelfIntersectMesh<
//        Kernel,
//        Eigen::MatrixXd,
//        Eigen::MatrixXi,
//        Eigen::MatrixXd,
//        Eigen::MatrixXi,
//        Eigen::MatrixXi,
//        Eigen::VectorXi,
//        Eigen::VectorXi
//    > SIM(V, F, params, VV, FF, IF, J, IM);
//
//    // 关键：自交三角形对数量
//    return static_cast<int>(SIM.count);
//}
//
//struct MeshCheckResult
//{
//    int num_boundary_edges = 0;
//    int num_boundary_faces = 0;
//    int num_nonmanifold_edges = 0;
//    int num_nonmanifold_vertices = 0;
//    int num_self_intersections = 0;
//};
//
//MeshCheckResult CheckMesh(
//    const Eigen::MatrixXd& V,
//    const Eigen::MatrixXi& F)
//{
//    MeshCheckResult result;
//
//    // =========================================================
//    // 1️⃣ Edge 拓扑
//    // =========================================================
//    Eigen::MatrixXi EV, FE, EF;
//    igl::edge_topology(V, F, EV, FE, EF);
//
//    // EF(e,0), EF(e,1) 为该 edge 相邻的两个面
//    // 如果有 -1，说明是边界 edge
//    for (int e = 0; e < EF.rows(); ++e)
//    {
//        int f0 = EF(e, 0);
//        int f1 = EF(e, 1);
//
//        if (f0 == -1 || f1 == -1)
//        {
//            result.num_boundary_edges++;
//        }
//    }
//
//    // =========================================================
//    // 2️⃣ 开放面统计
//    // =========================================================
//    // 如果一个面至少有一条 boundary edge，则视为开放面
//    std::vector<bool> is_boundary_face(F.rows(), false);
//
//    for (int e = 0; e < EF.rows(); ++e)
//    {
//        int f0 = EF(e, 0);
//        int f1 = EF(e, 1);
//
//        if (f0 == -1 || f1 == -1)
//        {
//            if (f0 != -1) is_boundary_face[f0] = true;
//            if (f1 != -1) is_boundary_face[f1] = true;
//        }
//    }
//
//    for (int i = 0; i < F.rows(); ++i)
//    {
//        if (is_boundary_face[i])
//            result.num_boundary_faces++;
//    }
//
//    // =========================================================
//    // 3️⃣ 非流形边统计
//    // =========================================================
//    // 非流形边 = 相邻面数 > 2
//    // EF 只存两个，需要通过统计 FE 来判断
//    // 非流形边
//    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> BF;
//    Eigen::Array<bool, Eigen::Dynamic, 1> BE;
//    Eigen::MatrixXi E;
//    Eigen::VectorXi EMAP;
//
//    igl::is_edge_manifold(F, BF, E, EMAP, BE);
//
//    for (int i = 0; i < BE.rows(); ++i)
//    {
//        if (!BE(i))
//            result.num_nonmanifold_edges++;
//    }
//
//    // 非流形点
//    Eigen::VectorXi vertex_manifold;
//    igl::is_vertex_manifold(F, vertex_manifold);
//    for (int i = 0; i < vertex_manifold.size(); ++i)
//        if (!vertex_manifold[i])
//            result.num_nonmanifold_vertices++;
//
//
//    result.num_self_intersections = CountSelfIntersections(V, F);
//
//    
//    std::cout << "Boundary edges: " << result.num_boundary_edges << std::endl;
//    std::cout << "Boundary faces: " << result.num_boundary_faces << std::endl;
//    std::cout << "Nonmanifold edges: " << result.num_nonmanifold_edges << std::endl;
//    std::cout << "Nonmanifold vertices: " << result.num_nonmanifold_vertices << std::endl;
//    std::cout << "Self-intersections: " << result.num_self_intersections << std::endl;
//
//    return result;
//}


/*采取更鲁棒的自写checkMesh函数*/
#include <unordered_set>
#include <iostream>

#include <Eigen/Core>
#include <igl/edge_topology.h>
#include <igl/boundary_facets.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/copyleft/cgal/SelfIntersectMesh.h>

int CountSelfIntersections(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F)
{
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

    // libigl 参数
    igl::copyleft::cgal::RemeshSelfIntersectionsParam params;
    params.detect_only = true;  // 关键：只检测，不remesh

    // 这些输出我们不关心，但构造器需要
    Eigen::MatrixXd VV;
    Eigen::MatrixXi FF;
    Eigen::MatrixXi IF;
    Eigen::VectorXi J;
    Eigen::VectorXi IM;

    // 构造 SelfIntersectMesh 实例
    igl::copyleft::cgal::SelfIntersectMesh<
        Kernel,
        Eigen::MatrixXd,
        Eigen::MatrixXi,
        Eigen::MatrixXd,
        Eigen::MatrixXi,
        Eigen::MatrixXi,
        Eigen::VectorXi,
        Eigen::VectorXi
    > SIM(V, F, params, VV, FF, IF, J, IM);

    // 关键：自交三角形对数量
    return static_cast<int>(SIM.count);
}

struct MeshCheckResult
{
    int num_boundary_edges = 0;
    int num_boundary_faces = 0;
    int num_nonmanifold_edges = 0;
    int num_nonmanifold_vertices = 0;
    int num_self_intersections = 0;
};


struct EdgeKey
{
    int v0, v1;

    EdgeKey(int a, int b)
    {
        if (a < b) { v0 = a; v1 = b; }
        else { v0 = b; v1 = a; }
    }

    bool operator==(const EdgeKey& other) const
    {
        return v0 == other.v0 && v1 == other.v1;
    }
};

struct EdgeKeyHash
{
    std::size_t operator()(const EdgeKey& e) const
    {
        return std::hash<long long>()(
            (static_cast<long long>(e.v0) << 32) | e.v1
            );
    }
};

MeshCheckResult CheckMesh(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F)
{
    MeshCheckResult result;

    if (F.cols() != 3)
    {
        std::cerr << "Mesh is not triangular.\n";
        return result;
    }

    // =====================================================
    // 1️⃣ 构建 非流形安全 Edge -> Faces 邻接
    // =====================================================

    std::unordered_map<EdgeKey, std::vector<int>, EdgeKeyHash> edgeFaces;

    edgeFaces.reserve(F.rows() * 3);

    for (int f = 0; f < F.rows(); ++f)
    {
        for (int i = 0; i < 3; ++i)
        {
            int v0 = F(f, i);
            int v1 = F(f, (i + 1) % 3);

            if (v0 < 0 || v0 >= V.rows() ||
                v1 < 0 || v1 >= V.rows())
            {
                std::cerr << "Invalid index in face " << f << "\n";
                continue;
            }

            EdgeKey key(v0, v1);
            edgeFaces[key].push_back(f);
        }
    }

    // =====================================================
    // 2️⃣ 统计边界 & 非流形边
    // =====================================================

    std::vector<bool> is_boundary_face(F.rows(), false);

    for (const auto& pair : edgeFaces)
    {
        const auto& faces = pair.second;

        if (faces.size() == 1)
        {
            result.num_boundary_edges++;
            is_boundary_face[faces[0]] = true;
        }

        if (faces.size() > 2)
        {
            result.num_nonmanifold_edges++;
        }
    }

    // =====================================================
    // 3️⃣ 统计边界面
    // =====================================================

    for (int i = 0; i < F.rows(); ++i)
        if (is_boundary_face[i])
            result.num_boundary_faces++;

    // =====================================================
    // 4️⃣ 非流形顶点检测
    // 定义：其 incident faces 不构成单连通扇形
    // =====================================================

    // vertex -> incident faces
    std::vector<std::vector<int>> vertexFaces(V.rows());

    for (int f = 0; f < F.rows(); ++f)
    {
        for (int i = 0; i < 3; ++i)
            vertexFaces[F(f, i)].push_back(f);
    }

    for (int v = 0; v < V.rows(); ++v)
    {
        const auto& incident = vertexFaces[v];

        if (incident.size() <= 1)
            continue;

        // 构建面邻接图（通过共享边）
        std::set<int> visited;
        std::vector<int> stack;

        stack.push_back(incident[0]);
        visited.insert(incident[0]);

        while (!stack.empty())
        {
            int f0 = stack.back();
            stack.pop_back();

            for (int f1 : incident)
            {
                if (visited.count(f1)) continue;

                // 检查 f0 和 f1 是否共享包含 v 的边
                int shared = 0;

                for (int i = 0; i < 3; ++i)
                {
                    int a0 = F(f0, i);
                    int b0 = F(f0, (i + 1) % 3);

                    if (a0 == v || b0 == v)
                    {
                        for (int j = 0; j < 3; ++j)
                        {
                            int a1 = F(f1, j);
                            int b1 = F(f1, (j + 1) % 3);

                            if ((a0 == a1 && b0 == b1) ||
                                (a0 == b1 && b0 == a1))
                            {
                                shared = 1;
                                break;
                            }
                        }
                    }
                    if (shared) break;
                }

                if (shared)
                {
                    visited.insert(f1);
                    stack.push_back(f1);
                }
            }
        }

        if (visited.size() != incident.size())
        {
            result.num_nonmanifold_vertices++;
        }
    }


    //result.num_self_intersections = CountSelfIntersections(V, F);

    // =====================================================
    // 输出
    // =====================================================
    std::cout << "Boundary edges: " << result.num_boundary_edges << "\n";
    std::cout << "Boundary faces: " << result.num_boundary_faces << "\n";
    std::cout << "Nonmanifold edges: " << result.num_nonmanifold_edges << "\n";
    std::cout << "Nonmanifold vertices: " << result.num_nonmanifold_vertices << "\n";
    //std::cout << "selfintersection vertices: " << result.num_self_intersections << "\n";
    return result;

}