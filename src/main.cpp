//#include <iostream>
//#include <vector>
//#include <string>
//#include <thread>
//#include <atomic>
//#include <cmath>
//#include <array>
//
//// Polyscope
//#include "polyscope/polyscope.h"
//#include "polyscope/surface_mesh.h"
//
//// libigl
//#include <igl/read_triangle_mesh.h>
//#include <igl/write_triangle_mesh.h>
//#include <igl/file_dialog_open.h>
//#include <igl/file_dialog_save.h>
//
//// 算法头文件
//#include "sparseGrid.hpp"
//#include "utils.hpp" // ✨ 直接使用你的 utils.hpp
//
//// ==========================================================
//// 全局状态与多网格数据
//// ==========================================================
//Eigen::MatrixXd V_target, V_opt, V_denoise;
//Eigen::MatrixXi F_target, F_opt, F_denoise;
//
//// 指向当前“最新”网格的指针（用于保存和质量检查）
//Eigen::MatrixXd* V_active = nullptr;
//Eigen::MatrixXi* F_active = nullptr;
//
//SparseSDFGrid* active_sgrid = nullptr;
//
//// ✨ 重构：使用单一整数记录当前激活的网格 (0:无, 1:Target, 2:Opt, 3:Denoise)
//int active_mesh_id = 0;
//
//// 渲染控制状态
//std::array<float, 3> bg_color = { 0.2f, 0.2f, 0.2f };
//std::array<float, 3> mesh_color = { 0.3f, 0.7f, 0.9f };
//bool show_edges = true;
//
//// UI 算法参数
//int   grid_resolution = 512;
//int   opt_iters = 50;
//float opt_lr = 0.6f;
//float denoise_sigma = 0.3f;
//int   denoise_normal_iters = 25;
//int   denoise_vertex_iters = 20;
//
//// 多线程控制与进度条
//std::atomic<bool> is_optimizing(false);
//std::atomic<bool> opt_finished(false);
//std::atomic<bool> is_denoising(false);
//std::atomic<bool> denoise_finished(false);
//
//// 质量检查结果状态
//bool show_check_results = false;
//MeshCheckResult ui_check_result;
//
//// ==========================================================
//// 统一刷新渲染材质
//// ==========================================================
//void ApplyGlobalRenderSettings() {
//    polyscope::view::bgColor = { bg_color[0], bg_color[1], bg_color[2] };
//    float edge_w = show_edges ? 1.0f : 0.0f;
//
//    std::vector<std::string> names = { "1. Target", "2. Optimized", "3. Denoised" };
//
//    for (const auto& name : names) {
//        if (polyscope::hasSurfaceMesh(name)) {
//            auto* m = polyscope::getSurfaceMesh(name);
//            m->setSurfaceColor({ mesh_color[0], mesh_color[1], mesh_color[2] });
//            m->setEdgeWidth(edge_w);
//            m->setSmoothShade(false);
//        }
//    }
//}
//
//// ==========================================================
//// ✨ 核心逻辑重构：严格单选互斥，彻底消灭“幽灵网格”
//// ==========================================================
//void SetActiveMesh(int mesh_id) {
//    active_mesh_id = mesh_id;
//
//    // 强行同步 Polyscope 底层的渲染状态，只允许选中的那个 mesh 渲染
//    if (polyscope::hasSurfaceMesh("1. Target"))   polyscope::getSurfaceMesh("1. Target")->setEnabled(mesh_id == 1);
//    if (polyscope::hasSurfaceMesh("2. Optimized")) polyscope::getSurfaceMesh("2. Optimized")->setEnabled(mesh_id == 2);
//    if (polyscope::hasSurfaceMesh("3. Denoised")) polyscope::getSurfaceMesh("3. Denoised")->setEnabled(mesh_id == 3);
//
//    // 更新指针，确保 CheckMesh 和 Save 永远操作你眼前的网格
//    if (mesh_id == 1) { V_active = &V_target;  F_active = &F_target; }
//    if (mesh_id == 2) { V_active = &V_opt;     F_active = &F_opt; }
//    if (mesh_id == 3) { V_active = &V_denoise; F_active = &F_denoise; }
//
//    // 切换网格时，强制隐藏旧的检测结果
//    show_check_results = false;
//}
//
//// ==========================================================
//// UI 布局回调
//// ==========================================================
//void ApplicationCallback() {
//
//    // 全局线程锁：只要后台在算，就锁定文件导入等危险操作
//    bool is_busy = is_optimizing || is_denoising;
//
//    ImGui::Begin("Geometry Processor", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
//    ImGui::PushItemWidth(120);
//
//    // --------------------------------------------------------
//    // 0. 渲染控制
//    // --------------------------------------------------------
//    ImGui::TextDisabled("--- Rendering ---");
//    if (ImGui::ColorEdit3("Background", bg_color.data(), ImGuiColorEditFlags_NoInputs)) {
//        polyscope::view::bgColor = { bg_color[0], bg_color[1], bg_color[2] };
//    }
//    if (ImGui::ColorEdit3("Mesh Color", mesh_color.data(), ImGuiColorEditFlags_NoInputs)) {
//        ApplyGlobalRenderSettings();
//    }
//    if (ImGui::Checkbox("Show Wireframe (Edges)", &show_edges)) {
//        ApplyGlobalRenderSettings();
//    }
//
//    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
//
//    // --------------------------------------------------------
//    // 1. 数据导入与严格单选查看
//    // --------------------------------------------------------
//    ImGui::TextDisabled("--- Mesh I/O & View ---");
//
//    ImGui::BeginDisabled(is_busy); // 防止在计算时手滑乱点
//    if (ImGui::Button("Load Mesh...", ImVec2(-1, 0))) {
//        std::string filename = igl::file_dialog_open();
//        if (!filename.empty() && igl::read_triangle_mesh(filename, V_target, F_target)) {
//
//            // ✨ 彻底清场：杀掉 Polyscope 里所有的旧结构，清空 Eigen 矩阵
//            polyscope::removeAllStructures();
//            V_opt.resize(0, 0); F_opt.resize(0, 0);
//            V_denoise.resize(0, 0); F_denoise.resize(0, 0);
//
//            // 注册新靶标网格，强制切换到 Target 状态
//            polyscope::registerSurfaceMesh("1. Target", V_target, F_target);
//            SetActiveMesh(1);
//
//            ApplyGlobalRenderSettings();
//            polyscope::view::resetCameraToHomeView();
//        }
//    }
//    ImGui::EndDisabled();
//
//    // ✨ 重构：将 Checkbox 替换为 RadioButton，实现“选项卡”式的完美互斥
//    if (V_target.rows() > 0) {
//        if (ImGui::RadioButton("Show 1. Target Mesh", active_mesh_id == 1)) SetActiveMesh(1);
//    }
//    if (V_opt.rows() > 0) {
//        if (ImGui::RadioButton("Show 2. Optimized Mesh", active_mesh_id == 2)) SetActiveMesh(2);
//    }
//    if (V_denoise.rows() > 0) {
//        if (ImGui::RadioButton("Show 3. Denoised Mesh", active_mesh_id == 3)) SetActiveMesh(3);
//    }
//
//    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
//
//    // --------------------------------------------------------
//    // 2. 网格拓扑质量检查
//    // --------------------------------------------------------
//    ImGui::TextDisabled("--- Quality Check ---");
//
//    ImGui::BeginDisabled(is_busy || active_mesh_id == 0);
//    if (ImGui::Button("Check Active Mesh", ImVec2(-1, 0))) {
//        if (V_active && F_active && V_active->rows() > 0) {
//            ui_check_result = CheckMesh(*V_active, *F_active);
//            show_check_results = true;
//        }
//    }
//    ImGui::EndDisabled();
//
//    if (show_check_results) {
//        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "Boundary Edges: %d", ui_check_result.num_boundary_edges);
//        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "Boundary Faces: %d", ui_check_result.num_boundary_faces);
//        ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.4f, 1.0f), "Non-Manifold Edges: %d", ui_check_result.num_nonmanifold_edges);
//        ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.4f, 1.0f), "Non-Manifold Verts: %d", ui_check_result.num_nonmanifold_vertices);
//    }
//
//    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
//
//    // --------------------------------------------------------
//    // 3. 隐式优化
//    // --------------------------------------------------------
//    ImGui::TextDisabled("--- SDF Optimization ---");
//    ImGui::InputInt("Resolution", &grid_resolution);
//    ImGui::InputInt("Iterations##Opt", &opt_iters);
//    ImGui::InputFloat("Learn Rate", &opt_lr);
//
//    if (is_optimizing) {
//        float time = (float)ImGui::GetTime();
//        float progress = (std::sin(time * 4.0f) + 1.0f) / 2.0f;
//        ImGui::ProgressBar(progress, ImVec2(-1, 0), "Optimizing... Please wait");
//    }
//    else {
//        ImGui::BeginDisabled(is_busy || V_target.rows() == 0);
//        if (ImGui::Button("Run Optimization", ImVec2(-1, 0))) {
//            is_optimizing = true;
//            show_check_results = false;
//
//            std::thread([&]() {
//                if (active_sgrid) delete active_sgrid;
//                active_sgrid = new SparseSDFGrid(V_target, F_target, grid_resolution);
//                active_sgrid->compute_sdf_values_floodfill(V_target, F_target);
//                active_sgrid->optimize_grid(V_target, F_target, opt_iters, (double)opt_lr);
//                active_sgrid->extract_mesh_mc(V_opt, F_opt);
//
//                F_opt.col(1).swap(F_opt.col(2));
//
//                is_optimizing = false;
//                opt_finished = true;
//                }).detach();
//        }
//        ImGui::EndDisabled();
//    }
//
//    if (opt_finished) {
//        opt_finished = false;
//        polyscope::registerSurfaceMesh("2. Optimized", V_opt, F_opt);
//        SetActiveMesh(2); // 强制切到 Opt 视图，自动关闭旧视图
//        ApplyGlobalRenderSettings();
//    }
//
//    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
//
//    // --------------------------------------------------------
//    // 4. 双边滤波去噪
//    // --------------------------------------------------------
//    ImGui::TextDisabled("--- Bilateral Denoising ---");
//    ImGui::InputFloat("Sigma Normal", &denoise_sigma);
//    ImGui::InputInt("Normal Iters", &denoise_normal_iters);
//    ImGui::InputInt("Vertex Iters", &denoise_vertex_iters);
//
//    if (is_denoising) {
//        float time = (float)ImGui::GetTime();
//        float progress = (std::sin(time * 4.0f) + 1.0f) / 2.0f;
//        ImGui::ProgressBar(progress, ImVec2(-1, 0), "Denoising... Please wait");
//    }
//    else {
//        ImGui::BeginDisabled(is_busy || V_opt.rows() == 0);
//        if (ImGui::Button("Run Denoising", ImVec2(-1, 0))) {
//            is_denoising = true;
//            show_check_results = false;
//
//            std::thread([&]() {
//                V_denoise = V_opt; F_denoise = F_opt;
//                active_sgrid->bilateral_mesh_denoise(V_denoise, F_denoise, (double)denoise_sigma, denoise_normal_iters, denoise_vertex_iters);
//
//                is_denoising = false;
//                denoise_finished = true;
//                }).detach();
//        }
//        ImGui::EndDisabled();
//    }
//
//    if (denoise_finished) {
//        denoise_finished = false;
//        polyscope::registerSurfaceMesh("3. Denoised", V_denoise, F_denoise);
//        SetActiveMesh(3); // 强制切到 Denoise 视图
//        ApplyGlobalRenderSettings();
//    }
//
//    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
//
//    // --------------------------------------------------------
//    // 5. 导出
//    // --------------------------------------------------------
//    ImGui::TextDisabled("--- Export ---");
//    ImGui::BeginDisabled(is_busy || active_mesh_id == 0);
//    if (ImGui::Button("Save Active Mesh...", ImVec2(-1, 0))) {
//        if (V_active && V_active->rows() > 0) {
//            std::string filename = igl::file_dialog_save();
//            if (!filename.empty()) {
//                igl::write_triangle_mesh(filename, *V_active, *F_active);
//            }
//        }
//    }
//    ImGui::EndDisabled();
//
//    ImGui::PopItemWidth();
//    ImGui::End();
//}
//
//// ==========================================================
//// 主函数
//// ==========================================================
//int main(int argc, char** argv) {
//    polyscope::options::programName = "Advanced Geometry Processor";
//    polyscope::options::buildGui = false;
//
//    polyscope::view::bgColor = { bg_color[0], bg_color[1], bg_color[2] };
//    polyscope::view::style = polyscope::view::NavigateStyle::Turntable;
//    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
//
//    polyscope::init();
//    polyscope::state::userCallback = ApplicationCallback;
//    polyscope::show();
//
//    if (active_sgrid) delete active_sgrid;
//    return 0;
//}


#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <cmath>
#include <array>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>

#include "sparseGrid.hpp"
#include "utils.hpp"

// ==========================================================
// 全局状态与多网格数据
// ==========================================================
Eigen::MatrixXd V_target, V_opt, V_denoise;
Eigen::MatrixXi F_target, F_opt, F_denoise;

// 指向当前“最新”网格的指针（用于保存和质量检查）
Eigen::MatrixXd* V_active = nullptr;
Eigen::MatrixXi* F_active = nullptr;

SparseSDFGrid* active_sgrid = nullptr;

//使用单一整数记录当前激活的网格 (0:无, 1:Target, 2:Opt, 3:Denoise)
int active_mesh_id = 0;

// 渲染控制状态
std::array<float, 3> bg_color = { 0.2f, 0.2f, 0.2f };
std::array<float, 3> mesh_color = { 0.3f, 0.7f, 0.9f };
bool show_edges = true;

// UI 算法参数
int   grid_resolution = 512;
int   opt_iters = 50;
float opt_lr = 0.6f;
float denoise_sigma = 0.3f;
int   denoise_normal_iters = 25;
int   denoise_vertex_iters = 20;

// 多线程控制与进度条
std::atomic<bool> is_optimizing(false);
std::atomic<bool> opt_finished(false);
std::atomic<bool> is_denoising(false);
std::atomic<bool> denoise_finished(false);

// 质量检查结果状态
bool show_check_results = false;
MeshCheckResult ui_check_result;

// SDF 生成参数
int sdf_method = 0; // 0: Flood Fill, 1: Winding Number
float winding_threshold = 0.5f;


void ApplyGlobalRenderSettings() {
    polyscope::view::bgColor = { bg_color[0], bg_color[1], bg_color[2] };
    float edge_w = show_edges ? 1.0f : 0.0f;

    std::vector<std::string> names = { "1. Target", "2. Optimized", "3. Denoised" };

    for (const auto& name : names) {
        if (polyscope::hasSurfaceMesh(name)) {
            auto* m = polyscope::getSurfaceMesh(name);
            m->setSurfaceColor({ mesh_color[0], mesh_color[1], mesh_color[2] });
            m->setEdgeWidth(edge_w);
            m->setSmoothShade(false);
        }
    }
}

void SetActiveMesh(int mesh_id) {
    active_mesh_id = mesh_id;

    if (polyscope::hasSurfaceMesh("1. Target"))   polyscope::getSurfaceMesh("1. Target")->setEnabled(mesh_id == 1);
    if (polyscope::hasSurfaceMesh("2. Optimized")) polyscope::getSurfaceMesh("2. Optimized")->setEnabled(mesh_id == 2);
    if (polyscope::hasSurfaceMesh("3. Denoised")) polyscope::getSurfaceMesh("3. Denoised")->setEnabled(mesh_id == 3);

    if (mesh_id == 1) { V_active = &V_target;  F_active = &F_target; }
    if (mesh_id == 2) { V_active = &V_opt;     F_active = &F_opt; }
    if (mesh_id == 3) { V_active = &V_denoise; F_active = &F_denoise; }

    show_check_results = false;
}

void ApplicationCallback() {

    // 全局线程锁：只要后台在算，就锁定文件导入等危险操作
    bool is_busy = is_optimizing || is_denoising;

    ImGui::Begin("Geometry Processor", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PushItemWidth(120);

    // --------------------------------------------------------
    // 0. 渲染控制
    // --------------------------------------------------------
    ImGui::TextDisabled("--- Rendering ---");
    if (ImGui::ColorEdit3("Background", bg_color.data(), ImGuiColorEditFlags_NoInputs)) {
        polyscope::view::bgColor = { bg_color[0], bg_color[1], bg_color[2] };
    }
    if (ImGui::ColorEdit3("Mesh Color", mesh_color.data(), ImGuiColorEditFlags_NoInputs)) {
        ApplyGlobalRenderSettings();
    }
    if (ImGui::Checkbox("Show Wireframe (Edges)", &show_edges)) {
        ApplyGlobalRenderSettings();
    }

    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    // --------------------------------------------------------
    // 1. 数据导入与严格单选查看
    // --------------------------------------------------------
    ImGui::TextDisabled("--- Mesh I/O & View ---");

    ImGui::BeginDisabled(is_busy); 
    if (ImGui::Button("Load Mesh...", ImVec2(-1, 0))) {
        std::string filename = igl::file_dialog_open();
        if (!filename.empty() && igl::read_triangle_mesh(filename, V_target, F_target)) {

            //清空 Eigen 矩阵
            polyscope::removeAllStructures();
            V_opt.resize(0, 0); F_opt.resize(0, 0);
            V_denoise.resize(0, 0); F_denoise.resize(0, 0);

            // 注册新网格，强制切换到 Target 状态
            polyscope::registerSurfaceMesh("1. Target", V_target, F_target);
            SetActiveMesh(1);

            ApplyGlobalRenderSettings();
            polyscope::view::resetCameraToHomeView();
        }
    }
    ImGui::EndDisabled();

    if (V_target.rows() > 0) {
        if (ImGui::RadioButton("Show 1. Target Mesh", active_mesh_id == 1)) SetActiveMesh(1);
    }
    if (V_opt.rows() > 0) {
        if (ImGui::RadioButton("Show 2. Optimized Mesh", active_mesh_id == 2)) SetActiveMesh(2);
    }
    if (V_denoise.rows() > 0) {
        if (ImGui::RadioButton("Show 3. Denoised Mesh", active_mesh_id == 3)) SetActiveMesh(3);
    }

    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    // --------------------------------------------------------
    // 2. 网格拓扑质量检查
    // --------------------------------------------------------
    ImGui::TextDisabled("--- Quality Check ---");

    ImGui::BeginDisabled(is_busy || active_mesh_id == 0);
    if (ImGui::Button("Check Active Mesh", ImVec2(-1, 0))) {
        if (V_active && F_active && V_active->rows() > 0) {
            ui_check_result = CheckMesh(*V_active, *F_active);
            show_check_results = true;
        }
    }
    ImGui::EndDisabled();

    if (show_check_results) {
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "Boundary Edges: %d", ui_check_result.num_boundary_edges);
        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.3f, 1.0f), "Boundary Faces: %d", ui_check_result.num_boundary_faces);
        ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.4f, 1.0f), "Non-Manifold Edges: %d", ui_check_result.num_nonmanifold_edges);
        ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.4f, 1.0f), "Non-Manifold Verts: %d", ui_check_result.num_nonmanifold_vertices);
    }

    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    // --------------------------------------------------------
    // 3. 优化
    // --------------------------------------------------------
    ImGui::TextDisabled("--- SDF Optimization ---");
    ImGui::InputInt("Resolution", &grid_resolution);
    ImGui::InputInt("Iterations##Opt", &opt_iters);
    ImGui::InputFloat("Learn Rate", &opt_lr);

    ImGui::Text("SDF Method:");
    ImGui::RadioButton("Flood Fill", &sdf_method, 0);
    ImGui::RadioButton("Winding Number", &sdf_method, 1);

    if (sdf_method == 1) {
        ImGui::InputFloat("WN Threshold", &winding_threshold, 0.05f, 0.1f, "%.2f");
    }

    if (is_optimizing) {
        float time = (float)ImGui::GetTime();
        float progress = (std::sin(time * 4.0f) + 1.0f) / 2.0f;
        ImGui::ProgressBar(progress, ImVec2(-1, 0), "Optimizing... Please wait");
    }
    else {
        ImGui::BeginDisabled(is_busy || V_target.rows() == 0);
        if (ImGui::Button("Run Optimization", ImVec2(-1, 0))) {
            is_optimizing = true;
            show_check_results = false;

            std::thread([&]() {
                if (active_sgrid) delete active_sgrid;
                active_sgrid = new SparseSDFGrid(V_target, F_target, grid_resolution);

                // 根据用户的选择调用不同的 SDF 生成函数
                if (sdf_method == 0) {
                    active_sgrid->compute_sdf_values_floodfill(V_target, F_target);
                }
                else {
                    active_sgrid->compute_sdf_values_windingnumber(V_target, F_target, (double)winding_threshold);
                }

                active_sgrid->optimize_grid(V_target, F_target, opt_iters, (double)opt_lr);
                active_sgrid->extract_mesh_mc(V_opt, F_opt);

                F_opt.col(1).swap(F_opt.col(2));

                is_optimizing = false;
                opt_finished = true;
                }).detach();
        }
        ImGui::EndDisabled();
    }

    if (opt_finished) {
        opt_finished = false;
        polyscope::registerSurfaceMesh("2. Optimized", V_opt, F_opt);
        SetActiveMesh(2); // 强制切到 Opt 视图，自动关闭旧视图
        ApplyGlobalRenderSettings();
    }

    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    // --------------------------------------------------------
    // 4. 双边滤波去噪
    // --------------------------------------------------------
    ImGui::TextDisabled("--- Bilateral Denoising ---");
    ImGui::InputFloat("Sigma Normal", &denoise_sigma);
    ImGui::InputInt("Normal Iters", &denoise_normal_iters);
    ImGui::InputInt("Vertex Iters", &denoise_vertex_iters);

    if (is_denoising) {
        float time = (float)ImGui::GetTime();
        float progress = (std::sin(time * 4.0f) + 1.0f) / 2.0f;
        ImGui::ProgressBar(progress, ImVec2(-1, 0), "Denoising... Please wait");
    }
    else {
        ImGui::BeginDisabled(is_busy || V_opt.rows() == 0);
        if (ImGui::Button("Run Denoising", ImVec2(-1, 0))) {
            is_denoising = true;
            show_check_results = false;

            std::thread([&]() {
                V_denoise = V_opt; F_denoise = F_opt;
                active_sgrid->bilateral_mesh_denoise(V_denoise, F_denoise, (double)denoise_sigma, denoise_normal_iters, denoise_vertex_iters);

                is_denoising = false;
                denoise_finished = true;
                }).detach();
        }
        ImGui::EndDisabled();
    }

    if (denoise_finished) {
        denoise_finished = false;
        polyscope::registerSurfaceMesh("3. Denoised", V_denoise, F_denoise);
        SetActiveMesh(3); // 强制切到 Denoise 视图
        ApplyGlobalRenderSettings();
    }

    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    // --------------------------------------------------------
    // 5. 导出
    // --------------------------------------------------------
    ImGui::TextDisabled("--- Export ---");
    ImGui::BeginDisabled(is_busy || active_mesh_id == 0);
    if (ImGui::Button("Save Active Mesh...", ImVec2(-1, 0))) {
        if (V_active && V_active->rows() > 0) {
            std::string filename = igl::file_dialog_save();
            if (!filename.empty()) {
                igl::write_triangle_mesh(filename, *V_active, *F_active);
            }
        }
    }
    ImGui::EndDisabled();

    ImGui::PopItemWidth();
    ImGui::End();
}

// ==========================================================
// 主函数
// ==========================================================
int main(int argc, char** argv) {
    polyscope::options::programName = "Advanced Geometry Processor";
    polyscope::options::buildGui = false;

    polyscope::view::bgColor = { bg_color[0], bg_color[1], bg_color[2] };
    polyscope::view::style = polyscope::view::NavigateStyle::Turntable;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;

    polyscope::init();
    polyscope::state::userCallback = ApplicationCallback;
    polyscope::show();

    if (active_sgrid) delete active_sgrid;
    return 0;
}