#pragma once

#include "Scene.h"

#include <utility>
#include "AutoMorphingModel.h"
#include "igl/min_heap.h"
#include <vector>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void Q_edge_and_midpoint(const int e,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& /*F*/,
        const Eigen::MatrixXi& E,
        const Eigen::VectorXi& /*EMAP*/,
        const Eigen::MatrixXi& /*EF*/,
        const Eigen::MatrixXi& /*EI*/,
        double& cost,
        Eigen::RowVectorXd& p);
    void reset_Q_matrices();
    /*
    void calculate_vertex_costs(const int e,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXi& E,
        double& cost,
        Eigen::RowVectorXd& p);
        */
private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cyl, sphere1, cube, snake;
    std::shared_ptr<cg3d::AutoMorphingModel> autoSnake, autoSphere;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi OF, F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd N, C, C2, OV, V, T;
    std::vector<Eigen::Matrix4d> Q_vertices;
    igl::min_heap< std::tuple<double, int, int> > Q, Q2;
    int max_iter;
};
