#pragma once

#include "Scene.h"

#include <utility>
#include "AutoMorphingModel.h"
#include "igl/min_heap.h"
#include "igl/AABB.h"
#include <vector>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    
    
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    bool OBB_intersect(Eigen::AlignedBox3d box1, Eigen::AlignedBox3d box2);
    bool collision(igl::AABB<Eigen::MatrixXd, 3> tree1, igl::AABB<Eigen::MatrixXd, 3> tree2);
    Eigen::Vector3d get_center(Eigen::AlignedBox3d B, Eigen::Matrix4d Transformation);
    void fit_cube_to_box(std::shared_ptr<cg3d::Model> cube, Eigen::AlignedBox<double, 3> B);
    void draw_line(Eigen::Vector3d a, Eigen::Vector3d b);
    
private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> sphere1, sphere2, cube1, cube2, collision1, collision2;
    igl::AABB<Eigen::MatrixXd, 3> sphere1_tree, sphere2_tree;       // AABB trees of mesh
    Eigen::MatrixXd V1, V2;                                         // Vertices of mesh
    Eigen::MatrixXi F1, F2;                                         // Faces of mesh
    Eigen::AlignedBox3d sphere1_B, sphere2_B;                       // bounding box of mesh
    Eigen::Matrix3f Rotate1, Rotate2;                               // Rotate matrices of mesh
    bool test_collision = false;
    
};
