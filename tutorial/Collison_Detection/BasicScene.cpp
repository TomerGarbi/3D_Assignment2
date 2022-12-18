#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include <Utility.h>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"

#include "igl/slice.h"

#include "igl/per_vertex_normals.h"
#include "igl/loop.h"
#include "igl/collapse_edge.h"
#include "Mesh.h"

using namespace cg3d;


double ABS(double d)
{
    return d >= 0 ? d : -1 * d;
}

Eigen::MatrixXd cube_vertices_transform(Eigen::MatrixXd V, Eigen::Matrix4d transform)
{
    Eigen::MatrixXd final_V = V;
    for (int v = 0; v < V.rows(); v++)
    {
        auto vertex = V.row(v);
        Eigen::Vector4d temp = transform * Eigen::Vector4d{ vertex(0), vertex(1), vertex(2), 1 };
        
        final_V.row(v) = Eigen::Vector3d{ temp(0), temp(1), temp(2) };
    }
    return final_V;
}

Eigen::MatrixXd vertices_from_box(Eigen::AlignedBox<double, 3> B)
{
    Eigen::MatrixXd V;
    V.resize(8, 3);
    V.row(0) = B.corner(Eigen::AlignedBox<double, 3>::BottomLeftCeil).transpose();
    V.row(1) = B.corner(Eigen::AlignedBox<double, 3>::BottomLeftFloor).transpose();
    V.row(2) = B.corner(Eigen::AlignedBox<double, 3>::BottomRightCeil).transpose();
    V.row(3) = B.corner(Eigen::AlignedBox<double, 3>::BottomRightFloor).transpose();
    V.row(4) = B.corner(Eigen::AlignedBox<double, 3>::TopLeftCeil).transpose();
    V.row(5) = B.corner(Eigen::AlignedBox<double, 3>::TopLeftFloor).transpose();
    V.row(6) = B.corner(Eigen::AlignedBox<double, 3>::TopRightCeil).transpose();
    V.row(7) = B.corner(Eigen::AlignedBox<double, 3>::TopRightFloor).transpose();
    return V;
}

void BasicScene::fit_cube_to_box(std::shared_ptr<cg3d::Model> cube, Eigen::AlignedBox<double, 3> B)
{
    auto sizes = B.sizes().cast<float>();
    cube->Scale({sizes(0), sizes(1), sizes(2)});
}


void BasicScene::draw_line(Eigen::Vector3d a, Eigen::Vector3d b)
{
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) };
    material->AddTexture(0, "textures/bricks.jpg", 2);
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
}

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) }; 
    auto material2{ std::make_shared<Material>("material", program) };
    auto material3{ std::make_shared<Material>("material", program) };

    material->AddTexture(0, "textures/bricks.jpg", 2);
    material2->AddTexture(0, "textures/pal.png", 2);
    material3->AddTexture(0, "textures/grass.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto bunnyMesh{ IglLoader::MeshFromFiles("bunny_igl", "data/bunny.off") };
    auto ACMesh{ IglLoader::MeshFromFiles("AC_igl", "data/AlignedCube.off") };
    
    collision1 = Model::Create("col1", ACMesh, material3);
    collision2 = Model::Create("col2", ACMesh, material3);
    collision1->isHidden = true;
    collision2->isHidden = true;
    collision1->showFaces = false;
    collision1->showWireframe = true;
    collision2->showFaces = false;
    collision2->showWireframe = true;
    root->AddChild(collision1);
    root->AddChild(collision2);

    sphere1 = Model::Create("sphere1", sphereMesh, material);
    sphere2 = Model::Create("sphere2", sphereMesh, material2);
    cube1 = Model::Create("cube1", ACMesh, material);
    cube1->showFaces = false;
    cube1->showWireframe = true;
    cube1->wireframeColor = { 1, 0, 0, 1 };

    cube2 = Model::Create("cube2", ACMesh, material2);
    cube2->showFaces = false;
    cube2->showWireframe = true;
    cube2->wireframeColor = { 0, 0, 0, 1 };

    // get vertices and faces
    V1 = sphere1->GetMeshList()[0]->data[0].vertices;
    V2 = sphere2->GetMeshList()[0]->data[0].vertices;
    F1 = sphere1->GetMeshList()[0]->data[0].faces;
    F2 = sphere2->GetMeshList()[0]->data[0].faces;
    // reset AABB trees
    sphere1_tree.init(V1, F1);
    sphere2_tree.init(V2, F2);

    
    camera->Translate(20, Axis::Z);
    root->AddChild(sphere1);
    root->AddChild(sphere2);

    //sphere1->Scale(10);
    //sphere2->Scale(10);

    sphere1->AddChild(cube1);
    sphere2->AddChild(cube2);
    sphere1->AddChild(collision1);
    sphere2->AddChild(collision2);
    
    sphere1->Translate({ 4, 0, 0 });
    sphere2->Translate({ -4, 0, 0 });
    sphere1->Scale({ 2, 1, 1 });
    sphere2->Scale({ 2, 1, 1 });
    
    sphere1_B = sphere1_tree.m_box;
    sphere2_B = sphere2_tree.m_box;

    Rotate1 = sphere1->GetRotation();
    Rotate2 = sphere2->GetRotation();

    fit_cube_to_box(cube1, sphere1_B);
    fit_cube_to_box(cube2, sphere2_B);
    cube1->Translate(sphere1_B.center().cast<float>());
    cube2->Translate(sphere2_B.center().cast<float>());

    

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    auto system = camera->GetRotation().transpose();
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.6f, 0.6f, 1.0f);
    if (sphere1->move && test_collision && collision(sphere1_tree, sphere2_tree))
    {
        sphere1->move = false;
    }
    if (sphere1->move)
    {
        sphere1->TranslateInSystem(system, sphere1->direction);
    }
    test_collision = !test_collision;
}

void BasicScene::KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            if (pickedModel)
            {
                pickedModel->direction = {0, 0.01f, 0};
                pickedModel->move = true;
            }
            else
            {
                camera->RotateInSystem(system, 0.1f, Axis::X);
            }
            break;
        case GLFW_KEY_DOWN:
            if (pickedModel)
            {
                pickedModel->direction = { 0, -0.01f, 0 };
                pickedModel->move = true;
            }
            else
            { 
                camera->RotateInSystem(system, -0.1f, Axis::X);
            }
            break;
        case GLFW_KEY_LEFT:
            if (pickedModel)
            {
                pickedModel->direction = { -0.01f, 0, 0 };
                pickedModel->move = true;
            }
            else
            {
                camera->RotateInSystem(system, 0.1f, Axis::Y);
            }
            break;
        case GLFW_KEY_RIGHT:
            if (pickedModel)
            {
                sphere1->direction = { 0.01f, 0, 0 };
                pickedModel->move = true;
            }
            else
            {
                camera->RotateInSystem(system, -0.1f, Axis::Y);
            }
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            if(pickedModel)
            {
                pickedModel->move = false;
                Eigen::Affine3f S = pickedModel->GetScaling(pickedModel->GetTransform());
                std::cout << "---\n" << "Scale: \n" << S.matrix() << std::endl;
            }
            else
            {
                camera->TranslateInSystem(system, { 0, -0.05f, 0 });
            }
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.05f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.05f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.05f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.05f });
            break;
        
        case GLFW_KEY_SPACE:
            if (pickedModel)
            {
                std::cout << "translatoin: \n" << pickedModel->GetTranslation() << std::endl;
            }
            std::cout << "res: " << OBB_intersect(sphere1_B, sphere2_B) << "\n";
            break;
        case GLFW_KEY_9:
            cube1->isHidden = !cube1->isHidden;
            cube2->isHidden = !cube2->isHidden;

        }
    }
}



Eigen::Vector3d BasicScene::get_center(Eigen::AlignedBox3d B, Eigen::Matrix4d transformation)
{
    auto C = B.center();
    Eigen::Vector4d temp = transformation * Eigen::Vector4d{ C(0), C(1), C(2), 1 };
    return { temp(0), temp(1), temp(2) };
}


bool BasicScene::OBB_intersect(Eigen::AlignedBox3d box1, Eigen::AlignedBox3d box2)
{

    // set parameters
    Eigen::Vector3d C0 = get_center(box1, sphere1->GetTransform().cast<double>()),
                    C1 = get_center(box2, sphere2->GetTransform().cast<double>());
    Rotate1 = sphere1->GetRotation();
    Rotate2 = sphere2->GetRotation();
   
    Eigen::Matrix3d A = Rotate1.cast<double>(), B = Rotate2.cast<double>();
    Eigen::Matrix3d C = A.transpose() * B;
    Eigen::RowVector3d A0 = A.col(0).transpose(), A1 = A.col(1).transpose(), A2 = A.col(2).transpose();
    Eigen::RowVector3d B0 = B.col(0).transpose(), B1 = B.col(1).transpose(), B2 = B.col(2).transpose();
    Eigen::Vector3d D = C1 - C0;
    Eigen::Matrix4d scale1 = sphere1->GetScaling(sphere1->GetTransform()).matrix().cast<double>();
    Eigen::Matrix4d scale2 = sphere2->GetScaling(sphere2->GetTransform()).matrix().cast<double>();
    
    double a0 = box1.sizes()(0) * scale1(0, 0), a1 = box1.sizes()(1) * scale1(1, 1), a2 = box1.sizes()(2) * scale1(2, 2),
          b0 = box2.sizes()(0) * scale2(0, 0), b1 = box2.sizes()(1) * scale2(1, 1), b2 = box2.sizes()(2) * scale2(2, 2);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C(i, j) = ABS(C(i, j));
    // test if the boxes intersects
    if (
           (ABS((A0 * D)(0)) <= a0 + b0 * C(0, 0) + b1 * C(0, 1) + b2 * C(0, 2))         // A0
        && (ABS((A1 * D)(0)) <= a1 + b0 * C(1, 0) + b1 * C(1, 1) + b2 * C(1, 2))         // A1
        && (ABS((A2 * D)(0)) <= a2 + b0 * C(2, 0) + b1 * C(2, 1) + b2 * C(2, 2))         // A2
        && (ABS((B0 * D)(0)) <= b0 + a0 * C(0, 0) + a1 * C(0, 1) + a2 * C(0, 2))         // B0
        && (ABS((B1 * D)(0)) <= b1 + a0 * C(1, 0) + a1 * C(1, 1) + a2 * C(1, 2))         // B1
        && (ABS((B2 * D)(0)) <= b2 + a0 * C(2, 0) + a1 * C(2, 1) + a2 * C(2, 2))         // B2
        && (ABS(C(1, 0) * (A2 * D)(0) - C(2, 0) * (A1 * D)(0)) <= a1 * C(2, 0) + a2 * C(1, 0) + b1 * C(0, 2) + b2 * C(0, 1))        // A0 x B0
        && (ABS(C(1, 1) * (A2 * D)(0) - C(2, 1) * (A1 * D)(0)) <= a1 * C(2, 1) + a2 * C(1, 1) + b0 * C(0, 2) + b2 * C(0, 0))        // A0 x B1
        && (ABS(C(1, 2) * (A2 * D)(0) - C(2, 2) * (A1 * D)(0)) <= a1 * C(2, 2) + a2 * C(1, 2) + b0 * C(0, 1) + b1 * C(0, 0))        // A0 x B2
        && (ABS(C(2, 0) * (A0 * D)(0) - C(0, 0) * (A2 * D)(0)) <= a0 * C(2, 0) + a2 * C(0, 0) + b1 * C(1, 2) + b2 * C(1, 1))        // A1 x B0
        && (ABS(C(2, 1) * (A0 * D)(0) - C(0, 1) * (A2 * D)(0)) <= a0 * C(2, 1) + a2 * C(0, 1) + b0 * C(1, 2) + b2 * C(1, 0))        // A1 x B1
        && (ABS(C(2, 2) * (A0 * D)(0) - C(0, 2) * (A2 * D)(0)) <= a0 * C(2, 2) + a2 * C(0, 2) + b0 * C(1, 1) + b1 * C(1, 0))        // A1 x B2
        && (ABS(C(0, 0) * (A1 * D)(0) - C(1, 0) * (A0 * D)(0)) <= a0 * C(1, 0) + a1 * C(0, 0) + b1 * C(2, 2) + b2 * C(2, 1))        // A2 x B0
        && (ABS(C(0, 1) * (A1 * D)(0) - C(1, 1) * (A0 * D)(0)) <= a0 * C(1, 1) + a1 * C(0, 1) + b0 * C(2, 2) + b2 * C(2, 0))        // A2 x B1
        && (ABS(C(0, 2) * (A1 * D)(0) - C(1, 2) * (A0 * D)(0)) <= a0 * C(1, 2) + a1 * C(0, 2) + b0 * C(2, 1) + b1 * C(2, 0)))       // A2 x B2
        return true;
    else
        return false;
}


bool BasicScene::collision(igl::AABB<Eigen::MatrixXd, 3> tree1, igl::AABB<Eigen::MatrixXd, 3> tree2)
{
    if (OBB_intersect(tree1.m_box, tree2.m_box))
    {
        if (tree1.is_leaf() && tree2.is_leaf())
        {
            fit_cube_to_box(collision1, tree1.m_box);
            fit_cube_to_box(collision2, tree2.m_box);
            collision1->Translate(tree1.m_box.center().cast<float>());
            collision2->Translate(tree2.m_box.center().cast<float>());
           // collision1->SetTransform(sphere1->GetTransform().cast<float>());
            //collision2->SetTransform(sphere2->GetTransform().cast<float>());

            collision1->isHidden = false;
            collision2->isHidden = false;
            return true;
        }
        else if (!tree1.is_leaf() && !tree2.is_leaf())
        {
            return collision(*tree1.m_left, *tree2.m_left) || collision(*tree1.m_left, *tree2.m_right) || collision(*tree1.m_right, *tree2.m_left) || collision(*tree1.m_right, *tree2.m_right);
        }
        else if (!tree1.is_leaf() && tree2.is_leaf())
        {
            return collision(*tree1.m_left, tree2) || collision(*tree1.m_right, tree2);
        }
        else if (tree1.is_leaf() && !tree2.is_leaf())
        {
            return collision(tree1, *tree2.m_left) || collision(tree1, *tree2.m_right);
        }     
    }
    return false;
}