#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include <Utility.h>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"

#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/per_vertex_normals.h"
#include "igl/loop.h"
#include "igl/collapse_edge.h"


using namespace cg3d;


void BasicScene::reset_Q_matrices()
{
    for (int i = 0; i < V.rows(); i++)
    {
        // calculate Q for each vertex and update the list
        auto v = V.row(i);
        auto n = autoSphere->GetMeshList()[0]->data[0].vertexNormals.row(i).normalized();
        double x, y, z, a, b, c, d;
        x = v[0];
        y = v[1];
        z = v[2];
        a = n[0];
        b = n[1];
        c = n[2];
        d = -1 * (a * x + b * y + z * c);
        Eigen::Vector4d p(a, b, c, d);
        auto temp = p * p.transpose();
        Eigen::Matrix4d Q_v = Eigen::Matrix4d::Zero();
        for (int k = 0; k < 4; k++)
        {
            for (int j = 0; j < 4; j++)
            {
                Q_v(k, j) = temp(k, j);
            }
        }
        Q_vertices.push_back(Q_v);
    }
    Eigen::VectorXd costs_Q(E.rows());
    Q2 = {};
    C2.resize(E.rows(), V.cols());
    for (int e = 0; e < E.rows(); e++)
    {
        double cost = e;
        Eigen::RowVectorXd p(1, 3);
        Q_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
        C2.row(e) = p;
        costs_Q(e) = cost;
    }
    for (int e = 0; e < E.rows(); e++)
    {
        Q2.emplace(costs_Q(e), e, 0);
    }
}

void BasicScene::Q_edge_and_midpoint(const int e,
    const Eigen::MatrixXd& V, const Eigen::MatrixXi& /*F*/, const Eigen::MatrixXi& E, const Eigen::VectorXi& /*EMAP*/, const Eigen::MatrixXi& /*EF*/, const Eigen::MatrixXi& /*EI*/, double& cost, Eigen::RowVectorXd& p)
{
    auto v1 = V.row(E(e, 0)), v2 = V.row(E(e, 1));
    Eigen::Vector4d mid((v1(0) + v2(0)) / 2, (v1(1) + v2(1)) / 2, (v1(2) + v2(2)) / 2, 1);
    cost = mid.transpose() * (Q_vertices[E(e, 0)] + Q_vertices[E(e, 1)]) * mid;
    p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
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
    auto material{ std::make_shared<Material>("material", program) }; // empty material
    //    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/camel_b.obj") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto handMesh{ IglLoader::MeshFromFiles("hand_igl", "data/hand.mesh") };
    auto bunnyMesh{ IglLoader::MeshFromFiles("bunny_igl", "data/bunny.off") };

    auto morphFunc = [&](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;
    };
    sphere1 = Model::Create("sphere", bunnyMesh, material);
    //cube = Model::Create("cube", cubeMesh, material);

    autoSphere = AutoMorphingModel::Create(*sphere1, morphFunc);
    autoSphere->showWireframe = true;
    autoSphere->Scale(10.0);
    camera->Translate(20, Axis::Z);

    root->AddChild(autoSphere);

    auto mesh = autoSphere->GetMeshList();
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;

    igl::edge_flaps(F, E, EMAP, EF, EI);
    reset_Q_matrices();  
    C.resize(E.rows(), V.cols());
    Eigen::VectorXd costs(E.rows());
    Q = {};
    EQ = Eigen::VectorXi::Zero(E.rows());

    {
        Eigen::VectorXd costs(E.rows());
        igl::parallel_for(E.rows(), [&](const int e)
            {
                double cost = e;
        Eigen::RowVectorXd p(1, 3);
        igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
        C.row(e) = p;
        costs(e) = cost;
            }, 10000);
        for (int e = 0; e < E.rows(); e++)
        {
            Q.emplace(costs(e), e, 0);
        }
    }
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.6f, 0.6f, 1.0f);
    //cube->Rotate(0.01f, Axis::All);
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
            camera->RotateInSystem(system, 0.1f, Axis::X);
            break;
        case GLFW_KEY_DOWN:
            camera->RotateInSystem(system, -0.1f, Axis::X);
            break;
        case GLFW_KEY_LEFT:
            camera->RotateInSystem(system, 0.1f, Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            camera->RotateInSystem(system, -0.1f, Axis::Y);
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.05f, 0 });
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
        case GLFW_KEY_1:
            if (pickedModel)
            {
                if (pickedModel->meshIndex + 1 < pickedModel->GetMeshList()[0]->data.size())
                {
                    pickedModel->meshIndex++;
                }
            }
            break;
        case GLFW_KEY_2:
            if (pickedModel)
            {
                if (pickedModel->meshIndex > 0)
                {
                    pickedModel->meshIndex--;
                }
            }
            break;
        case GLFW_KEY_SPACE:
            if (pickedModel)
            {   
                int num_collapsed = 0;
                int max_iter = std::ceil(0.10 * Q.size());
                for (int j = 0; j < max_iter; j++)
                {
                    if (!igl::collapse_edge(igl::shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
                    {
                        break;
                    }
                    num_collapsed++;
                }
                igl::per_vertex_normals(V, F, N);
                T = Eigen::MatrixXd::Zero(V.rows(), 2);
                auto mesh = pickedModel->GetMeshList();
                mesh[0]->data.push_back({ V, F, N, T });
                autoSphere->SetMeshList(mesh);
                autoSphere->meshIndex = autoSphere->GetMeshList()[0]->data.size() - 1;
            }
            else
            {
                std::cout << "no object pickced!\n";
            }
            break;       
        }
    }
}

