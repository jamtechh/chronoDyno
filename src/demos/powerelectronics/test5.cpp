// ======== Multi-purposes headers ========///////
#if defined(_DEBUG)
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else
// #include <Python.h>
#endif
// #include <pybind11/pybind11.h>
// #include <pybind11/embed.h>
// #include <pybind11/numpy.h>
// #include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <filesystem>
#include <exception>
#define _USE_MATH_DEFINES 
#include <cmath>
// #include "externals/json.hpp"
#include <future>
#include <sstream>
// #include <chrono> // Header, which provides a high-resolution clock
// #include <nlohmann/json.hpp>
#include <thread>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRandom.h"
#include "chrono/core/ChFrame.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
// #include "chrono_irrlicht/ChIrrMeshTools.h"
// #include "chrono_irrlicht/ChIrrCamera.h"
#include <chrono/physics/ChSystem.h>

    // ======== ChElectronics headers ========
// #include "chrono_powerelectronics/ChElectronics.h"
// #include "chrono_powerelectronics/ChElectronicsManipulator.h"
// #include "chrono_powerelectronics/ChElectronicsSources.h"
#include "chrono_powerelectronics/ChElectronicsCosimulation.h"
#include "chrono_powerelectronics/circuits/ChElectronicMotor.h"
// #include "swFiles/cpp/dyno2.h"

// ============================
// ======== NAMESPACES ========
// ============================
using namespace ::chrono;
using namespace ::chrono::irrlicht;
using namespace ::chrono::powerelectronics;
// namespace py = pybind11;
// namespace fs = std::filesystem;
// using json = nlohmann::json;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// =====================================
// ======== FUNCTIONS & CLASSES ========
// =====================================

void _sleep(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds)); 
}

enum class JointType {
    FIXED,
    REVOLUTE,
    PRISMATIC
};

ChSystemNSC GravetySetup(){
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys; // Create a Chrono physical system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    ChVector3d gravity_acc = sys.GetGravitationalAcceleration(); 
    std::cout << "The gravity acceleration  vector is: " << gravity_acc << "\n\n";
    double gravity = 9.81e3; //[mm/s^2]
    sys.SetGravitationalAcceleration(ChVector3d(gravity, 0, 0));
    ChVector3d gravity_acc_new = sys.GetGravitationalAcceleration(); 
    std::cout << "The new gravity acceleration  vector is: " << gravity_acc_new << "\n\n";
    return sys;
}           
void AddVisualizationBall(ChSystemNSC& sys, const ChVector3d& position, const ChColor& color = ChColor(1,0,0), int rad = 8){
    // set a ball on the center of mass of the frame for visualization purposes        
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(rad, 10, true, true, mat);
    mrigidBall->SetPos(position);
    mrigidBall->SetPosDt(ChVector3d(0, 0, 0));  // set initial speed
    auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(rad);
    sphere_shape->SetColor(color);
    mrigidBall->AddVisualShape(sphere_shape);
    mrigidBall->SetFixed(true);
    sys.Add(mrigidBall);
}
void AddAxis(ChSystemNSC& sys, const ChVector3d& position, float x = 2, float y = 2, float z = 2, const ChColor& color = ChColor(1.0f, 0.0f, 0.0f)){
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto x_axis = chrono_types::make_shared<ChBodyEasyBox>(
        x, y, z, 1000, true, false, mat);
    x_axis->SetPos(ChVector3d(position[0]-x/2, position[1]-y/2, position[2]-z/2));  // Position along X-axis
    x_axis->SetFixed(true);
    x_axis->GetVisualShape(0)->SetColor(color); // Red color
    sys.Add(x_axis);
}
void CreateJoint(std::shared_ptr<ChBody> bodyA, std::shared_ptr<ChBody> bodyB, ChSystemNSC& sys, JointType jointType, bool showAxis = false) {
    ChVector3d jointPosition(bodyA->GetPos());

    ChQuaternion<> jointOrientation;
    if (jointType == JointType::PRISMATIC)jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(0, 0, 1));
    else jointOrientation.SetFromAngleX(0);
    
    ChFrame<> jointFrame(jointPosition, jointOrientation);
    std::shared_ptr<ChLinkLock> Joint;
    switch (jointType) {
        case JointType::FIXED:      {Joint = chrono_types::make_shared<ChLinkLockLock>();      break;}
        case JointType::REVOLUTE:   {Joint = chrono_types::make_shared<ChLinkLockRevolute>();  break;}
        case JointType::PRISMATIC:  {Joint = chrono_types::make_shared<ChLinkLockPrismatic>(); break;}
        default:    throw std::invalid_argument("Invalid joint type.");
    }

    Joint->Initialize(bodyA, bodyB, jointFrame);
    sys.AddLink(Joint);

    if(showAxis){
        auto axisShape = chrono_types::make_shared<ChVisualShapeCylinder>(2.5, 100); // Radius = 2, Length = 50
        axisShape->SetColor(ChColor(1, 0, 0)); // Green color for rotation axis

        auto axisBody = chrono_types::make_shared<ChBody>();
        axisBody->SetPos(jointPosition);
        axisBody->SetFixed(true); // The axis is just for visualization
        axisBody->AddVisualShape(axisShape, ChFrame<>(ChVector3d(0, 0, 0), jointOrientation));
        sys.Add(axisBody);
    } 
}

class RigidBody {
    public:
        RigidBody(ChSystemNSC& sys, const std::string& file_name, const ChVector3d& positionn, ChQuaternion<> rotts, ChColor color = ChColor(0.8f,0.8f,0.8f), bool is_fixed = false)
            : system(sys), obj_file(file_name), positionn(positionn), rotts(rotts),colorr(color), is_fixed(is_fixed) {
            SetupRigidBody();
        }
    
        std::shared_ptr<ChBody> GetBody() const {
            return body;
        }
    
        ChVector3d GetCOG() const {
            return cog;
        }

        std::tuple<std::shared_ptr<ChBody>, ChVector3d> GetBodyAndCOG() const {
            return std::make_tuple(body, cog);
        }
        void hideBody(){mesh->SetVisible(false);}
        void setPos(ChVector3d poss){body->SetPos(poss);}
        void setRot(ChQuaternion<>rott){body->SetRot(rott);}
        void setMass(double masss){body->SetMass(masss);}
        void setInertiaXX(){body->SetInertiaXX(false);}
        void setInertiaXY(){body->SetInertiaXY(false);}
        void showCG(){AddVisualizationBall(system, body->GetPos());}
    private:
        ChSystemNSC& system;
        std::string obj_file;
        bool is_fixed;
        std::shared_ptr<ChBody> body;
        std::shared_ptr<ChVisualShapeTriangleMesh>mesh;
        ChVector3d cog;
        ChVector3d positionn;
        ChQuaternion<> rotts;
        ChColor colorr;
    
        void SetupRigidBody() {
            // Load mesh
            const std::string fName = std::string("powerelectronics/dynoCPP_shapes/") + obj_file + std::string(".obj");
            std::cout<<fName<<std::endl;
            // std::cout<<obj_file<<std::endl;
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(fName));
            // auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));
    
            // Compute mass properties
            double volume;
            ChMatrix33<> geometric_inertia;
            trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia);
    
            // Calculate mass and inertia
            double mass = 7500.00 / (1e9) * volume;
            ChMatrix33<> inertia = 7500.00 / (1e9) * geometric_inertia;
    
            // Create rigid body
            body = chrono_types::make_shared<ChBody>();
            body->SetFixed(is_fixed);
            body->SetMass(mass);
            body->SetInertiaXX(ChVector3d(inertia(0, 0), inertia(1, 1), inertia(2, 2)));
            body->SetPos(positionn);
            body->SetRot(rotts);

            system.Add(body);
    
            // Visualization
            mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            mesh->SetMesh(trimesh);
            // mesh->SetVisible(true);
            mesh->SetColor(colorr);
            body->AddVisualShape(mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
    
            // Debug Output
            if(0){
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
                std::cout << "!!!!!!! " << obj_file << " -> Inertia properties !!!!!!!" << "\n";
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
                std::cout << "Mass: " << mass << " [kg]" << "\n";
                std::cout << "Center of Gravity: " << cog << " [mm]" << "\n";
                std::cout << "Inertia Tensor:\n" << inertia << " [kg*mm^2]" << "\n\n";
            }
            if(0){AddVisualizationBall(system, positionn, ChColor(0,1,0));}
        }
    };
        
int main(int argc, char* argv[]) {
    ChSystemNSC sys = GravetySetup();

    std::vector<std::unique_ptr<RigidBody>> bodies(11);     // Total 10 parts with 0 index Null
    std::vector<std::shared_ptr<ChBody>> body_ptrs(11);
    std::vector<ChVector3d> cogs(11);
    struct BodyData {
        std::string file_name;
        ChVector3d position;
        ChQuaternion<> rotation;
    };
    // Initialize data manually
    std::vector<BodyData> body_data = {{"null", ChVector3d(0.0f,0.0f,0.0f), ChQuaternion<>(0.0f,0.0f,0.0f,0.0f)},
        {"body_4_1",    ChVector3d(10.0501781487224,127.271341649174,219.573424974887),       ChQuaternion<>(1,0,0,0)},                                                                     // frame    
        {"body_1_1",    ChVector3d(-153.681408502864,232.071341649174,257.256405065421),      ChQuaternion<>(0.344860417986101,-0.344860417986101,0.617309721376921,0.617309721376921)},    // Motor    
        {"body_10_1",   ChVector3d(-153.681408502864,15.4713416491736,257.256405065421),      ChQuaternion<>(0.679959230544171,0.679959230544171,0.194050108986773,-0.194050108986773)},    // dyno     
        {"body_9_1",    ChVector3d(-213.749821851277,127.271341649174,280.873424974888),      ChQuaternion<>(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594)},    // flywheel     
        {"body_2_1",    ChVector3d(-153.681408502864,316.571341649174,257.256405065421),      ChQuaternion<>(0.69219181681608,-0.69219181681608,0.144466220040721,0.144466220040721)},      // GearA     
        {"body_3_1",    ChVector3d(-153.681408502864,-62.0286583508263,257.256405065421),     ChQuaternion<>(0.682163121105785,0.682163121105785,0.186154441803611,-0.186154441803611)},    // GearF     
        {"body_5_1",    ChVector3d(-183.793091163963,320.571341649174,254.366929798653),      ChQuaternion<>(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382)},    // GearB     
        {"body_6_1",    ChVector3d(-213.749821851278,320.571341649174,280.873424974888),      ChQuaternion<>(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627)},// GearC    
        {"body_7_1",    ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888),     ChQuaternion<>(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0)},                 // GearD     
        {"body_8_1",    ChVector3d(-183.793091163963,-66.0286583508263,254.366929798653),     ChQuaternion<>(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918)}     // GearE     
    };
    float bri = 0.9;
    for (size_t i = 1; i < body_data.size(); i++) {
        auto& [name, pos, rot] = body_data[i];  // Structured binding
        if(i == 1)              bodies[i] = std::make_unique<RigidBody>(sys, name, (pos - body_data[1].position), rot, ChColor(bri, bri, bri));
        else if(i == 2)         bodies[i] = std::make_unique<RigidBody>(sys, name, (pos - body_data[1].position), rot, ChColor(0.0f, 0.0f, 1.0f));
        else if(i == 3)         bodies[i] = std::make_unique<RigidBody>(sys, name, (pos - body_data[1].position), rot, ChColor(0.0f, 1.0f, 0.0f));
        else if(i>=5 && i<=9 && i!=7)  bodies[i] = std::make_unique<RigidBody>(sys, name, (pos - body_data[1].position), rot, ChColor(1.0f, 0.1f, 0.2f));
        else if(i==7||i==10)    bodies[i] = std::make_unique<RigidBody>(sys, name, (pos - body_data[1].position), rot, ChColor(1.0f, 1.0f, 0.0f));
        else                    bodies[i] = std::make_unique<RigidBody>(sys, name, (pos - body_data[1].position), rot, ChColor(1.0f, 0.1f, 1.0f));
    }

    // Extract body and COG values (starting from index 1)
    for (size_t i = 1; i < bodies.size(); ++i) {
        auto [body, cog] = bodies[i]->GetBodyAndCOG();
        body_ptrs[i] = body;
        cogs[i] = cog;
    }


    if (cogs.size() > 4) {
        AddAxis(sys, ChVector3d(0,0,0), 100, 2, 2);
        AddAxis(sys, ChVector3d(0,0,0), 2, 100, 2, ChColor(0,1,0));
        AddAxis(sys, ChVector3d(0,0,100), 2, 2, 100, ChColor(0,0,1));
    }
    
    // CreateJoint(Stator_body,    FrameGlobal_body,   sys, JointType::FIXED);
    // CreateJoint(Rotor_body, Stator_body, sys, JointType::REVOLUTE, true);
    // CreateJoint(Gear_body,      FrameGlobal_body,   sys, JointType::FIXED);
    
    // ===============================================
    // ======== IRRLICHT VISUALIZATION SYSTEM ========
    // ===============================================
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1200, 800);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 0, 500), ChVector3d(0, 0, 0));
    bri = 0.9;
    vis->AddLight(ChVector3d(50.f, -400.f, -400.f), 300, ChColor(1, 1, 1));
    vis->AddLight(ChVector3d(-50.f, 400.f, 400.f), 300, ChColor(bri, bri, bri));
    vis->AddLight(ChVector3d(-50.f, -400.f, -400.f), 300, ChColor(bri, bri, bri));
    vis->AddLight(ChVector3d(-50.f, -400.f, 400.f), 300, ChColor(bri, bri, bri));
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);

    while (vis->Run()) {         
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
    }
    return 0;
}
