    // ======== Multi-purposes headers ========
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
    #include "chrono/physics/ChLinkMotorRotationSpeed.h"
    #include "chrono/physics/ChSystemNSC.h"
    #include "chrono/core/ChRealtimeStep.h"
    #include "chrono/core/ChRandom.h"
    
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
    #include "chrono_irrlicht/ChIrrMeshTools.h"
    // #include "chrono_irrlicht/ChIrrCamera.h"
    #include <chrono/physics/ChSystem.h>
    
        // ======== ChElectronics headers ========
    // #include "chrono_powerelectronics/ChElectronics.h"
    // #include "chrono_powerelectronics/ChElectronicsManipulator.h"
    // #include "chrono_powerelectronics/ChElectronicsSources.h"
    #include "chrono_powerelectronics/ChElectronicsCosimulation.h"
    #include "chrono_powerelectronics/circuits/ChElectronicMotor.h"
    
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
    
    // ======== Method: allows to put in pause the execution for sec ========
    void _sleep(int seconds) {
        std::this_thread::sleep_for(std::chrono::seconds(seconds)); 
    }
    
    void AddVisualizationBall(ChSystemNSC& sys, const ChVector3d& position, const ChColor& color, int rad = 10){
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
    // ===========================
    // ======== MAIN LOOP ========
    // ===========================
    int main(int argc, char* argv[]) {

        std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
        ChSystemNSC sys; // Create a Chrono physical system
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        
        ChVector3d gravity_acc = sys.GetGravitationalAcceleration(); 
        std::cout << "The gravity acceleration  vector is: " << gravity_acc << "\n\n";
        double gravity = 9.81e3; //[mm/s^2]
        sys.SetGravitationalAcceleration(ChVector3d(gravity, 0, 0));
        ChVector3d gravity_acc_new = sys.GetGravitationalAcceleration(); 
        std::cout << "The new gravity acceleration  vector is: " << gravity_acc_new << "\n\n";
      
    
        // =======================================================================
        // ======== RIGID BODY DEFINITION: WaveFront Shape -> FrameGlobal ========
        // =======================================================================
        // ======== File name ========
        std::string FrameGlobal_file_name = "powerelectronics/obj/Part4_frame.obj";
        // ======== Meshes ========
        // ======== Visualization Mesh ========
        auto FrameGlobal_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(FrameGlobal_file_name));
        auto FrameGlobal_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        FrameGlobal_mesh->SetMesh(FrameGlobal_trimesh);
        FrameGlobal_mesh->SetVisible(true);
    
    
        // ======== Visualization Collision Mesh ========
        // auto FrameGlobal_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(FrameGlobal_file_name));
        // auto FrameGlobal_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        // FrameGlobal_coll_mesh->SetMesh(FrameGlobal_coll_trimesh);
        // FrameGlobal_coll_mesh->SetVisible(true);
    
        // ======== Triangle Mesh for collision the model ========
        auto trimesh_FrameGlobal = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(FrameGlobal_file_name));
        // ======== Compute mass inertia from mesh ========
        double FrameGlobal_volume; // [mm^3]  
        ChVector3d FrameGlobal_cog; // [mm]   
        ChMatrix33<> FrameGlobal_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: FrameGlobal_mesh->ComputeMassProperties) 
        FrameGlobal_trimesh->ComputeMassProperties(true, FrameGlobal_volume, FrameGlobal_cog, FrameGlobal_geometric_inertia); // It returns: FrameGlobal_volume:[mm^3], FrameGlobal_cog:[mm], FrameGlobal_inertia:[mm^5] that is the geometric inertia tensor 
        double FrameGlobal_density = 7500.00 / (1e9); // [kg/mm^3]
        double FrameGlobal_mass = FrameGlobal_density * FrameGlobal_volume; // [kg]
        ChMatrix33<> FrameGlobal_inertia = FrameGlobal_density * FrameGlobal_geometric_inertia; // [kg*mm^2]
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "!!!!!!! FrameGlobal -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "The FrameGlobal mass is: " << FrameGlobal_mass << " [kg]" << "\n\n";
        std::cout << "The FrameGlobal cog is: " << FrameGlobal_cog << " [mm]" << "\n\n";
        std::cout << "The FrameGlobal inertia tensor is:\n" << FrameGlobal_inertia << " [kg*mm^2]" << "\n\n";
        // ======== Define the rigid body ========
        auto FrameGlobal_body = chrono_types::make_shared<ChBody>();
        sys.Add(FrameGlobal_body);
        
        FrameGlobal_body->SetFixed(true);
        FrameGlobal_body->SetMass(FrameGlobal_mass);
        FrameGlobal_body->SetInertiaXX(ChVector3d(FrameGlobal_inertia(0, 0), FrameGlobal_inertia(1, 1), FrameGlobal_inertia(2, 2)));
        FrameGlobal_body->SetPos(ChVector3d(FrameGlobal_cog[0], FrameGlobal_cog[1], FrameGlobal_cog[2]));
        
        // ======== Visulaization ========
        // FrameGlobal_mesh->SetMutable(false);
        FrameGlobal_mesh->SetColor(ChColor(1.0f, 0.0f, 1.0f));
        //FrameGlobal_mesh->SetOpacity(0.5f);
        FrameGlobal_mesh->SetBackfaceCull(true);
        FrameGlobal_body->AddVisualShape(FrameGlobal_mesh, ChFrame<>(-FrameGlobal_cog, ChMatrix33<>(1)));
    
        // FrameGlobal_body->AddVisualShape(FrameGlobal_coll_mesh, ChFrame<>(-FrameGlobal_cog, ChMatrix33<>(1)));
    
        // ======== Collision ========
        auto FrameGlobal_coll_model = chrono_types::make_shared<ChCollisionModel>();
        FrameGlobal_coll_model->SetSafeMargin(0.1f);  // inward safe margin
        FrameGlobal_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
        trimesh_FrameGlobal->Transform(-FrameGlobal_cog, ChMatrix33<>(1));
        auto FrameGlobal_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        FrameGlobal_mat->SetFriction(0.30);
        FrameGlobal_mat->SetRestitution(0.001); //In the range[0, 1].
        auto FrameGlobal_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(FrameGlobal_mat, trimesh_FrameGlobal, false, false, 0.001);
        FrameGlobal_coll_model->AddShape(FrameGlobal_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
        FrameGlobal_body->AddCollisionModel(FrameGlobal_coll_model);
        FrameGlobal_body->EnableCollision(false);
    
        // Crank_body, Piston_body, Rod_body, Screw1_body, Screw2_body, ScrewNut_body

        // =================================================================  
        // ======== RIGID BODY DEFINITION: WaveFront Shape -> Rotor ========
        // =================================================================
        // ======== File name ========
        std::string Rotor_file_name = "powerelectronics/obj/Part5_motor.obj";
        // ======== Meshes ========
            // ======== Visualization Mesh ========
        auto Rotor_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rotor_file_name));
        auto Rotor_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        Rotor_mesh->SetMesh(Rotor_trimesh);
        Rotor_mesh->SetVisible(true);
        // ======== Visualization Collision Mesh ========
        auto Rotor_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rotor_file_name));
        auto Rotor_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        Rotor_coll_mesh->SetMesh(Rotor_coll_trimesh);
        Rotor_coll_mesh->SetVisible(false);
        // ======== Triangle Mesh for collision the model ========
        auto trimesh_Rotor = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rotor_file_name));
        // ======== Compute mass inertia from mesh ========
        double Rotor_volume; // [mm^3]  
        ChVector3d Rotor_cog; // [mm]   
        ChMatrix33<> Rotor_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Rotor_mesh->ComputeMassProperties) 
        Rotor_trimesh->ComputeMassProperties(true, Rotor_volume, Rotor_cog, Rotor_geometric_inertia); // It returns: Rotor_volume:[mm^3], Rotor_cog:[mm], Rotor_inertia:[mm^5] that is the geometric inertia tensor 
        double Rotor_density = 5820.00 / (1e9); // [kg/mm^3]
        double Rotor_mass = Rotor_density * Rotor_volume; // [kg]
        ChMatrix33<> Rotor_inertia = Rotor_density * Rotor_geometric_inertia; // [kg*mm^2]
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "!!!!!!! Rotor -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "The Rotor mass is: " << Rotor_mass << " [kg]" << "\n\n";
        std::cout << "The Rotor cog is: " << Rotor_cog << " [mm]" << "\n\n";
        std::cout << "The Rotor inertia tensor is:\n" << Rotor_inertia << " [kg*mm^2]" << "\n\n";
        // ======== Define the rigid body ========
        auto Rotor_body = chrono_types::make_shared<ChBody>();
        sys.Add(Rotor_body);
        Rotor_body->SetFixed(false);
        Rotor_body->SetMass(Rotor_mass);
        Rotor_body->SetInertiaXX(ChVector3d(Rotor_inertia(0, 0), Rotor_inertia(1, 1), Rotor_inertia(2, 2)));
        Rotor_body->SetPos(Rotor_cog);
        // ======== Visulaization ========
        //Rotor_mesh->SetMutable(false);
        // Rotor_mesh->SetColor(ChColor(5.0f, 0.0f, 1.0f));
        //Rotor_mesh->SetOpacity(0.5f);
        //Rotor_mesh->SetBackfaceCull(true);
        Rotor_body->AddVisualShape(Rotor_mesh, ChFrame<>(-Rotor_cog, ChMatrix33<>(1)));
        Rotor_body->AddVisualShape(Rotor_coll_mesh, ChFrame<>(-Rotor_cog, ChMatrix33<>(1)));
        // ======== Collision ========
        auto Rotor_coll_model = chrono_types::make_shared<ChCollisionModel>();
        Rotor_coll_model->SetSafeMargin(0.1f);  // inward safe margin
        Rotor_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
        trimesh_Rotor->Transform(-Rotor_cog, ChMatrix33<>(1));
        auto Rotor_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        Rotor_mat->SetFriction(3.0);
        Rotor_mat->SetRestitution(0.001); //In the range[0, 1].
        auto Rotor_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Rotor_mat, trimesh_Rotor, false, false, 0.001);
        Rotor_coll_model->AddShape(Rotor_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
        Rotor_body->AddCollisionModel(Rotor_coll_model);
        Rotor_body->EnableCollision(false);
    
        // =================================================================  
        // ======== RIGID BODY DEFINITION: WaveFront Shape -> Stator =======
        // =================================================================
        // ======== File name ========
        std::string Stator_file_name = "powerelectronics/obj/Part5_motor.obj";
        // ======== Meshes ========
            // ======== Visualization Mesh ========
        auto Stator_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
        auto Stator_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        Stator_mesh->SetMesh(Stator_trimesh);
        Stator_mesh->SetVisible(true);
        // ======== Visualization Collision Mesh ========
        auto Stator_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
        auto Stator_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        Stator_coll_mesh->SetMesh(Stator_coll_trimesh);
        Stator_coll_mesh->SetVisible(false);
        // ======== Triangle Mesh for collision the model ========
        auto trimesh_Stator = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
        // ======== Compute mass inertia from mesh ========
        double Stator_volume; // [mm^3]  
        ChVector3d Stator_cog; // [mm]   
        ChMatrix33<> Stator_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Stator_mesh->ComputeMassProperties) 
        Stator_trimesh->ComputeMassProperties(true, Stator_volume, Stator_cog, Stator_geometric_inertia); // It returns: Stator_volume:[mm^3], Stator_cog:[mm], Stator_inertia:[mm^5] that is the geometric inertia tensor 
        double Stator_density = 7920.00 / (1e9); // [kg/mm^3]
        double Stator_mass = Stator_density * Stator_volume; // [kg]
        ChMatrix33<> Stator_inertia = Stator_density * Stator_geometric_inertia; // [kg*mm^2]
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "!!!!!!! Stator -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "The Stator mass is: " << Stator_mass << " [kg]" << "\n\n";
        std::cout << "The Stator cog is: " << Stator_cog << " [mm]" << "\n\n";
        std::cout << "The Stator inertia tensor is:\n" << Stator_inertia << " [kg*mm^2]" << "\n\n";
        // ======== Define the rigid body ========
        auto Stator_body = chrono_types::make_shared<ChBody>();
        sys.Add(Stator_body);
        Stator_body->SetFixed(false);
        Stator_body->SetMass(Stator_mass);
        Stator_body->SetInertiaXX(ChVector3d(Stator_inertia(0, 0), Stator_inertia(1, 1), Stator_inertia(2, 2)));
        Stator_body->SetPos(Stator_cog);
        // ======== Visulaization ========
        //Stator_mesh->SetMutable(false);
        //Stator_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
        //Stator_mesh->SetOpacity(0.5f);
        //Stator_mesh->SetBackfaceCull(true);
        Stator_body->AddVisualShape(Stator_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));
        Stator_body->AddVisualShape(Stator_coll_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));
        // ======== Collision ========
        auto Stator_coll_model = chrono_types::make_shared<ChCollisionModel>();
        Stator_coll_model->SetSafeMargin(0.1f);  // inward safe margin
        Stator_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
        trimesh_Stator->Transform(-Stator_cog, ChMatrix33<>(1));
        auto Stator_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        Stator_mat->SetFriction(0.30);
        Stator_mat->SetRestitution(0.001); //In the range[0, 1].
        auto Stator_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Stator_mat, trimesh_Stator, false, false, 0.001);
        Stator_coll_model->AddShape(Stator_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
        Stator_body->AddCollisionModel(Stator_coll_model);
        Stator_body->EnableCollision(false);
    
        AddVisualizationBall(sys, FrameGlobal_cog, ChColor(0.0f, 0.0f, 1.0f));
        AddVisualizationBall(sys, Rotor_cog, ChColor(1.0f, 0.0f, 0.0f), 5);
        AddVisualizationBall(sys, Stator_cog, ChColor(0.0f, 1.0f, 1.0f));
        AddVisualizationBall(sys, FrameGlobal_cog, ChColor(1.0f, 0.0f, 0.0f));
        // AddVisualizationBall(sys, Rotor_cog, ChColor(1.0f, 0.0f, 0.0f), 5);
        // AddVisualizationBall(sys, Rotor_cog, ChColor(1.0f, 0.0f, 0.0f), 5);

        AddAxis(sys, FrameGlobal_cog, 100, 2, 2);
        AddAxis(sys, FrameGlobal_cog, 2, 100, 2, ChColor(0,1,0));
        AddAxis(sys, FrameGlobal_cog, 2, 2, 100, ChColor(0,0,1));
            
        // ==========================================
        // ======== KINEMATIC LINKS CREATION ========
        // ==========================================
        // ======================================================================
        // ======== LINK DEFINITION -> FIXED JOINT: Stator - FrameGlobal ========
        // ======================================================================
        ChVector3d Stator_FrameGlobal_Link_Position(Stator_body->GetPos());   // [mm] set the position in the 3D space of the link respect to the absolute frame
        //Stator_FrameGlobal_Link_Position[2] = Stator_FrameGlobal_Link_Position[2] + 7.0;  
        ChQuaternion<> Stator_FrameGlobal_Link_Orientation; 
        Stator_FrameGlobal_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
        ChFrame<> Stator_FrameGlobal_Link_Frame(Stator_FrameGlobal_Link_Position, Stator_FrameGlobal_Link_Orientation);
        auto Stator_FrameGlobal_Link_Fixed = chrono_types::make_shared<ChLinkLockLock>();
        Stator_FrameGlobal_Link_Fixed->Initialize(Stator_body,                      // Body 1  
            FrameGlobal_body,                     // Body 2  
            Stator_FrameGlobal_Link_Frame);        // Location and orientation of the frame   
        sys.AddLink(Stator_FrameGlobal_Link_Fixed);
    
        // ===================================================================
        // ======== LINK DEFINITION -> REVOLUTE JOINT: Rotor - Stator ========
        // ===================================================================
        ChVector3d Rotor_Stator_Link_Position(Rotor_body->GetPos());            // [mm] set the position in the 3D space of the link respect to the absolute frame
        // Rotor_Stator_Link_Position[0] += 100.0;
        // Rotor_Stator_Link_Position[1] += 70.0;
        // Rotor_Stator_Link_Position[2] += 50.0;
        ChQuaternion<> Rotor_Stator_Link_Orientation;
        Rotor_Stator_Link_Orientation.SetFromAngleAxis(0 * (M_PI / 180.0), ChVector3d(0, 0, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
        ChFrame<> Rotor_Stator_Link_Frame(Rotor_Stator_Link_Position, Rotor_Stator_Link_Orientation);

        // ChVector3d joint_position(0, 0, 0); // Replace with your joint's position
        // ChQuaternion<> z_to_y_rotation;
        // z_to_y_rotation.Q_from_AngAxis(-M_PI / 2, ChVector<>(1, 0, 0));
        // ChCoordsys<> joint_coordsys(joint_position, z_to_y_rotation);

        auto Rotor_Stator_Link_Revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        Rotor_Stator_Link_Revolute->Initialize(Rotor_body,                      // Body 1  
            Stator_body,                     // Body 2  
            Rotor_Stator_Link_Frame);        // Location and orientation of the frame  
        sys.AddLink(Rotor_Stator_Link_Revolute);
    
        // =============================================================================
        // ======== F / T DEFINITION -> TORSIONAL SPRING/DAMPER: Rotor - Stator ========
        // =============================================================================
        // ======== Torsional spring coefficient ========
        double k_eq_Rotor_Stator_spr = 0.0; // [(N * m) / rad]
        k_eq_Rotor_Stator_spr = k_eq_Rotor_Stator_spr * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s]) 
    
        // ======== Torsional damping coefficient ========
        double r_ShaftBushing_experimental = 0.407e-4; //[(N*m*s)/rad]
        double r_eq_Rotor_Stator_spr = r_ShaftBushing_experimental * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])  
    
        // ======== Torsional spring/damper implementation ========
        auto Rotor_Stator_Torsional_Spring = chrono_types::make_shared<ChLinkRSDA>();
        ChVector3d Rotor_Stator_Torsional_Spring_Position(Stator_body->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
        Rotor_Stator_Torsional_Spring_Position[2] += 6.0;  //[mm] Rise the position of the spring along y-axis in order to see it better in the animation
        ChQuaternion<> Rotor_Stator_Torsional_Spring_Orientation;
        Rotor_Stator_Torsional_Spring_Orientation.SetFromAngleAxis(0.0 * M_PI / 180.0, ChVector3d(0, 1, 0)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
        ChFrame<> Rotor_Stator_Torsional_Spring_Frame(Rotor_Stator_Torsional_Spring_Position, Rotor_Stator_Torsional_Spring_Orientation);
        Rotor_Stator_Torsional_Spring->Initialize(Rotor_body,                                   // Body 1 
            Stator_body,                                  // Body 2 
            false,                                        // the two following frames are in absolute, not relative, coords.
            Rotor_Stator_Torsional_Spring_Frame,          // Location and orientation of the Body 1 frame 
            Rotor_Stator_Torsional_Spring_Frame);         // Location and orientation of the Body 1 frame
        Rotor_Stator_Torsional_Spring->SetRestAngle(0.0 * (M_PI / 180.0)); //[rad] Starting angular position
        Rotor_Stator_Torsional_Spring->SetSpringCoefficient(k_eq_Rotor_Stator_spr); // [(kg mm mm)/(s^2 rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m/rad]
        Rotor_Stator_Torsional_Spring->SetDampingCoefficient(r_eq_Rotor_Stator_spr); // [(kg mm mm s)/(s^2 mm rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m s/rad]
        sys.AddLink(Rotor_Stator_Torsional_Spring);
        Rotor_Stator_Torsional_Spring->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(10, 15)); // var1 = radius of the spring, var2 = graphical resolution of the spring
        
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
        vis->AddCamera(ChVector3d(200, 400, -10), Rotor_cog);
        vis->AddLight(ChVector3d(200.f, 400.f, -10.f), 300, ChColor(0.7f, 0.7f, 0.7f));
        vis->EnableBodyFrameDrawing(true);
        vis->EnableLinkFrameDrawing(true);
    
        // =================================
        // ======== SOLVER SETTINGS ========
        // =================================
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
        //sys.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
        sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        sys.GetSolver()->AsIterative()->SetMaxIterations(100000.0);
        sys.SetMaxPenetrationRecoverySpeed(1000.1);
        sys.SetMinBounceSpeed(0.001);
        ChRealtimeStepTimer realtime_timer;
    
        // =============================================================
        // ======== SET THE MULTI-PHYSICS SYMULATION PARAMETERS ========
        // =============================================================
        // ======== Mechanical domain ========
        double f_ToSample_mechanic = 0.5e4; // [Hz]
        double t_step_mechanic = 1 / f_ToSample_mechanic; // [s]
        // ======== Electronic domain ======== 
        double f_ToSample_electronic = 0.5e4; // [Hz]                              Frequency at which the electronic domain is called respect to the global time line
        double T_ToSample_electronic = 1 / f_ToSample_electronic;               // Period at which the electronic domain is called respect to the global time line
        double T_sampling_electronic = t_step_mechanic;                         // Time window of the electronic (SPICE) simulation
        double t_step_electronic = 1.0e-6; // [s]                                  Discretization of the electronic time window
    
        // ==================================================
        // ======== INITIALIZE THE MOTOR ========
        // ==================================================
    
        ChElectronicMotor motor(Rotor_body, t_step_electronic);
        double kt_motor = 0.1105*1e6;//*1e3*1e3;  // Motor torque constant [Nm/A]
        double ke_motor = -0.0953*1.0;  // Motor back EMF constant [V/(rad/s)]
    
        motor.InitParams(kt_motor,ke_motor);
        // motor.SetShaftAngVel(0.0);
        motor.Initialize();
    
    
        // ==================================================
        // ======== MULTI-PHYSICS CO-SYMULATION LOOP ========
        // ==================================================
    
        // ======== SET -> the Multi-physics timeline ========
        double t_simulation_STOP = 400.0e-3; //[s]
        double t_sim_mechanics = 0.0; //[s] 
        double t_sim_electronics = 0.0; //[s]
        double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
        int brake_flag = 1; // Set a brake flag in the case you want to stop the simulation before: t_simulation_STOP 
    
        // ======== INITIALIZE -> some needed variables ========
        double IVprobe1 = 0.0; //[A] Current circulating in the Motor
        double T_PWM = 4000.0e-6; //[s] PWM Period
        double Duty_PWM = 0.0 / 100; //[s] PWM Duty
        double t_PWM_counter = 0.0; //[s] PWM Period
        
        // RTSCamera cameruu;
        // const s32 LEFT_MOUSE_BUTTON = 0; // Assuming 0 corresponds to the left mouse button
        auto camera = vis->GetActiveCamera();
        // camera->setPosition(core::vector3df(0,0,0));

        while (t_sim_mechanics < t_simulation_STOP && brake_flag == 1) {
            // ======== RUN -> the Irrlicht visualizer ========
            vis->Run();
            // tools::drawGrid(vis.get(), 2, 2, 30, 30, ChCoordsys<>(ChVector3d(0, 0.01, 0), QuatFromAngleX(CH_PI_2)),ChColor(0.0f, 0.0f, 0.0f), true);

            // auto camera_pos = camera->getPosition();
            // auto camera_tar = camera->getTarget();
            // printf("x = %lf\t y=%lf \t z=%lf \n", camera_pos.X, camera_pos.Y, camera_pos.Z);
                            
            if (vis->Run()) { brake_flag = 1; } // Check if the User wanted to stop de simulation before: t_simulation_STOP
            else { brake_flag = 0; }
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
    
            // ======== SOLVE & UPDATE -> the Electronic domain ========
            if (t_sim_mechanics > 1.0e-4){
            // {printf("Hi \t\t\t\t\t\t\t\t !!!!!!!!!!!!!!!! \n");
                if (t_PWM_counter < T_PWM * Duty_PWM)
                {
                    motor.SetPWM(5.2); //[V]
                    t_PWM_counter += t_step_mechanic;
                    // printf("pwm \n");
                }
                else
                {
                    motor.SetPWM(0.0); //[V]
                    t_PWM_counter += t_step_mechanic;
                    // printf("0 \n");
                }
                if (t_PWM_counter >= T_PWM)
                {
                    t_PWM_counter = 0.0;
                }
            }
    
            if (t_sampling_electronic_counter >= T_ToSample_electronic)
            {
                // ======== COSIMULATE -> the SPICE circuit ========
                motor.Advance(t_step_mechanic);
                auto res = motor.GetResult();
                // std::cout << "IVprobe1 " << res["vprobe1"].back() << std::endl;
                assert(!res.empty());
                t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
            }
    
            _sleep(300.0e-3); // Wait until Python circuit solution is completed
    
            // ======== RUN -> the Mechanic solver ========
            sys.DoStepDynamics(t_step_mechanic);
            realtime_timer.Spin(t_step_mechanic);
    
            // ======== UPDATE -> the Multi-physics timeline ======== 
            t_sampling_electronic_counter += t_step_mechanic;
            t_sim_electronics += t_step_mechanic;
            t_sim_mechanics += t_step_mechanic;
        }
        return 0;
    }
    