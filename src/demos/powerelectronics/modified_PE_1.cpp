// ==================================================================================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ==================================================================================================================================================
// Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
// ==================================================================================================================================================

// =========================
// ======== HEADERS ========
// =========================
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
// #include "chrono_multicore/physics/ChSystemMulticoreNSC.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
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
void AddAxis(ChSystemNSC& sys, const ChVector3d& position, float x = 2, float y = 2, float z = 2, const ChColor& color = ChColor(0.0f, 0.0f, 0.0f)){
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto x_axis = chrono_types::make_shared<ChBodyEasyBox>(
        x, y, z, 1000, true, false, mat);
    x_axis->SetPos(ChVector3d(position[0]-x/2, position[1]-y/2, position[2]-z/2));  // Position along X-axis
    x_axis->SetFixed(true);
    x_axis->GetVisualShape(0)->SetColor(color); // Red color
    sys.Add(x_axis);
}

enum class JointType {
    FIXED,
    REVOLUTE,
    PRISMATIC
};

void CreateJoint(std::shared_ptr<ChBody> bodyA, std::shared_ptr<ChBody> bodyB, ChSystemNSC& sys, JointType jointType) {
    ChVector3d jointPosition(bodyA->GetPos());

    ChQuaternion<> jointOrientation;
    if (jointType == JointType::PRISMATIC)jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(0, 1, 0));
    else jointOrientation.SetFromAngleAxis(0.0 * (CH_PI / 180.0), ChVector3d(0, 1, 0));
    
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
}

class RigidBody {
public:
    RigidBody(ChSystem& sys, const std::string& file_name, double density, bool is_fixed, const ChColor& color = ChColor(1.0f, 0.0f, 0.0f))
        : system(sys), obj_file(file_name), density(density), is_fixed(is_fixed) {
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
private:
    ChSystem& system;
    std::string obj_file;
    double density;
    bool is_fixed;
    std::shared_ptr<ChBody> body;
    ChVector3d cog;
    const ChColor color;

    void SetupRigidBody() {
        // Load mesh
        const std::string fName = std::string("powerelectronics/Test_Model/") + obj_file + std::string(".obj");
        std::cout<<fName<<std::endl;
        std::cout<<obj_file<<std::endl;
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(fName));
        // auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));

        // Compute mass properties
        double volume;
        ChMatrix33<> geometric_inertia;
        trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia);

        // Calculate mass and inertia
        double mass = density * volume;
        ChMatrix33<> inertia = density * geometric_inertia;

        // Create rigid body
        body = chrono_types::make_shared<ChBody>();
        system.Add(body);
        body->SetFixed(is_fixed);
        body->SetMass(mass);
        body->SetInertiaXX(ChVector3d(inertia(0, 0), inertia(1, 1), inertia(2, 2)));
        body->SetPos(cog);

        // Visualization
        auto mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        mesh->SetMesh(trimesh);
        // mesh->SetColor(color);
        mesh->SetVisible(true);
        body->AddVisualShape(mesh, ChFrame<>(-cog, ChMatrix33<>(1)));

        // Collision
        auto coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(fName));
        coll_trimesh->Transform(-cog, ChMatrix33<>(1));

        auto coll_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        coll_mat->SetFriction(0.30);
        coll_mat->SetRestitution(0.001);

        auto coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(coll_mat, coll_trimesh, false, false, 0.001);
        auto coll_model = chrono_types::make_shared<ChCollisionModel>();
        coll_model->SetSafeMargin(0.1f);
        coll_model->SetEnvelope(0.001f);
        coll_model->AddShape(coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
        body->AddCollisionModel(coll_model);
        body->EnableCollision(false);

        // Debug Output
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "!!!!!!! " << obj_file << " -> Inertia properties !!!!!!!" << "\n";
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
        std::cout << "Mass: " << mass << " [kg]" << "\n";
        std::cout << "Center of Gravity: " << cog << " [mm]" << "\n";
        std::cout << "Inertia Tensor:\n" << inertia << " [kg*mm^2]" << "\n\n";
    }

// This is how you declare call the class in the main function
// here auto will catch the data type of the function
// auto = std::shared_ptr<ChBody>
// auto FrameGlobal_body = frameGlobal.GetBody();          ChVector3d FrameGlobal_cog = frameGlobal.GetCOG();
// std::shared_ptr<ChBody> Rotor_body = rotor.GetBody();   ChVector3d Rotor_cog = rotor.GetCOG();
// std::shared_ptr<ChBody> Stator_body = stator.GetBody(); ChVector3d Stator_cog = stator.GetCOG();

};

int main(int argc, char* argv[]) {
    ChSystemNSC sys = GravetySetup();

    RigidBody FrameGlobal(sys, "FrameGlobal_OBJ", 7500.00 / (1e9), true);
    RigidBody Crank(sys, "Crank_OBJ", 2700.00 / (1e9), false);
    RigidBody Piston(sys, "Piston_OBJ", 2700.00 / (1e9), false);
    RigidBody Rod(sys, "Rod_OBJ", 7500.00 / (1e9), false);
    RigidBody Rotor(sys, "Rotor_OBJ", 5820.00 / (1e9), false);
    RigidBody Screw1(sys, "Screw1_OBJ", 7500.00 / (1e9), false);
    RigidBody Screw2(sys, "Screw2_OBJ", 7500.00 / (1e9), false);
    RigidBody ScrewNut(sys, "ScrewNut_OBJ", 7500.00 / (1e9), false);
    RigidBody Stator(sys, "Stator_OBJ", 7920.00 / (1e9), false);
    
    auto [FrameGlobal_body, FrameGlobal_cog] = FrameGlobal.GetBodyAndCOG();
    auto [Crank_body,       Crank_cog]      = Crank.GetBodyAndCOG();
    auto [Piston_body,      Piston_cog]     = Piston.GetBodyAndCOG();
    auto [Rod_body,         Rod_cog]        = Rod.GetBodyAndCOG();
    auto [Rotor_body,       Rotor_cog]      = Rotor.GetBodyAndCOG();
    auto [Screw1_body,      Screw1_cog]     = Screw1.GetBodyAndCOG();
    auto [Screw2_body,      Screw2_cog]     = Screw2.GetBodyAndCOG();
    auto [ScrewNut_body,    ScrewNut_cog]   = ScrewNut.GetBodyAndCOG();
    auto [Stator_body,      Stator_cog]     = Stator.GetBodyAndCOG();

    // AddVisualizationBall(sys, FrameGlobal_cog, ChColor(0.0f, 0.0f, 1.0f));
    // AddVisualizationBall(sys, Rotor_cog, ChColor(1.0f, 0.0f, 0.0f), 5);
    // AddVisualizationBall(sys, Stator_cog, ChColor(0.0f, 1.0f, 1.0f));
    // AddVisualizationBall(sys, FrameGlobal_cog, ChColor(1.0f, 0.0f, 0.0f));
    // AddAxis(sys, FrameGlobal_cog, 100, 2, 2);
    // AddAxis(sys, FrameGlobal_cog, 2, 100, 2, ChColor(0,1,0));
    // AddAxis(sys, FrameGlobal_cog, 2, 2, 100, ChColor(0,0,1));

    CreateJoint (Stator_body,       FrameGlobal_body,   sys, JointType::FIXED);
    CreateJoint (Rotor_body,        Stator_body,        sys, JointType::REVOLUTE);
    CreateJoint (Rotor_body,        Crank_body,         sys, JointType::FIXED);
    CreateJoint (Screw1_body,       Crank_body,         sys, JointType::FIXED);
    CreateJoint (Screw2_body,       Crank_body,         sys, JointType::FIXED);
    CreateJoint (Screw2_body,       Rod_body,           sys, JointType::REVOLUTE);
    CreateJoint (ScrewNut_body,     Rod_body,           sys, JointType::REVOLUTE);
    CreateJoint (ScrewNut_body,     Piston_body,        sys, JointType::REVOLUTE);
    CreateJoint (FrameGlobal_body,  Piston_body,        sys, JointType::PRISMATIC);


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
    Rotor_Stator_Torsional_Spring_Orientation.SetFromAngleAxis(0.0 * M_PI / 180.0, ChVector3d(1, 0, 0)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
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


    auto Piston_Frame_Spring = chrono_types::make_shared<ChLinkTSDA>();
    ChVector3d Piston_Frame_Spring_Position(Piston_body->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
    ChVector3d Piston_Frame_Spring_Position_1 = Piston_Frame_Spring_Position;
    Piston_Frame_Spring_Position_1[0] -= 6.0;  //[mm] Rise the position of the spring along y-axis in order to see it better in the animation
    Piston_Frame_Spring->Initialize(Piston_body, FrameGlobal_body, true, Piston_Frame_Spring_Position, Piston_Frame_Spring_Position_1);
    Piston_Frame_Spring->SetRestLength(0.0);
    Piston_Frame_Spring->SetSpringCoefficient(0.0);
    Piston_Frame_Spring->SetDampingCoefficient(0.0005);
    sys.AddLink(Piston_Frame_Spring);

    // =%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%
    // =%=%=%=% MULTI-PHYSICS SIMULATION =%=%=%=%
    // =%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%

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
    vis->AddCamera(ChVector3d(0, 100, 300));
    vis->AddLight(ChVector3d(30.f, 100.f, -30.f), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->EnableBodyFrameDrawing(false);
    vis->EnableLinkFrameDrawing(false);

    // =================================
    // ======== SOLVER SETTINGS ========
    // =================================
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    //sys.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);

    // sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.SetSolverType(ChSolver::Type::PARDISO_MKL);
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
    double Duty_PWM = 85.0 / 100; //[s] PWM Duty
    double t_PWM_counter = 0.0; //[s] PWM Period

    int frame_count = 0;
    double fps = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    int i = 0;
    while (brake_flag == 1) {
        // ======== RUN -> the Irrlicht visualizer ========
        //tools::drawGrid(vis.get(), 2, 2, 30, 30, ChCoordsys<>(ChVector3d(0, 0.01, 0), QuatFromAngleX(CH_PI_2)),ChColor(0.3f, 0.3f, 0.3f), true);
        if (vis->Run()) { brake_flag = 1; } // Check if the User wanted to stop de simulation before: t_simulation_STOP
        else { brake_flag = 0; }
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // ======== SOLVE & UPDATE -> the Electronic domain ========
        if (t_sim_mechanics > 1.0e-4)
        {
            if (t_PWM_counter < T_PWM * Duty_PWM)
            {
                motor.SetPWM(5.2); //[V]
                t_PWM_counter += t_step_mechanic;
            }
            else
            {
                motor.SetPWM(0.0); //[V]
                t_PWM_counter += t_step_mechanic;
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
            // std::cout << "IVprobe1 " << res["vprobe1"].back();
            std::cout << i << "\t\tfps =  " << fps << std::endl;
            assert(!res.empty());
            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }
        
        frame_count++;
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();

        if (elapsed_time >= 1.0) {  // Every second
            fps = frame_count / elapsed_time;
            frame_count = 0;
            start_time = std::chrono::high_resolution_clock::now();
        }

        // _sleep(300.0e-3); // Wait until Python circuit solution is completed

        // ======== RUN -> the Mechanic solver ========
        sys.DoStepDynamics(t_step_mechanic);
        realtime_timer.Spin(t_step_mechanic);

        // ======== UPDATE -> the Multi-physics timeline ======== 
        t_sampling_electronic_counter += t_step_mechanic;
        t_sim_electronics += t_step_mechanic;
        t_sim_mechanics += t_step_mechanic;
        i++;
    }

    // ============================================================
    // ======== CLOSE THE MULTI-PHYSICS CO-SIMULATION LOOP ========
    // ============================================================
    // system("pause>0"); // Pause the execution of the code to see the results onto the cmd terminal
    return 0;
}
