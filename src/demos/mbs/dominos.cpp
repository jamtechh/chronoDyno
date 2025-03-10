#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/assets/ChTexture.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Create a bunch of rigid bodies that represent bricks in a large wall.
void create_wall_bodies_backup(ChSystemNSC& sys) {
    // Create a material that will be shared among all collision shapes
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);
    int height = 0;

    // Create bricks
    height = 4;
    for (int ui = 0; ui < 15; ui++) {  // N. of hor. bricks
        auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(1, height, 2,  // x,y,z size
                                                                    100,         // density
                                                                    true,        // visualization?
                                                                    true,        // collision?
                                                                    mat);        // contact material
        mrigidBody->SetPos(ChVector3d(ui * 2, height/2, 0));
        mrigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_borders.png"));
        sys.Add(mrigidBody);
    }
    
    // Create a ball that will collide with wall
    height = 2;
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(height,     // radius
                                                                  10,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector3d(-4, height+2, 0));
    mrigidBall->SetPosDt(ChVector3d(10, 0, 0));  // set initial speed
    mrigidBall->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sys.Add(mrigidBall);


    // Create the floor using fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visulization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector3d(0, -2, 0));
    mrigidFloor->SetFixed(true);
    sys.Add(mrigidFloor);
}

void create_wall_bodies(ChSystemNSC& sys) {
    // Create a material that will be shared among all collision shapes
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);

    // Create bricks
    int height = 4;

    for (int ui = 0; ui < 16; ui++) {  // N. of hor. bricks
        auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(1, height, 2,  // x,y,z size
                                                                    100,         // density
                                                                    true,        // visualization?
                                                                    true,        // collision?
                                                                    mat);        // contact material
        mrigidBody->SetPos(ChVector3d(ui * 2, height/2, 0));
        mrigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_borders.png"));
        sys.Add(mrigidBody);
    }
    for (int ui = 0; ui < 16; ui++) {  // N. of hor. bricks
        auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(1, height, 2,  // x,y,z size
                                                                    100,         // density
                                                                    true,        // visualization?
                                                                    true,        // collision?
                                                                    mat);        // contact material
        mrigidBody->SetPos(ChVector3d(ui * 2, height/2, 20));
        mrigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_borders.png"));
        sys.Add(mrigidBody);
    }

    double radius = 10.0;
    int noOfBricks = 15;
    double angle_step = M_PI / (noOfBricks-1);  // Use M_PI instead of CH_C_PI
    for (int i = 0; i < noOfBricks; i++) {
        double angle = i * angle_step - M_PI / 2;
        double x = radius * cos(angle) + 20 + radius + 2;
        double z = radius * sin(angle) + radius;

        auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(1, height, 2, 100, true, true, mat);
        mrigidBody->SetPos(ChVector3d(x, height / 2, z));

        // Manually calculate quaternion rotation around the Y-axis
        double half_angle = (-angle+M_PI/2) / 2.0;
        double qw = cos(half_angle);
        double qx = 0.0;
        double qy = sin(half_angle);
        double qz = 0.0;

        mrigidBody->SetRot(ChQuaternion<>(qw, qx, qy, qz));  // Manually set rotation quaternion

        sys.Add(mrigidBody);
    }
    // Create a ball that will collide with wall
    height = 2;
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(height,     // radius
                                                                  10,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector3d(-4, height+2, 0));
    mrigidBall->SetPosDt(ChVector3d(10, 0, 0));  // set initial speed
    mrigidBall->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sys.Add(mrigidBall);


    // Create the floor using fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visulization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector3d(0, -2, 0));
    mrigidFloor->SetFixed(true);
    mrigidFloor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/grid.png")); // Apply grid texture
    sys.Add(mrigidFloor);

    int noGrid = 40;
    // Create X-axis marker (Red Line)
    for(int i=0;i<=noGrid;i++){
        double thickness = (i == 0) ? 0.2 : 0.1;
        auto x_axis = chrono_types::make_shared<ChBodyEasyBox>(
            50, thickness, thickness, 1000, true, false, mat);
        x_axis->SetPos(ChVector3d(25, 0.1, i));  // Position along X-axis
        x_axis->SetFixed(true);
        x_axis->GetVisualShape(0)->SetColor((i == 0) ? ChColor(1, 0, 0) : ChColor(0, 0, 0)); // Red color
        sys.Add(x_axis);   
        if (i % 5 == 0) {
            auto marker = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 1000, true, false, mat);
            marker->SetPos(ChVector3d(i, 0.5, -2));  // Positioning along X-axis
            marker->SetFixed(true);
            marker->GetVisualShape(0)->SetColor(ChColor(0, 0, 1));  // Red for X-axis markers
            sys.Add(marker);
        }     
    }
    
    // Create Z-axis marker (Blue Line)
    for(int i=0;i<=noGrid;i++){
        double thickness = (i == 0) ? 0.2 : 0.1;
        auto z_axis = chrono_types::make_shared<ChBodyEasyBox>(thickness, thickness, 50, 1000, true, false, mat);
        z_axis->SetPos(ChVector3d(i, 0.1, 25));  // Position along Z-axis
        z_axis->SetFixed(true);
        z_axis->GetVisualShape(0)->SetColor((i == 0) ? ChColor(0, 0, 1) : ChColor(0, 0, 0));
        sys.Add(z_axis);
        if (i % 5 == 0) {
            auto marker = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 1000, true, false, mat);
            marker->SetPos(ChVector3d(0, 0.5, i));  // Positioning along X-axis
            marker->SetFixed(true);
            marker->GetVisualShape(0)->SetColor(ChColor(1, 0, 0));  // Red for X-axis markers
            sys.Add(marker);
        }
    }
        


}

// Create a bunch of ChronoENGINE rigid bodies that represent bricks in a Jenga tower

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    sys.SetNumThreads(1,8,7);

    // Create all the rigid bodies.
    create_wall_bodies(sys);
    std::cout << "Here1" << std::endl;

    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    int cx=20, cy=30, cz=10;
    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
            #ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Bricks test");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddLight(ChVector3d(70, 120, -90), 290, ChColor(0.7f, 0.7f, 0.7f));
            vis_irr->AddLight(ChVector3d(30, 80, 60), 190, ChColor(0.7f, 0.8f, 0.8f));
            vis_irr->AddCamera(ChVector3d(-10, cy, -10), ChVector3d(cx, 0, cz)); // (CamPos, CamTarget)

            vis = vis_irr;
            #endif
            break;
        }
    }
    // Prepare the physical system for the simulation
    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(40);
    solver->EnableWarmStart(true);
    sys.SetSolver(solver);

    // sys.SetSleepingAllowed(true);
    sys.SetMaxPenetrationRecoverySpeed(1.0);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        
        // auto vis_irrr = std::dynamic_pointer_cast<ChVisualSystemIrrlicht>(vis);

        // ChVector3d cam_pos = vis_irrr->GetCameraPosition();
        // ChVector3d cam_target = vis_irrr->GetCameraTarget();
        // std::cout << "Position: " << cam_pos << " | Target: " << cam_target << std::endl;
    
        sys.DoStepDynamics(0.02);
    }

    return 0;
}


