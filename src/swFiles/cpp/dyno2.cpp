// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: C:\Users\Jamiul Haque\OneDrive - UW-Madison\Design\chronoMotor\Assem1.SLDASM


#include <string>
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "dyno2.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


using namespace ::chrono;
using namespace ::chrono::irrlicht;
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

/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(chrono::ChSystemNSC& system) {

    std::string shapes_dir = "dyno_shapes/";

// Prepare some data for later use
std::shared_ptr<chrono::ChVisualShapeModelFile> body_shape;
std::shared_ptr<ChVisualShapeTriangleMesh>mesh;

// Rigid body part
// auto body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_1->SetName("Part1-1");
// body_1->SetPos(chrono::ChVector3d(-153.681408502864,232.071341649174,257.256405065421));
// body_1->SetRot(chrono::ChQuaternion<>(0.344860417986101,-0.344860417986101,0.617309721376921,0.617309721376921));
// body_1->SetMass(0.016029213366013);
// body_1->SetInertiaXX(chrono::ChVector3d(5.77976073009449,7.96307088073259,9.73060284283223));
// body_1->SetInertiaXY(chrono::ChVector3d(2.41692490861642,0.0837790436764728,-0.0530757027275727));
// body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.139083582689284,-0.107870586674347,27.5964334240644),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_1_1.obj");
// body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_1);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("frame-1");
body_4->SetPos(chrono::ChVector3d(10.0501781487224,127.271341649174,219.573424974887));
body_4->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_4->SetMass(1.54162568183963);
body_4->SetInertiaXX(chrono::ChVector3d(29568.9201124321,45865.3318996888,73541.0736970342));
body_4->SetInertiaXY(chrono::ChVector3d(-0.0138791051526975,-1731.0556183721,-0.0228813539240712));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(37.4376931356531,-0.000143977620523125,19.8392235533775),chrono::ChQuaternion<>(1,0,0,0)));
body_4->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

system.Add(body_4);
// Visualization
auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(shapes_dir + "body_4_1.obj"));
mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
mesh->SetMesh(trimesh);
// mesh->SetVisible(true);
body_4->AddVisualShape(mesh, ChFrame<>(ChVector3d(10.0501781487224,127.271341649174,219.573424974887), ChMatrix33<>(1)));


// // Rigid body part
// auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_2->SetName("GearA-2");
// body_2->SetPos(chrono::ChVector3d(-153.681408502864,316.571341649174,257.256405065421));
// body_2->SetRot(chrono::ChQuaternion<>(0.69219181681608,-0.69219181681608,0.144466220040721,0.144466220040721));
// body_2->SetMass(0.000870269104083074);
// body_2->SetInertiaXX(chrono::ChVector3d(0.0245232834116315,0.0348513174025659,0.0220932767457664));
// body_2->SetInertiaXY(chrono::ChVector3d(0.00556795508597199,-1.39917117738131e-17,9.19843766249956e-18));
// body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-1.96368346622425e-15,2.76276818372194e-15,0.394433254710844),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_2_1.obj");
// body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_2);



// // Rigid body part
// auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_3->SetName("GearF-1");
// body_3->SetPos(chrono::ChVector3d(-153.681408502864,-62.0286583508263,257.256405065421));
// body_3->SetRot(chrono::ChQuaternion<>(0.682163121105785,0.682163121105785,0.186154441803611,-0.186154441803611));
// body_3->SetMass(0.000870269104083074);
// body_3->SetInertiaXX(chrono::ChVector3d(0.0260120055120133,0.0333625953021842,0.0220932767457664));
// body_3->SetInertiaXY(chrono::ChVector3d(-0.00664540463802123,1.36335435030819e-17,1.15565959252578e-17));
// body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-2.02308898578366e-15,2.67365990438282e-15,0.394433254710843),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_3_1.obj");
// body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_3);


// // Rigid body part
// auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_5->SetName("GearB-1");
// body_5->SetPos(chrono::ChVector3d(-183.793091163963,320.571341649174,254.366929798653));
// body_5->SetRot(chrono::ChQuaternion<>(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382));
// body_5->SetMass(0.00573763521549478);
// body_5->SetInertiaXX(chrono::ChVector3d(1.09268072169601,0.695141740951902,0.603909758681937));
// body_5->SetInertiaXY(chrono::ChVector3d(-0.211167099311834,-4.86196921754609e-17,-1.20434881916057e-16));
// body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.4993650536692e-15,2.77437524831689e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_5_1.obj");
// body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_5);



// // Rigid body part
// auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_6->SetName("GearC-1");
// body_6->SetPos(chrono::ChVector3d(-213.749821851278,320.571341649174,280.873424974888));
// body_6->SetRot(chrono::ChQuaternion<>(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627));
// body_6->SetMass(0.00573763521549478);
// body_6->SetInertiaXX(chrono::ChVector3d(0.634526836838321,1.15329562580959,0.603909758681937));
// body_6->SetInertiaXY(chrono::ChVector3d(-0.129694217418744,1.25647520447832e-16,3.28806303875533e-17));
// body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.59890820730129e-15,2.65462361729245e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_6_1.obj");
// body_6->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_6);



// // Rigid body part
// auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_7->SetName("GearD-1");
// body_7->SetPos(chrono::ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888));
// body_7->SetRot(chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0));
// body_7->SetMass(0.00573763521549478);
// body_7->SetInertiaXX(chrono::ChVector3d(0.603909758681937,1.18391270396598,0.603909758681937));
// body_7->SetInertiaXY(chrono::ChVector3d(-6.68097235284266e-17,1.28759415537667e-16,-3.13275101861351e-18));
// body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.27453129794006e-15,2.61858173847454e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_7_1.obj");
// body_7->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_7);



// // Rigid body part
// auto body_8 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_8->SetName("GearE-1");
// body_8->SetPos(chrono::ChVector3d(-183.793091163963,-66.0286583508263,254.366929798653));
// body_8->SetRot(chrono::ChQuaternion<>(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918));
// body_8->SetMass(0.00573763521549478);
// body_8->SetInertiaXX(chrono::ChVector3d(0.81879554426816,0.969026918379755,0.603909758681937));
// body_8->SetInertiaXY(chrono::ChVector3d(0.280104422836692,-8.50253870869868e-18,2.02232343781072e-16));
// body_8->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.4993650536692e-15,2.77437524831689e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_8_1.obj");
// body_8->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_8);



// // Rigid body part
// auto body_9 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_9->SetName("Part2_flywheel-1");
// body_9->SetPos(chrono::ChVector3d(-213.749821851277,127.271341649174,280.873424974888));
// body_9->SetRot(chrono::ChQuaternion<>(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594));
// body_9->SetMass(1.02740423420996);
// body_9->SetInertiaXX(chrono::ChVector3d(6609.88825669769,5209.36075890446,11319.9638141394));
// body_9->SetInertiaXY(chrono::ChVector3d(-5364.83913451316,-0.00029120923437435,0.000265996941534389));
// body_9->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(8.92766810034472e-07,8.92766809937858e-07,1.29505962482037e-14),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_9_1.obj");
// body_9->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_9);



// // Rigid body part
// auto body_10 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
// body_10->SetName("Part2_dyno-1");
// body_10->SetPos(chrono::ChVector3d(-153.681408502864,15.4713416491736,257.256405065421));
// body_10->SetRot(chrono::ChQuaternion<>(0.679959230544171,0.679959230544171,0.194050108986773,-0.194050108986773));
// body_10->SetMass(0.0195543893524915);
// body_10->SetInertiaXX(chrono::ChVector3d(8.64325141424593,6.23841160373604,10.0061631055424));
// body_10->SetInertiaXY(chrono::ChVector3d(2.47769437441725,0.136114194517743,-0.20644974843194));
// body_10->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.0176453705510941,0.529772669645291,17.8045092646342),chrono::ChQuaternion<>(1,0,0,0)));

// // Visualization shape
// body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
// body_shape->SetFilename(shapes_dir + "body_10_1.obj");
// body_10->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// system.Add(body_10);

}

void printSom(){
    std::cout<<"\n\n\n\n\t\tgot the function \t\t\t\t !!!!!!!!!!!!!!!!!!!\n\n\n\n\n"<<std::endl;
}
