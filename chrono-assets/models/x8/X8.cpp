// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: D:\Virginia-Tech-PhD\PHD_research\PyChrono\UAV_CAD_Models\X8_Chrono\Chrono_Assembly.SLDASM


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
#include "X8.h"


/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(chrono::ChSystem& system, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
ImportSolidworksSystemCpp(bodylist, linklist, motfun_map);
for (auto& body : bodylist)
    system.Add(body);
for (auto& link : linklist)
    system.Add(link);
}


/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist, std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {

// Some global settings
double sphereswept_r = 0.001;
chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.003);
chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.002);

std::string shapes_dir = "X8_shapes/";

// Prepare some data for later use
std::shared_ptr<chrono::ChVisualShapeModelFile> body_shape;
chrono::ChMatrix33<> mr;
std::shared_ptr<chrono::ChLinkBase> link;
chrono::ChVector3d cA;
chrono::ChVector3d cB;
chrono::ChVector3d dA;
chrono::ChVector3d dB;

// Assembly ground body
auto body_0 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_0->SetName("SLDW_GROUND");
body_0->SetFixed(true);
bodylist.push_back(body_0);

// Rigid body part
auto body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_1->SetName("frame-1");
body_1->SetPos(chrono::ChVector3d(0,0,0));
body_1->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_1->SetMass(1.52039110869982);
body_1->SetInertiaXX(chrono::ChVector3d(0.0154869282101269,0.0136133467493814,0.0134064606478442));
body_1->SetInertiaXY(chrono::ChVector3d(-1.21648368653966e-05,0.000721380361655569,1.05235970977448e-05));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.0118119639499877,-0.00303486723088082,0.0334178902325843),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_1->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_1 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_1;
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=7.02385995171017E-15;
mr(0,1)=-1.48723864357112E-16; mr(1,1)=-1; mr(2,1)=-1.48723864357112E-16;
mr(0,2)=7.02385995171017E-15; mr(1,2)=-1.48723864357113E-16; mr(2,2)=1;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.245,0.373249790618444,0.0285);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.00304557838099178,-0.00322141559125175,0.239749999999996), mr));
mr(0,0)=0; mr(1,0)=1; mr(2,0)=1.40930265789226E-16;
mr(0,1)=-1; mr(1,1)=0; mr(2,1)=7.08961260536019E-15;
mr(0,2)=7.08961260536019E-15; mr(1,2)=-1.40930265789226E-16; mr(2,2)=1;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.246181821379837,0.231961596037371,0.09175);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.0100647803623046,-0.00351913709958371,-0.00337500000000418), mr));
body_1->EnableCollision(true);

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("3_blade_prop-4");
body_2->SetPos(chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0251400000000042));
body_2->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_2->SetMass(0.00364120675790278);
body_2->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_2->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("3_blade_prop-1");
body_3->SetPos(chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0251400000000041));
body_3->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_3->SetMass(0.00364120675790278);
body_3->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_3->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("box_big_200x200x100-1");
body_4->SetPos(chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996));
body_4->SetRot(chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,3.34685337296841e-30,4.99485663418776e-15));
body_4->SetMass(0.356294160000001);
body_4->SetInertiaXX(chrono::ChVector3d(0.00230557943592285,0.00230557943592285,0.00384243450096002));
body_4->SetInertiaXY(chrono::ChVector3d(-2.9866381865429e-19,1.14461248885815e-17,4.97754647660404e-19));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-1.55192863101547e-16,0.0343231818337972,3.72402913198362e-17),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_4->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_4 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_4;

// Triangle mesh collision shape
auto body_4_1_collision_mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
body_4_1_collision_mesh->LoadWavefrontMesh(shapes_dir + "body_4_1_collision.obj", false, true);
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=0;
mr(0,2)=0; mr(1,2)=0; mr(2,2)=1;
body_4_1_collision_mesh->Transform(chrono::ChVector3d(0,0,0),mr);
collshape_4 = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(mat_4,body_4_1_collision_mesh,false,false, sphereswept_r);
body_4->GetCollisionModel()->AddShape(collshape_4);
body_4->EnableCollision(true);

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("3_blade_prop-3");
body_5->SetPos(chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0251400000000059));
body_5->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_5->SetMass(0.00364120675790278);
body_5->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_5->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_5);



// Rigid body part
auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_6->SetName("3_blade_prop-8");
body_6->SetPos(chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0321399999999942));
body_6->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_6->SetMass(0.00364120675790278);
body_6->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_6->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_6->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_6);



// Rigid body part
auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_7->SetName("3_blade_prop-7");
body_7->SetPos(chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0321399999999958));
body_7->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_7->SetMass(0.00364120675790278);
body_7->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_7->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_7->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_7);



// Rigid body part
auto body_8 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_8->SetName("3_blade_prop-2");
body_8->SetPos(chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0251400000000041));
body_8->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_8->SetMass(0.00364120675790278);
body_8->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_8->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_8->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_8->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_8);



// Rigid body part
auto body_9 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_9->SetName("3_blade_prop-5");
body_9->SetPos(chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.0321399999999959));
body_9->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_9->SetMass(0.00364120675790278);
body_9->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_9->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_9->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_9->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_9);



// Rigid body part
auto body_10 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_10->SetName("3_blade_prop-6");
body_10->SetPos(chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0321399999999959));
body_10->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_10->SetMass(0.00364120675790278);
body_10->SetInertiaXX(chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06));
body_10->SetInertiaXY(chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13));
body_10->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_10->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_10);




// Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: 3_blade_prop-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.0883883629628807,0.106432065455224,-0.0210000000000055);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0210000000000041);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0883883629628807,0.106432065455224,-0.0210000000000055);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0210000000000041);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident1");
linklist.push_back(link);


// Mate constraint: Concentric1 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: 3_blade_prop-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.090651591965577,0.105370088769343,-0.0330000000000055);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0292800000000041);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.090651591965577,0.105370088769343,-0.0330000000000055);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0292800000000041);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
linklist.push_back(link);


// Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_8 , SW name: 3_blade_prop-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.0889161874738823,-0.112643041133974,-0.021000000000004);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0210000000000041);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_8,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0889161874738823,-0.112643041133974,-0.021000000000004);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0210000000000041);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident2");
linklist.push_back(link);


// Mate constraint: Concentric2 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_8 , SW name: 3_blade_prop-2 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0911724945380793,-0.111566436084732,-0.033000000000004);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0292800000000041);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.0911724945380793,-0.111566436084732,-0.033000000000004);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0292800000000041);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric2");
linklist.push_back(link);


// Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: 3_blade_prop-3 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0868236642062918,-0.110750943265968,-0.0210000000000045);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0210000000000059);
dA = chrono::ChVector3d(-7.06379399417759e-15,-6.86049799777155e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_5,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0868236642062918,-0.110750943265968,-0.0210000000000045);
dA = chrono::ChVector3d(-7.06379399417759e-15,-6.86049799777155e-15,-1);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0210000000000059);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident3");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_5 , SW name: 3_blade_prop-3 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0845604352035955,-0.111812919951849,-0.0330000000000045);
dA = chrono::ChVector3d(7.06379399417759e-15,6.86049799777155e-15,1);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0292800000000058);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.0845604352035955,-0.111812919951849,-0.0330000000000045);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0292800000000058);
dA = chrono::ChVector3d(7.06379399417759e-15,6.86049799777155e-15,1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: 3_blade_prop-4 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.087337644840296,0.104046999852986,-0.0210000000000042);
cB = chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0210000000000042);
dA = chrono::ChVector3d(-7.06379399417755e-15,-6.86049799777154e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.087337644840296,0.104046999852986,-0.0210000000000042);
dA = chrono::ChVector3d(-7.06379399417755e-15,-6.86049799777154e-15,-1);
cB = chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0210000000000042);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident4");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_2 , SW name: 3_blade_prop-4 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.085081337776099,0.105123604902229,-0.0330000000000043);
dA = chrono::ChVector3d(7.06379399417755e-15,6.86049799777154e-15,1);
cB = chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0292800000000041);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.085081337776099,0.105123604902229,-0.0330000000000043);
cB = chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0292800000000041);
dA = chrono::ChVector3d(7.06379399417755e-15,6.86049799777154e-15,1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_9 , SW name: 3_blade_prop-5 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.0900958894888372,-0.109310129020535,0.027999999999996);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.027999999999996);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_9,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0900958894888372,-0.109310129020535,0.027999999999996);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.027999999999996);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident5");
linklist.push_back(link);


// Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_9 , SW name: 3_blade_prop-5 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0911724945380798,-0.111566436084732,0.0399999999999961);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.0362799999999959);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.0911724945380798,-0.111566436084732,0.0399999999999961);
cB = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.0362799999999959);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric5");
linklist.push_back(link);


// Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_10 , SW name: 3_blade_prop-6 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.091713568651458,0.10763331777204,0.0279999999999945);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0279999999999959);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_10,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident6");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.091713568651458,0.10763331777204,0.0279999999999945);
dA = chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0279999999999959);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident6");
linklist.push_back(link);


// Mate constraint: Concentric6 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_10 , SW name: 3_blade_prop-6 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0906515919655775,0.105370088769343,0.0399999999999945);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0362799999999959);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric6");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.0906515919655775,0.105370088769343,0.0399999999999945);
cB = chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0362799999999959);
dA = chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric6");
linklist.push_back(link);


// Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_7 , SW name: 3_blade_prop-7 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0861579428253412,0.107379911966426,0.0279999999999957);
cB = chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0279999999999958);
dA = chrono::ChVector3d(7.06379399417755e-15,6.86049799777154e-15,1);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_7,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident7");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0861579428253412,0.107379911966426,0.0279999999999957);
dA = chrono::ChVector3d(7.06379399417755e-15,6.86049799777154e-15,1);
cB = chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0279999999999958);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident7");
linklist.push_back(link);


// Mate constraint: Concentric7 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_7 , SW name: 3_blade_prop-7 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0850813377760986,0.105123604902229,0.0399999999999957);
dA = chrono::ChVector3d(-7.06379399417755e-15,-6.86049799777154e-15,-1);
cB = chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0362799999999958);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric7");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.0850813377760986,0.105123604902229,0.0399999999999957);
cB = chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0362799999999958);
dA = chrono::ChVector3d(-7.06379399417755e-15,-6.86049799777154e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric7");
linklist.push_back(link);


// Mate constraint: Coincident8 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_6 , SW name: 3_blade_prop-8 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0834984585177144,-0.109549690949152,0.0279999999999955);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0279999999999942);
dA = chrono::ChVector3d(7.06379399417759e-15,6.86049799777155e-15,1);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_6,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident8");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0834984585177144,-0.109549690949152,0.0279999999999955);
dA = chrono::ChVector3d(7.06379399417759e-15,6.86049799777155e-15,1);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0279999999999942);
dB = chrono::ChVector3d(0,0,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident8");
linklist.push_back(link);


// Mate constraint: Concentric8 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_6 , SW name: 3_blade_prop-8 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.084560435203595,-0.111812919951848,0.0399999999999955);
dA = chrono::ChVector3d(-7.06379399417759e-15,-6.86049799777155e-15,-1);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0362799999999942);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric8");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.084560435203595,-0.111812919951848,0.0399999999999955);
cB = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0362799999999942);
dA = chrono::ChVector3d(-7.06379399417759e-15,-6.86049799777155e-15,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric8");
linklist.push_back(link);


// Mate constraint: Coincident9 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: box_big_200x200x100-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.0880455783809917,0.176778584408747,0.225499999999995);
cB = chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996);
dA = chrono::ChVector3d(-7.06379399417756e-15,1.22464679914735e-16,-1);
dB = chrono::ChVector3d(7.06379399417755e-15,9.95799250102464e-17,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident9");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0880455783809917,0.176778584408747,0.225499999999995);
dA = chrono::ChVector3d(-7.06379399417756e-15,1.22464679914735e-16,-1);
cB = chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996);
dB = chrono::ChVector3d(7.06379399417755e-15,9.95799250102464e-17,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident9");
linklist.push_back(link);


// Mate constraint: Distance2 [MateDistanceDim] type:5 align:0 flip:True
//   Entity 0: C::E name: body_0 , SW name: frame-1 ,  SW ref.type:4 (4)
//   Entity 1: C::E name: body_0 , SW name: box_big_200x200x100-1 ,  SW ref.type:4 (4)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0,0,0);
cB = chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996);
dA = chrono::ChVector3d(0,1,0);
dB = chrono::ChVector3d(-7.06379399417755e-15,1,3.44509284839767e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0.003);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distance2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0,0,0);
dA = chrono::ChVector3d(0,1,0);
cB = chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996);
dB = chrono::ChVector3d(-7.06379399417755e-15,1,3.44509284839767e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distance2");
linklist.push_back(link);


// Mate constraint: Distance1 [MateDistanceDim] type:5 align:0 flip:False
//   Entity 0: C::E name: body_0 , SW name: frame-1 ,  SW ref.type:4 (4)
//   Entity 1: C::E name: body_0 , SW name: box_big_200x200x100-1 ,  SW ref.type:4 (4)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0,0,0);
cB = chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996);
dA = chrono::ChVector3d(1,0,0);
dB = chrono::ChVector3d(1,7.06379399417756e-15,-7.06379399417756e-15);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(-0.003);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distance1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0,0,0);
dA = chrono::ChVector3d(1,0,0);
cB = chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996);
dB = chrono::ChVector3d(1,7.06379399417756e-15,-7.06379399417756e-15);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distance1");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("Coordinate System1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.090651591965577,0.105370088769343,-0.0330000000000055),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("Coordinate System2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0911724945380793,-0.111566436084732,-0.033000000000004),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("Coordinate System3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0845604352035955,-0.111812919951849,-0.0330000000000045),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("Coordinate System4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.085081337776099,0.105123604902229,-0.0330000000000043),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_5 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_5->SetName("Coordinate System5");
body_0->AddMarker(marker_0_5);
marker_0_5->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0911724945380798,-0.111566436084732,0.0399999999999961),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_6 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_6->SetName("Coordinate System6");
body_0->AddMarker(marker_0_6);
marker_0_6->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0906515919655775,0.105370088769343,0.0399999999999945),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_7 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_7->SetName("Coordinate System7");
body_0->AddMarker(marker_0_7);
marker_0_7->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0850813377760986,0.105123604902229,0.0399999999999957),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_8 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_8->SetName("Coordinate System8");
body_0->AddMarker(marker_0_8);
marker_0_8->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.084560435203595,-0.111812919951848,0.0399999999999955),chrono::ChQuaternion<>(1,0,0,0)));


} // end function