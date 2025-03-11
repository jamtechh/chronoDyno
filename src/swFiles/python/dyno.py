# PyChrono model automatically generated using Chrono::SolidWorks add-in
# Assembly: C:\Users\Jamiul Haque\OneDrive - UW-Madison\Design\chronoMotor\Assem1.SLDASM


import pychrono as chrono 
import builtins 

# Some global settings 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'dyno_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0 = chrono.ChBodyAuxRef()
body_0.SetName('SLDW_GROUND')
body_0.SetFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1 = chrono.ChBodyAuxRef()
body_1.SetName('frame-1')
body_1.SetPos(chrono.ChVector3d(0.0100501781487224,0.127271341649174,0.219573424974887))
body_1.SetRot(chrono.ChQuaterniond(1,0,0,0))
body_1.SetMass(1.54162568183963)
body_1.SetInertiaXX(chrono.ChVector3d(0.0295689201124321,0.0458653318996888,0.0735410736970342))
body_1.SetInertiaXY(chrono.ChVector3d(-1.38791051528139e-08,-0.0017310556183721,-2.28813539243213e-08))
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(0.037437693135653,-1.43977620517403e-07,0.0198392235533775),chrono.ChQuaterniond(1,0,0,0)))
body_1.SetFixed(True)

# Visualization shape 
body_1_1_shape = chrono.ChVisualShapeModelFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_1)



# Rigid body part
body_2 = chrono.ChBodyAuxRef()
body_2.SetName('Part1-1')
body_2.SetPos(chrono.ChVector3d(-0.153681408502864,0.232071341649174,0.257256405065421))
body_2.SetRot(chrono.ChQuaterniond(0.344860417986101,-0.344860417986101,0.617309721376921,0.617309721376921))
body_2.SetMass(0.016029213366013)
body_2.SetInertiaXX(chrono.ChVector3d(5.77976073009449e-06,7.96307088073259e-06,9.73060284283223e-06))
body_2.SetInertiaXY(chrono.ChVector3d(2.41692490861642e-06,8.37790436764729e-08,-5.30757027275727e-08))
body_2.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(0.000139083582689285,-0.000107870586674347,0.0275964334240644),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_2)



# Rigid body part
body_3 = chrono.ChBodyAuxRef()
body_3.SetName('GearA-2')
body_3.SetPos(chrono.ChVector3d(-0.153681408502864,0.316571341649174,0.257256405065421))
body_3.SetRot(chrono.ChQuaterniond(0.69219181681608,-0.69219181681608,0.144466220040721,0.144466220040721))
body_3.SetMass(0.000870269104083074)
body_3.SetInertiaXX(chrono.ChVector3d(2.45232834116315e-08,3.48513174025659e-08,2.20932767457664e-08))
body_3.SetInertiaXY(chrono.ChVector3d(5.56795508597199e-09,-1.39935665117364e-23,8.94234922184334e-24))
body_3.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.94233460763258e-18,3.00039026195962e-18,0.000394433254710844),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_3)



# Rigid body part
body_4 = chrono.ChBodyAuxRef()
body_4.SetName('GearB-1')
body_4.SetPos(chrono.ChVector3d(-0.183793091163963,0.320571341649174,0.254366929798653))
body_4.SetRot(chrono.ChQuaterniond(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382))
body_4.SetMass(0.00573763521549478)
body_4.SetInertiaXX(chrono.ChVector3d(1.09268072169601e-06,6.95141740951902e-07,6.03909758681937e-07))
body_4.SetInertiaXY(chrono.ChVector3d(-2.11167099311834e-07,-4.81908280928055e-23,-1.19442225809283e-22))
body_4.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(2.28877170423011e-18,2.90691676901785e-18,0.0025),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChVisualShapeModelFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4.AddVisualShape(body_4_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_4)



# Rigid body part
body_5 = chrono.ChBodyAuxRef()
body_5.SetName('GearD-1')
body_5.SetPos(chrono.ChVector3d(-0.213749821851277,-0.0660286583508263,0.280873424974888))
body_5.SetRot(chrono.ChQuaterniond(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0))
body_5.SetMass(0.00573763521549478)
body_5.SetInertiaXX(chrono.ChVector3d(6.03909758681937e-07,1.18391270396598e-06,6.03909758681937e-07))
body_5.SetInertiaXY(chrono.ChVector3d(-6.68097235284266e-23,1.28759415537667e-22,-3.13275101861351e-24))
body_5.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(2.49936505366921e-18,2.65334179213175e-18,0.0025),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChVisualShapeModelFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5.AddVisualShape(body_5_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_5)



# Rigid body part
body_6 = chrono.ChBodyAuxRef()
body_6.SetName('GearC-1')
body_6.SetPos(chrono.ChVector3d(-0.213749821851278,0.320571341649174,0.280873424974888))
body_6.SetRot(chrono.ChQuaterniond(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627))
body_6.SetMass(0.00573763521549478)
body_6.SetInertiaXX(chrono.ChVector3d(6.34526836838321e-07,1.15329562580959e-06,6.03909758681937e-07))
body_6.SetInertiaXY(chrono.ChVector3d(-1.29694217418744e-07,1.12016171588533e-22,2.96626605990115e-23))
body_6.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(2.28311378076172e-18,2.62662036785091e-18,0.0025),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChVisualShapeModelFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6.AddVisualShape(body_6_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_6)



# Rigid body part
body_7 = chrono.ChBodyAuxRef()
body_7.SetName('GearE-1')
body_7.SetPos(chrono.ChVector3d(-0.183793091163963,-0.0660286583508263,0.254366929798653))
body_7.SetRot(chrono.ChQuaterniond(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918))
body_7.SetMass(0.00573763521549478)
body_7.SetInertiaXX(chrono.ChVector3d(8.1879554426816e-07,9.69026918379755e-07,6.03909758681937e-07))
body_7.SetInertiaXY(chrono.ChVector3d(2.80104422836692e-07,-7.64458818798232e-24,2.01574155774716e-22))
body_7.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(2.38265693439381e-18,2.5104561020208e-18,0.0025),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_7_1_shape = chrono.ChVisualShapeModelFile() 
body_7_1_shape.SetFilename(shapes_dir +'body_7_1.obj') 
body_7.AddVisualShape(body_7_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_7)



# Rigid body part
body_8 = chrono.ChBodyAuxRef()
body_8.SetName('Part2_flywheel-1')
body_8.SetPos(chrono.ChVector3d(-0.213749821851277,0.127271341649174,0.280873424974888))
body_8.SetRot(chrono.ChQuaterniond(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594))
body_8.SetMass(1.02740423420996)
body_8.SetInertiaXX(chrono.ChVector3d(0.00660988825669769,0.00520936075890446,0.0113199638141394))
body_8.SetInertiaXY(chrono.ChVector3d(-0.00536483913451316,-2.9120923437435e-10,2.65996941534389e-10))
body_8.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(8.92766810038497e-10,8.92766809944299e-10,1.29505962482037e-17),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_8_1_shape = chrono.ChVisualShapeModelFile() 
body_8_1_shape.SetFilename(shapes_dir +'body_8_1.obj') 
body_8.AddVisualShape(body_8_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_8)



# Rigid body part
body_9 = chrono.ChBodyAuxRef()
body_9.SetName('Part2_dyno-1')
body_9.SetPos(chrono.ChVector3d(-0.153681408502864,0.0154713416491736,0.257256405065421))
body_9.SetRot(chrono.ChQuaterniond(0.679959230544171,0.679959230544171,0.194050108986773,-0.194050108986773))
body_9.SetMass(0.0195543893524918)
body_9.SetInertiaXX(chrono.ChVector3d(8.64325141424603e-06,6.23841160373614e-06,1.00061631055425e-05))
body_9.SetInertiaXY(chrono.ChVector3d(2.47769437441729e-06,1.3611419451779e-07,-2.06449748431985e-07))
body_9.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(1.76453705511435e-05,0.000529772669645473,0.017804509264634),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_9_1_shape = chrono.ChVisualShapeModelFile() 
body_9_1_shape.SetFilename(shapes_dir +'body_9_1.obj') 
body_9.AddVisualShape(body_9_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_9)



# Rigid body part
body_10 = chrono.ChBodyAuxRef()
body_10.SetName('GearF-1')
body_10.SetPos(chrono.ChVector3d(-0.153681408502864,-0.0620286583508263,0.257256405065421))
body_10.SetRot(chrono.ChQuaterniond(0.682163121105785,0.682163121105785,0.186154441803611,-0.186154441803611))
body_10.SetMass(0.000870269104083074)
body_10.SetInertiaXX(chrono.ChVector3d(2.60120055120133e-08,3.33625953021842e-08,2.20932767457664e-08))
body_10.SetInertiaXY(chrono.ChVector3d(-6.64540463802123e-09,1.43736440830122e-23,1.1419234343844e-23))
body_10.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.98131947984345e-18,2.82217370328136e-18,0.000394433254710844),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_10_1_shape = chrono.ChVisualShapeModelFile() 
body_10_1_shape.SetFilename(shapes_dir +'body_10_1.obj') 
body_10.AddVisualShape(body_10_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_10)




# Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Part1-1 ,  SW ref.type:2 (2)
link_1 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.213749821851277,0.305071341649174,0.280873424974888)
cB = chrono.ChVector3d(-0.153681408502864,0.305071341649174,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(1.150251786806e-16,1,-2.13289118002379e-31)
link_1.Initialize(body_1,body_2,False,cA,cB,dB)
link_1.SetDistance(0)
link_1.SetName("Coincident5")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,0.305071341649174,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.153681408502864,0.305071341649174,0.257256405065421)
dB = chrono.ChVector3d(1.150251786806e-16,1,-2.13289118002379e-31)
link_2.SetFlipped(True)
link_2.Initialize(body_1,body_2,False,cA,cB,dA,dB)
link_2.SetName("Coincident5")
exported_items.append(link_2)


# Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: GearA-2 ,  SW ref.type:2 (2)
link_3 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.153681408502864,0.311571341649174,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.153681408502864,0.311571341649174,0.257256405065421)
dB = chrono.ChVector3d(1.150251786806e-16,1,1.29422492262822e-31)
link_3.SetFlipped(True)
link_3.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_3.SetName("Concentric1")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateGeneric()
link_4.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.153681408502864,0.311571341649174,0.257256405065421)
cB = chrono.ChVector3d(-0.153681408502864,0.311571341649174,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(1.150251786806e-16,1,1.29422492262822e-31)
link_4.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_4.SetName("Concentric1")
exported_items.append(link_4)


# Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: GearC-1 ,  SW ref.type:2 (2)
link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,0.311571341649174,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.213749821851278,0.315571341649174,0.280873424974888)
dB = chrono.ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31)
link_5.SetFlipped(True)
link_5.Initialize(body_1,body_6,False,cA,cB,dA,dB)
link_5.SetName("Concentric3")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.213749821851277,0.311571341649174,0.280873424974888)
cB = chrono.ChVector3d(-0.213749821851278,0.315571341649174,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31)
link_6.Initialize(body_1,body_6,False,cA,cB,dA,dB)
link_6.SetName("Concentric3")
exported_items.append(link_6)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: GearD-1 ,  SW ref.type:2 (2)
link_7 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,-0.0570286583508263,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
cB = chrono.ChVector3d(-0.213749821851277,-0.0610286583508263,0.280873424974888)
dB = chrono.ChVector3d(1.15025178680601e-16,-1,0)
link_7.SetFlipped(True)
link_7.Initialize(body_1,body_5,False,cA,cB,dA,dB)
link_7.SetName("Concentric4")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateGeneric()
link_8.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.213749821851277,-0.0570286583508263,0.280873424974888)
cB = chrono.ChVector3d(-0.213749821851277,-0.0610286583508263,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
dB = chrono.ChVector3d(1.15025178680601e-16,-1,0)
link_8.Initialize(body_1,body_5,False,cA,cB,dA,dB)
link_8.SetName("Concentric4")
exported_items.append(link_8)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: GearF-1 ,  SW ref.type:2 (2)
link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
cB = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
dB = chrono.ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31)
link_9.SetFlipped(True)
link_9.Initialize(body_1,body_10,False,cA,cB,dA,dB)
link_9.SetName("Concentric5")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
cB = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
dB = chrono.ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31)
link_10.Initialize(body_1,body_10,False,cA,cB,dA,dB)
link_10.SetName("Concentric5")
exported_items.append(link_10)


# Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: GearF-1 ,  SW ref.type:2 (2)
link_11 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.213749821851277,-0.0570286583508263,0.280873424974888)
cB = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
dA = chrono.ChVector3d(1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(-1.150251786806e-16,1,-2.82315246648629e-31)
link_11.Initialize(body_1,body_10,False,cA,cB,dB)
link_11.SetDistance(0)
link_11.SetName("Coincident6")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,-0.0570286583508263,0.280873424974888)
dA = chrono.ChVector3d(1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
dB = chrono.ChVector3d(-1.150251786806e-16,1,-2.82315246648629e-31)
link_12.SetFlipped(True)
link_12.Initialize(body_1,body_10,False,cA,cB,dA,dB)
link_12.SetName("Coincident6")
exported_items.append(link_12)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: GearA-2 ,  SW ref.type:2 (2)
link_13 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.213749821851277,0.311571341649174,0.280873424974888)
cB = chrono.ChVector3d(-0.153681408502864,0.311571341649174,0.257256405065421)
dA = chrono.ChVector3d(1.15025178680601e-16,1,0)
dB = chrono.ChVector3d(-1.150251786806e-16,-1,-1.29422492262822e-31)
link_13.Initialize(body_1,body_3,False,cA,cB,dB)
link_13.SetDistance(0)
link_13.SetName("Coincident7")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,0.311571341649174,0.280873424974888)
dA = chrono.ChVector3d(1.15025178680601e-16,1,0)
cB = chrono.ChVector3d(-0.153681408502864,0.311571341649174,0.257256405065421)
dB = chrono.ChVector3d(-1.150251786806e-16,-1,-1.29422492262822e-31)
link_14.SetFlipped(True)
link_14.Initialize(body_1,body_3,False,cA,cB,dA,dB)
link_14.SetName("Coincident7")
exported_items.append(link_14)


# Mate constraint: Coincident8 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: GearA-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: GearC-1 ,  SW ref.type:2 (2)
link_15 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.153681408502864,0.320571341649174,0.257256405065421)
cB = chrono.ChVector3d(-0.213749821851278,0.320571341649174,0.280873424974888)
dA = chrono.ChVector3d(1.150251786806e-16,1,1.29422492262822e-31)
dB = chrono.ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31)
link_15.Initialize(body_3,body_6,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident8")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.153681408502864,0.320571341649174,0.257256405065421)
dA = chrono.ChVector3d(1.150251786806e-16,1,1.29422492262822e-31)
cB = chrono.ChVector3d(-0.213749821851278,0.320571341649174,0.280873424974888)
dB = chrono.ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31)
link_16.Initialize(body_3,body_6,False,cA,cB,dA,dB)
link_16.SetName("Coincident8")
exported_items.append(link_16)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: GearD-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: GearF-1 ,  SW ref.type:2 (2)
link_17 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.213749821851277,-0.0660286583508263,0.280873424974888)
cB = chrono.ChVector3d(-0.153681408502864,-0.0660286583508263,0.257256405065421)
dA = chrono.ChVector3d(1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31)
link_17.Initialize(body_5,body_10,False,cA,cB,dB)
link_17.SetDistance(0)
link_17.SetName("Coincident9")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,-0.0660286583508263,0.280873424974888)
dA = chrono.ChVector3d(1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.153681408502864,-0.0660286583508263,0.257256405065421)
dB = chrono.ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31)
link_18.Initialize(body_5,body_10,False,cA,cB,dA,dB)
link_18.SetName("Coincident9")
exported_items.append(link_18)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: GearB-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: GearC-1 ,  SW ref.type:2 (2)
link_19 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.183793091163963,0.320571341649174,0.254366929798653)
cB = chrono.ChVector3d(-0.213749821851278,0.320571341649174,0.280873424974888)
dA = chrono.ChVector3d(1.150251786806e-16,1,1.63318859284038e-30)
dB = chrono.ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31)
link_19.Initialize(body_4,body_6,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident11")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.183793091163963,0.320571341649174,0.254366929798653)
dA = chrono.ChVector3d(1.150251786806e-16,1,1.63318859284038e-30)
cB = chrono.ChVector3d(-0.213749821851278,0.320571341649174,0.280873424974888)
dB = chrono.ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31)
link_20.Initialize(body_4,body_6,False,cA,cB,dA,dB)
link_20.SetName("Coincident11")
exported_items.append(link_20)


# Mate constraint: Coincident13 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: GearD-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: GearE-1 ,  SW ref.type:2 (2)
link_21 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.213749821851277,-0.0660286583508263,0.280873424974888)
cB = chrono.ChVector3d(-0.183793091163963,-0.0660286583508263,0.254366929798653)
dA = chrono.ChVector3d(1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16)
link_21.Initialize(body_5,body_7,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident13")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,-0.0660286583508263,0.280873424974888)
dA = chrono.ChVector3d(1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.183793091163963,-0.0660286583508263,0.254366929798653)
dB = chrono.ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16)
link_22.Initialize(body_5,body_7,False,cA,cB,dA,dB)
link_22.SetName("Coincident13")
exported_items.append(link_22)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,0.311571341649174,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.213749821851278,-0.0477286583508263,0.280873424974888)
dB = chrono.ChVector3d(1.150251786806e-16,1,-7.53206554000315e-31)
link_23.SetFlipped(True)
link_23.Initialize(body_1,body_8,False,cA,cB,dA,dB)
link_23.SetName("Concentric16")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.213749821851277,0.311571341649174,0.280873424974888)
cB = chrono.ChVector3d(-0.213749821851278,-0.0477286583508263,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(1.150251786806e-16,1,-7.53206554000315e-31)
link_24.Initialize(body_1,body_8,False,cA,cB,dA,dB)
link_24.SetName("Concentric16")
exported_items.append(link_24)


# Mate constraint: Width1 [MateWidth] type:11 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)

# Mate constraint: Concentric18 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: GearE-1 ,  SW ref.type:2 (2)
link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.183793091163963,-0.0570286583508263,0.254366929798653)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
cB = chrono.ChVector3d(-0.183793091163963,-0.0610286583508263,0.254366929798653)
dB = chrono.ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16)
link_25.SetFlipped(True)
link_25.Initialize(body_1,body_7,False,cA,cB,dA,dB)
link_25.SetName("Concentric18")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.183793091163963,-0.0570286583508263,0.254366929798653)
cB = chrono.ChVector3d(-0.183793091163963,-0.0610286583508263,0.254366929798653)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
dB = chrono.ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16)
link_26.Initialize(body_1,body_7,False,cA,cB,dA,dB)
link_26.SetName("Concentric18")
exported_items.append(link_26)


# Mate constraint: Concentric19 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: GearB-1 ,  SW ref.type:2 (2)
link_27 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.183793091163963,0.311571341649174,0.254366929798653)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
cB = chrono.ChVector3d(-0.183793091163963,0.315571341649174,0.254366929798653)
dB = chrono.ChVector3d(1.150251786806e-16,1,1.63318859284038e-30)
link_27.SetFlipped(True)
link_27.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_27.SetName("Concentric19")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateGeneric()
link_28.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.183793091163963,0.311571341649174,0.254366929798653)
cB = chrono.ChVector3d(-0.183793091163963,0.315571341649174,0.254366929798653)
dA = chrono.ChVector3d(-1.15025178680601e-16,-1,0)
dB = chrono.ChVector3d(1.150251786806e-16,1,1.63318859284038e-30)
link_28.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_28.SetName("Concentric19")
exported_items.append(link_28)


# Mate constraint: Concentric20 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)
link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
cB = chrono.ChVector3d(-0.153681408502864,-0.0395286583508263,0.257256405065421)
dB = chrono.ChVector3d(-1.15025274152838e-16,1,-1.45428225327743e-23)
link_29.Initialize(body_1,body_9,False,cA,cB,dA,dB)
link_29.SetName("Concentric20")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-0.153681408502864,-0.0570286583508263,0.257256405065421)
cB = chrono.ChVector3d(-0.153681408502864,-0.0395286583508263,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
dB = chrono.ChVector3d(-1.15025274152838e-16,1,-1.45428225327743e-23)
link_30.Initialize(body_1,body_9,False,cA,cB,dA,dB)
link_30.SetName("Concentric20")
exported_items.append(link_30)


# Mate constraint: Coincident17 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: frame-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)
link_31 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.213749821851277,-0.0505286583508263,0.280873424974888)
cB = chrono.ChVector3d(-0.153681408502864,-0.0505286583508263,0.257256405065421)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
dB = chrono.ChVector3d(1.15025274152838e-16,-1,1.45428225327743e-23)
link_31.Initialize(body_1,body_9,False,cA,cB,dB)
link_31.SetDistance(0)
link_31.SetName("Coincident17")
exported_items.append(link_31)

link_32 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-0.213749821851277,-0.0505286583508263,0.280873424974888)
dA = chrono.ChVector3d(-1.15025178680601e-16,1,0)
cB = chrono.ChVector3d(-0.153681408502864,-0.0505286583508263,0.257256405065421)
dB = chrono.ChVector3d(1.15025274152838e-16,-1,1.45428225327743e-23)
link_32.SetFlipped(True)
link_32.Initialize(body_1,body_9,False,cA,cB,dA,dB)
link_32.SetName("Coincident17")
exported_items.append(link_32)

