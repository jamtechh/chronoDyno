if(NOT ENABLE_MODULE_IRRLICHT AND NOT ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List of all MBS demos

# Add demos that can work with either Irrlicht or VSG
set(DEMOS
  # demo_MBS_aux_ref
  # demo_MBS_ballSMC
  demo_MBS_bricks
  dominos
  # demo_MBS_callbackNSC
  # demo_MBS_callbackSMC
  # demo_MBS_collisionNSC
  # demo_MBS_collisionSMC
  # demo_MBS_earthquake
  # demo_MBS_friction
  # demo_MBS_motors
  # demo_MBS_spring  
  # demo_MBS_external_dynamics
  # demo_MBS_hydraulic_actuator
  # demo_MBS_hydraulic_crane
  # demo_MBS_hydraulic_crane_cosim
)

# Add demos that can only work with Irrlicht
if(ENABLE_MODULE_IRRLICHT)
    set(DEMOS ${DEMOS}
      # demo_MBS_cohesion
      # demo_MBS_conveyor
      # demo_MBS_feeder
      # demo_MBS_crank
      # demo_MBS_forklift
      # demo_MBS_fourbar
      demo_MBS_gears
      # demo_MBS_mecanum
      # demo_MBS_pendulum
      # demo_MBS_suspension
      # demo_MBS_tracks
      # demo_MBS_convergence
      # demo_MBS_collision_trimesh
      # demo_MBS_collision_2d
      # demo_MBS_emitter
      # demo_MBS_emit_cluster
      # demo_MBS_emit_processor
      # demo_MBS_rev_sph
      # demo_MBS_rev_trans
      # demo_MBS_rev_limits
      # demo_MBS_rev_reactions
      # demo_MBS_rot_spring
      # demo_MBS_ujoint
      # demo_MBS_planar
      # demo_MBS_link_bushing
      # demo_MBS_prismatic_limits
      # demo_MBS_prismatic_force
      # demo_MBS_report_collisions
      # demo_MBS_imposed_motion
      # demo_MBS_custom_contact
    )

    if(THRUST_FOUND)
      set(DEMOS ${DEMOS}
        demo_MBS_collision_object
      )
    endif()
endif()

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CH_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHT_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  include_directories(${CH_MKL_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_VSG)
  include_directories(${CH_VSG_INCLUDES})
  list(APPEND LIBS "ChronoEngine_vsg")
endif()

#--------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

#--------------------------------------------------------------

# Add executables

message(STATUS "Demo programs for MBS module...")

foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBS})  

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()

if(ENABLE_MODULE_PARDISO_MKL)
  set(PROGRAM demo_MBS_link_mate)
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  if(MSVC)
    target_compile_options(${PROGRAM} PRIVATE /bigobj)
  endif()
  target_link_libraries(${PROGRAM} ${LIBS})  

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endif()
