# Semantic differences (role-mapped, derived metrics excluded)

| Path | Base | LCAS | Class | Reason |
|---|---|---|---|---|
| `/gazebo` | [] | (absent) | C | Gazebo/ROS2-specific integration; do not copy to Isaac |
| `/gazebo/0` | (absent) | "<gazebo reference=\"rear_left_wheel\"><mu1 value=\"1.0\" /><mu2 value=\"0.5\" /><kp value=\"10000000.0\" /><kd value=\"1.0\" /><fdir1 value=\"1 0 0\" /></gazebo>" | C | Gazebo/ROS2-specific integration; do not copy to Isaac |
| `/gazebo/1` | (absent) | "<gazebo reference=\"rear_right_wheel\"><mu1 value=\"1.0\" /><mu2 value=\"0.5\" /><kp value=\"10000000.0\" /><kd value=\"1.0\" /><fdir1 value=\"1 0 0\" /></gazebo>" | C | Gazebo/ROS2-specific integration; do not copy to Isaac |
| `/gazebo/2` | (absent) | "<gazebo><plugin filename=\"libgazebo_ros2_control.so\" name=\"gazebo_ros2_control\"><parameters>third_party/hunter_robot/hunter_description/config/ackermann_like_controller.yaml</parameters></plugin></gazebo>" | C | Gazebo/ROS2-specific integration; do not copy to Isaac |
| `/joints/chassis_joint/axis_local/0` | (absent) | 0 | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/chassis_joint/axis_local/1` | (absent) | 0 | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/chassis_joint/axis_local/2` | (absent) | 0 | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/chassis_joint/child` | (absent) | "chassis" | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/chassis_joint/dynamics` | (absent) | null | E | Simulation tuning; hardware value unverified |
| `/joints/chassis_joint/limit` | (absent) | null | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/chassis_joint/mimic` | (absent) | null | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/chassis_joint/origin/rpy/0` | (absent) | 0.0 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/chassis_joint/origin/rpy/1` | (absent) | 0.0 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/chassis_joint/origin/rpy/2` | (absent) | 0.0 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/chassis_joint/origin/xyz/0` | (absent) | 0.0 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/chassis_joint/origin/xyz/1` | (absent) | 0.0 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/chassis_joint/origin/xyz/2` | (absent) | 0.37 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/chassis_joint/original_name` | (absent) | "chassis_joint" | D | Naming/structural organization, not a hardware correction |
| `/joints/chassis_joint/parent` | (absent) | "root" | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/chassis_joint/type` | (absent) | "fixed" | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/front_left/axis_local/2` | 1.0 | -1.0 | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/front_steer_joint/axis_local/0` | 0.0 | (absent) | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/front_steer_joint/axis_local/1` | 0.0 | (absent) | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/front_steer_joint/axis_local/2` | 0.0 | (absent) | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/front_steer_joint/child` | "front_steer_link" | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/front_steer_joint/dynamics` | null | (absent) | E | Simulation tuning; hardware value unverified |
| `/joints/front_steer_joint/limit` | null | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/front_steer_joint/mimic` | null | (absent) | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/front_steer_joint/origin/rpy/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/front_steer_joint/origin/rpy/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/front_steer_joint/origin/rpy/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/front_steer_joint/origin/xyz/0` | 0.44792 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/front_steer_joint/origin/xyz/1` | -1.1431e-05 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/front_steer_joint/origin/xyz/2` | -0.121 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/front_steer_joint/original_name` | "front_steer_joint" | (absent) | D | Naming/structural organization, not a hardware correction |
| `/joints/front_steer_joint/parent` | "chassis" | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/front_steer_joint/type` | "fixed" | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/rear_left/axis_local/2` | 1.0 | -1.0 | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/rear_left/dynamics` | null | (absent) | E | Simulation tuning; hardware value unverified |
| `/joints/rear_left/dynamics/damping` | (absent) | "0.2" | E | Simulation tuning; hardware value unverified |
| `/joints/rear_left/dynamics/friction` | (absent) | "15" | E | Simulation tuning; hardware value unverified |
| `/joints/rear_left/limit/effort` | "100" | "10.0" | E | Simulation tuning; hardware value unverified |
| `/joints/rear_left/limit/velocity` | "1" | "10.0" | B | Permits official straight speed; exact 10 rad/s hardware limit is still unverified |
| `/joints/rear_left/origin/rpy/0` | 1.5708 | 1.5707963267948966 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_left/origin/xyz/1` | 0.29249 | 0.2925 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_left/original_name` | "left_rear_joint" | "rear_left_wheel_joint" | D | Naming/structural organization, not a hardware correction |
| `/joints/rear_right/dynamics` | null | (absent) | E | Simulation tuning; hardware value unverified |
| `/joints/rear_right/dynamics/damping` | (absent) | "0.2" | E | Simulation tuning; hardware value unverified |
| `/joints/rear_right/dynamics/friction` | (absent) | "15" | E | Simulation tuning; hardware value unverified |
| `/joints/rear_right/limit/effort` | "100" | "10.0" | E | Simulation tuning; hardware value unverified |
| `/joints/rear_right/limit/velocity` | "1" | "10.0" | B | Permits official straight speed; exact 10 rad/s hardware limit is still unverified |
| `/joints/rear_right/mimic` | null | (absent) | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/rear_right/mimic/joint` | (absent) | "rear_left_wheel_joint" | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/rear_right/mimic/multiplier` | (absent) | "1.0" | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/rear_right/mimic/offset` | (absent) | "0.0" | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/rear_right/origin/rpy/0` | 1.5708 | 1.5707963267948966 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_right/origin/rpy/1` | 0.0026575 | 0.0 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_right/origin/xyz/0` | -0.28018 | -0.28 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_right/origin/xyz/1` | -0.29251 | -0.2925 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_right/original_name` | "right_rear_joint" | "rear_right_wheel_joint" | D | Naming/structural organization, not a hardware correction |
| `/joints/rear_wheel_joint/axis_local/0` | 0.0 | (absent) | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/rear_wheel_joint/axis_local/1` | 0.0 | (absent) | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/rear_wheel_joint/axis_local/2` | 0.0 | (absent) | D | Wheel sign convention standardized; physical axle direction already transverse |
| `/joints/rear_wheel_joint/child` | "rear_wheel_link" | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/rear_wheel_joint/dynamics` | null | (absent) | E | Simulation tuning; hardware value unverified |
| `/joints/rear_wheel_joint/limit` | null | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/rear_wheel_joint/mimic` | null | (absent) | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/rear_wheel_joint/origin/rpy/0` | 0.0032313 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_wheel_joint/origin/rpy/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_wheel_joint/origin/rpy/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_wheel_joint/origin/xyz/0` | -0.42408 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_wheel_joint/origin/xyz/1` | -1.1431e-05 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_wheel_joint/origin/xyz/2` | -0.085581 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/rear_wheel_joint/original_name` | "rear_wheel_joint" | (absent) | D | Naming/structural organization, not a hardware correction |
| `/joints/rear_wheel_joint/parent` | "chassis" | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/rear_wheel_joint/type` | "fixed" | (absent) | D | Structural addition/removal; see full diff and report for adoption decision |
| `/joints/steer_left/axis_local/1` | 0.0 | 1.0 | A | Vertical steering correction supported by transforms and AgileX Gazebo |
| `/joints/steer_left/axis_local/2` | 1.0 | 0.0 | A | Vertical steering correction supported by transforms and AgileX Gazebo |
| `/joints/steer_left/dynamics` | null | (absent) | E | Simulation tuning; hardware value unverified |
| `/joints/steer_left/dynamics/damping` | (absent) | "0.2" | E | Simulation tuning; hardware value unverified |
| `/joints/steer_left/limit/effort` | "100" | "10.0" | E | Simulation tuning; hardware value unverified |
| `/joints/steer_left/limit/lower` | "0" | "-0.461" | E | Unlocking is an A improvement, but this exact 0.461 individual bound is unverified |
| `/joints/steer_left/limit/upper` | "0" | "0.461" | E | Unlocking is an A improvement, but this exact 0.461 individual bound is unverified |
| `/joints/steer_left/origin/rpy/0` | 1.5708 | 1.5707963267948966 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/steer_left/origin/xyz/1` | 0.29199 | 0.2925 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/steer_left/origin/xyz/2` | -0.1955 | -0.19558 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/steer_left/original_name` | "front_steer_left_joint" | "front_left_steering_joint" | D | Naming/structural organization, not a hardware correction |
| `/joints/steer_right/axis_local/1` | 0.0 | 1.0 | A | Vertical steering correction supported by transforms and AgileX Gazebo |
| `/joints/steer_right/axis_local/2` | -1.0 | 0.0 | A | Vertical steering correction supported by transforms and AgileX Gazebo |
| `/joints/steer_right/dynamics` | null | (absent) | E | Simulation tuning; hardware value unverified |
| `/joints/steer_right/dynamics/damping` | (absent) | "0.2" | E | Simulation tuning; hardware value unverified |
| `/joints/steer_right/limit/effort` | "100" | "10.0" | E | Simulation tuning; hardware value unverified |
| `/joints/steer_right/limit/lower` | "0" | "-0.461" | E | Unlocking is an A improvement, but this exact 0.461 individual bound is unverified |
| `/joints/steer_right/limit/upper` | "0" | "0.461" | E | Unlocking is an A improvement, but this exact 0.461 individual bound is unverified |
| `/joints/steer_right/mimic` | null | (absent) | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/steer_right/mimic/joint` | (absent) | "front_left_steering_joint" | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/steer_right/mimic/multiplier` | (absent) | "1.0" | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/steer_right/mimic/offset` | (absent) | "0.0" | E | 1:1 coupling is not exact Ackermann; do not adopt |
| `/joints/steer_right/origin/rpy/0` | 1.5708 | 1.5707963267948966 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/steer_right/origin/xyz/1` | -0.29201 | -0.2925 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/steer_right/origin/xyz/2` | -0.1955 | -0.19558 | E | Frame offset/symmetrization not established from official dimensions |
| `/joints/steer_right/original_name` | "front_steer_right_joint" | "front_right_steering_joint" | D | Naming/structural organization, not a hardware correction |
| `/links/chassis/collision/0/geometry/filename` | "package://hunter2_base/meshes/base_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/chassis.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/chassis/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/base_link.STL" | "third_party/hunter_robot/hunter_description/meshes/chassis.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/chassis/inertial/mass` | 6.02945169536679 | 72.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/chassis/original_name` | "base_link" | "chassis" | D | Naming/structural organization, not a hardware correction |
| `/links/chassis/visual/0/geometry/filename` | "package://hunter2_base/meshes/base_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/chassis.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/chassis/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.792156862745098 0.819607843137255 0.933333333333333 1\" />\n      </material>" | null | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/chassis/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/base_link.STL" | "third_party/hunter_robot/hunter_description/meshes/chassis.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_left/collision/0/geometry/filename` | "package://hunter2_base/meshes/front_left_wheel_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/front_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_left/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_left_wheel_link.STL" | "third_party/hunter_robot/hunter_description/meshes/front_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_left/inertial/mass` | 7.7007 | 7.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_left/inertial/tensor/ixy` | -3.8745e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_left/inertial/tensor/ixz` | 2.2753e-07 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_left/inertial/tensor/iyz` | -4.88e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_left/original_name` | "front_left_wheel_link" | "front_left_wheel" | D | Naming/structural organization, not a hardware correction |
| `/links/front_left/visual/0/geometry/filename` | "package://hunter2_base/meshes/front_left_wheel_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/front_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_left/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.79216 0.81961 0.93333 1\" />\n      </material>" | null | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_left/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_left_wheel_link.STL" | "third_party/hunter_robot/hunter_description/meshes/front_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_right/collision/0/geometry/filename` | "package://hunter2_base/meshes/front_right_wheel_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/front_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_right/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_right_wheel_link.STL" | "third_party/hunter_robot/hunter_description/meshes/front_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_right/inertial/mass` | 7.5613 | 7.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/origin/xyz/0` | -9.1148e-08 | -6.5774e-06 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/origin/xyz/1` | 1.1284e-07 | 2.0242e-05 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/origin/xyz/2` | 0.0095049 | -0.0096386 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/tensor/ixx` | 0.063626 | 0.063724 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/tensor/ixy` | -6.4092e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/tensor/ixz` | -2.0164e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/tensor/iyy` | 0.063626 | 0.063724 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/tensor/iyz` | 6.4721e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/inertial/tensor/izz` | 0.11054 | 0.11072 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_right/original_name` | "front_right_wheel_link" | "front_right_wheel" | D | Naming/structural organization, not a hardware correction |
| `/links/front_right/visual/0/geometry/filename` | "package://hunter2_base/meshes/front_right_wheel_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/front_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_right/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.79216 0.81961 0.93333 1\" />\n      </material>" | null | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_right/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_right_wheel_link.STL" | "third_party/hunter_robot/hunter_description/meshes/front_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/collision/0/geometry/filename` | "package://hunter2_base/meshes/front_steer_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/collision/0/geometry_type` | "mesh" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/collision/0/material_xml` | null | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/collision/0/origin/rpy/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/collision/0/origin/rpy/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/collision/0/origin/rpy/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/collision/0/origin/xyz/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/collision/0/origin/xyz/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/collision/0/origin/xyz/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_steer_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/inertial/mass` | 0.0049179 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/origin/rpy/0` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/origin/rpy/1` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/origin/rpy/2` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/origin/xyz/0` | 0.049836 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/origin/xyz/1` | 8.1046e-15 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/origin/xyz/2` | 0.017912 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/tensor/ixx` | 1.6846e-07 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/tensor/ixy` | 7.6396e-23 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/tensor/ixz` | -1.0974e-08 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/tensor/iyy` | 7.549e-07 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/tensor/iyz` | -1.8957e-22 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/inertial/tensor/izz` | 7.334e-07 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/front_steer_link/original_name` | "front_steer_link" | (absent) | D | Naming/structural organization, not a hardware correction |
| `/links/front_steer_link/visual/0/geometry/filename` | "package://hunter2_base/meshes/front_steer_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/visual/0/geometry_type` | "mesh" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.79216 0.81961 0.93333 1\" />\n      </material>" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/front_steer_link/visual/0/origin/rpy/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/visual/0/origin/rpy/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/visual/0/origin/rpy/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/visual/0/origin/xyz/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/visual/0/origin/xyz/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/visual/0/origin/xyz/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/front_steer_link/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_steer_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_left/collision/0/geometry/filename` | "package://hunter2_base/meshes/left_rear_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/rear_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_left/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/left_rear_link.STL" | "third_party/hunter_robot/hunter_description/meshes/rear_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_left/inertial/mass` | 7.6692 | 7.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/origin/xyz/0` | -8.826e-05 | -6.5774e-06 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/origin/xyz/1` | 6.4342e-05 | 2.0242e-05 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/origin/xyz/2` | -0.0096491 | -0.0096386 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/tensor/ixx` | 0.063699 | 0.063724 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/tensor/ixy` | -2.542e-07 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/tensor/ixz` | 2.638e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/tensor/iyy` | 0.063699 | 0.063724 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/tensor/iyz` | -5.9934e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/inertial/tensor/izz` | 0.11067 | 0.11072 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_left/original_name` | "left_rear_link" | "rear_left_wheel" | D | Naming/structural organization, not a hardware correction |
| `/links/rear_left/visual/0/geometry/filename` | "package://hunter2_base/meshes/left_rear_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/rear_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_left/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.79216 0.81961 0.93333 1\" />\n      </material>" | null | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_left/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/left_rear_link.STL" | "third_party/hunter_robot/hunter_description/meshes/rear_left_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_right/collision/0/geometry/filename` | "package://hunter2_base/meshes/right_rear_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/rear_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_right/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/right_rear_link.STL" | "third_party/hunter_robot/hunter_description/meshes/rear_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_right/inertial/mass` | 7.7133 | 7.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/origin/xyz/0` | 8.6344e-05 | -6.5774e-06 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/origin/xyz/1` | 7.4488e-05 | 2.0242e-05 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/origin/xyz/2` | 0.0097824 | -0.0096386 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/tensor/ixx` | 0.06373 | 0.063724 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/tensor/ixy` | 2.5408e-07 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/tensor/ixz` | 2.6379e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/tensor/iyy` | 0.06373 | 0.063724 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/tensor/iyz` | 5.9941e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/inertial/tensor/izz` | 0.11073 | 0.11072 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_right/original_name` | "right_rear_link" | "rear_right_wheel" | D | Naming/structural organization, not a hardware correction |
| `/links/rear_right/visual/0/geometry/filename` | "package://hunter2_base/meshes/right_rear_link.STL" | "file://third_party/hunter_robot/hunter_description/meshes/rear_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_right/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.79216 0.81961 0.93333 1\" />\n      </material>" | null | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_right/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/right_rear_link.STL" | "third_party/hunter_robot/hunter_description/meshes/rear_right_wheel.dae" | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/collision/0/geometry/filename` | "package://hunter2_base/meshes/rear_wheel_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/collision/0/geometry_type` | "mesh" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/collision/0/material_xml` | null | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/collision/0/origin/rpy/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/collision/0/origin/rpy/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/collision/0/origin/rpy/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/collision/0/origin/xyz/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/collision/0/origin/xyz/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/collision/0/origin/xyz/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/rear_wheel_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/inertial/mass` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/origin/rpy/0` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/origin/rpy/1` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/origin/rpy/2` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/origin/xyz/0` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/origin/xyz/1` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/origin/xyz/2` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/tensor/ixx` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/tensor/ixy` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/tensor/ixz` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/tensor/iyy` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/tensor/iyz` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/inertial/tensor/izz` | 0.0 | (absent) | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/rear_wheel_link/original_name` | "rear_wheel_link" | (absent) | D | Naming/structural organization, not a hardware correction |
| `/links/rear_wheel_link/visual/0/geometry/filename` | "package://hunter2_base/meshes/rear_wheel_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/visual/0/geometry_type` | "mesh" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"1 1 1 1\" />\n      </material>" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/rear_wheel_link/visual/0/origin/rpy/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/visual/0/origin/rpy/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/visual/0/origin/rpy/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/visual/0/origin/xyz/0` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/visual/0/origin/xyz/1` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/visual/0/origin/xyz/2` | 0.0 | (absent) | E | Frame offset/symmetrization not established from official dimensions |
| `/links/rear_wheel_link/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/rear_wheel_link.STL" | (absent) | D | DAE/material/triangle count change; shape fidelity still needs checking |
| `/links/root/collision` | (absent) | [] | D | Structural addition/removal; see full diff and report for adoption decision |
| `/links/root/inertial` | (absent) | null | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/root/original_name` | (absent) | "base_link" | D | Naming/structural organization, not a hardware correction |
| `/links/root/visual` | (absent) | [] | D | Structural addition/removal; see full diff and report for adoption decision |
| `/links/steer_left/collision` | (absent) | [] | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/geometry/filename` | "package://hunter2_base/meshes/front_steer_left_link.STL" | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/geometry_type` | "mesh" | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/material_xml` | null | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/origin/rpy/0` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/origin/rpy/1` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/origin/rpy/2` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/origin/xyz/0` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/origin/xyz/1` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/origin/xyz/2` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_steer_left_link.STL" | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_left/inertial/mass` | 7.8526 | 0.005 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/origin/xyz/0` | -0.012614 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/origin/xyz/1` | 1.8116e-05 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/origin/xyz/2` | 0.0023132 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/tensor/ixx` | 0.063827 | 0.1 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/tensor/ixy` | 1.5154e-07 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/tensor/ixz` | 3.1921e-07 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/tensor/iyy` | 0.063827 | 0.1 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/tensor/iyz` | 2.9854e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/inertial/tensor/izz` | 0.11091 | 0.1 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_left/original_name` | "front_steer_left_link" | "front_left_steering_link" | D | Naming/structural organization, not a hardware correction |
| `/links/steer_left/visual/0/geometry/filename` | "package://hunter2_base/meshes/front_steer_left_link.STL" | (absent) | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_left/visual/0/geometry/length` | (absent) | "0.01" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_left/visual/0/geometry/radius` | (absent) | "0.01" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_left/visual/0/geometry_type` | "mesh" | "cylinder" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_left/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.79216 0.81961 0.93333 1\" />\n      </material>" | "<material name=\"invisible\"><color rgba=\"0.0 0.0 0.0 0.0\" /></material>" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_left/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_steer_left_link.STL" | (absent) | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_right/collision` | (absent) | [] | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/geometry/filename` | "package://hunter2_base/meshes/front_steer_right_link.STL" | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/geometry_type` | "mesh" | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/material_xml` | null | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/origin/rpy/0` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/origin/rpy/1` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/origin/rpy/2` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/origin/xyz/0` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/origin/xyz/1` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/origin/xyz/2` | 0.0 | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/collision/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_steer_right_link.STL" | (absent) | A | Removes duplicated wheel collider |
| `/links/steer_right/inertial/mass` | 7.696 | 0.005 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/origin/xyz/0` | -8.9374e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/origin/xyz/1` | 1.0997e-07 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/origin/xyz/2` | 0.0095958 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/tensor/ixx` | 0.063704 | 0.1 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/tensor/ixy` | -6.3892e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/tensor/ixz` | -2.0159e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/tensor/iyy` | 0.063704 | 0.1 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/tensor/iyz` | 6.4721e-08 | 0.0 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/inertial/tensor/izz` | 0.11068 | 0.1 | E | LCAS mass/COM/inertia lacks per-link manufacturer validation |
| `/links/steer_right/original_name` | "front_steer_right_link" | "front_right_steering_link" | D | Naming/structural organization, not a hardware correction |
| `/links/steer_right/visual/0/geometry/filename` | "package://hunter2_base/meshes/front_steer_right_link.STL" | (absent) | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_right/visual/0/geometry/length` | (absent) | "0.01" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_right/visual/0/geometry/radius` | (absent) | "0.01" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_right/visual/0/geometry_type` | "mesh" | "cylinder" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_right/visual/0/material_xml` | "<material name=\"\">\n        <color rgba=\"0.79216 0.81961 0.93333 1\" />\n      </material>" | "<material name=\"invisible\"><color rgba=\"0.0 0.0 0.0 0.0\" /></material>" | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/links/steer_right/visual/0/resolved_mesh` | "third_party/ugv_gazebo_sim/hunter/hunter2_base/meshes/front_steer_right_link.STL" | (absent) | A | Removes duplicated wheel visual; exact marker size is not a hardware value |
| `/ros2_control` | [] | (absent) | C | Gazebo/ROS2-specific integration; do not copy to Isaac |
| `/ros2_control/0` | (absent) | "<ros2_control name=\"GazeboSystem\" type=\"system\"><hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware><joint name=\"front_left_steering_joint\"><command_interface name=\"position\" /><state_interface name=\"position\" /></joint><joint name=\"front_right_steering_joint\"><param name=\"mimic\">front_left_steering_joint</param><param name=\"multiplier\">1</param><state_interface name=\"position\" /></joint><joint name=\"rear_left_wheel_joint\"><command_interface name=\"velocity\" /><state_interface name=\"velocity\" /><state_interface name=\"position\" /></joint><joint name=\"rear_right_wheel_joint\"><param name=\"mimic\">rear_left_wheel_joint</param><param name=\"multiplier\">1</param><state_interface name=\"velocity\" /><state_interface name=\"position\" /></joint><joint name=\"front_right_wheel_joint\"><state_interface name=\"velocity\" /><state_interface name=\"position\" /></joint><joint name=\"front_left_wheel_joint\"><state_interface name=\"velocity\" /><state_interface name=\"position\" /></joint></ros2_control>" | C | Gazebo/ROS2-specific integration; do not copy to Isaac |
