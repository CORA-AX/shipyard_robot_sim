# 자동 산출 상세표

단위: m, rad, kg, kg·m², N·m, rad/s. `—`는 미지정이며 0과 다르다.

## AgileX Base

총 mass **52.2274695954 kg**; links 9, joints 8.

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Local axis | Base axis (q=0) | Limit lower/upper/velocity/effort | Mimic |
|---|---|---|---|---|---|---|---|---|
| front_steer_left_joint | revolute | base_link → front_steer_left_link | 0.37142 0.29199 -0.1955 | 1.5708 0 0 | 0 0 1 | 0 -1 -3.6732051e-06 | 0 / 0 / 1 / 100 | — |
| front_left_wheel_joint | continuous | front_steer_left_link → front_left_wheel_link | 0 0 0 | 0 0 0 | 0 0 1 | 0 -1 -3.6732051e-06 | — / — / — / — | — |
| front_steer_right_joint | revolute | base_link → front_steer_right_link | 0.37142 -0.29201 -0.1955 | 1.5708 0 0 | 0 0 -1 | 0 1 3.6732051e-06 | 0 / 0 / 1 / 100 | — |
| front_right_wheel_joint | continuous | front_steer_right_link → front_right_wheel_link | 0 0 0 | 0 0 0 | 0 0 -1 | 0 1 3.6732051e-06 | — / — / — / — | — |
| left_rear_joint | continuous | base_link → left_rear_link | -0.28 0.29249 -0.19558 | 1.5708 0 0 | 0 0 1 | 0 -1 -3.6732051e-06 | — / — / 1 / 100 | — |
| right_rear_joint | continuous | base_link → right_rear_link | -0.28018 -0.29251 -0.19558 | 1.5708 0.0026575 0 | 0 0 -1 | 9.76153107e-09 1 3.67319213e-06 | — / — / 1 / 100 | — |
| front_steer_joint | fixed | base_link → front_steer_link | 0.44792 -1.1431e-05 -0.121 | 0 0 0 | 0 0 0 | 0 0 0 | — / — / — / — | — |
| rear_wheel_joint | fixed | base_link → rear_wheel_link | -0.42408 -1.1431e-05 -0.085581 | 0.0032313 0 0 | 0 0 0 | 0 0 0 | — / — / — / — | — |

| Link | Mass | COM xyz / rpy (link frame) | Inertia [ixx,iyy,izz; ixy,ixz,iyz] |
|---|---|---|---|
| base_link | 6.02945169537 | 0.0266989328 0.000409203501 -0.135901125 / 0 0 0 | 0.033237096 0.111072002 0.126608628 -1.01315049e-06 -0.00188859201 -5.13308151e-07 |
| front_steer_left_link | 7.8526 | -0.012614 1.8116e-05 0.0023132 / 0 0 0 | 0.063827 0.063827 0.11091 1.5154e-07 3.1921e-07 2.9854e-08 |
| front_left_wheel_link | 7.7007 | -6.5774e-06 2.0242e-05 -0.0096386 / 0 0 0 | 0.063724 0.063724 0.11072 -3.8745e-08 2.2753e-07 -4.88e-08 |
| front_steer_right_link | 7.696 | -8.9374e-08 1.0997e-07 0.0095958 / 0 0 0 | 0.063704 0.063704 0.11068 -6.3892e-08 -2.0159e-08 6.4721e-08 |
| front_right_wheel_link | 7.5613 | -9.1148e-08 1.1284e-07 0.0095049 / 0 0 0 | 0.063626 0.063626 0.11054 -6.4092e-08 -2.0164e-08 6.4721e-08 |
| left_rear_link | 7.6692 | -8.826e-05 6.4342e-05 -0.0096491 / 0 0 0 | 0.063699 0.063699 0.11067 -2.542e-07 2.638e-08 -5.9934e-08 |
| right_rear_link | 7.7133 | 8.6344e-05 7.4488e-05 0.0097824 / 0 0 0 | 0.06373 0.06373 0.11073 2.5408e-07 2.6379e-08 5.9941e-08 |
| front_steer_link | 0.0049179 | 0.049836 8.1046e-15 0.017912 / 0 0 0 | 1.6846e-07 7.549e-07 7.334e-07 7.6396e-23 -1.0974e-08 -1.8957e-22 |
| rear_wheel_link | 0 | 0 0 0 / 0 0 0 | 0 0 0 0 0 0 |

| Link | Visual geometry | Collision geometry |
|---|---|---|
| base_link | mesh: {"filename": "package://hunter2_base/meshes/base_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/base_link.STL"} |
| front_steer_left_link | mesh: {"filename": "package://hunter2_base/meshes/front_steer_left_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/front_steer_left_link.STL"} |
| front_left_wheel_link | mesh: {"filename": "package://hunter2_base/meshes/front_left_wheel_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/front_left_wheel_link.STL"} |
| front_steer_right_link | mesh: {"filename": "package://hunter2_base/meshes/front_steer_right_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/front_steer_right_link.STL"} |
| front_right_wheel_link | mesh: {"filename": "package://hunter2_base/meshes/front_right_wheel_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/front_right_wheel_link.STL"} |
| left_rear_link | mesh: {"filename": "package://hunter2_base/meshes/left_rear_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/left_rear_link.STL"} |
| right_rear_link | mesh: {"filename": "package://hunter2_base/meshes/right_rear_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/right_rear_link.STL"} |
| front_steer_link | mesh: {"filename": "package://hunter2_base/meshes/front_steer_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/front_steer_link.STL"} |
| rear_wheel_link | mesh: {"filename": "package://hunter2_base/meshes/rear_wheel_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/rear_wheel_link.STL"} |

## AgileX Gazebo

총 mass **67.2323874954 kg**; links 9, joints 8.

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Local axis | Base axis (q=0) | Limit lower/upper/velocity/effort | Mimic |
|---|---|---|---|---|---|---|---|---|
| front_steer_left_joint | revolute | base_link → front_steer_left_link | 0.37142 0.29199 -0.1955 | 1.5708 0 0 | 0 1 0 | 0 -3.6732051e-06 1 | -0.69 / 0.69 / 0 / 0 | — |
| front_left_wheel_joint | continuous | front_steer_left_link → front_left_wheel_link | 0 0 0 | 0 0 0 | 0 0 -1 | 0 1 3.6732051e-06 | — / — / — / — | — |
| front_steer_right_joint | revolute | base_link → front_steer_right_link | 0.37142 -0.29201 -0.1955 | 1.5708 0 0 | 0 1 0 | 0 -3.6732051e-06 1 | -0.69 / 0.69 / 0 / 0 | — |
| front_right_wheel_joint | continuous | front_steer_right_link → front_right_wheel_link | 0 0 0 | 0 0 0 | 0 0 -1 | 0 1 3.6732051e-06 | — / — / — / — | — |
| left_rear_joint | continuous | base_link → left_rear_link | -0.28 0.29249 -0.19558 | 1.5708 0 0 | 0 0 -1 | 0 1 3.6732051e-06 | — / — / — / — | — |
| right_rear_joint | continuous | base_link → right_rear_link | -0.28018 -0.29251 -0.19558 | 1.5708 0.0026575 0 | 0 0 -1 | 9.76153107e-09 1 3.67319213e-06 | — / — / — / — | — |
| front_steer_joint | revolute | base_link → front_steer_link | 0.44792 -1.1431e-05 -0.121 | 0 0 0 | 0 0 1 | 0 0 1 | -0.72 / 0.72 / 0 / 0 | — |
| rear_wheel_joint | continuous | base_link → rear_wheel_link | -0.42408 -1.1431e-05 -0.085581 | 0.0032313 0 0 | 0 0 1 | 0 -0.00323129438 0.999994779 | — / — / — / — | — |

| Link | Mass | COM xyz / rpy (link frame) | Inertia [ixx,iyy,izz; ixy,ixz,iyz] |
|---|---|---|---|
| base_link | 21.0294516954 | 0.0266989328 0.000409203501 -0.135901125 / 0 0 0 | 0.033237096 0.111072002 0.126608628 -1.01315049e-06 -0.00188859201 -5.13308151e-07 |
| front_steer_left_link | 7.8526 | -0.012614 1.8116e-05 0.0023132 / 0 0 0 | 0.063827 0.063827 0.11091 0 0 0 |
| front_left_wheel_link | 7.7007 | -6.5774e-06 2.0242e-05 -0.0096386 / 0 0 0 | 0.063724 0.063724 0.11072 0 0 0 |
| front_steer_right_link | 7.696 | -8.9374e-08 1.0997e-07 0.0095958 / 0 0 0 | 0.063704 0.063704 0.11068 0 0 0 |
| front_right_wheel_link | 7.5613 | -9.1148e-08 1.1284e-07 0.0095049 / 0 0 0 | 0.063626 0.063626 0.11054 0 0 0 |
| left_rear_link | 7.6692 | -8.826e-05 6.4342e-05 -0.0096491 / 0 0 0 | 0.063699 0.063699 0.11067 0 0 0 |
| right_rear_link | 7.7133 | 8.6344e-05 7.4488e-05 0.0097824 / 0 0 0 | 0.06373 0.06373 0.11073 0 0 0 |
| front_steer_link | 0.0049179 | 0.049836 8.1046e-15 0.017912 / 0 0 0 | 1.6846e-07 7.549e-07 7.334e-07 7.6396e-23 -1.0974e-08 -1.8957e-22 |
| rear_wheel_link | 0.0049179 | -0.02 0.0031269 -0.12735 / 0 0 0 | 1.1821e-06 9.5276e-07 1.0913e-06 9.6965e-13 -3.1553e-12 1.2428e-08 |

| Link | Visual geometry | Collision geometry |
|---|---|---|
| base_link | mesh: {"filename": "package://hunter2_base/meshes/base_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/base_link.STL"} |
| front_steer_left_link | cylinder: {"length": "0.001", "radius": "0.005"} | cylinder: {"length": "0.001", "radius": "0.005"} |
| front_left_wheel_link | mesh: {"filename": "package://hunter2_base/meshes/front_left_wheel_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/front_left_wheel_link.STL"} |
| front_steer_right_link | cylinder: {"length": "0.001", "radius": "0.005"} | cylinder: {"length": "0.001", "radius": "0.005"} |
| front_right_wheel_link | mesh: {"filename": "package://hunter2_base/meshes/front_left_wheel_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/front_right_wheel_link.STL"} |
| left_rear_link | mesh: {"filename": "package://hunter2_base/meshes/left_rear_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/left_rear_link.STL"} |
| right_rear_link | mesh: {"filename": "package://hunter2_base/meshes/right_rear_link.STL"} | mesh: {"filename": "package://hunter2_base/meshes/right_rear_link.STL"} |
| front_steer_link | cylinder: {"length": "0.001", "radius": "0.005"} | cylinder: {"length": "0.001", "radius": "0.005"} |
| rear_wheel_link | cylinder: {"length": "0.001", "radius": "0.005"} | cylinder: {"length": "0.001", "radius": "0.005"} |

## LCAS

총 mass **100.01 kg**; links 8, joints 7.

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Local axis | Base axis (q=0) | Limit lower/upper/velocity/effort | Mimic |
|---|---|---|---|---|---|---|---|---|
| chassis_joint | fixed | base_link → chassis | 0 0 0.37 | 0 0 0 | 0 0 0 | 0 0 0 | — / — / — / — | — |
| rear_left_wheel_joint | continuous | chassis → rear_left_wheel | -0.28 0.2925 -0.19558 | 1.57079633 0 0 | 0 0 -1 | 0 1 -6.123234e-17 | — / — / 10.0 / 10.0 | — |
| rear_right_wheel_joint | continuous | chassis → rear_right_wheel | -0.28 -0.2925 -0.19558 | 1.57079633 0 0 | 0 0 -1 | 0 1 -6.123234e-17 | — / — / 10.0 / 10.0 | {'joint': 'rear_left_wheel_joint', 'multiplier': '1.0', 'offset': '0.0'} |
| front_left_steering_joint | revolute | chassis → front_left_steering_link | 0.37142 0.2925 -0.19558 | 1.57079633 0 0 | 0 1 0 | 0 6.123234e-17 1 | -0.461 / 0.461 / 1 / 10.0 | — |
| front_left_wheel_joint | continuous | front_left_steering_link → front_left_wheel | 0 0 0 | 0 0 0 | 0 0 -1 | 0 1 -6.123234e-17 | — / — / — / — | — |
| front_right_steering_joint | revolute | chassis → front_right_steering_link | 0.37142 -0.2925 -0.19558 | 1.57079633 0 0 | 0 1 0 | 0 6.123234e-17 1 | -0.461 / 0.461 / 1 / 10.0 | {'joint': 'front_left_steering_joint', 'multiplier': '1.0', 'offset': '0.0'} |
| front_right_wheel_joint | continuous | front_right_steering_link → front_right_wheel | 0 0 0 | 0 0 0 | 0 0 -1 | 0 1 -6.123234e-17 | — / — / — / — | — |

| Link | Mass | COM xyz / rpy (link frame) | Inertia [ixx,iyy,izz; ixy,ixz,iyz] |
|---|---|---|---|
| base_link | 미지정 | — | — |
| chassis | 72 | 0.0266989328 0.000409203501 -0.135901125 / 0 0 0 | 0.033237096 0.111072002 0.126608628 -1.01315049e-06 -0.00188859201 -5.13308151e-07 |
| rear_left_wheel | 7 | -6.5774e-06 2.0242e-05 -0.0096386 / 0 0 0 | 0.063724 0.063724 0.11072 0 0 0 |
| rear_right_wheel | 7 | -6.5774e-06 2.0242e-05 -0.0096386 / 0 0 0 | 0.063724 0.063724 0.11072 0 0 0 |
| front_left_steering_link | 0.005 | 0 0 0 / 0 0 0 | 0.1 0.1 0.1 0 0 0 |
| front_left_wheel | 7 | -6.5774e-06 2.0242e-05 -0.0096386 / 0 0 0 | 0.063724 0.063724 0.11072 0 0 0 |
| front_right_steering_link | 0.005 | 0 0 0 / 0 0 0 | 0.1 0.1 0.1 0 0 0 |
| front_right_wheel | 7 | -6.5774e-06 2.0242e-05 -0.0096386 / 0 0 0 | 0.063724 0.063724 0.11072 0 0 0 |

| Link | Visual geometry | Collision geometry |
|---|---|---|
| base_link | 없음 | 없음 |
| chassis | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/chassis.dae"} | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/chassis.dae"} |
| rear_left_wheel | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/rear_left_wheel.dae"} | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/rear_left_wheel.dae"} |
| rear_right_wheel | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/rear_right_wheel.dae"} | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/rear_right_wheel.dae"} |
| front_left_steering_link | cylinder: {"length": "0.01", "radius": "0.01"} | 없음 |
| front_left_wheel | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/front_left_wheel.dae"} | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/front_left_wheel.dae"} |
| front_right_steering_link | cylinder: {"length": "0.01", "radius": "0.01"} | 없음 |
| front_right_wheel | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/front_right_wheel.dae"} | mesh: {"filename": "file://third_party/hunter_robot/hunter_description/meshes/front_right_wheel.dae"} |

## Current

총 mass **52.2274695954 kg**; links 9, joints 8.

| Joint | Type | Parent → Child | Origin xyz | Origin rpy | Local axis | Base axis (q=0) | Limit lower/upper/velocity/effort | Mimic |
|---|---|---|---|---|---|---|---|---|
| front_steer_left_joint | revolute | base_link → front_steer_left_link | 0.37142 0.29199 -0.1955 | 1.5708 0 0 | 0 0 1 | 0 -1 -3.6732051e-06 | -0.58 / 0.58 / 1 / 100 | — |
| front_left_wheel_joint | continuous | front_steer_left_link → front_left_wheel_link | 0 0 0 | 0 0 0 | 0 0 1 | 0 -1 -3.6732051e-06 | — / — / — / — | — |
| front_steer_right_joint | revolute | base_link → front_steer_right_link | 0.37142 -0.29201 -0.1955 | 1.5708 0 0 | 0 0 -1 | 0 1 3.6732051e-06 | -0.58 / 0.58 / 1 / 100 | — |
| front_right_wheel_joint | continuous | front_steer_right_link → front_right_wheel_link | 0 0 0 | 0 0 0 | 0 0 -1 | 0 1 3.6732051e-06 | — / — / — / — | — |
| left_rear_joint | continuous | base_link → left_rear_link | -0.28 0.29249 -0.19558 | 1.5708 0 0 | 0 0 1 | 0 -1 -3.6732051e-06 | — / — / 1 / 100 | — |
| right_rear_joint | continuous | base_link → right_rear_link | -0.28018 -0.29251 -0.19558 | 1.5708 0.0026575 0 | 0 0 -1 | 9.76153107e-09 1 3.67319213e-06 | — / — / 1 / 100 | — |
| front_steer_joint | fixed | base_link → front_steer_link | 0.44792 -1.1431e-05 -0.121 | 0 0 0 | 0 0 0 | 0 0 0 | — / — / — / — | — |
| rear_wheel_joint | fixed | base_link → rear_wheel_link | -0.42408 -1.1431e-05 -0.085581 | 0.0032313 0 0 | 0 0 0 | 0 0 0 | — / — / — / — | — |

| Link | Mass | COM xyz / rpy (link frame) | Inertia [ixx,iyy,izz; ixy,ixz,iyz] |
|---|---|---|---|
| base_link | 6.02945169537 | 0.0266989328 0.000409203501 -0.135901125 / 0 0 0 | 0.033237096 0.111072002 0.126608628 -1.01315049e-06 -0.00188859201 -5.13308151e-07 |
| front_steer_left_link | 7.8526 | -0.012614 1.8116e-05 0.0023132 / 0 0 0 | 0.063827 0.063827 0.11091 1.5154e-07 3.1921e-07 2.9854e-08 |
| front_left_wheel_link | 7.7007 | -6.5774e-06 2.0242e-05 -0.0096386 / 0 0 0 | 0.063724 0.063724 0.11072 -3.8745e-08 2.2753e-07 -4.88e-08 |
| front_steer_right_link | 7.696 | -8.9374e-08 1.0997e-07 0.0095958 / 0 0 0 | 0.063704 0.063704 0.11068 -6.3892e-08 -2.0159e-08 6.4721e-08 |
| front_right_wheel_link | 7.5613 | -9.1148e-08 1.1284e-07 0.0095049 / 0 0 0 | 0.063626 0.063626 0.11054 -6.4092e-08 -2.0164e-08 6.4721e-08 |
| left_rear_link | 7.6692 | -8.826e-05 6.4342e-05 -0.0096491 / 0 0 0 | 0.063699 0.063699 0.11067 -2.542e-07 2.638e-08 -5.9934e-08 |
| right_rear_link | 7.7133 | 8.6344e-05 7.4488e-05 0.0097824 / 0 0 0 | 0.06373 0.06373 0.11073 2.5408e-07 2.6379e-08 5.9941e-08 |
| front_steer_link | 0.0049179 | 0.049836 8.1046e-15 0.017912 / 0 0 0 | 1.6846e-07 7.549e-07 7.334e-07 7.6396e-23 -1.0974e-08 -1.8957e-22 |
| rear_wheel_link | 0 | 0 0 0 / 0 0 0 | 0 0 0 0 0 0 |

| Link | Visual geometry | Collision geometry |
|---|---|---|
| base_link | mesh: {"filename": "meshes/base_link.STL"} | mesh: {"filename": "meshes/base_link.STL"} |
| front_steer_left_link | mesh: {"filename": "meshes/front_steer_left_link.STL"} | mesh: {"filename": "meshes/front_steer_left_link.STL"} |
| front_left_wheel_link | mesh: {"filename": "meshes/front_left_wheel_link.STL"} | mesh: {"filename": "meshes/front_left_wheel_link.STL"} |
| front_steer_right_link | mesh: {"filename": "meshes/front_steer_right_link.STL"} | mesh: {"filename": "meshes/front_steer_right_link.STL"} |
| front_right_wheel_link | mesh: {"filename": "meshes/front_right_wheel_link.STL"} | mesh: {"filename": "meshes/front_right_wheel_link.STL"} |
| left_rear_link | mesh: {"filename": "meshes/left_rear_link.STL"} | mesh: {"filename": "meshes/left_rear_link.STL"} |
| right_rear_link | mesh: {"filename": "meshes/right_rear_link.STL"} | mesh: {"filename": "meshes/right_rear_link.STL"} |
| front_steer_link | mesh: {"filename": "meshes/front_steer_link.STL"} | mesh: {"filename": "meshes/front_steer_link.STL"} |
| rear_wheel_link | mesh: {"filename": "meshes/rear_wheel_link.STL"} | mesh: {"filename": "meshes/rear_wheel_link.STL"} |

