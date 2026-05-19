# RG2 Migration Notes

- RG2-only Isaac URDF worked: `generated_urdf/rg2_only/rg2_only_isaac.urdf`
- Full UR5e URDF patched: `ros2_ws/src/robot_description/urdf/ur5e_ros.urdf`
- New RG2 source package: `ros2_ws/src/onrobot_rg_description`
- MoveIt config updated in `ros2_ws/src/ur5e_moveit_config/config`
- Isaac publishes the new RG2 moving joints with unprefixed names.
- The full URDF and MoveIt config were aligned to those same unprefixed joint names so `robot_state_publisher` can resolve the RG2 moving-link TFs.
- `generated_urdf/rg2_only/rg2_only_isaac.urdf` was also aligned to the final prefixed RG2 link names while keeping the unprefixed joint names that Isaac publishes.

## Final RG2 Root/Base Link

- `onrobot_rg2_base_link`

## Final RG2 Joint Names

- `wrist_3_link_to_TCP`
- `Gripper_joint`
- `finger_joint`
- `left_inner_knuckle_joint`
- `left_inner_finger_joint`
- `right_outer_knuckle_joint`
- `right_inner_knuckle_joint`
- `right_inner_finger_joint`

## Final RG2 Link Names

- `onrobot_rg2_base_link`
- `onrobot_rg2_left_outer_knuckle`
- `onrobot_rg2_left_inner_knuckle`
- `onrobot_rg2_left_inner_finger`
- `onrobot_rg2_right_outer_knuckle`
- `onrobot_rg2_right_inner_knuckle`
- `onrobot_rg2_right_inner_finger`
- `tcp`

## Attachment Strategy

- `tcp` is preserved exactly as a fixed child of `wrist_3_link` by `wrist_3_link_to_TCP`.
- `wrist_3_link_to_TCP` uses `xyz="0 0 0.2" rpy="0 0 0"` to keep the IK tcp chain unchanged.
- The new RG2 root is attached to `wrist_3_link` by `Gripper_joint` as a fixed joint.
- `Gripper_joint` uses `xyz="0 0 0.0" rpy="0 0 -1.5707963267948966"` to match the Isaac attachment orientation shown in the screenshot.

## Isaac / Joint State Alignment

Isaac publishes these RG2 moving-joint names:

- `finger_joint`
- `left_inner_knuckle_joint`
- `right_inner_knuckle_joint`
- `right_outer_knuckle_joint`
- `left_inner_finger_joint`
- `right_inner_finger_joint`

The full URDF previously used prefixed names such as `onrobot_rg2_finger_joint`. Those names were renamed in the full URDF, SRDF, and active MoveIt YAML files to match Isaac exactly.

## Deprecated Old RG2 Joint Names

The following old custom linkage joints are deprecated and intentionally removed from the active model:

- `onrobot_rg2_finger_1_origin_to_moment_arm_joint`
- `onrobot_rg2_finger_1_origin_to_truss_arm_joint`
- `onrobot_rg2_finger_2_origin_to_moment_arm_joint`
- `onrobot_rg2_finger_2_origin_to_truss_arm_joint`
- `onrobot_rg2_finger_1_truss_arm_to_finger_tip_joint`
- `onrobot_rg2_finger_2_truss_arm_to_finger_tip_joint`

## Removed Old Active Geometry References

- `package://robot_description/meshes/onrobot_rg2ft/visual/angle_bracket.stl`
- `package://robot_description/meshes/onrobot_rg2ft/collision/angle_bracket.stl`
- old `onrobot_description/meshes/rg2_v1/*` custom full-robot RG2 mesh references

## Manual Verification Checklist

1. Open `ros2_ws/src/robot_description/urdf/ur5e_ros.urdf` and confirm both fixed joints exist: `wrist_3_link -> tcp` and `wrist_3_link -> onrobot_rg2_base_link`.
2. Confirm the RG2 moving joints in the full URDF are named exactly: `finger_joint`, `left_inner_knuckle_joint`, `left_inner_finger_joint`, `right_outer_knuckle_joint`, `right_inner_knuckle_joint`, `right_inner_finger_joint`.
3. Confirm there are no active references to the old prefixed RG2 moving-joint names in active URDF/SRDF/YAML config.
4. Confirm all RG2 meshes in the full URDF now resolve from `package://onrobot_rg_description/meshes/rg2/...`.
5. Confirm `ur_manipulator` in `ros2_ws/src/ur5e_moveit_config/config/ur5e.srdf` still ends at `tcp`.
6. After the file review looks right, do your manual build and launch verification outside this edit pass.
