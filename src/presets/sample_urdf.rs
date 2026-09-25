/// Built-in sample URDF XML model of a 3-DOF Planar Arm.
pub const SAMPLE_URDF_PLANAR_3DOF: &str = r#"<?xml version="1.0"?>
<robot name="planar_3dof_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.05"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.25 0.25 0.28 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.96" upper="2.96" effort="50.0" velocity="3.14"/>
  </joint>

  <link name="link_1">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.5 0.85 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.6" upper="2.6" effort="30.0" velocity="3.14"/>
  </joint>

  <link name="link_2">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.3"/>
      </geometry>
      <material name="orange">
        <color rgba="0.95 0.5 0.2 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="joint_3" type="revolute">
    <parent link="link_2"/>
    <child link="link_3"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.8" upper="2.8" effort="15.0" velocity="3.14"/>
  </joint>

  <link name="link_3">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.15"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.4 1.0"/>
      </material>
    </visual>
  </link>
</robot>
"#;

/// Built-in sample URDF XML model of a 6-DOF Industrial Arm (UR5/Puma style).
pub const SAMPLE_URDF_INDUSTRIAL_6DOF: &str = r#"<?xml version="1.0"?>
<robot name="industrial_6dof_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="dark">
        <color rgba="0.2 0.2 0.25 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="base_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="150" velocity="3.14"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.12"/>
      </geometry>
      <material name="cyan">
        <color rgba="0.1 0.65 0.85 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.6" upper="2.6" effort="150" velocity="3.14"/>
  </joint>

  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.42"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.95 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 -0.1 0.42" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="150" velocity="3.14"/>
  </joint>

  <link name="forearm_link">
    <visual>
      <geometry>
        <cylinder radius="0.045" length="0.38"/>
      </geometry>
      <material name="cyan_forearm">
        <color rgba="0.1 0.65 0.85 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="wrist_1_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_1_link"/>
    <origin xyz="0 0.1 0.38" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/>
  </joint>

  <link name="wrist_1_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.09"/>
      </geometry>
      <material name="orange">
        <color rgba="0.95 0.45 0.15 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link"/>
    <child link="wrist_2_link"/>
    <origin xyz="0 0.09 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/>
  </joint>

  <link name="wrist_2_link">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.09"/>
      </geometry>
      <material name="gold">
        <color rgba="0.9 0.75 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="ee_link"/>
    <origin xyz="0 0 0.09" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="3.14"/>
  </joint>

  <link name="ee_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="emerald">
        <color rgba="0.1 0.85 0.5 1.0"/>
      </material>
    </visual>
  </link>
</robot>
"#;
