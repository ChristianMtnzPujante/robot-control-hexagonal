# Robotiq 2F-85 (URDF + mallas)

Copiado del paquete `ros-humble-robotiq-description` 0.0.1 (PickNik,
[ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)),
licencia **BSD**. Descargado el 30/09/2026 con `apt-get download` + `dpkg -x`,
sin instalarlo.

- `urdf/robotiq_2f_85.urdf`: salida de xacro sobre
  `urdf/robotiq_2f_85_gripper.urdf.xacro` (`use_fake_hardware:=true`), con
  tres cambios:
  - sin el link `world` ni su joint fijo, para que la raíz sea
    `robotiq_85_base_link` y se pueda colgar de la brida de cualquier robot;
  - sin el bloque `ros2_control`;
  - las mallas apuntando a `package://robotiq_2f_85/meshes/...`. Con
    `simURDF`, el prefijo que sustituye a `package://` es el directorio
    `assets/`.
- `meshes/`: `visual/*.dae` y `collision/*.stl` tal cual, sin el acoplador
  de UR (`ur_to_robotiq_adapter`), que el CR5 no usa.

Cinemática: el único joint que se manda es `robotiq_85_left_knuckle_joint`
(0 = abierta, 0,8 rad = cerrada). Los otros cinco joints móviles son
`<mimic>` suyos, con multiplicador ±1.

Para qué se usa: `commander/coppeliasim_scene_builder.py`
(`ROBOTIQ_2F_85_ON_CR5`) y
`robot_node/adapters/coppeliasim_gripper_adapter.py`.
