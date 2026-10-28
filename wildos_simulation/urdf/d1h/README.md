# D1H navigation model

Source:

- Repository: <https://github.com/DDTRobot/ddt_ros2_control>
- Upstream path: `urdfs/d1h_description`
- Commit: `68cdc88ae65616f4c7431c8baa66e7718a48fb02`

Imported files:

- `d1h_navigation.urdf.xacro`
- Official `d1h_description/meshes/*.STL`

Local navigation adaptation:

- Hip, thigh and calf joints are fixed at the neutral wheel-legged stance.
- The two official wheel joints remain visible but passive.
- The first navigation proxy uses Gazebo velocity control to isolate
  localization, perception and Nav2 from wheel slip.
- Invisible low-friction supports keep the headless navigation proxy upright.
- A rear navigation tower carries one 128-line 3D LiDAR, three cameras and the IMU.
- Gazebo ground-truth odometry is published for Nav2 evaluation.

The upstream `d1h_description/package.xml` currently declares
`TODO: License declaration`. Confirm the official license with Direct Drive
Technology before redistributing the meshes or using them commercially.
