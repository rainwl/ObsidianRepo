## pos + rot -> Matrix
```cpp
#include <Eigen/Dense>
#include <Eigen/Geometry> 

Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();  // 创建一个单位4x4矩阵

// 用欧拉角创建旋转矩阵
Eigen::Matrix3f rotation_matrix;
rotation_matrix = Eigen::AngleAxisf(rongeur_euler_angles[0], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(rongeur_euler_angles[1], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(rongeur_euler_angles[2], Eigen::Vector3f::UnitZ());
// 第二种写法
const Eigen::AngleAxisf rongeur_rotate_x(euler_x, Eigen::Vector3f::UnitX());
const Eigen::AngleAxisf rongeur_rotate_y(euler_y, Eigen::Vector3f::UnitY());
const Eigen::AngleAxisf rongeur_rotate_z(euler_z, Eigen::Vector3f::UnitZ());
const Eigen::Matrix3f rongeur_rotation_matrix = (rongeur_rotate_z * rongeur_rotate_y * rongeur_rotate_x).matrix();


// 将旋转矩阵设置到4x4矩阵的左上角
transform_matrix.block<3,3>(0,0) = rotation_matrix;

// 设置平移
transform_matrix.block<3,1>(0,3) = rongeur_pos;

// 现在 transform_matrix 是你需要的4x4变换矩阵

```

## local pos z transform -> Matrix

```cpp
const Eigen::Vector3f pivot_pos(tool_data[21], tool_data[22], tool_data[23]);
const Eigen::Vector3f offset(0.0f, 0.0f, offset_z);

const Eigen::Matrix3f rongeur_rotation_matrix;
const Eigen::Vector3f rongeur_rotated_offset = rongeur_rotation_matrix * offset;

Eigen::Vector3f rongeur_pos = pivot_pos + rongeur_rotated_offset;

```

## Matrix3x3 -> euler angle


```cpp
Eigen::Vector3f rongeur_euler_angles = rongeur_rotation_matrix.eulerAngles(0, 1, 2);
```



## euler -> matrix3x3
```cpp
#include <Eigen/Geometry>

// 假设 new_tube_rot 是一个包含三个欧拉角的 Eigen::Vector3f
Eigen::Vector3f new_tube_rot; // new_tube_rot需要被赋值

// 创建旋转矩阵
Eigen::Matrix3f rotation_matrix;
rotation_matrix = 
    Eigen::AngleAxisf(new_tube_rot[2], Eigen::Vector3f::UnitZ()) * // 绕Z轴旋转
    Eigen::AngleAxisf(new_tube_rot[1], Eigen::Vector3f::UnitY()) * // 绕Y轴旋转
    Eigen::AngleAxisf(new_tube_rot[0], Eigen::Vector3f::UnitX());  // 绕X轴旋转

// rotation_matrix 现在是一个根据 new_tube_rot 的欧拉角构造的3x3旋转矩阵
```



