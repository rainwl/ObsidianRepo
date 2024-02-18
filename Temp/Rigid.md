如果发生了碰撞并且没有发送接触的时候(一瞬间)
持续记录碰撞位置collision_position_,以及更新actual_pos_,
这时候只记录pos就可以,rot是不需要的

>之前做的接触力,是怎么给到力反馈设备的,有没有经过sofa或者其他的转换,在Geomagic里面

```cpp
Vec2 *world_xz_force = nullptr;

const Vec3 motor_force = Rotate(rDevice, Vec3(1, 0, 0)) * forces[0][0] +  Rotate(rDevice, Vec3(0, 1, 0)) * forces[1][0];

(*geomagic->world_xz_force)[1] = motor_force.y - motor_force.z;  
(*geomagic->world_xz_force)[0] = motor_force.x;

if (geomagic->is_left_hand_frame) (*geomagic->world_xz_force)[0] = -motor_force.x;
```

>感觉疑惑,这个分解出来的电机的力,还需要怎么个分解法啊,直接x,z不行吗?

