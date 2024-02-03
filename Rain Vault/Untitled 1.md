```cpp
const Eigen::Vector3f dir = static_cast<Eigen::Vector3f>(*pivot_pos) - static_cast<Eigen::Vector3f>(collision_position_);  
const Eigen::Vector3f cur = static_cast<Eigen::Vector3f>(*pivot_pos) - static_cast<Eigen::Vector3f>(actual_pos_);  
Eigen::Vector3f axis = cur.cross(dir);  
axis.normalize();  
const float angle = std::acos(cur.normalized().dot(dir.normalized()));
```
我现在有如上数据,dir,cur,axis,angle,
然后我有一个对象,包含pos和rot数据,我需要把这个对象绕着点pivot_pos以及轴axis,旋转angle得到新的pos和rot数据.
帮我实现.