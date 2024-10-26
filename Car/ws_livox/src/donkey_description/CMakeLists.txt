cmake_minimum_required(VERSION 3.0.2)
project(donkey_description)

# 查找catkin的构建工具
find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

# catkin构建时需要的信息
catkin_package()

# 安装URDF文件
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

# # 安装网格文件
# install(DIRECTORY meshes/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
# )

# # 安装材质文件
# install(DIRECTORY materials/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/materials
# )

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
