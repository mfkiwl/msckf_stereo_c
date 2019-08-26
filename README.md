# msckf_vio_mynt

modified version of msckf_vio

-----

## Build

```sh
mkdir build
cd build
cmake ..
make -j3
```

## Run

* main project
  ```sh
  cd build
  ./run_euroc
  ```

* unit test
  ```sh
  cd build
  ./test/run_unit_test
  ```

## Test Dataset

Remember change the value of variable `std::string euroc_dir` defined in **run_euroc.cpp**. 

* [The EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
  - [V1_01_easy.zip](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip)

## TODO

- [x] 编写EuRoC数据接口：Image和IMU两线程结构
- [x] 编写YAML配置文件读写接口
- [x] 添加单元测试
- [x] 添加 ImageProcessor
  - [x] 将 ROS message 改为 C++结构体
  - [x] 修改其他ROS数据结构为Plain C++数据结构
  - [x] 测试、调试（1 Day）
- [x] 添加 msckf_vio
  - [x] 将ROS数据结构为Plain C++数据结构（1 Day）
  - [x] 测试、调试（1 Day）
- [x] 添加 Draw 线程：轨迹渲染显示（1 Day）
- [ ] 系统测试、调试（1-2 Day）
- [ ] 代码重构（1 Day）
- [ ] 第三方库替换
  - [ ] PCL（1 Day）
  - [ ] OpenCV（1-2 Day）
  - [ ] Eigen3（1-2 Day）
  - [ ] 测试、调试（1-2 Day）
- [ ] 功能优化
- [ ] C代码重构
- [ ] FPGA移植
