# allan_variance_ros2
從 [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros) 修改而來
## BUILD
```shell
cd ~/ros2_ws/src
git clone https://github.com/t112360140/allan_variance_ros2.git
cd ~/ros2_ws
colcon build --packages-select allan_variance_ros2
```
## USE
```shell
ros2 run allan_variance_ros2 allan_variance /path/to/bag/folder /path/to/config/file/imu.yaml

python3 ~/ros2_ws/src/scripts/analysis.py --data /path/to/data/file/allan_variance.csv --config /path/to/config/file/imu.yaml
```
