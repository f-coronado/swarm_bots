# Swarm_Bots

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![codecov](https://codecov.io/gh/f-coronado/swarm_bots/branch/main/graph/badge.svg)](https://codecov.io/gh/f-coronado/swarm_bots)
![CICD Workflow status](https://github.com/f-coronado/swarm_bots/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)


# Introduction
Swarm Robotics is an innovative field of robotics that draws inspiration from the collective behavior observed in natural swarms, such as flocks of birds or colonies of ants. In the realm of robotics, it involves the coordination and collaboration of numerous simple robots, known as agents or drones, to perform tasks in a decentralized and self-organized manner. These autonomous agents communicate and interact with each other to achieve a common goal, demonstrating emergent behavior that is often more robust, adaptable, and efficient than that of individual robots. The principles of Swarm Robotics have found applications in various domains, including agriculture, search and rescue missions, environmental monitoring, and industrial automation. This GitHub repository aims to provide a self contained cpp based module, swarm_library, that can easily swapped any ros2 package for trajectory generation. Furthermore, this repo contains a ros2 package, turtlebot_swarm, which showcases using the swarm_library for a swarm demo. This repo was created in a 4 week log AIP process, all planning meetings, logs and more can be found [here](https://drive.google.com/drive/folders/1ltB3tIcugKpeDje7qDEQy7WUJzHPNTlJ?usp=sharing)

|Name|Github UserID|Email|
|:---:|:---:|:---:|
|Fabrizio Coronado|f-coronado|fcoronad@umd.edu|
|Kautilya Chappidi|1412kauti|kautilya@umd.edu|


#### Demo Video, UML Diagram, Logs, Planning Meetings and more can be found [here](https://drive.google.com/drive/folders/1ltB3tIcugKpeDje7qDEQy7WUJzHPNTlJ?usp=sharing)


## System Requirements
Ubuntu 22.04 LTS with ROS2 Humble are recommended for the turtlebot_swarm package

## Building and Dependencies
First make sure you have the required dependencies
```bash
sudo apt-get update
rosdep install -i --from-path src --rosdistro humble -y
sudo apt-get install doxygen
sudo apt-get install lcov
sudo apt install ros-humble-turtlebot3*
sudo apt install pandoc
```
Next build the swarm_library cpp module
```bash
colcon build --packages-select swarm_library
source install/setup.bash
```
Lastly, build the turtlebot_swarm ros2 package
```bash
colcon build --packages-select turtlebot_swarm
source install/setup.bash
```

## Running the Demo
By default run_demo.launch.py spawns 20 turtlebot
```bash
ros2 launch turtlebot_swarm run_demo.launch.py 
# alternatively
ros2 launch turtlebot_swarm demo_python.launch.py
```
If you want to spawn 10 bots or any number of bots
```bash
ros2 launch turtlebot_swarm run_demo.launch.py node_number:=10 
```
To see the bots move
```bash
ros2 run turtlebot_swarm velocity_pub
```

## How to build for tests (unit test and integration test)

```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
```

## How to generate coverage reports after running colcon test

First make sure we have run the unit test already.

```bash
colcon test
```

### Test coverage report for `turtlebot_swarm`:

``` bash
ros2 run turtlebot_swarm generate_coverage_report.bash
open build/turtlebot_swarm/test_coverage/index.html
```

### Test coverage report for `swarm_library`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select swarm_library \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
open build/swarm_library/test_coverage/index.html
```

### combined test coverage report

``` bash
./do-tests.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
```

## Known bugs
If you are unable to build turtlebot_swarm then please try the following:
1. Download the repo
2. git checkout bca92568a594303528d870e336a2b1f37440cec2
3. Retry steps from above

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Disclaimer

This software is provided "as is" and any expressed or implied warranties, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose are disclaimed. In no event shall the authors or contributors be liable for any direct, indirect, incidental, special, exemplary, or consequential damages (including, but not limited to, procurement of substitute goods or services; loss of use, data, or profits; or business interruption) however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence or otherwise) arising in any way out of the use of this software, even if advised of the possibility of such damage.

  - GitHub CI
    - "main" branch runs in a ROS 2 Humble container
  - Codecov badges
  - Colcon workspace structure
  - C++ library that depends on other system libraries such as OpenCV and rclcpp.
    - The library is *self-contained*
    - In real life, we download source code of third-party modules all
      the time and often just stick the modules as-is into our colcon
      workspace.
  - ROS 2 package that depends on a C++ library built in the same colcon workspace
  - Establishing package dependency within the colcon workspace.
    - ie. the ROS 2 package will not be built before all of its dependent C++ libraries are built first
  - Multiple subscriptions within a ROS2 node all listening to the same topic.
    - Only one callback function is needed.
    - More efficient than to have N callback functions.
    - More efficient than to have N ROS nodes.
  - Unit test and integration test.
  - Doxygen setup
  - ROS2 launch file
  - Bash scripts that can be invoked by the "ros2 run ..." command