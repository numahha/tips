# 目標
産業用ロボットの運動学パラメータを推定する問題を考える。具体例として、ユニバーサルロボット社のUR5を考える。運動学パラメータの表現方法として、DHパラメータ表現を用いる。なお、[ユニバーサルロボット社のページ](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)より、UR5のDHパラメータは以下である（thetaは関節角度なので変化する）。

本稿では、DHパラメータのノミナル値（大まかな値・カタログ値の意味）を所与として、関節角ー手先位置姿勢のデータに基づきDHパラメータの個体差を推定する、という状況を考える。

|  |theta [rad] | a [m] |	d [m] 	| alpha [rad] |
| ---- | ---- | ---- | ---- | ---- |
|Joint 1 |	0 |	0 |	0.089159 |	π/2 |
|Joint 2 |	0 |	-0.425 |	0 |	0 |
|Joint 3 |	0 |	-0.39225 |	0 |	0 |
|Joint 4 |	0 |	0 |	0.10915 |	π/2 |
|Joint 5 |	0 |	0 |	0.09465 |	-π/2 |
|Joint 6 |	0 |	0 |	0.0823 |	0 |
<img width="800" alt="dh_ur.jpg (44.3 kB)" src="https://img.esa.io/uploads/production/attachments/15545/2021/05/05/102139/fe954c9d-3d64-4856-9a66-cdf749028da1.jpg">





# 推定方法
関節角度を$\theta$、手先位置姿勢を $y$、推定したいパラメータを$\phi$ とする。運動学を次のように表現する。
```math
y = f_\phi(\theta)
```
あるパラメータ$\phi_0$に対して、推定誤差を$\Delta \phi=\phi_\ast-\phi_{0}$と表現すると、手先の誤差$\Delta y = y-y_{pred}$は次ように書ける。
```math
\Delta y = \left.\frac{\partial f_\phi(\theta)}{\partial \phi}\right|_{\phi=\phi_{0}}\Delta \phi = J(\phi_0,\theta)\Delta \phi
```
データ$(\theta^{(1)}, y^{(1)})\cdots(\theta^{(n)}, y^{(n)})$に対して、$\delta Y = (\delta y^{(1)},\cdots,\delta y^{(n)})$と$J(\phi_0) = (J(\phi_0,\theta^{(1)}),\cdots,J(\phi_0,\theta^{(1)}))$のように並べて表現する。
```math
\Delta Y = J(\phi_0) \Delta \phi
```

●●●もう少し書く必要●●●


## 参考
[Robot Kinematics Identification: KUKA LWR4+Redundant Manipulator Example](https://iopscience.iop.org/article/10.1088/1742-6596/659/1/012011/pdf)

[1] https://github.com/nnadeau/pybotics/blob/master/examples/calibration.ipynb


# 関節角ー手先位置姿勢の数値実験データの生成方法
Ubuntu 18.04における方法をまとめる。
## ROS1 (melodic)のインストール
[公式ページ](http://wiki.ros.org/melodic/Installation/Ubuntu)に従ってインストール
```
$ sudo apt update
$ sudo apt upgrade -y 
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full -y
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
$ sudo apt install python-rosdep -y 
$ sudo rosdep init
$ rosdep update
$ sudo apt install ros-melodic-moveit -y
```
チュートリアルに従い、ワークスペースを作成
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
いくつかパッケージを追加でインストール
```
$ sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers
$ sudo apt-get install ros-melodic-gazebo-ros-control
$ sudo apt install ros-melodic-joint-state-publisher-gui
```

`~/.ignition/fuel/config.yaml`のurlを以下のように修正する。
```
url: https://api.ignitionrobotics.org
```

## UR5のROSパッケージのインストール

[出村先生のページ](https://demura.net/robot/athome/14710.html)などを参考に、ur5アームとrobotiqグリッパーのパッケージをダウンロード
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/utecrobotics/ur5
$ git clone https://github.com/utecrobotics/robotiq
```
以下のようにlaunchする。
```
$ roslaunch ur5_description display_with_gripper.launch
```
手先の位置姿勢は、「世界座標系からエンドエフェクタリンク座標系への変換」として表現される。[チュートリアル](http://wiki.ros.org/ja/tf)を参考に、別ターミナルを開いて以下を実行する。
```
$ rosrun tf tf_echo /world /ee_link
```
`joint_state_publisher`のスライダを動かしながら、TranslationとRotationの値が変わることを確認する。
例えば、[仕様](https://cobot-system.com/maker/cr_ur/)を見ると、リーチは850mmである。`shoulder_lift_joint`のみを動かしてアームを立てた場合、エンドエフェクタのz軸位置はおよそ0.85m＋台座くらいの値に変化するはずである。

関節座標は、次のコマンドで見ることができる。
```
$ rostopic echo /joint_states
```



## 関節角ー手先位置姿勢データを取得するパッケージの作成
`ur_endeffector_joint` という名前で、パッケージを作成する。
```
$ cd ~/catkin_ws/srs
$ catkin_create_pkg ur_endeffector_joint std_msgs sensor_msgs roscpp tf
```

 [tf_echoのノード](https://github.com/ros/geometry/blob/melodic-devel/tf/src/tf_echo.cpp) を参考に、下記のソースファイルを作成する。
```cpp:ur_endeffector_joint/src/ur_endeffector_joint.cpp
#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include<fstream>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#define _USE_MATH_DEFINES

std::string filename;

class echoListener
{
public:

  tf::TransformListener tf;
  std::ofstream writing_file;
  std::string filename;

  //constructor with name
  echoListener()
  {
    const char *home_path = getenv("HOME");
    std::string env_var(home_path ? home_path : "");
    filename = env_var + "/endeffector_data.csv";
    writing_file.open(filename, std::ios::out);
    writing_file << "t1,rx,ry,rz,qx,qy,qz,qw,t2,th1,th2,th3,th4,th5,th6\n";
    std::cout << "\nStart recording.\n" << std::endl;
    std::cout << "Type 'Ctrl + C' to finish recording.\n" << std::endl;
  }

  ~echoListener()
  {
    writing_file.close();
    std::cout << "\n\nData is saved at " << filename << "\n" << std::endl;
  }

  void write_file(double t1, 
                  double rx, double ry, double rz, double qx, double qy, double qz, double qw, 
                  double t2, 
                  double th1, double th2, double th3, double th4, double th5, double th6){
    static int local_count = 0;
    if (local_count<=10000){
        writing_file << t1 << ", "
          << rx << ", " << ry << ", " << rz << ", " << qx << ", " << qy << ", " << qz << ", " << qw << ","
          << t2 << ", "
          << th1 << ", " << th2 << ", " << th3 << ", " << th4 << ", " << th5 << ", " << th6 << "\n";
    }
    local_count++;
  }

private:

};

//void chatterCallback(const std_msgs::String::ConstPtr& msg)
static double joint_angle[6];
static double timestamp;
void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  timestamp = msg->header.stamp.toSec();
  joint_angle[0] = msg->position[0];
  joint_angle[1] = msg->position[1];
  joint_angle[2] = msg->position[2];
  joint_angle[3] = msg->position[3];
  joint_angle[4] = msg->position[4];
  joint_angle[5] = msg->position[5];
}

int main(int argc, char ** argv)
{
  static int init_flag=1;
  double start_time;

  //Initialize ROS
  ros::init(argc, argv, "ur_endeffector_joint", ros::init_options::AnonymousName);

  // Allow 2 or 3 command line arguments
  if (argc < 3 || argc > 4)
  {
    printf("Usage: tf_echo source_frame target_frame [echo_rate]\n\n");
    printf("This will echo the transform from the coordinate frame of the source_frame\n");
    printf("to the coordinate frame of the target_frame. \n");
    printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
    printf("Default echo rate is 1 if echo_rate is not given.\n");
    return -1;
  }

  ros::NodeHandle nh("~");

  double rate_hz;
  if (argc == 4)
  {
    // read rate from command line
    rate_hz = atof(argv[3]);
  }
  else
  {
    // read rate parameter
    nh.param("rate", rate_hz, 10.);
  }
  if (rate_hz <= 0.0)
  {
    std::cerr << "Echo rate must be > 0.0\n";
    return -1;
  }
  ros::Rate rate(rate_hz);

  int precision(3);
  if (nh.getParam("precision", precision))
  {
    if (precision < 1)
    {
      std::cerr << "Precision must be > 0\n";
      return -1;
    }
    printf("Precision default value was overriden, new value: %d\n", precision);
  }

  //Instantiate a local listener
  echoListener echoListener;

  std::string source_frameid = std::string(argv[1]);
  std::string target_frameid = std::string(argv[2]);

  // Wait for up to one second for the first transforms to become avaiable. 
  echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.));

  ros::Subscriber sub = nh.subscribe("/joint_states", 100, chatterCallback);

  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while(nh.ok())
    {
      try
      {
        tf::StampedTransform echo_transform;
        echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
        std::cout.precision(precision);
        std::cout.setf(std::ios::fixed,std::ios::floatfield);
        double yaw, pitch, roll;
        echo_transform.getBasis().getRPY(roll, pitch, yaw);
        tf::Quaternion q = echo_transform.getRotation();
        tf::Vector3 v = echo_transform.getOrigin();

        if (init_flag){
          init_flag = 0;
          start_time = echo_transform.stamp_.toSec();
        }
        std::cout << "At time " << echo_transform.stamp_.toSec() - start_time << "\r";

        echoListener.write_file( echo_transform.stamp_.toSec() - start_time, 
                                 v.getX(), v.getY(), v.getZ(), q.getX(), q.getY(), q.getZ(), q.getW(),
                                 timestamp - start_time, 
                                 joint_angle[0], joint_angle[1], joint_angle[2], joint_angle[3], joint_angle[4], joint_angle[5] );
        //print transform
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << echoListener.tf.allFramesAsString()<<std::endl;
        
      }
      ros::spinOnce();

      rate.sleep();
     
    }

  return 0;
}

```

`ur_endeffector_joint/CMakeLists.txt`に、下記を追加する。

```txt:ur_endeffector_joint/CMakeLists.txt
add_executable(ur_endeffector_joint src/ur_endeffector_joint.cpp)
target_link_libraries(ur_endeffector_joint
   ${catkin_LIBRARIES}
)
```

リビルド
```
$ cd ~/catkin_ws
$ catkin_make
```
実行
```
$ source ~/.bashrc
$ rosrun  ur_endeffector_joint ur_endeffector_joint /base_link /ee_link
```

Ctrl+Cで終了するまでの各時刻における関節角ー手先位置姿勢データが`~/endeffector_data.csv`に保存される。joint_state_publisherのRandomizeというボタンを押して、様々な関節各データを取ることがおすすめ。

## 本稿では必要無しの備忘録：GazeboのためのUR5関連ROSパッケージの修正 
[robotiqグリッパーのIssue](https://github.com/crigroup/robotiq/issues/4)を参考に、 `robotiq/robotiq_gazebo/src/mimic_joint_plugin.cpp`の中身を次のように修正する

* `event::Events::DisconnectWorldUpdateBegin(this->updateConnection)` →`this->updateConnection.reset()`
* `GetPhysicsEngine()` →`Physics()`
* `GetAngle(0).Radian()` →`Position(0)`
* `gazebo::math::clamp` →`ignition::math::clamp`

リビルドする
```
$ cd ~/catkin_ws
$ catkin_make
```

色合いが気に入らない場合には、[最新のurdf](https://github.com/ros-industrial/universal_robot/tree/melodic-devel)を取ってきて入れ替えれば良い。

`ur5/ur5_description/config/gazebo_ros_control_params.yaml` というファイルを作成する。
```yaml:ur5/ur5_description/config/gazebo_ros_control_params.yaml
/gazebo_ros_control:
  pid_gains:
    shoulder_pan_joint:
      p: 100.0 
      i: 0.01 
      d: 10.0
    shoulder_lift_joint:
      p: 100.0 
      i: 0.01 
      d: 10.0
    elbow_joint:
      p: 100.0 
      i: 0.01 
      d: 10.0
    wrist_1_joint:
      p: 100.0 
      i: 0.01 
      d: 10.0
    wrist_2_joint:
      p: 100.0 
      i: 0.01 
      d: 10.0
    wrist_3_joint:
      p: 100.0 
      i: 0.01 
      d: 10.0
```
また、`ur5/ur5_gazebo/launch/ur5_setup.launch`を以下のように修正する。
```launch:ur5/ur5_gazebo/launch/ur5_setup.launch
省略

  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -param robot_description -model robot -z 0.594
              -J shoulder_lift_joint -1.8
              -J elbow_joint 1.0"
        output="screen">
        <rosparam file="$(find ur5_description)/config/gazebo_ros_control_params.yaml" command="load"/>
  </node>
  
  <include file="$(find ur5_gazebo)/launch/ur5_controllers.launch" />
</launch>
```

下記のようにlaunchする。
```
$ roslaunch ur5_gazebo ur5_cubes.launch
```



