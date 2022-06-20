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

