#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sstream>

class Odometry {
public:
  Odometry(double _sigma_vx, double _sigma_vy, double _sigma_vth) {
    delta_pub  = n.advertise<geometry_msgs::Pose>("odometry/delta_pose", 50);
    Delta_pub  = n.advertise<geometry_msgs::Pose>("odometry/Delta_pose", 50);
    global_pub = n.advertise<geometry_msgs::Pose>("odometry/global_pose", 50);
    C_D_pub    = n.advertise<std_msgs::String>("odometry/C_D", 50);
    vel_sub    = n.subscribe("/cmd_vel", 50, &Odometry::callback, this);

    current_time = ros::Time::now();
    last_time = current_time;

    //### Initialize motion data ###
    vx  = 0.0;
    vy  = 0.0;
    vth = 0.0;

    //### Set constant twist covariance ###
    C_v      = Eigen::Matrix3d::Zero(3, 3);
    C_v(0,0) = pow(_sigma_vx, 2);
    C_v(1,1) = pow(_sigma_vy, 2);
    C_v(2,2) = pow(_sigma_vth, 2);

    //### Initial reset: pose ###
    x  = 0.0;
    y  = 0.0;
    th = 0.0;

    //### Initial reset: factor pose increment ###
    Delta_x  = 0.0;
    Delta_y  = 0.0;
    Delta_th = 0.0;

    //### Initial reset: factor pose increment's covariance ###
    C_D = Eigen::Matrix3d::Zero(3, 3);
  }

  void run() {
    while(n.ok()) {
      usleep(100);
      current_time   = ros::Time::now();
      double delta_t = (current_time - last_time).toSec();

      //### Placeholder Variable Initializations ###
      // partim del punt 0,0 en el pla i un angle de 0º
      double delta_x  = 0;
      double delta_y  = 0;
      double delta_th = 0;
      //### Time Integration of Velocity Data ###
      // Increment de x be donat per multiplicar la velocitat de x per el delta t, i això per y i per theta
      Delta_x=vx*delta_t;
      Delta_y=vy*delta_t;
      Delta_th=vth*delta_t;
      //### Time Integration of Velocity Data - Jacobian stage ###
      // la Jacobiana de delta t es una matriu d'identitat de 3x3.
      J_delta_t= Eigen::Matrix3d::Identity(3,3);
      //### Integrate Factor ###
      // apliquem la fòrmula de la covariança: jacovianax la caovariança del twist x la jacoviana transposada
      C_d=J_delta_t*C_v*J_delta_t.transpose()*delta_t;
      //### Integrate Factor pose increment
      Delta_x = Delta_x + delta_x * cos(Delta_th) - delta_y * sin(Delta_th);
      Delta_y = Delta_y + delta_x * sin(Delta_th) + delta_y * cos(Delta_th);
      Delta_th = Delta_th + delta_th;
      //Jacobians of the equations above, repect to D ###
      J_D_D << 	1,	0,	-delta_x * sin(Delta_th) - delta_y * cos(Delta_th),
                0,	1,	 delta_x * cos(Delta_th) - delta_y * sin(Delta_th),
                0,  0,   1;
      //Jacobians of the equations above, repect to d ###
      J_D_d <<	cos(Delta_th),	-sin(Delta_th),	0,
                sin(Delta_th),	cos(Delta_th),	0,
                0, 0, 1;
      //### Integrate Factor - Covariance stage ###
      C_D = J_D_D * C_D * J_D_D.transpose() + J_D_d * C_d * J_D_d.transpose();
      //### Integrate Global Pose ###
      x = x + delta_x * cos(th) - delta_y * sin(th);
      y = y + delta_x * sin(th) + delta_y * cos(th);
      th = th + delta_th;
      //### Publish d, D and global poses, and covariance C_D ###
      delta_pub.publish(create_pose_msg(delta_x, delta_y, delta_th));
      Delta_pub.publish(create_pose_msg(Delta_x, Delta_y, Delta_th));
      C_D_pub.publish(create_covariance_msg(C_D));
      global_pub.publish(create_pose_msg(x, y, th));

      last_time = current_time;
      ros::spinOnce();
    }
  }

  void keyframe_callback() {
    //### Publish factor data
    // JS: publish Delta_x, Delta_y, Delta_th
    // DS: Delta_pub.publish(create_pose_msg(Delta_x, Delta_y, Delta_th));
    // JS: publish C_D;
    // DS: C_D_pub.publish(create_covariance_msg(C_D));

    //### Keyframe reset: factor pose increment ###
    //Delta_x  = 0.0;
    //Delta_y  = 0.0;
    //Delta_th = 0.0;

    //### Keyframe reset: factor pose increment's covariance ###
    //C_D = Eigen::Matrix3d::Zero(3, 3);
  }

private:
  ros::NodeHandle n;
  ros::Publisher delta_pub, Delta_pub, global_pub, C_D_pub;
  ros::Subscriber vel_sub;
  ros::Time current_time, last_time;
  Eigen::Matrix3d C_v, C_D;
  double x, y, th;
  double vx, vy, vth;
  double Delta_x, Delta_y, Delta_th;

  geometry_msgs::Pose create_pose_msg(double x, double y, double th) {
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::Pose pose;

    pose.position.x  = x;
    pose.position.y  = y;
    pose.orientation = pose_quat;

    return pose;
  }

  std_msgs::String create_covariance_msg(Eigen::Matrix3d m) {
    std::stringstream ss;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
	ss << m(i, j) << " ";
      }
    }

    std_msgs::String covar;
    covar.data = ss.str().substr(0, ss.str().size()-1);

    return covar;
  }

  void callback(const geometry_msgs::Twist::ConstPtr& input) {
    vx  = input->linear.x;
    vy  = input->linear.y;
    vth = input->angular.z;
  }
};

int main(int argc, char **argv) {

  //### SET THESE USER-DEFINED VALUES ###
  double sigma_vx  = 0.1; // [m/s/sqrt(s)]
  double sigma_vy  = 0.1; // [m/s/sqrt(s)]
  double sigma_vth = 0.1; // [m/s/sqrt(s)]

  ros::init(argc, argv, "odometry");
  Odometry odom(sigma_vx, sigma_vy, sigma_vth);
  odom.run();
  return 0;
}
