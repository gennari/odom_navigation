#include "ros/ros.h"
#include <cstdlib>
#include "odom_navigation/PID.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "move_base_msgs/MoveBaseAction.h"


class OdomAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> _as;
  std::string _action_name;
  // create messages that are used to published feedback/result
  move_base_msgs::MoveBaseActionFeedback _feedback;
  move_base_msgs::MoveBaseActionResult _result;

public:

  OdomAction(std::string name) :
    _as(nh_, name, boost::bind(&OdomAction::executeCB, this, _1), false),
    _action_name(name)
  {
    _as.start();
  }

  ~OdomAction(void)
  {
  }

  void executeCB(const move_base_msgs::MoveBaseActionGoalConstPtr &goal)
  {

      geometry_msgs::PoseStamped ps = move_base_goal->target_pose;

      geometry_msgs::PoseStampedConstPtr  ps_ptr (new geometry_msgs::PoseStamped(ps));
      _action_result="";
      setGoalCallback(ps_ptr);
      move_base_msgs::MoveBaseFeedback feed;
      ros::Rate r(10); // 10 hz
      while(_action_result=="" && _as.isActive()){
          r.sleep();
          _as.publishFeedback(feed);
      }
      ROS_INFO("Action finished with result: %s",_action_result.c_str());
      if(_action_result=="SUCCEEDED"){
          _as.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
      }else{
          _as.setAborted(move_base_msgs::MoveBaseResult(), _action_result);
      }
  }
  void setGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

      tf::Pose pose;
      tf::poseMsgToTF(msg->pose, pose);
      Eigen::Vector3f new_pose(pose.getOrigin().x(),
                               pose.getOrigin().y(),
                               getYaw(pose));

      ROS_INFO("Setting goal (%.6f): %.3f %.3f %.3f",
               ros::Time::now().toSec(),
               new_pose.x(),
               new_pose.y(),
               new_pose.z());
      Eigen::Isometry2f inverse_origin=v2t(_map_origin).inverse();
      Eigen::Isometry2f global_pose=v2t(new_pose);
      Eigen::Vector3f map_pose=t2v(inverse_origin*global_pose);
      setGoal(map_pose);
      _have_goal=true;
  }

};


nav_msgs::Odometry pose_,old_pose;
geometry_msgs::Twist velocity;
float alpha,tv_kp,tv_kd,tv_ki,rv_kp,rv_kd,rv_ki,_max_tv,_max_rv;
geometry_msgs::PointStamped goal,goal_b;
tf::TransformListener* listener;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{       pose_ = *msg;
        // transform  point.
        geometry_msgs::PoseStamped in,out;
        in.header=old_pose.header;
        in.pose=old_pose.pose.pose;

        listener->transformPose("/base_link",ros::Time(0),in,"/odom",out);

        velocity.linear.x=alpha*velocity.linear.x+(1-alpha)*(-1*out.pose.position.x)/(old_pose.header.stamp-pose_.header.stamp).toSec();
        velocity.angular.z=alpha*velocity.angular.z+(1-alpha)*(-1*tf::getYaw(out.pose.orientation))/(old_pose.header.stamp-pose_.header.stamp).toSec();

        old_pose=pose_;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    listener=new tf::TransformListener(n,ros::Duration(10));
    listener->waitForTransform("/odom","/base_link",ros::Time::now(),ros::Duration(10.0));


    alpha=0.5;
    old_pose.pose.pose.orientation.w=1;
    old_pose.header.frame_id="/odom";
    srand((unsigned)time(NULL));
    //Services
    //    ros::ServiceClient spawn_circles_srv = n.serviceClient<turtlesim::SpawnCircle>("/spawnCircle");
    //    ros::ServiceClient remove_circles_srv = n.serviceClient<turtlesim::RemoveCircle>("/removeCircle");
    //    ros::ServiceClient get_circles_srv = n.serviceClient<turtlesim::GetCircles>("/getCircles");
    //    ros::ServiceClient clear_circles_srv = n.serviceClient<std_srvs::Empty>("/clear");

    //Topics
    ros::Subscriber pose_sub_ = n.subscribe("/odom", 1, odometryCallback);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    nh.param("/tv/kp", tv_kp, 1.0f);
    nh.param("/tv/kd", tv_kd, 1.0f);
    nh.param("/tv/ki", tv_ki, 1.0f);

    nh.param("/rv/kp", rv_kp, 0.5f);
    nh.param("/rv/kd", rv_kd, 0.2f);
    nh.param("/rv/ki", rv_ki, 1.0f);

    nh.param("max_rv", _max_rv, 1.0f);
    nh.param("max_tv", _max_tv, 0.5f);

    //ros::spin();


    pid::PID pid_x_, pid_y_;

    pid_x_.paramInit(tv_kp,tv_kd,tv_ki,0.005);
    pid_y_.paramInit(rv_kp,rv_kd,rv_ki,0.005);
    goal.header.stamp=ros::Time::now();
    goal.header.frame_id="/odom";
    goal.point.x=std::rand()%20-40;
    goal.point.y=std::rand()%20-10;

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        listener->transformPoint("/base_link",ros::Time(0),goal,"/odom",goal_b);


        float tv = pid_x_.getCommand(goal_b.point.x, velocity.linear.x);
        float rv = pid_y_.getCommand(goal_b.point.y,velocity.angular.z);


        geometry_msgs::Twist cmd_vel_;
        if (fabs(rv)>_max_rv){
            float scale=_max_rv/fabs(rv);
            tv*=scale;
            rv*=scale;
        }

        if (fabs(tv)>_max_tv){
            float scale=_max_tv/fabs(tv);
            tv*=scale;
            rv*=scale;
        }
        cmd_vel_.linear.x = tv;
        cmd_vel_.angular.z = rv;

        cmd_vel_pub.publish(cmd_vel_);

        if( ((pose_.pose.pose.position.x- goal.point.x)*(pose_.pose.pose.position.x- goal.point.x) +
             (pose_.pose.pose.position.y- goal.point.y)*(pose_.pose.pose.position.y- goal.point.y)) <= 0.1 )
        {

            ROS_INFO("reached");
            goal.point.x=std::rand()%20-40;
            goal.point.y=std::rand()%20-10;

        }
        loop_rate.sleep();

    }
    return 0;
}
