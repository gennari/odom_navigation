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
    std::string _action_name, _action_result;
    // create messages that are used to published feedback/result
    move_base_msgs::MoveBaseActionFeedback _feedback;
    move_base_msgs::MoveBaseActionResult _result;
    tf::TransformListener* listener;
    geometry_msgs::PoseStamped goal_,goal_b;
    nav_msgs::Odometry pose_,old_pose;
    geometry_msgs::Twist velocity;
    bool _have_goal;
    float alpha,tv_kp,tv_kd,tv_ki,rv_kp,rv_kd,rv_ki,_max_tv,_max_rv;
    pid::PID pid_x_, pid_y_;
    ros::Publisher cmd_vel_pub;
ros::Subscriber pose_sub_,goal_sub;
public:

    OdomAction(std::string name) :
        _as(nh_, "move_base", boost::bind(&OdomAction::executeCB, this, _1), false),_action_name("move_base")

    {

        _as.start();



        listener=new tf::TransformListener(nh_,ros::Duration(10));
        listener->waitForTransform("/odom","/base_link",ros::Time::now(),ros::Duration(10.0));


        alpha=0.5;
        old_pose.pose.pose.orientation.w=1;
        old_pose.header.frame_id="/odom";

        //Topics
        pose_sub_ = nh_.subscribe("/odom", 1, &OdomAction::odometryCallback,this);
        goal_sub =  nh_.subscribe("move_base_simple/goal", 2, &OdomAction::setGoalCallback, this);
        cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        nh_.param("/tv/kp", tv_kp, 1.0f);
        nh_.param("/tv/kd", tv_kd, 1.0f);
        nh_.param("/tv/ki", tv_ki, 1.0f);

        nh_.param("/rv/kp", rv_kp, 0.5f);
        nh_.param("/rv/kd", rv_kd, 0.2f);
        nh_.param("/rv/ki", rv_ki, 1.0f);

        nh_.param("max_rv", _max_rv, 1.0f);
        nh_.param("max_tv", _max_tv, 1.0f);

        //ros::spin();

        _have_goal=false;


        pid_x_.paramInit(tv_kp,tv_kd,tv_ki);
        pid_y_.paramInit(rv_kp,rv_kd,rv_ki);

    }

    ~OdomAction(void)
    {
    }

    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr& goal)
    {

        geometry_msgs::PoseStamped ps = goal->target_pose;

        //geometry_msgs::PoseStampedConstPtr  ps_ptr (new geometry_msgs::PoseStamped(ps));
        _action_result="";
        setGoalCallback(ps);
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

        _as.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");

    }
    void setGoalCallback(const geometry_msgs::PoseStamped& msg) {

         ROS_INFO("SETTING POSE X: %f Y: %f THETA: %f",msg.pose.position.x,msg.pose.position.y,tf::getYaw(msg.pose.orientation));

        //listener->transformPose("/base_link",ros::Time(0),msg,msg.header.frame_id,goal_b);
         goal_=msg;
        ROS_INFO("SET POSE X: %f Y: %f THETA: %f",msg.pose.position.x,msg.pose.position.y,tf::getYaw(msg.pose.orientation));
        _have_goal=true;
    }
    void run(){
        ros::Rate r(10);
        while(ros::ok()){
            if (_have_goal){
                listener->transformPose("/base_link",ros::Time(0),goal_,goal_.header.frame_id,goal_b);



                float tv = pid_x_.getCommand(goal_b.pose.position.x, velocity.linear.x,0.005);
                float rv = pid_y_.getCommand(goal_b.pose.position.y,velocity.angular.z,0.005);


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

                if( ((pose_.pose.pose.position.x- goal_.pose.position.x)*(pose_.pose.pose.position.x- goal_.pose.position.x) +
                     (pose_.pose.pose.position.y- goal_.pose.position.y)*(pose_.pose.pose.position.y- goal_.pose.position.y)) <= 0.1 )
                {

                    ROS_INFO("reached");
                    _action_result="SUCCEEDED";
                    _have_goal=false;
                    cmd_vel_.linear.x = 0;
                    cmd_vel_.angular.z = 0;

                    cmd_vel_pub.publish(cmd_vel_);

                }
            }
            r.sleep();
            ros::spinOnce();
        }
    }

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

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client");

    OdomAction odom(ros::this_node::getName());
    odom.run();
    return 0;
}
