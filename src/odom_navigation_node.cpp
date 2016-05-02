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
    bool _have_goal, _pure_rotation;
    double alpha,tv_kp,tv_kd,tv_ki,rv_kp,rv_kd,rv_ki,_max_tv,_max_rv,_tresh,delta_t;
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
        nh_.param("tv_kp", tv_kp, 1.0);
        nh_.param("tv_kd", tv_kd, 0.5);
        nh_.param("tv_ki", tv_ki, 0.2);

        nh_.param("rv_kp", rv_kp, 0.1);
        nh_.param("rv_kd", rv_kd, 0.1);
        nh_.param("rv_ki", rv_ki, 0.1);

        nh_.param("max_rv", _max_rv, 1.0);
        nh_.param("max_tv", _max_tv, 0.3);
	nh_.param("tresh", _tresh, 0.001);

           
	ROS_INFO("_tresh:=%f",_tresh);
	ROS_INFO("_max_tv:=%f",_max_tv);
        //ros::spin();

        _have_goal=false;
  	 _pure_rotation=false;

        pid_x_.paramInit(tv_kp,tv_kd,tv_ki);
        pid_y_.paramInit(rv_kp,rv_kd,rv_ki);

    }

    ~OdomAction(void)
    {
    }

    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr& goal)
    {
	ROS_ERROR("ACTION CALLED");
        geometry_msgs::PoseStamped ps = goal->target_pose;

        //geometry_msgs::PoseStampedConstPtr  ps_ptr (new geometry_msgs::PoseStamped(ps));
        _action_result="";

        setGoalCallback(ps);
        move_base_msgs::MoveBaseFeedback feed;
        ros::Rate r(10); // 10 hz

        while(_action_result=="" && _as.isActive()){
	    // check that preempt has not been requested by the client
	      if (_as.isPreemptRequested() || !ros::ok())
	      {
		ROS_INFO("Preempted");
		// set the action state to preempted
		_action_result = "CANCELLED";
		_have_goal=false;
		break;
	      }

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
    void setGoalCallback(const geometry_msgs::PoseStamped& msg) {

         ROS_INFO("SETTING POSE X: %f Y: %f THETA: %f",msg.pose.position.x,msg.pose.position.y,tf::getYaw(msg.pose.orientation));

        listener->transformPose("/odom",ros::Time(0),msg,msg.header.frame_id,goal_);
         //goal_=msg;
        ROS_INFO("SET POSE X: %f Y: %f THETA: %f",goal_.pose.position.x,goal_.pose.position.y,tf::getYaw(msg.pose.orientation));
        _have_goal=true;
	_pure_rotation=false;
    }
    void run(){
        ros::Rate r(10);
        while(ros::ok()){
            if (_have_goal){
                listener->transformPose("/base_link",ros::Time(0),goal_,goal_.header.frame_id,goal_b);

                

                float des_yaw;
		if(_pure_rotation){
			des_yaw=tf::getYaw(goal_b.pose.orientation)*180/M_PI;
		}else{
			des_yaw = atan2(goal_b.pose.position.y,goal_b.pose.position.x)*180/M_PI;
		}
		//ROS_INFO("ANGLE: %f",des_yaw);
                float tv=0;
		float rv=0;
		if(fabs(des_yaw)<=5){
			tv= pid_x_.getCommand(goal_b.pose.position.x, velocity.linear.x,delta_t);		
		}
		if(fabs(des_yaw)>=1){		
                	rv = pid_y_.getCommand(des_yaw,velocity.angular.z,delta_t);
		}



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
		float distance=((pose_.pose.pose.position.x- goal_.pose.position.x)*(pose_.pose.pose.position.x- goal_.pose.position.x) +
                     (pose_.pose.pose.position.y- goal_.pose.position.y)*(pose_.pose.pose.position.y- goal_.pose.position.y));
		
                if( distance<= _tresh )
                {   
		    //angle=tf::getYaw(pose_.pose.pose.orientation)-tf::getYaw(goal_b.pose.orientation);
		/*    tf::Quaternion my,g;
		    tf::quaternionMsgToTF(pose_.pose.pose.orientation,my);
		    tf::quaternionMsgToTF(goal_b.pose.orientation,g);
 		    float a =my.angleShortestPath(g);
		    ROS_INFO("DISTANCE: %f",fabs(a));*/
		    if(fabs(tf::getYaw(goal_b.pose.orientation)*180/M_PI)<=2){
		            ROS_INFO("reached");
		            _action_result="SUCCEEDED";
		            _have_goal=false;
			    _pure_rotation=false;
		            cmd_vel_.linear.x = 0;
		            cmd_vel_.angular.z = 0;

		            cmd_vel_pub.publish(cmd_vel_);
		   }else{
			//ROS_INFO("PURE_ROTATION");
			_pure_rotation=true;
		   }
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
            delta_t=(old_pose.header.stamp-pose_.header.stamp).toSec();
            velocity.linear.x=alpha*velocity.linear.x+(1-alpha)*(-1*out.pose.position.x)/delta_t;
            velocity.angular.z=alpha*velocity.angular.z+(1-alpha)*(-1*tf::getYaw(out.pose.orientation))/delta_t;
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
