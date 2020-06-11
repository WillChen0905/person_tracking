#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class Person_Tracking {
public:
    Person_Tracking ( ros::NodeHandle& nodehandle )
            : nh_( nodehandle )
    {
        camera_action_sub_ = nh_.subscribe( "person_tracking_action", 10, &Person_Tracking::CameraActionCB, this );
        target_position_pub_ = nh_.advertise< geometry_msgs::Point > ( "target_point", 10 );
        aiv_position_pub_ = nh_.advertise< geometry_msgs::Point > ( "aiv_point", 10 );
        simple_goal_pub_ = nh_.advertise< geometry_msgs::PoseStamped > ( "move_base_simple/goal", 10 );
    }
private:

    void CameraActionCB( const std_msgs::String& str )
    {
        bool is_Tracking = true;
        if ( !( str.data == "start" ) )
        {
            std::cout << "CameraTF_convert shutdown" << std::endl;
            camera_action_sub_.shutdown();
            is_Tracking = false;
        }

        while(is_Tracking)
        {
            tf2_ros::Buffer tfBuffer_ob;
            tf2_ros::Buffer tfBuffer_aiv;
            tf2_ros::TransformListener tfListener_ob(tfBuffer_ob);
            tf2_ros::TransformListener tfListener_aiv(tfBuffer_aiv);
            geometry_msgs::TransformStamped transformStamped_ob;
            geometry_msgs::TransformStamped transformStamped_aiv;
            bool is_Transform = true;

            do
            {
                try
                {
                    transformStamped_ob = tfBuffer_ob.lookupTransform("map", "object_0", ros::Time(0));
                    transformStamped_aiv = tfBuffer_aiv.lookupTransform("map", "base_link", ros::Time(0));
                    std::cout << "Get Transform" << std::endl;
                    is_Transform = false;
                }
                catch( tf2::TransformException& ex )
                {
                    ROS_WARN("%s", ex.what());
                }
                ros::Duration(0.2).sleep();
            }
            while( is_Transform );


            geometry_msgs::Pose tmp;
            geometry_msgs::Pose target_position;
            geometry_msgs::Pose aiv_position;
            geometry_msgs::Point target_point;
            geometry_msgs::Point aiv_point;

            tf2::doTransform( tmp, target_position, transformStamped_ob );
            target_point.x = target_position.position.x;
            target_point.y = target_position.position.y;
//            target_position_pub_.publish( target_point );

            tf2::doTransform( tmp, aiv_position, transformStamped_aiv );
            aiv_point.x = aiv_position.position.x;
            aiv_point.y = aiv_position.position.y;
//            aiv_position_pub_.publish( aiv_point );


            geometry_msgs::PoseStamped simple_goal;
            tf2::Quaternion quat_tf;
            geometry_msgs::Quaternion quat_msg;
            double diff_x = target_point.x - aiv_point.x;
            double diff_y = target_point.y - aiv_point.y;

            quat_tf.setRPY(0, 0, atan2( diff_y, diff_x ));
            tf2::convert( quat_tf, quat_msg );

            simple_goal.pose.position.x = target_point.x - 0.2;
            simple_goal.pose.position.y = target_point.y - 0.2;
            simple_goal.pose.orientation = quat_msg;
            simple_goal.header.frame_id = "map";



            simple_goal_pub_.publish( simple_goal );


        }
    }


    ros::NodeHandle nh_;
    ros::Subscriber camera_action_sub_;
    ros::Publisher target_position_pub_;
    ros::Publisher aiv_position_pub_;
    ros::Publisher simple_goal_pub_;
};// end of class


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Person_Tracking");
  ros::NodeHandle nh;
  Person_Tracking  PT(nh);
  ros::spin();
  return 0;
}
