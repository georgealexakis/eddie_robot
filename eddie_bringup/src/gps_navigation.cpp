#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <iostream>
#include <string>

// Create a type definition for a client called MoveBaseClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GPSNavigator
{
private:
    // Variables
    ros::NodeHandle n;
    ros::Publisher pub_status;
    ros::Subscriber sub_points;
    std::vector<std::pair<double, double>>::iterator iter;
    geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
    int count = 0, wait_count = 0;
    double latiGoal, longiGoal, latiNext, longiNext;
    std::string utm_zone;
    std::string path_local, path_abs;

public:
    GPSNavigator()
    {
        ROS_INFO("Initiated GPS navigation node");
        // Initiate publisher to send end of node message
        pub_status = n.advertise<std_msgs::Bool>("/gps_navigation/status", 10);
        // Subscriber for GPS points
        sub_points = n.subscribe("/gps_navigation/points", 10, &GPSNavigator::pointsCallback, this);
    }

    // Callback to receive GPS points
    void pointsCallback(const std_msgs::String::ConstPtr &msg)
    {
        // Variables
        int pointsCounter = 0;
        // Handle incomming coordinates
        std::vector<std::pair<double, double>> points;
        std::vector<int> coordinatesCounter;
        std::string coordinates = msg->data.c_str();

        // Separate pairs of coordinates
        for (int i = 0; i < coordinates.length(); i++)
        {
            if (coordinates[i] == '|')
            {
                coordinatesCounter.push_back(i);
            }
        }

        // Number of points
        pointsCounter = coordinatesCounter.size() - 1;
        if (pointsCounter > 0)
        {
            ROS_INFO("Valid incoming coordinates");
            // Translate string to coordinates vector
            for (int i = 0; i < pointsCounter; i++)
            {
                std::string record = coordinates.substr(coordinatesCounter[i] + 1, coordinatesCounter[i + 1] - coordinatesCounter[i] - 1);
                std::size_t pos = record.find("&");
                std::string lat = record.substr(0, pos);
                std::string lng = record.substr(pos + 1, record.length() - 1 - pos);
                points.push_back(std::make_pair(std::stod(lat), std::stod(lng)));
            }

            std::cout << pointsCounter << " incoming coordinates" << std::endl;
            std::vector<std::pair<double, double>>::iterator pointIterator;
            for (pointIterator = points.begin(); pointIterator < points.end(); pointIterator++)
            {
                std::cout << pointIterator->first << " " << pointIterator->second << std::endl;
            }

            // Set variables to init
            bool moveBaseStatus = false;
            count = 0;
            wait_count = 0;

            // Construct an action client that we use to communication with the action named move_base
            // Setting true is telling the constructor to start ros::spin()
            MoveBaseClient ac("/move_base", true);

            // Wait for the action server to come up
            while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok() && moveBaseStatus == false)
            {
                wait_count++;
                if (wait_count > 5)
                {
                    ROS_ERROR("Action server did not come up");
                    moveBaseStatus = true;
                }
                else
                {
                    ROS_INFO("Waiting for the move_base action server to come up");
                    moveBaseStatus = false;
                }
            }
            if (moveBaseStatus == false)
            {
                // Iterate through vector of waypoints for setting goals
                for (iter = points.begin(); iter < points.end(); iter++)
                {
                    // Setting goal
                    latiGoal = iter->first;
                    longiGoal = iter->second;
                    bool final_point = false;

                    if (iter < (points.end() - 1))
                    {
                        iter++;
                        latiNext = iter->first;
                        longiNext = iter->second;
                        iter--;
                    }
                    else
                    {
                        latiNext = iter->first;
                        longiNext = iter->second;
                        final_point = true;
                    }

                    ROS_INFO("Received Latitude goal:%.8f", latiGoal);
                    ROS_INFO("Received longitude goal:%.8f", longiGoal);

                    //Convert lat/long to utm:
                    UTM_point = latLongtoUTM(latiGoal, longiGoal);
                    UTM_next = latLongtoUTM(latiNext, longiNext);

                    // Transform UTM to map point in odom frame
                    map_point = UTMtoMapPoint(UTM_point);
                    map_next = UTMtoMapPoint(UTM_next);

                    // Build goal to send to move_base
                    move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point);

                    // Send Goal
                    ROS_INFO("Sending goal");
                    ac.sendGoal(goal); //push goal to move_base node

                    // Wait for result
                    // Waiting to see if move_base was able to reach goal
                    ac.waitForResult();

                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Robot has reached its goal!");
                        // Switch to next waypoint and repeat
                    }
                    else
                    {
                        ROS_ERROR("Robot was unable to reach its goal. GPS Waypoint unreachable.");
                        // Notify that waypoint following is complete
                        std_msgs::Bool navStatus;
                        navStatus.data = false;
                        pub_status.publish(navStatus);
                    }
                } // End for loop iterating through waypoint vector

                ROS_INFO("Robot has reached all of its goals!!!\n");
                // Notify that waypoint following is complete
                std_msgs::Bool navStatus;
                navStatus.data = true;
                pub_status.publish(navStatus);
            }
            else
            {
                std_msgs::Bool navStatus;
                navStatus.data = false;
                pub_status.publish(navStatus);
            }
        }
        else
        {
            ROS_INFO("No valid incoming coordinates");
        }
    }

    // Point construction
    geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
    {
        double utm_x = 0, utm_y = 0;
        geometry_msgs::PointStamped UTM_point_output;

        // Convert lat/long to utm
        RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

        // Construct UTM_point and map_point geometry messages
        UTM_point_output.header.frame_id = "utm";
        UTM_point_output.header.stamp = ros::Time(0);
        UTM_point_output.point.x = utm_x;
        UTM_point_output.point.y = utm_y;
        UTM_point_output.point.z = 0;

        return UTM_point_output;
    }

    // From GPS point to loal point
    geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
    {
        geometry_msgs::PointStamped map_point_output;
        bool notDone = true;
        tf::TransformListener listener;
        ros::Time time_now = ros::Time::now();
        while (notDone)
        {
            try
            {
                UTM_point.header.stamp = ros::Time::now();
                listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
                listener.transformPoint("odom", UTM_input, map_point_output);
                notDone = false;
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.01).sleep();
            }
        }
        return map_point_output;
    }

    // Build a goal
    move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point)
    {
        move_base_msgs::MoveBaseGoal goal;

        // Specify what frame we want the goal to be published in
        goal.target_pose.header.frame_id = "odom";
        goal.target_pose.header.stamp = ros::Time::now();

        // Specify x and y goal
        goal.target_pose.pose.position.x = map_point.point.x;
        goal.target_pose.pose.position.y = map_point.point.y;

        // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
        if (last_point == false)
        {
            tf::Matrix3x3 rot_euler;
            tf::Quaternion rot_quat;

            // Calculate quaternion
            float x_curr = map_point.point.x, y_curr = map_point.point.y;
            float x_next = map_next.point.x, y_next = map_next.point.y;
            float delta_x = x_next - x_curr, delta_y = y_next - y_curr;
            float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
            yaw_curr = atan2(delta_y, delta_x);

            // Specify quaternions
            rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
            rot_euler.getRotation(rot_quat);

            goal.target_pose.pose.orientation.x = rot_quat.getX();
            goal.target_pose.pose.orientation.y = rot_quat.getY();
            goal.target_pose.pose.orientation.z = rot_quat.getZ();
            goal.target_pose.pose.orientation.w = rot_quat.getW();
        }
        else
        {
            goal.target_pose.pose.orientation.w = 1.0;
        }

        return goal;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_navigator");
    GPSNavigator navigator;
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    ros::shutdown();
    std::cout << "GPS Navigator shutted down" << std::endl;
    return 0;
}