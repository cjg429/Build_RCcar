//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "race/drive_param.h"

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
int margin = 6;
int K = 500;
double MaxStep = 1;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//control
nav_msgs::Path visual_path;

//robot
point robot_pose;
race::drive_param cmd;

//FSM state
int state;

//function definition
void set_waypoints();
void generate_path_RRT();
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<race::drive_param>("/drive_parameters",100); 
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/rrt_path",100);
    
    ros::Subscriber robot_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);
    printf("Initialize topics\n");

    // Load Map

    //char* user = getlogin();
    map = cv::imread((std::string("/home/scarab1/")+
                      std::string("catkin_ws/src/hwsetup/src/dongacpsfuck.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -9.1;
    world_x_max = 9.1;
    world_y_min = -6.225;
    world_y_max = 6.225;
    res = 0.05;
    printf("Load map\n");


     if(! map.data )                              // Check for invalid input
    {
        printf("Could not open or find the image\n");
        return -1;
    }

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    int look_ahead_idx;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;
	    printf("path size : %d\n", path_RRT.size());
            state = RUNNING;
        } break;

        case RUNNING: {
		int current_goal = 1;
   		PID pid_ctrl;
                visual_path.poses.resize(path_RRT.size());
                for(int i =0; i < path_RRT.size(); i++){
                    visual_path.header.frame_id = "map";
		    visual_path.header.stamp = ros::Time::now();
		    visual_path.poses[i].pose.position.x = path_RRT[i].x;			
		    visual_path.poses[i].pose.position.y = path_RRT[i].y;
                }
		while(ros::ok()) {
                        path_pub.publish(visual_path);
			cmd.velocity = 17.0;
			point temp_goal;
			temp_goal.x = path_RRT[current_goal].x;
			temp_goal.y = path_RRT[current_goal].y;
			temp_goal.th = path_RRT[current_goal].th;
                        cmd.angle = -400*pid_ctrl.get_control(robot_pose, temp_goal)/3.141592;
			cmd_vel_pub.publish(cmd);

			float check_x = robot_pose.x - path_RRT[current_goal].x;
			float check_y = robot_pose.y - path_RRT[current_goal].y;
			if (fabs(check_x) < 0.5 && fabs(check_y) < 0.5) {
			    printf("arrived goal : %d with x : %f, y : %f \n", current_goal, fabs(check_x), fabs(check_y));
			    pid_ctrl.reset();
			    current_goal++;
			    if (current_goal == path_RRT.size()) {
				printf("reached all point");
			        state = FINISH;
				break;
			    }
			}
			ros::spinOnce();
			control_rate.sleep();
		}
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }
    }
    return 0;
}

void generate_path_RRT()
{

    rrtTree temp_tree;
    traj temp_traj;
    temp_traj.x = waypoints[0].x;
    temp_traj.y = waypoints[0].y;
    temp_traj.th = waypoints[0].th;	
    path_RRT.push_back(temp_traj);
    for(int i = 0; i < waypoints.size() - 1; i++) {
        temp_tree = rrtTree(waypoints[i],waypoints[i+1], map, map_origin_x, map_origin_y, res, margin); 
	temp_tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
	std::vector<traj> temp_traj = temp_tree.backtracking_traj();
        if(temp_traj.size() > 0) {
		point temp_point;
		temp_point.x = temp_traj[0].x;
		temp_point.y = temp_traj[0].y;
		temp_point.th = temp_traj[0].th;
		waypoints[i+1] = temp_point;
		for(int j = 0; j < temp_traj.size(); j++) {
		    path_RRT.push_back(temp_traj[temp_traj.size() - j - 1]);
		}
        }
    }
    //temp_tree.visualizeTree(path_RRT);
    sleep(1);
}

void set_waypoints()
{
    point waypoint_candid[8];
    
    waypoint_candid[0].x = 7.39;
    waypoint_candid[0].y = -1.52;
    waypoint_candid[0].th = 3.141592;
    waypoint_candid[1].x = -5.145;
    waypoint_candid[1].y = -4.328;
    waypoint_candid[2].x = -6.111;
    waypoint_candid[2].y = 0.781;
    waypoint_candid[3].x = 6.499;
    waypoint_candid[3].y = 3.018;    
    waypoint_candid[4].x = 6.206;
    waypoint_candid[4].y = -1.837;
    waypoint_candid[5].x = -3.564;
    waypoint_candid[5].y = -1.013;
    waypoint_candid[6].x = 4.290;
    waypoint_candid[6].y = 0.518;
    waypoint_candid[7].x = 1.381;
    waypoint_candid[7].y = -2.668;

    int order[] = {0,1,2,3,4,5,6,7};
    int order_size = 8;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
}

void setcmdvel(double vel, double deg){
    cmd.velocity = vel;
    cmd.angle = deg;
}
