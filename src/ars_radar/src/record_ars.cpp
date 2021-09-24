#include <ros/ros.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "can_msgs/Object.h"
using namespace std;

FILE *fp;

void chatterCalllback_record_ars(const can_msgs::Object::ConstPtr& msg)
{
    fprintf(fp, "%.3f\t%u\t%u\t%u\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\n",msg->header.stamp.toSec(),msg->id,msg->obj_type,msg->status,msg->theta,
                    msg->x,msg->y,msg->distance,msg->x_speed, msg->y_speed, msg->speed);
    fflush(fp);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "record_ars_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    string FilePath = nh_private.param<string>("file_path","$(find ars_radar)/data/");
    string FileName = nh_private.param<string>("file_name","ars_radar");

    // 将当前时间作为文件名后缀
	time_t timep;
	time(&timep);
	string file_name_time = ctime(&timep);
	file_name_time.pop_back();
	string filetype = ".txt";
	FileName = FileName + " " + file_name_time + filetype;
    fp = fopen((FilePath+FileName).c_str(),"w");

	if(fp == NULL)
	{
		ROS_INFO("open record data file %s failed !!!",(FilePath+FileName).c_str());
		return false;
	}
    // id tpye status theta x y dis vx vy v
    fprintf(fp, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n","stamp","id","type","status","theta","x",
                    "y","distance","x_speed","y_speed","speed");

    ros::Subscriber sub_record_ars = nh.subscribe( nh_private.param<string>("ars_object","/ars_object"), 1, chatterCalllback_record_ars );
    ros::spin();
    return 0;
}
