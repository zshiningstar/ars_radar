#ifndef ARS_RADAR_H_
#define ARS_RADAR_H_
#include <ros/ros.h>
#include <string>
#include <vector>
#include <thread>
//#include"gps_msgs/gpsUtm.h"

#include <can_msgs/Object.h>
#include <can_msgs/ObjectArray.h>
#include <can_msgs/FrameArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <cmath>
#include <unordered_map>

class ArsRadar
{
public:
    ArsRadar();
    ~ArsRadar(){
		pthread_rwlock_destroy(&egoSpeedLock);
		pthread_rwlock_destroy(&countLock);
	};

    bool init();
    void run();
private:
    void sendMsgToArs(const ros::TimerEvent&);
    void configArs(const ros::TimerEvent&);
    void canMsg_callback(const can_msgs::FrameArray::ConstPtr& msg);
    //void gps_callback(const gps_msgs::gpsUtm::ConstPtr& msg);
    void parse_msg(const can_msgs::Frame &frame, int index, int n);
    void pubBoundingBoxArray();

	enum Cluster_DynProp
	{
		Target_Move = 0,   				//移动
		Target_Static = 1, 				//静止
		Target_Come = 2,   				//来向
		Target_May_Static = 3,			//可能静止
		Target_Unknow = 4,				//未知
		Target_Across = 5,				//横穿
		Target_Across_Static = 6,	    //横穿静止
		Target_Stop = 7					//停止
	};

	typedef struct _Info
	{
		std::string type;
		uint8_t r;
		uint8_t g;
		uint8_t b;
		_Info(const std::string& t, uint8_t _r, uint8_t _g, uint8_t _b)  
		{
			type = t;
			r = _r;
			g = _g;
			b = _b;
		}
	}Info;

    std::vector<Info> Infos;

private:
    ros::Subscriber sub_can_; // 订阅can分析仪消息
    ros::Subscriber sub_gps_;
    ros::Publisher  pub_can_; // 向can分析仪发布消息
    ros::Publisher  pub_can_config; // 向can分析仪发布配置消息
    ros::Publisher  pub_object_; // 发布离我最近的障碍物消息
    ros::Publisher  pub_objects_; // 发布所有障碍物消息

    ros::Publisher  pub_cloud_; // 发布点云消息

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointXYZRGB point; 

    sensor_msgs::PointCloud2 output;



    std::string from_can_topic_;
    std::string to_can_topic_;
    std::string gpsUtm_topic;


	can_msgs::ObjectArray ars_objects_;
	can_msgs::Object  ars_object_;
	can_msgs::Object  ars_object_car;

	bool is_sendMsgToArs_;
	
	ros::Timer timer_1, timer_2;

    std::unordered_map<int, int> MapObjectId2indexInObjectArray;

    double egoSpeed; // m/s
    double yawRate; // deg/s

	pthread_rwlock_t egoSpeedLock, countLock;
	int readCount;


};

#endif
