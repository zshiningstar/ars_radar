#include "ars_radar.h"
#include <iostream>
#include <ros/ros.h>
#include <math.h>


using std::cout;
using std::endl;
using std::hex;

ArsRadar::ArsRadar()
{
    ars_objects_.header.frame_id = "ars_radar";
    output.header.frame_id = "ars_radar";
    //ars_object_car.header.frame_id = "ars_radar";
    ars_object_car.id = 255;
    ars_object_car.x_speed = 0;
    ars_object_car.y_speed = 0;
    ars_object_car.theta = 0;
    ars_object_car.distance = 100;
    ars_object_car.x = 0;
    ars_object_car.y = 0;
    ars_object_car.status = 0;
    ars_object_car.obj_type = 7;

    egoSpeed = 0;
    yawRate = 0;
    readCount = 0;
    pthread_rwlock_init(&egoSpeedLock,NULL);
    pthread_rwlock_init(&countLock,NULL);

    Infos.emplace_back("point",255,0,0);//红色
    Infos.emplace_back("car",255,125,0);//橙色
    Infos.emplace_back("truck",255,255,0);//黄色
    Infos.emplace_back("person",0,255,0);//绿色
    Infos.emplace_back("motorcycle",0,0,255);//蓝色
    Infos.emplace_back("bicycle", 0,255,255);//青色
    Infos.emplace_back("wide", 255,0,255);
    Infos.emplace_back("reserved", 125,125,125);
}

bool ArsRadar::init()
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("from_can_topic", from_can_topic_, "/from_usbcan");
    nh_private.param<std::string>("to_can_topic", to_can_topic_, "/to_usbcan");
    nh_private.param<std::string>("gpsUtm_topic", gpsUtm_topic, "/gpsUtm");

    nh_private.param<bool> ("is_sendMsgToArs",is_sendMsgToArs_,true);

	// 定义一个发布者，发布 ObjectArray，发布到 /ars_objects，这个是自定义消息类型
	pub_objects_ = nh.advertise<can_msgs::ObjectArray>("/ars_objects",10);

	// 发布离我最近的障碍物消息
	pub_object_  = nh.advertise<can_msgs::Object>("/ars_object",1);

    // 可视化
    pub_cloud_   = nh.advertise<sensor_msgs::PointCloud2>("/ars_cloud",10); 

	// 订阅者，订阅 from_can_topic_ 消息
	sub_can_     = nh.subscribe(from_can_topic_,1,&ArsRadar::canMsg_callback, this);
    // 订阅者，订阅 gpsUtm_topic 消息
    //sub_gps_     = nh.subscribe(gpsUtm_topic,1,&ArsRadar::gps_callback, this);


    return true;
}

void ArsRadar::run()
{
	if(is_sendMsgToArs_)
	{
		ros::NodeHandle nh;
		// 定义一个发布者，发布 FrameArray，发布到 to_can_topic_
		pub_can_ = nh.advertise<can_msgs::FrameArray>(to_can_topic_,1);
        pub_can_config = nh.advertise<can_msgs::FrameArray>(to_can_topic_,1);

        timer_1 = nh.createTimer(ros::Duration(1),&ArsRadar::configArs,this);

		timer_2 = nh.createTimer(ros::Duration(0.05),&ArsRadar::sendMsgToArs,this);

	}

    ros::MultiThreadedSpinner s(4);
	ros::spin(s);
}

// 收到gps消息后进入回调函数
//void ArsRadar::gps_callback(const gps_msgs::gpsUtm::ConstPtr& msg)
//{
//    pthread_rwlock_rdlock(&egoSpeedLock);
//    egoSpeed = sqrt(pow(msg->north_velocity,2) + pow(msg->east_velocity,2) + pow(msg->up_velocity, 2));
//    yawRate = -1 * msg->angle_velocity_z * 180 / M_PI; // 左转为正
//    pthread_rwlock_unlock(&egoSpeedLock);
//}

// 收到CAN分析仪的消息后进入回调
void ArsRadar::canMsg_callback(const can_msgs::FrameArray::ConstPtr& msg)
{
    // ROS_INFO("***********************");
	// 如果不是通道1发过来的数据，直接返回，因为通道1是从毫米波雷达接收数据
	// // 通道2没有连接
	if(msg->header.frame_id != "channel1") 
		return;
    int n = msg->frames.size();
    // ROS_INFO("nums of frame = %d", n);
    static int NumObjects = -1; // 设置为静态变量的原因：在下次进入回调的时候，能够保持上一时期的NumObjects
    // ROS_INFO("nums of shoule Objects = %d", NumObjects);
    int MeasCounter = 0;
    static int numsOfFrameD = 0; // 用来统计 0x60D的个数
    // ROS_INFO("nums of frameD = %d", numsOfFrameD);
    int CurObjectsNumsInVec = ars_objects_.objects.size();
    // ROS_INFO("nums of now Objects = %d", CurObjectsNumsInVec);
	for(int i=0; i<n; ++i)
    {
        // ROS_INFO("0X%X", msg->frames[i].id);
        if (0x201 == msg->frames[i].id)
        {
            // ROS_INFO("0X201");
            int RadarState_NVMwriteStatus = (msg->frames[i].data[0] & 0x80) >> 7;
            int RadarState_NVMreadStatus = (msg->frames[i].data[0] & 0x40) >> 6;
            int RadarState_MaxDistanceCfg = ((msg->frames[i].data[1] << 2) + (msg->frames[i].data[2] >> 6)) * 2;
            int RadarState_OutputTypeCfg  = (msg->frames[i].data[5] & 0x0C) >> 2;
            int RadarState_MotionRxState = (msg->frames[i].data[5] & 0xC0) >> 6;

            // ROS_INFO("NVMwriteStatus:%d, NVMreadStatus:%d, MaxDistance:%d, OutputType:%d, MotionRxState:%d", 
            //         RadarState_NVMwriteStatus,RadarState_NVMreadStatus,RadarState_MaxDistanceCfg, RadarState_OutputTypeCfg, RadarState_MotionRxState);
        }
        else if (msg->frames[i].id == 0x203)
        {
            int numsFilterofObj = (msg->frames[i].data[1] >> 3) & 0x1F;
            int numsFilterofClu = (msg->frames[i].data[0] >> 3) & 0x1F;
        }

        else if (msg->frames[i].id == 0x204)
        {
            // ROS_INFO("0X204");
            // ROS_INFO("********************************");
            int index = (msg->frames[i].data[0] >> 3) & 0x0F;
            if (index == 2)
            {
                int thetaMinH = (msg->frames[i].data[1] & 0x0F) << 8;
                int thetaMinL = msg->frames[i].data[2] & 0xFF;
                double thetaMin = (thetaMinH + thetaMinL) * 0.025 - 50;

                int thetaMaxH = (msg->frames[i].data[3] & 0x0F) << 8;
                int thetaMaxL = msg->frames[i].data[4] & 0xFF;
                double thetaMax = (thetaMinH + thetaMaxL) * 0.025 - 50;
                // ROS_INFO("theta = [%.1f, %.1f]",thetaMin, thetaMax);
            }
            else if (index == 9)
            {
                int yMinH = (msg->frames[i].data[1] & 0x0F) << 8;
                int yMinL = msg->frames[i].data[2] & 0xFF;
                double yMin = (yMinH + yMinL) * 0.2 - 409.5;

                int yMaxH = (msg->frames[i].data[3] & 0x0F) << 8;
                int yMaxL = msg->frames[i].data[4] & 0xFF;
                double yMax = (yMaxH + yMaxL) * 0.2 - 409.5;
                // ROS_INFO("Y = [%.1f, %.1f]",yMin, yMax);
            }
        }

        // 如果遇到一个0x60a，说明即将开始一轮新的障碍物，这时，如果 NumObjects == ars_objects_.size(),说明收集全了ox60b
        // 如果 numsOfFrameD == NumObjects 说明收集全了0x60d,并且0x60b和0x60d是对应的
        // 说明上一轮的数据是全的，就在开始新的一轮之前publish，并清空数据
        else if (0x60A == msg->frames[i].id)
        {
//            ROS_INFO("0X60A");
            if (NumObjects == ars_objects_.objects.size() && numsOfFrameD == NumObjects)
            {
                ROS_INFO("***********************");
                pub_objects_.publish(ars_objects_);
                pcl::toROSMsg(cloud, output);
                output.header.frame_id = "ars_radar";
                
                pub_cloud_.publish(output);
                for (int i = 0; i< ars_objects_.objects.size(); ++i)
                {
                    // ROS_INFO("%.1f, %.1f, %.1f, %d, %d", ars_objects_.objects[i].y, ars_objects_.objects[i].x, ars_objects_.objects[i].distance,ars_objects_.objects[i].obj_type, ars_objects_.objects[i].id);
                    // ROS_INFO("%.1f", ars_objects_.objects[i].y);
                    // 0 点目标；1 小汽车；2 卡车/客车； 3 行人； 4 摩托车； 5 自行车； 6 宽大目标 7 保留
                    double relV_x = ars_objects_.objects[i].x_speed;
                    double relV_y = ars_objects_.objects[i].y_speed;
                    double relV = relV_x > 0 ? sqrt(pow(relV_x, 2) + pow(relV_y, 2)) : -1 * sqrt(pow(relV_x, 2) + pow(relV_y, 2)); //   相对速度

                    pthread_rwlock_rdlock(&countLock);
                    if (readCount == 0)
                        pthread_rwlock_rdlock(&egoSpeedLock);
                    readCount++;
                    pthread_rwlock_unlock(&countLock);
                    double curEgoSpeed = egoSpeed;
                    pthread_rwlock_rdlock(&countLock);
                    readCount--;
                    if (readCount == 0)
                        pthread_rwlock_unlock(&egoSpeedLock);
                    pthread_rwlock_unlock(&countLock);

                    double preV = relV + curEgoSpeed; // 前车的绝对速度
                    // ROS_INFO("curEgoSpeed = %.1f", curEgoSpeed);
                    // ROS_INFO("relV = %.1f", relV);
                    // ROS_INFO("preV = %.1f", preV);
                    ROS_INFO("%.1f, %.1f, %.1f, %u, %.1f", ars_objects_.objects[i].y, ars_objects_.objects[i].x, ars_objects_.objects[i].distance,ars_objects_.objects[i].obj_type, preV * 3.6);
                    //if ((ars_objects_.objects[i].y < 0.8 && ars_objects_.objects[i].y > -1.5) && ars_objects_.objects[i].x > 0.3 && preV > -0.2)
                    //此处计算出来距离我最近障碍物的距离信息
                    if ((ars_objects_.objects[i].y < 0.5 && ars_objects_.objects[i].y > -0.5) && ars_objects_.objects[i].x > 0.25 && preV > -0.25)
                    // if (abs(ars_objects_.objects[i].y) < 2 && ars_objects_.objects[i].x > 0.3)
                    // if (abs(ars_objects_.objects[i].y) < 3 )
                    {
                        // ROS_INFO("%.1f, %.1f, %.1f, %d, %d", ars_objects_.objects[i].y, ars_objects_.objects[i].x, ars_objects_.objects[i].distance,ars_objects_.objects[i].obj_type, ars_objects_.objects[i].id);
                        // ROS_INFO("%.1f, %.1f, %d, %d", ars_object_car.y, ars_object_car.x, ars_object_car.obj_type, ars_object_car.id);
                        if (ars_objects_.objects[i].distance < ars_object_car.distance)
                        {
                            ars_object_car.distance = ars_objects_.objects[i].distance;
                            ars_object_car.x = ars_objects_.objects[i].x;
                            ars_object_car.y = ars_objects_.objects[i].y;
                            ars_object_car.theta = ars_objects_.objects[i].theta;
                            ars_object_car.x_speed = ars_objects_.objects[i].x_speed;
                            ars_object_car.y_speed = ars_objects_.objects[i].y_speed;
                            ars_object_car.speed = preV;
                            // ROS_INFO("***");
                            // ROS_INFO("preV = %.1f", preV);
                            // ROS_INFO("***");
                            ars_object_car.status = ars_objects_.objects[i].status;
                            ars_object_car.obj_type = ars_objects_.objects[i].obj_type;
                            ars_object_car.id = ars_objects_.objects[i].id;
                        }
                    }
        
                }
                ROS_INFO("y = %.1f, x = %.1f, dis = %.1f, type = %u, id = %u, speed = %.1f", ars_object_car.y, ars_object_car.x, ars_object_car.distance,ars_object_car.obj_type, ars_object_car.id, ars_object_car.speed);
                // ROS_INFO("%.1f, %.1f, %d, %d", ars_object_car.y, ars_object_car.x, ars_object_car.obj_type, ars_object_car.id);
                // ROS_INFO("ars_object_car_ID = %d", ars_object_car.id);
                //ars_object_car.header.stamp = ars_objects_.header.stamp;
                pub_object_.publish(ars_object_car);
                // 找到一组完整数据之后，就清空
                //ars_object_car.header.frame_id = "ars_radar";
                ars_object_car.id = 255;
                ars_object_car.x_speed = 0;
                ars_object_car.y_speed = 0;
                ars_object_car.speed = 0;
                ars_object_car.theta = 0;
                ars_object_car.distance = 100;
                ars_object_car.x = 0;
                ars_object_car.y = 0;
                ars_object_car.status = 0;
                ars_object_car.obj_type = 7;
            }
            ars_objects_.objects.clear();
            MapObjectId2indexInObjectArray.clear();
            cloud.points.clear();
            numsOfFrameD = 0;
            NumObjects = msg->frames[i].data[0];
            int MeasCounterH = msg->frames[i].data[1] << 8;
            int MeasCounterL = msg->frames[i].data[2];
            MeasCounter = (MeasCounterH + MeasCounterL);
            // ROS_INFO("MeasCounter = %d", MeasCounter);
        }
        else if (msg->frames[i].id == 0x60B)
        {
//            ROS_INFO("0X60b");
            uint8_t  id         = msg->frames[i].data[0]; //id
            float xh_pos        = msg->frames[i].data[1] << 5;
            float xl_pos        = msg->frames[i].data[2] >> 3;
            float x_pos         = (xh_pos + xl_pos) * 0.2 -500; //distance_x
            float yh_pos        = (msg->frames[i].data[2] & 0x07) << 8;
            float yl_pos        = msg->frames[i].data[3];
            float y_pos         = (yh_pos + yl_pos) * 0.2 -204.6;//distance_y
            float xh_speed      = msg->frames[i].data[4] << 2;
            float xl_speed      = msg->frames[i].data[5] >> 6;
            float x_speed       = (xh_speed + xl_speed) * 0.25 - 128; //speed
            float yh_speed      = (msg->frames[i].data[5] & 0x3f) << 3;
            float yl_speed      = msg->frames[i].data[6] >> 5;
            float y_speed       =  (yh_speed + yl_speed) * 0.25 - 64; //speed
            uint8_t  t_status       = (msg->frames[i].data[6] & 0x07);
            float distance      = sqrt((pow(x_pos,2) + pow(y_pos,2)));
            float theta         = atan2(y_pos, x_pos);

            ars_object_.id          = id;
            ars_object_.x		    = x_pos;
            ars_object_.y			= y_pos;
            ars_object_.x_speed 	= x_speed;
            ars_object_.y_speed     = y_speed;
            ars_object_.theta		= theta;
            ars_object_.distance    = distance;
            ars_object_.status      = t_status;
            ars_objects_.objects.push_back(ars_object_);

            point.x = x_pos;
            point.y = y_pos;
            point.z = 1.0;
            cloud.points.push_back(point);

            MapObjectId2indexInObjectArray.insert(std::make_pair(id, ars_objects_.objects.size()-1));
            if (ars_objects_.objects.size() == 1)
            {
                ars_objects_.header.stamp = ros::Time::now(); // 时间戳
                pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
            }
        }
        else if (msg->frames[i].id == 0x60D)
        {
//             ROS_INFO("0X60d");
            uint8_t obj_id = msg->frames[i].data[0];
            uint8_t obj_type = msg->frames[i].data[3] & 0x07; 
            if (MapObjectId2indexInObjectArray.find(obj_id) != MapObjectId2indexInObjectArray.end())
            {
                numsOfFrameD++;
                ars_objects_.objects[ MapObjectId2indexInObjectArray[obj_id] ].obj_type = obj_type;

                cloud.points[ MapObjectId2indexInObjectArray[obj_id] ].r = Infos[obj_type].r;
                cloud.points[ MapObjectId2indexInObjectArray[obj_id] ].g = Infos[obj_type].g;
                cloud.points[ MapObjectId2indexInObjectArray[obj_id] ].b = Infos[obj_type].b;
            }
        }
    }
}

// 发送车速和横摆角速度信息
void ArsRadar::sendMsgToArs(const ros::TimerEvent&)
{
//     ROS_INFO("111111111111111111111");
    can_msgs::FrameArray frame_array;

    can_msgs::Frame frameSpeed;
    frameSpeed.id = 0x300;
    frameSpeed.len = 2;

    pthread_rwlock_rdlock(&countLock);
    if (readCount == 0)
        pthread_rwlock_rdlock(&egoSpeedLock);
    readCount++;
    pthread_rwlock_unlock(&countLock);

    double curEgoSpeedCfg = egoSpeed;
    double curYawRateCfg = yawRate;

    pthread_rwlock_rdlock(&countLock);
    readCount--;
    if (readCount == 0)
        pthread_rwlock_unlock(&egoSpeedLock);
    pthread_rwlock_unlock(&countLock);

    if (curEgoSpeedCfg < 0.1)
    {
        frameSpeed.data[0] = 0x00;
    }
    else
    {
        frameSpeed.data[0] = 0x40;

    }
    uint16_t egoSpeedinCanTable = uint16_t(curEgoSpeedCfg / 0.02);
    frameSpeed.data[0] |= (egoSpeedinCanTable >> 8) & 0x1F;
    frameSpeed.data[1] = egoSpeedinCanTable & 0xFF;
    frameSpeed.is_rtr = false;
    frameSpeed.is_extended = false;
    frameSpeed.is_error = false;
    frame_array.frames.push_back(frameSpeed);


    can_msgs::Frame frameYawRate;
    frameYawRate.id = 0x301;
    frameYawRate.len = 2;
    uint16_t YawRateinCanTable = (curYawRateCfg + 327.68) / 0.01;
    frameYawRate.data[0] = YawRateinCanTable >> 8;
    frameYawRate.data[1] = YawRateinCanTable;
    frameYawRate.is_rtr = false;
    frameYawRate.is_extended = false;
    frameYawRate.is_error = false;
    frame_array.frames.push_back(frameYawRate);

    frame_array.header.frame_id = "channel1";

    pub_can_.publish(frame_array);
}

// 发送配置信息
void ArsRadar::configArs(const ros::TimerEvent&)
{
    can_msgs::FrameArray frame_array;

    can_msgs::Frame frameCfg;
    frameCfg.id = 0x200;
    frameCfg.len = 8;
    // 是否可以更改的标志位
    // 1111 1001 存储到nvm、排序、扩展信息、质量信息、  输出模式、雷达功率、传感器id、最大距离
    frameCfg.data[0] = 0xF9;
    double Maxdistance = 200;
    uint16_t MaxdistanceInCanTable = Maxdistance / 2;
    frameCfg.data[1] = (MaxdistanceInCanTable >> 2) & 0xFF;
    frameCfg.data[2] = (MaxdistanceInCanTable << 6) & 0xC0;
    frameCfg.data[4] = 0x08;
    frameCfg.data[5] = 0x18; // 0001 1000 不存储到nvm,采用距离排序，不输出扩展信息
    frameCfg.data[6] = 0x00;
    frameCfg.data[7] = 0x00;
    frameCfg.is_rtr = false;
    frameCfg.is_extended = false;
    frameCfg.is_error = false;
    frame_array.frames.push_back(frameCfg);

    can_msgs::Frame frameFilterY;
    frameFilterY.id = 0x202;
    frameFilterY.len = 5;
    frameFilterY.data[0] = 0xCE;// 1 1001 1 1 0 
    double yMin = -3;
    uint16_t yMinInCanTable = ceil((yMin + 409.5) / 0.2);
    frameFilterY.data[1] = (yMinInCanTable >> 8) & 0x0F;
    frameFilterY.data[2] = yMinInCanTable & 0xFF;
    double yMax = 3;
    uint16_t yMaxInCanTable = ceil((yMax + 409.5) / 0.2);
    frameFilterY.data[3] = (yMaxInCanTable >> 8) & 0x0F;
    frameFilterY.data[4] = yMaxInCanTable & 0xFF;
    frameFilterY.is_rtr = false;
    frameFilterY.is_extended = false;
    frameFilterY.is_error = false;
    frame_array.frames.push_back(frameFilterY);

    can_msgs::Frame frameFilterTheta;
    frameFilterTheta.id = 0x202;
    frameFilterTheta.len = 5;
    frameFilterTheta.data[0] = 0x96; // 1 0010 1 1 0
    double thetaMin = -25;
    uint16_t thetaMinInCanTable = uint16_t((thetaMin + 50) / 0.025);
    frameFilterTheta.data[1] = (thetaMinInCanTable >> 8) & 0x0F;
    frameFilterTheta.data[2] = thetaMinInCanTable & 0xFF;
    double thetaMax = 25;
    uint16_t thetaMaxInCanTable = uint16_t((thetaMax + 50) / 0.025);
    frameFilterTheta.data[3] = (thetaMaxInCanTable >> 8) & 0x0F;
    frameFilterTheta.data[4] = thetaMaxInCanTable & 0xFF;
    frameFilterTheta.is_rtr = false;
    frameFilterTheta.is_extended = false;
    frameFilterTheta.is_error = false;
    frame_array.frames.push_back(frameFilterTheta);

    frame_array.header.frame_id = "channel1";
    pub_can_config.publish(frame_array);

}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"ars_radar_node");
	ArsRadar app;
	
	if(app.init())
		app.run();
	
	return 0;

}
