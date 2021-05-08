#include <iostream>
#include <ros/ros.h>
#include <replan_control/PredObsState.h>
#include <replan_control/PredStateArray.h>

#include <nav_msgs/Odometry.h>
#include <math.h>
#include <cmath>
#include <geometry_msgs/Twist.h>

#include <fstream>

//实现订阅两个话题
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>

using namespace message_filters;

class subreplan
{
private:
    /* data */
    ros::Subscriber sub_state;
    ros::Subscriber sub_cmd_vel_;

    ros::Publisher pub_cmd;
    replan_control::PredStateArray prestates;
    replan_control::PredObsState prestate;
    
    
    int direc;
    float distance;
    double rob_obs_angle;
    double obs_v_angle;
    double vz_add;
    geometry_msgs::Twist cmd_add;

    bool static_obstacle;
    
    int dynamic_vy;  //0为正转，1为反转，2保持不变
public:
    subreplan(ros::NodeHandle &nh);
    ~subreplan();
    // 判断距离是否超出阈值
    void replancallback(const replan_control::PredStateArray &in_prestates);
    // 获取当前机器人经过TEB规划后的速度
    void recmd_velcallback(const geometry_msgs::Twist& in_cmd_vel);
    // 根据机器人和障碍物位置与障碍物速度关系，确定机器人额外需要的速度
    int vel_direction(replan_control::PredObsState pose);
};

subreplan::subreplan(ros::NodeHandle &nh)
{
    // message_filters::Subscriber<geometry_msgs::Twist> sub_cmd_vel_(nh,"cmd_vel",1,ros::TransportHints().tcpNoDelay());
    // message_filters::Subscriber<replan_control::PredStateArray> sub_state(nh,"pre_state",1,ros::TransportHints().tcpNoDelay());
    
    // typedef sync_policies::ApproximateTime<geometry_msgs::Twist, replan_control::PredStateArray> syncPolicy;
    // Synchronizer<syncPolicy> sync(syncPolicy(10), sub_cmd_vel_, sub_state);  
    
    // sync.registerCallback(boost::bind(&replancallback, _1, _2));
    sub_cmd_vel_ = nh.subscribe("cmd_vel",5, &subreplan::recmd_velcallback, this);
    pub_cmd = nh.advertise<geometry_msgs::Twist>("p3_cmd_vel",1);
    sub_state = nh.subscribe("pre_state", 5, &subreplan::replancallback, this);
    ros::spin();
}

subreplan::~subreplan()
{
}

void subreplan::replancallback(const replan_control::PredStateArray &in_prestates)
{
    
    distance = 100;
    std::cout << "into replancallback" << std::endl;
    for (int i=0;i<in_prestates.PredStates.size();i++)
    {
        distance = pow(in_prestates.PredStates[i].x,2)+pow(in_prestates.PredStates[i].y,2);
        distance = sqrt(distance);
        std::cout << distance << std::endl;
        std::cout << "predict x:"<<in_prestates.PredStates[i].x << std::endl;
        std::cout << "predict y:"<<in_prestates.PredStates[i].y << std::endl;

        std::cout << "predict vx:" << in_prestates.PredStates[i].vx << std::endl;
        std::cout << "predict vy:" << in_prestates.PredStates[i].vy << std::endl;
                
        // std::cout << in_prestates.PredStates[i].index << std::endl;
        if(distance < 2)
        {
            direc = vel_direction(in_prestates.PredStates[i]);
            std::cout << direc << std::endl;
            //只有一个障碍，如果多个，应该是vx_add +=; 并且大小应该与预测的障碍物的速度，方向等有关
            // if(direc == 2)
            // {
            //     static_obstacle = true;
            //     // 返回一个和目标点的角度？还有障碍的角度
            //     return ;
            // }
            if(static_obstacle)
            {
                if(direc == 0)
                {
                    vz_add = -0.2;
                }
                else
                {
                    vz_add = 0.2;
                }
            }
            else
            {
                if(in_prestates.PredStates[i].vy > 0)
                {
                    vz_add = -0.4;
                }
                else
                {
                    vz_add = 0.4;
                }
                
            }
            
            
        }
        
        
    }

    
}

int subreplan::vel_direction(replan_control::PredObsState pose)
{
    double x,y,vx,vy;
    //还有个问题是，机器人坐标系是以机器人为原点__|。上是x左边是y。但是我们经常 |__上是y，右是x
    
    x = -pose.y;
    vx = -pose.vy;
    y = pose.x;
    vy = pose.vx;
    rob_obs_angle = atan2(-y, -x); //负号原因是，在机器人坐标系下障碍物坐标
                                            //但是计算夹角的是障碍物的姿态。emmm画图更好一些
    obs_v_angle = atan2(vy, vx);


    //先判断是不是静态障碍物,T和真实的不一样。。
    double v;
    v = pow(vx,2) + pow(vy,2);
    v = sqrt(v);

    if(v <= 0.6 && v >= 0.4)
    {
        static_obstacle = true; //静态障碍物
    }
    else
    {
        static_obstacle = false;
        return 2;
    }
    
    std::cout << "velocity angle:" << obs_v_angle << std::endl;

    std::cout << "rob_obs angle:" << rob_obs_angle << std::endl;
    // if(abs(rob_obs_angle-obs_v_angle) )
    if(rob_obs_angle >= obs_v_angle)  //
    {
        // vx_add = -pose.vy;
        // vy_add = pose.vx;
        return 0;
    }
    else
    {
        // vx_add = pose.vy;
        // vy_add = -pose.vx;
        return 1;
    }
    

}


//第一次仅对速度加上额外的速度，会造成，速度一直增加。然后跑飞的现象。
//所以在大于阈值后，应该对速度修改回来。同时，小于阈值时，也应该限制速度大小
void subreplan::recmd_velcallback(const geometry_msgs::Twist& in_cmd_vel)
{
    std::cout << "into cmd_vel callback" << std::endl;
    std::cout << in_cmd_vel << std::endl;
    std::cout << distance << std::endl;
    cmd_add = in_cmd_vel;

    std::cout << static_obstacle << std::endl;

    if(distance < 2)
    {
    //     cmd_add.angular.z = in_cmd_vel.angular.z + vz_add;
        cmd_add.angular.z = vz_add;
        std::cout << "in there, vz_z = "<<cmd_add.angular.z << std::endl;
        if(distance < 0.5)
        {
            cmd_add.angular.z = 0;
            cmd_add.linear.x = 0;
        }

    }
    // std::ofstream deposit_vec("/home/pzc/samples/velocity.txt",std::ios::app);
    // deposit_vec << "z: " <<cmd_add.angular.z << std::endl;
    // deposit_vec.close();
    vz_add = in_cmd_vel.angular.z; //之前测试静态的时候没加，那是怎么对的？？？
    //不加上边这行好像也确实没啥问题。
    pub_cmd.publish(cmd_add);
    distance = 100; //要不然会出现最近是小于2的，然后不发布障碍话题后，distance还是小于2的。
                    //会导致继续旋转。
    
    static_obstacle = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "replan");
    ros::NodeHandle nh;
    subreplan replan(nh);
    return 0;
}