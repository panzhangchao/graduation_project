#include <iostream>
#include <ros/ros.h>
#include <cluster/obstaclePose.h>
#include <cluster/obstaclePoseArray.h>

//发布预测的障碍状态
#include <replan_control/PredObsState.h>
#include <replan_control/PredStateArray.h>

#include "kalman.h"

#include<math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include <Eigen/Dense>

//old_obstacle 存放旧障碍物
//optimum_obstacle_pose 存放最优估计的x
//data_address 存放P
//pre_obstacle 存放预测x

using namespace std;


//可以在声明时初始化，A, H和状态大小是不变的。
class sub_and_predict
{
private:
    ros::Subscriber obs_pose_;

    ros::Publisher pub_pred_state;

    replan_control::PredObsState pre_state;
    replan_control::PredStateArray pre_states;

    int stateSize; //state variable's dimenssion  
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    float T;
    Eigen::VectorXd x_pose;

    Eigen::VectorXd x_pose_;

    Eigen::VectorXd z;
    Eigen::VectorXd z_;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd u;
    Eigen::MatrixXd P; //coveriance
    Eigen::MatrixXd H; //
    Eigen::MatrixXd R; //measurement noise covariance
    Eigen::MatrixXd Q; //process noise covariance

public:
    sub_and_predict(ros::NodeHandle &nh);
    void posecallback(const cluster::obstaclePoseArray &in_pose);
    ~sub_and_predict();
    void kalman_init(int &stateSize, int &measSize, int &uSize);
    void kalmaninit_(Eigen::VectorXd &x_, Eigen::MatrixXd& P_, Eigen::MatrixXd& R, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_); //似乎用不到那个A_ B u的。把加速度作为一个状态，而不是控制
    Eigen::VectorXd update(Eigen::MatrixXd& H_, Eigen::VectorXd z_means);
    //读取P的字符串转成double然后放到矩阵中对应的位置
    double read_P(string str);
    //返回index对应的地址，P0,P1这种
    string back_address(int index);
    //返回index对应的地址，记录最优x的
    string back_x_address(int index);
    //记录P矩阵信息
    void record_P_mess(string str,Eigen::MatrixXd P_record);
    //记录最优估计x值
    void record_x_mess(string str, Eigen::VectorXd optimum_x_record);

    Eigen::VectorXd read_x(string str);

    bool del_file_or_not(int index, int arr[], int lens);

};

sub_and_predict::sub_and_predict(ros::NodeHandle &nh)
{
    obs_pose_ = nh.subscribe("obstacle_pose", 1, &sub_and_predict::posecallback, this);
    pub_pred_state = nh.advertise<replan_control::PredStateArray>("pre_state",1);
    ros::spin();
}

sub_and_predict::~sub_and_predict()
{
}

void sub_and_predict::posecallback(const cluster::obstaclePoseArray &in_pose)
{
    
    /* 读取上一次障碍物信息*/
    ifstream ifs("/home/pzc/rob_cat_ws/src/kalman/out/old_obstacle_pose.txt");
    string str;
    int count = 0;
    int index;
    float x,y;
    cluster::obstaclePoseArray old_pose_array;
    cluster::obstaclePoseArray in_pose_cpy = in_pose;  //copy的原因是回调函数参数只读，不能修改
    cluster::obstaclePose old_pose;
    while (ifs >> str)
    {
        /* code */
        if(count%3 == 0)
            index = atoi(str.c_str()); 
        if(count%3 == 1)
            x = atof(str.c_str());
        if(count%3 ==2)
            y = atof(str.c_str());
        
        count++;
        if(count%3 == 0)
        {
            old_pose.index = index;
            old_pose.x = x;
            old_pose.y = y;
            old_pose.z = 0;
            old_pose.new_obstacle = false;
            old_pose_array.obstaclePoses.push_back(old_pose);
            // cout << "x: " <<x<<"  y:"<<y<<"  index:"<<index<<endl;
        }
    }    
    
    ifs.close();
    /*读取上一次障碍物信息并且存入old_pose_array中*/

    /*  进行匹配，最近点就是匹配
        超出阈值即为新的障碍点，进行更新
        还需要删除旧的障碍点  */     
    
    /* 测试匹配是否正确 */
    
    /*
    // 测试用的数据
    in_pose_cpy.obstaclePoses.clear();//先把传进来的删除，测试用的。
    
    cluster::obstaclePose example;
    for(int k = 0;k<4;k++)
    {
        if(k==0)
        {
            example.x = 0;
            example.y = -1;
            example.index = 0;
            example.new_obstacle = false;
        }
        if(k == 1)
        {
            example.x = 3;
            example.y = 8;
            example.index = 1;
            example.new_obstacle = false;
        }
        if(k == 2)
        {
            example.x = 0.5;
            example.y = -11;
            example.index = 2;
            example.new_obstacle = false;
        }
        if(k == 3)
        {
            example.x = -1;
            example.y = 11;
            example.index = 3;
            example.new_obstacle = false;
        }
        in_pose_cpy.obstaclePoses.push_back(example);
    }

    */

    int new_obs_size = in_pose_cpy.obstaclePoses.size(); //回调中当前的障碍信息
    int old_obs_size = old_pose_array.obstaclePoses.size();
    // cout << old_obs_size <<endl;
    int new_index = 0; //用来记录新增加障碍的个数的

    float error;
    int min_dis_index = -1;
    float temp = 1000; 
    bool param_init = false;

    if(new_obs_size!=0)
    {
    
    for(int i = 0;i<new_obs_size;i++)
    {
        // cout << "I'm come!" <<endl;
        //old_obs_size == 0表示之前没有障碍，对应预测那块所有参数都应该初始化, 可能用不到
        // cout << old_obs_size <<endl;
        
        if(old_obs_size == 0)
        {
            // cout<<"That's break!"<<endl;
            param_init = true;
            break;  //old_obs_size == 0表示之前没有障碍

        }

            
        for( int j = 0;j<old_obs_size;j++)
        {
            // cout<<"I come secondly!"<<endl;
            error = pow(in_pose_cpy.obstaclePoses[i].x-old_pose_array.obstaclePoses[j].x,2) + 
                    pow(in_pose_cpy.obstaclePoses[i].y-old_pose_array.obstaclePoses[j].y,2);
            // cout<<"The error is:"<<error<<endl;
            if(error<temp)
            {
                temp = error;
                min_dis_index = old_pose_array.obstaclePoses[j].index;
            }    
            //  如果最小的距离都大于阈值，就是新障碍

        }
        if(temp > 1 && (new_obs_size>old_obs_size))  //似乎一个条件就行，障碍减少不会有新障碍
        {
            in_pose_cpy.obstaclePoses[i].new_obstacle = true;
            /*如果是新的障碍，还得判断index是否需要更改。

            仅仅考虑障碍仅仅增加或者减少，不考虑这一次和上一次
            具有相同的障碍数目，但是却存在不同的障碍
            
            如果是障碍减少了，那肯定不存在新障碍，仅仅是障碍减少了
            如果是增加了，就将新障碍的Index赋为障碍个数的index。这也不会和之前重复 
            增加的障碍个数>1的话，
            */
            in_pose_cpy.obstaclePoses[i].index = old_obs_size+new_index;
            new_index++;

            //障碍如果是减少的，原来是 0 1 2 3 4，现在是0 1 2 4.那最后应该变成0,1,2,3.
            // 0 1 2 3 现在是 1 3 应该变成 0，1
            //如果是 0 1 2 4.再增加一个新障碍时，就是 0 1 2  4.但是这块会涉及到卡尔曼
            //预测参数对应，再最后也得把那个参数的index改成3
        }
        else
        {
            in_pose_cpy.obstaclePoses[i].index = min_dis_index;
        }
        temp = 1000;//如果不改为1000，在下次循环就是小的值;
    }
    /* 匹配完成 */
    
    /* kalman predict*/
    /*
    利用卡尔曼进行预测
    读取上一次预测的这次的值
    结合当前的障碍信息，更新这次最优解，利用 A×X(最优) 预测
    保存预测信息，更新k,p等
    */
    //  定义多个类？还是一个类里多个参数？
    //多个txt存储吧。。。。。
    int measSize;
    int stateSize;
    int uSize;
    measSize = 2;stateSize = 6; uSize = 0;
    //这里应该对x_pose,x_pose_也进行存储
    kalman_init(stateSize,measSize,uSize);
    
    //应该有个循环对多个障碍进行预测，但是现在先一次。
    string data_address; //读取存放P的文件地址
    std::ofstream pre_obstacle("/home/pzc/rob_cat_ws/src/kalman/out/predic_obstacle_pose.txt",std::ios::app);
    string optimum_obstacle_pose;
    string update_P;

    int count_P;
    for(int i = 0;i<new_obs_size;i++)
    {
        /* 读取P和最优x, 如果是新障碍，这个P就应该是初始化的，新文件里没有 */
        if(! in_pose_cpy.obstaclePoses[i].new_obstacle) 
        {
        count_P = 0;
        optimum_obstacle_pose = back_x_address(in_pose_cpy.obstaclePoses[i].index);
        data_address = back_address(in_pose_cpy.obstaclePoses[i].index);
        ifstream ifs_P(data_address.c_str());
        while(ifs_P >> update_P)
        {
            double P_array;
            P_array = read_P(update_P);
            switch (count_P%21)
            {
            case 0:
                P(0,0) = P_array;
                break;
            case 1:
                P(0,1) = P_array;
                break;
            case 2:
                P(0,2) = P_array;
                break;
            case 3:
                P(0,3) = P_array;
                break;
            case 4:
                P(0,4) = P_array;
                break;
            case 5:
                P(0,5) = P_array;
                break;
            case 6:
                P(1,1) = P_array;
                break;
            case 7:
                P(1,2) = P_array;
                break;
            case 8:
                P(1,3) = P_array;
                break;
            case 9:
                P(1,4) = P_array;
                break;
            case 10:
                P(1,5) = P_array;
                break;
            case 11:
                P(2,2) = P_array;
                break;
            case 12:
                P(2,3) = P_array;
                break;
            case 13:
                P(2,4) = P_array;
                break;
            case 14:
                P(2,5) = P_array;
                break;
            case 15:
                P(3,3) = P_array;
                break;
            case 16:
                P(3,4) = P_array;
                break;
            case 17:
                P(3,5) = P_array;
                break;
            case 18:
                P(4,4) = P_array;
                break;
            case 19:
                P(4,4) = P_array;
                break;
            case 20:
                P(5,5) = P_array;
                break;
            default:
                break;
            }
            count_P++;
        }
        
        for(int m = 0;m<6;m++)
        {
            for(int n = 0;n<6;n++)
            {
                if(m==n)
                    continue;
                else
                {
                    P(n,m) = P(m,n);
                }
            }
        }
        
        
        }
        if(in_pose_cpy.obstaclePoses[i].new_obstacle || param_init) //这个障碍是新的 
        {                                                           //或者是这是刚开始调用这个。没有旧障碍读取
            x_pose_ << in_pose_cpy.obstaclePoses[i].x,in_pose_cpy.obstaclePoses[i].y,0,0,0,0;
            // cout << "init "<< endl;
            kalmaninit_(x_pose_,P,R,Q);
            //测试问题
            // cout << "初始化没问题" << endl;
            // std::cout << P << endl;
        }
        else
        {
            x_pose_ = read_x(optimum_obstacle_pose); //读取最优的
            // cout << "in there" << endl;
            // cout << x_pose_ << endl;
        }
        
        x_pose << predict(A); //用来更新最优的，需要上一次最优的进行这次的估计

        z << in_pose_cpy.obstaclePoses[i].x,in_pose_cpy.obstaclePoses[i].y; //观测
        // z_ << x_pose_[0],x_pose_[1];
        x_pose_ << update(H,z); //更新这次的最优，以及下次的P
        
        x_pose << A*x_pose_;//用这次最优，预测下一次 ## 这个才是预测！ 
                            //之前用的是predict函数，会把P多更新一次。
    
        /*
        如果，障碍会碰到，就重新规划，那在规划期间，如何让这个预测节点停止，
        在规划好以后，再重新开启？同时所有内容清空？
        */
        pre_obstacle << in_pose_cpy.obstaclePoses[i].index<<" "
        <<x_pose[0]<<" "<<x_pose[1]<<endl;

        pre_state.x = x_pose[0];
        pre_state.y = x_pose[1];
        pre_state.vx = x_pose[2];
        pre_state.vy = x_pose[3];
        pre_state.index = in_pose_cpy.obstaclePoses[i].index;
        //预测完一个后，保存相关信息，P，最优的x(?)，如果最后是障碍减少，再进行修改
        
        record_x_mess(optimum_obstacle_pose, x_pose_);
        record_P_mess(data_address,P);

        pre_states.PredStates.push_back(pre_state);

    }
    //x预测 = update(H,z,z_);
    pub_pred_state.publish(pre_states);
    pre_states.PredStates.clear();

    //预测结束后，对减少的障碍进行Index修改，应该重新排序。
    //之前0 1 2 3，之后 1 3.先把这俩留下，其他的都删除，然后，就可以重命名了

    string recity_data_address;
    string recity_obstacle_pose_address;
    int index_array[] = {10,10,10,10,10,10,10,10,10,10};
    //先把1,3放在数组里
    for(int q = 0;q<new_obs_size;q++)
    {
        index_array[q] = in_pose_cpy.obstaclePoses[q].index;
    }
    int len_index = 10;
    //检查旧文件中，不是1,3的，删除。则就剩下0,2
    for(int l = 0;l<old_obs_size;l++)
    {
        if(!del_file_or_not(old_pose_array.obstaclePoses[l].index, index_array,len_index))
        {
            data_address = back_address(old_pose_array.obstaclePoses[l].index);
            remove(data_address.c_str());
            optimum_obstacle_pose = back_x_address(old_pose_array.obstaclePoses[l].index);
            remove(optimum_obstacle_pose.c_str());
        }
    }
    //把0,2重命名为0,1
    if(new_obs_size<old_obs_size)
    {
        for(int k=0;k<new_obs_size;k++)
        {
            
            data_address = back_address(in_pose_cpy.obstaclePoses[k].index);
            recity_data_address =  back_address(k);
            int result_P;//进行重命名
            result_P = rename(data_address.c_str(),recity_data_address.c_str());
            /*这里需要对卡尔曼相关参数的index也进行修改，但是还没写预测的*/
            //还是写个函数吧。。减少，所以Index一定是>=之前的，那就从最小的到最小的写就行
            //直接重命名/ww
            // 起初存在错误，返回的是p的地址。 back_address
            optimum_obstacle_pose = back_x_address(in_pose_cpy.obstaclePoses[k].index);
            recity_obstacle_pose_address = back_x_address(k);
            int result_x;
            result_x = rename(optimum_obstacle_pose.c_str(),recity_obstacle_pose_address.c_str());
            
            in_pose_cpy.obstaclePoses[k].index = k;
            
        }
    }
    
    /* 把新的障碍存入old_obstacle_pose.txt文件 是追加形式，测试对不对的。后边需要改为覆盖*/
    //然后存放更新后的，下一次来
    
    std::ofstream old_obstacle_pose("/home/pzc/rob_cat_ws/src/kalman/out/old_obstacle_pose.txt", std::ios::out);
    
    // std::ofstream obstacle_pose("/home/pzc/rob_cat_ws/src/kalman/out/obstacle_pose.txt",std::ios::out);
    
    for(int q=0;q<in_pose_cpy.obstaclePoses.size();q++)
    {
        // cout <<"come old_obstacle!"<<endl;
        old_obstacle_pose<<fixed<<setprecision(4)<<in_pose_cpy.obstaclePoses[q].index<<
        " "<<in_pose_cpy.obstaclePoses[q].x<<
        " "<<in_pose_cpy.obstaclePoses[q].y<<endl;

        // cout<<"print the second"<<endl;
        // obstacle_pose<<fixed<<setprecision(4)<<in_pose_cpy.obstaclePoses[q].index<<
        // " "<<in_pose_cpy.obstaclePoses[q].x<<
        // " "<<in_pose_cpy.obstaclePoses[q].y<<endl;

    }
    //old_obstacle_pose<<"\n"<<endl;
    old_obstacle_pose.close();

    //存一个到obstacle，用来更新

    std::ofstream obstacle_pose("/home/pzc/rob_cat_ws/src/kalman/out/obstacle_pose.txt", std::ios::app);
    for(int p=0;p<in_pose_cpy.obstaclePoses.size();p++)
    {
        obstacle_pose<<fixed<<setprecision(4)<<in_pose_cpy.obstaclePoses[p].index<<
        " "<<in_pose_cpy.obstaclePoses[p].x<<
        " "<<in_pose_cpy.obstaclePoses[p].y<<endl;
    }
    obstacle_pose.close();

    /* 测试匹配 */
    // for(int n = 0;n<in_pose_cpy.obstaclePoses.size();n++)
    // {
    //     std::cout << "x1: " << in_pose_cpy.obstaclePoses[n].x << 
    //     "  y:" << in_pose_cpy.obstaclePoses[n].y << 
    //     "  index:" << in_pose_cpy.obstaclePoses[n].index <<
    //     " new? :"<<std::endl;
    //     if(!in_pose_cpy.obstaclePoses[n].new_obstacle)
    //         cout << "false\n" <<endl;
    // }


    // 需要2个txt文件，上一次的障碍物信息，上一次预测的这次的信息  注意匹配
    // std::ofstream out("/home/pzc/rob_cat_ws/src/kalman/out/predic_obstacle_pose.txt");
    // out.close();
    old_pose_array.obstaclePoses.clear();
    in_pose_cpy.obstaclePoses.clear();

    pre_obstacle.close();

    }
}

void sub_and_predict::kalman_init(int &stateSize, int &measSize, int &uSize)
{
    if(stateSize == 0 || measSize == 0)
        std::cerr << "Error,State size and measurement size must bigger than 0\n";
    
    T = 0.05;

    x_pose.resize(stateSize);
    x_pose.setZero();

    x_pose_.resize(stateSize);
    x_pose_.setZero();

    A.resize(stateSize, stateSize);
    A.setIdentity();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    Q.resize(stateSize, stateSize);
    Q.setIdentity()*0.001;

    R.resize(measSize, measSize);
    R.setIdentity()*0.01;
    
    Eigen::MatrixXd B(0,0);


    A << 1, 0, T, 0, 1 / 2 * T*T, 0,
        0, 1, 0, T, 0, 1 / 2 * T*T,
        0, 0, 1, 0, T, 0, 
        0, 0, 0, 1, 0, T,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;
}

void sub_and_predict::kalmaninit_(Eigen::VectorXd &x_, Eigen::MatrixXd& P_, Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_)
{
    x_pose_ = x_;
    P = P_;
    R = R_;
    Q = Q_;
}

Eigen::VectorXd sub_and_predict::predict(Eigen::MatrixXd& A_)
{
    A = A_;
    x_pose = A*x_pose_;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q; 
//  cout << "P-=" << P<< endl;
    return x_pose;
}

Eigen::VectorXd sub_and_predict::update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas)
{
    H = H_;
//    cout << "come update no problem" << endl;
    Eigen::MatrixXd temp1, temp2,Ht;
    Ht = H.transpose();
//    cout << "Transpose no problem" << endl; 
    temp1 = H*P*Ht + R;
//    cout << "S no problem"<<endl;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
//    cout << "inverse no problem" << endl;

    Eigen::MatrixXd K = P*Ht*temp2;
//    cout << "caculate K no problem" << endl;
    z = H*x_pose;
//    cout << "HX no problem" << endl;

    x_pose_ = x_pose + K*(z_meas-z);
//    cout << "x_pose no problem" << endl;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P = (I - K*H)*P;
//    cout << "update P no problem" << endl;
//  cout << "P=" << P << endl;
    return x_pose_;
}

double sub_and_predict::read_P(string str)
{
    return atof(str.c_str());
}

string sub_and_predict::back_address(int index)
{
    string back_str;
    switch (index)
    {
    case 0:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/P0.txt";
        break;
    case 1:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/P1.txt";
        break;
    case 2:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/P2.txt";
        break;
    case 3:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/P3.txt";
        break;
    case 4:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/P4.txt";
        break;
    case 5:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/P5.txt";
        break;
    case 6:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/P6.txt";
        break;
    default:
        break;
    }
    return back_str;
}

string sub_and_predict::back_x_address(int index)
{
    string back_str;
    switch (index)
    {
    case 0:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/x0.txt";
        break;
    case 1:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/x1.txt";
        break;
    case 2:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/x2.txt";
        break;
    case 3:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/x3.txt";
        break;
    case 4:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/x4.txt";
        break;
    case 5:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/x5.txt";
        break;
    case 6:
        back_str = "/home/pzc/rob_cat_ws/src/kalman/out/x6.txt";
        break;
    default:
        break;
    }
    return back_str;
}

void sub_and_predict::record_P_mess(string str, Eigen::MatrixXd P_record)
{
    std::ofstream record_message_P(str.c_str(),std::ios::out);
    record_message_P << P_record(0,0)<<" "<<P_record(0,1)<<" "<<P_record(0,2)<<" "
                    <<P_record(0,3)<<" "<<P_record(0,4)<<" "<<P_record(0,5)<<" "
                    <<P_record(1,1)<<" "<<P_record(1,2)<<" "<<P_record(1,3)<<" "
                    <<P_record(1,4)<<" "<<P_record(1,5)<<" "<<P_record(2,2)<<" "
                    <<P_record(2,3)<<" "<<P_record(2,4)<<" "<<P_record(2,5)<<" "
                    <<P_record(3,3)<<" "<<P_record(3,4)<<" "<<P_record(3,5)<<" "
                    <<P_record(4,4)<<" "<<P_record(4,5)<<" "<<P_record(5,5)<<std::endl;
    record_message_P.close();
}

void sub_and_predict::record_x_mess(string str, Eigen::VectorXd optimum_x_record)
{
    std::ofstream record_op_x_mess(str.c_str(),std::ios::out);
    record_op_x_mess <<optimum_x_record(0)<<" "<<optimum_x_record(1)
    <<" "<<optimum_x_record(2)<<" "<<optimum_x_record(3)<<" "<<optimum_x_record(4)
    <<" "<<optimum_x_record(5)<<std::endl;
    record_op_x_mess.close();
}

Eigen::VectorXd sub_and_predict::read_x(string str)
{
    Eigen::VectorXd optimum_x(6);
    ifstream ifs_x(str.c_str());
    string optimum_str_x;
    int count_x = 0;
    while (ifs_x >> optimum_str_x)
    {
        switch (count_x%6)
        {
        case 0:
            optimum_x(0) = atof(optimum_str_x.c_str());
            break;
        case 1:
            optimum_x(1) = atof(optimum_str_x.c_str());
            break;
        case 2:
            optimum_x(2) = atof(optimum_str_x.c_str());
            break;
        case 3:
            optimum_x(3) = atof(optimum_str_x.c_str());
            break;
        case 4: 
            optimum_x(4) = atof(optimum_str_x.c_str());
            break;
        case 5:
            optimum_x(5) = atof(optimum_str_x.c_str());
            break;
        default:
            break;
        }
        count_x++;
    }

    return optimum_x;
    
}

bool sub_and_predict::del_file_or_not(int index, int arr[], int lens)
{
    bool in_file;
    in_file = false;
    int *p = arr;
    for(int i = 0;i < lens;i++)
    {
        if(index == *p)
        {
            in_file = true;
            break;
        }    
        p++;
    }
    return in_file;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"pub_and_pre");
    ros::NodeHandle nh;
    sub_and_predict sub_and_predict(nh);
    return 0;
}

