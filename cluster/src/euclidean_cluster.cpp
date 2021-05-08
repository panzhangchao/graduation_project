#include"euclidean_cluster.h"

namespace sensor_lidar
{
    //构造函数
    sensor_lidar::Euclidean::Euclidean(ros::NodeHandle &nh)
    {
        seg_distance_ = {15, 30, 45, 60};
        cluster_distance_ = {0.3, 0.8, 1.2, 2.0, 2.5};
        sub_point_colud_ = nh.subscribe("PointCloud2", 5, &sensor_lidar::Euclidean::cluster_CallBack, this);
        pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);
        //测试质心发布
        pub_center_point_ = nh.advertise<sensor_msgs::PointCloud2>("CenterPointCloud",1);
        //add
        pub_center_ = nh.advertise<cluster::obstaclePoseArray>("obstacle_pose",1);
        ros::spin();
    }
    //析构函数
    sensor_lidar::Euclidean::~Euclidean(){}

    //回调函数
    void sensor_lidar::Euclidean::cluster_CallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        point_cloud_header_ = in_cloud_ptr->header;
        pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
        
        // down sampling the point cloud before cluster
        //sensor_lidar::Euclidean::voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

        std::vector<sensor_lidar::Euclidean::Detected_Obj> global_obj_list;
        //sensor_lidar::Euclidean::cluster_by_distance(filtered_pc_ptr, global_obj_list);
        //原来进行了降采样，现在删除这一步测试效果。
        sensor_lidar::Euclidean::cluster_by_distance(current_pc_ptr, global_obj_list);
        // pcl::io::savePCDFileASCII("/home/pzc/rob_cat_ws/src/learning-pcl-master/sensor_lidar_perception/cluster/test_2.pcd", *filtered_pc_ptr);


        pcl::io::savePCDFileASCII("/home/pzc/rob_cat_ws/src/learning-pcl-master/sensor_lidar_perception/cluster/test_2.pcd", *current_pc_ptr);

        jsk_recognition_msgs::BoundingBoxArray bbox_array;

        for (size_t i = 0; i < global_obj_list.size(); i++)
        {
            
            bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
        }
        bbox_array.header = point_cloud_header_;
        
        pub_bounding_boxs_.publish(bbox_array);
    }

    //点云降采样处理
    void sensor_lidar::Euclidean::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, 
                                                                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr out, 
                                                                                                    double leaf_size
                                                                                                )
    {
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(in);
        filter.setLeafSize(leaf_size,leaf_size,leaf_size);
        filter.filter(*out);
    }

    //
    void sensor_lidar::Euclidean::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<sensor_lidar::Euclidean::Detected_Obj> &obj_list)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);
        for (size_t i = 0; i < segment_pc_array.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            segment_pc_array[i] = tmp;
        }
    #pragma omp for
        for (size_t i = 0; i < in_pc->points.size(); i++)
        {
            pcl::PointXYZ current_point;
            current_point.x = in_pc->points[i].x;
            current_point.y = in_pc->points[i].y;
            current_point.z = in_pc->points[i].z;

            float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

            // 如果点的距离大于120m, 忽略该点
            if (origin_distance >= 10)
            {
                continue;
            }

            if (origin_distance < seg_distance_[0])
            {
                segment_pc_array[0]->points.push_back(current_point);
            }
            else if (origin_distance < seg_distance_[1])
            {
                segment_pc_array[1]->points.push_back(current_point);
            }
            else if (origin_distance <  seg_distance_[2])
            {
                segment_pc_array[2]->points.push_back(current_point);
            }
            else if (origin_distance < seg_distance_[3])
            {
                segment_pc_array[3]->points.push_back(current_point);
            }
            else
            {
                segment_pc_array[4]->points.push_back(current_point);
            }
        }

        std::vector<pcl::PointIndices> final_indices;
        std::vector<pcl::PointIndices> tmp_indices;

        for (size_t i = 0; i < segment_pc_array.size(); i++)
        {
            sensor_lidar::Euclidean::cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list);
        }
    }

    //细分聚类
    void sensor_lidar::Euclidean::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                                                                                    double in_max_cluster_distance, 
                                                                                                    std::vector<sensor_lidar::Euclidean::Detected_Obj> & obj_list)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        //2d PointColud
        pcl::PointCloud<pcl::PointXYZ>::Ptr colud_2d(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*in_pc,*colud_2d); //in_pc到cloud_2d中

        double length_;
        double width_;
        double height_;

        //XY平面（Z==0）
        for(size_t i=0;i<colud_2d->points.size();i++)
        {
            colud_2d->points[i].z=0;
        }

        //判断是否有点
        if (colud_2d->points.size()>5)
        {
            tree->setInputCloud(colud_2d);
        }
        //人为给几个点。。防止出现 Cannot create a KDTree with an empty input cloud
        else
        {
            pcl::PointXYZ insert_pose;
            for(int i=0;i<41;i++)
            {
                insert_pose.x = 10+i*0.1;
                insert_pose.y = 10+i*0.1;
                insert_pose.z = 0;
                colud_2d->points.push_back(insert_pose);
            }
            tree->setInputCloud(colud_2d);
            // std::cout << "no error in insert" << std::endl;
        }

        //  点云分割
        std::vector<pcl::PointIndices> local_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ>euclid;
        euclid.setInputCloud(colud_2d);
        euclid.setClusterTolerance(in_max_cluster_distance);
        euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
        euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
        euclid.setSearchMethod(tree);
        euclid.extract(local_indices);

    #pragma omp for  //并行计算
        for(size_t i=0;i<local_indices.size();i++)
        {
            sensor_lidar::Euclidean::Detected_Obj obj_info;
            float min_x=std::numeric_limits<float>::max();
            float min_y=std::numeric_limits<float>::max();
            float min_z=std::numeric_limits<float>::max();
            
            float max_x=-std::numeric_limits<float>::max();
            float max_y=-std::numeric_limits<float>::max();    
            float max_z=-std::numeric_limits<float>::max();
            
            for(auto pit=local_indices[i].indices.begin(); pit !=local_indices[i].indices.end(); ++pit)
            {
                //填充新的点（点到点填充）
                pcl::PointXYZ p_xyz;
                // std::cout << "no error in begin p_xyz" << std::endl;
                // p_xyz.x=in_pc->points[*pit].x;
                // p_xyz.y=in_pc->points[*pit].y;
                // p_xyz.z=in_pc->points[*pit].z;  
                p_xyz.x = colud_2d->points[*pit].x;
                p_xyz.y = colud_2d->points[*pit].y;              
                p_xyz.z = colud_2d->points[*pit].z;
                
                // std::cout << "no error in end p_xyz" << std::endl;
                obj_info.centroid_.x  +=p_xyz.x;
                obj_info.centroid_.y  +=p_xyz.y;
                obj_info.centroid_.z  +=p_xyz.z;

                if(p_xyz.x<min_x)  min_x=p_xyz.x;
                if(p_xyz.y<min_y) min_y=p_xyz.y;
                if(p_xyz.z<min_z) min_z=p_xyz.y;
                if(p_xyz.x>max_x) max_x=p_xyz.x;
                if(p_xyz.y>max_y) max_y=p_xyz.y;
                if(p_xyz.z>max_z) max_z=p_xyz.z;

                obj_info.min_point_.x = min_x;
                obj_info.min_point_.y = min_y;
                obj_info.min_point_.z = min_z;

                obj_info.max_point_.x = max_x;
                obj_info.max_point_.y = max_y;
                obj_info.max_point_.z = max_z;
                
                //计算质心(均值)
                if(local_indices[i].indices.size()>0)
                {
                    obj_info.centroid_.x  /=local_indices[i].indices.size();
                    obj_info.centroid_.y  /=local_indices[i].indices.size();
                    obj_info.centroid_.z  /=local_indices[i].indices.size();
                }
                
                //计算bounding box
                // double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
                // double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
                // double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

                length_ = obj_info.max_point_.x - obj_info.min_point_.x;
                width_ = obj_info.max_point_.y - obj_info.min_point_.y;
                height_ = obj_info.max_point_.z - obj_info.min_point_.z;

                obj_info.bounding_box_.header=point_cloud_header_;
                obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
                obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
                obj_info.bounding_box_.pose.position.z = 0; 
                // obj_info.min_point_.z + height_ / 2;

                obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
                obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
                obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

                obj_list.push_back(obj_info);                
            }          
            
            /* 测试质心计算是否正确 */
            
            float cen_x = obj_info.min_point_.x;
            float cen_y = obj_info.min_point_.y;
            cen_x = cen_x + 0.5*length_;
            cen_y = cen_y + 0.5*width_;
            centroid_point.x = cen_x;
            centroid_point.y = cen_y;
            centroid_point.z = 0;
            if(centroid_point.x!= 12 && centroid_point.y!=12)
            {
                centroid_cloud.points.push_back(centroid_point);
            }
            


            /*转换到map坐标系*/
            // tf::TransformListener listener;
            // geometry_msgs::PointStamped pose_base_link;
            // pose_base_link.header.stamp = ros::Time(0);
            // pose_base_link.header.frame_id = "base_link";
            // pose_base_link.point.x = cen_x;
            // pose_base_link.point.y = cen_y;
            // pose_base_link.point.z = 0;


            // geometry_msgs::PointStamped pose_map;

            // try{
            //     listener.transformPoint("odom", pose_base_link, pose_map);                
            // }
            // catch( tf::TransformException ex)
            // {
            //     ROS_WARN("transfrom exception : %s",ex.what());
            // }
            // obstacle.x = pose_map.point.x;
            // obstacle.y = pose_map.point.y;
            // obstacle.z = 0;
            /* 以上是转换到map坐标系*/

            obstacle.x = cen_x;
            obstacle.y = cen_y;
            obstacle.z = 0;

            obstacle.index = i;
            if(cen_x!= 12 && cen_y!=12)
            {
                obs_pose.obstaclePoses.push_back(obstacle);
            }
            
        }
        pcl::toROSMsg(centroid_cloud,centroid_msg_);
        centroid_msg_.header.frame_id = "base_link";
        pub_center_point_.publish(centroid_msg_);
        centroid_cloud.clear();
        pub_center_.publish(obs_pose);
        obs_pose.obstaclePoses.clear();
    }
}


// tf::TransformListener listener;
// geometry_msgs::PoseStamped pose_base_link;
// pose_base_link.header.frame_id = "base_link";
// pose_base_link.pose.position.x = ;
// pose_base_link.pose.position.y = ;
// pose_base_link.pose.position.z = ;
// pose_base_link.pose.orientation.x = 1;
// pose_base_link.pose.orientation.y = 1;
// pose_base_link.pose.orientation.z = 1;
// pose_base_link.pose.orientation.w = 1;


// geometry_msgs::PoseStamped pose_map;

// try{
//     listener.transformPose("map", pose_base_link, pose_map);
// }
// catch( tf::TransformException ex)
// {
//     ROS_WARN("transfrom exception : %s",ex.what());
// }
// pose_map.pose.position.x;
