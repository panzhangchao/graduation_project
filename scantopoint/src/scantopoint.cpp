#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>

class scantopoint
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    sensor_msgs::LaserScan _laser_scan;

    sensor_msgs::PointCloud2 pointcloud2;
public:
    scantopoint(/* args */);
    ~scantopoint();
    void chattercallback(const sensor_msgs::LaserScan::ConstPtr& _laser_sacn);
};

scantopoint::scantopoint(/* args */)
{
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("PointCloud2",1);
    sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 1, &scantopoint::chattercallback,this);
}

scantopoint::~scantopoint()
{

}

void scantopoint::chattercallback(const sensor_msgs::LaserScan::ConstPtr& _laser_scan)
    {
        pcl::PointCloud<pcl::PointXYZI> _pointcloud;
        pcl::PointXYZI newPoint;
        newPoint.z = 0.0;
        double newPointAngle;

        int beamNum = _laser_scan->ranges.size();
        for (int i = 0; i < beamNum; i++)
        {
            newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
            newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
            newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
            newPoint.intensity = _laser_scan->intensities[i];
            _pointcloud.push_back(newPoint);
        }
        pcl::toROSMsg(_pointcloud, pointcloud2);
        pointcloud2.header.frame_id = "base_link";
        pub_.publish(pointcloud2);
        _pointcloud.clear();
    }

int main (int argc, char **argv)
{
    ros::init(argc,argv,"sub_scan_to_pc2");
    scantopoint scan2p;
    ros::spin();
    return 0;
}