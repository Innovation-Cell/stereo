#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>


bool allCloudsProcessed = 0;
static bool LC_called = 0, CR_called = 0, LR_called = 0;

static pcl::PointCloud<pcl::PointXYZ> LC_cloud, CR_cloud, LR_cloud;


void LC_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //ROS_INFO("LC Called back!");
	if((input->height*input->width)>0) pcl::fromROSMsg(*input,LC_cloud);
	LC_called = 1;
}

void CR_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //ROS_INFO("CR Called back!");
	if((input->height*input->width)>0) pcl::fromROSMsg(*input,CR_cloud);	
  CR_called = 1;
}

void LR_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //ROS_INFO("LR Called back!");
	if((input->height*input->width)>0) pcl::fromROSMsg(*input,LR_cloud);
  LR_called = 1;	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planeSegmenter");
  ros::NodeHandle n;

  ros::Publisher concatenated_visualizer = n.advertise<sensor_msgs::PointCloud2>("/concatenated_pointCloud", 1);

  ros::Subscriber sub_LC = n.subscribe("/camera/stereo_camera_LC/points2", 1, LC_callback);
  ros::Subscriber sub_CR = n.subscribe("/camera/stereo_camera_CR/points2", 1, CR_callback);
  ros::Subscriber sub_LR = n.subscribe("/camera/stereo_camera_LR/points2", 1, LR_callback);

  while(ros::ok()){
    ros::spinOnce();

    if(LC_called&&CR_called&&LR_called){
      //ROS_INFO("Entered publishsing loop");

      pcl::PointCloud<pcl::PointXYZ> final_cloud;	

      final_cloud += LC_cloud;
      final_cloud += CR_cloud;
      final_cloud += LR_cloud;

      sensor_msgs::PointCloud2 finalCloud;
      pcl::toROSMsg(final_cloud, finalCloud);
      finalCloud.header.frame_id = "/base_link";
      finalCloud.header.stamp = ros::Time::now();

      concatenated_visualizer.publish(finalCloud);

      LC_called = 0;
      CR_called = 0;
      LR_called = 0;				
    }
  }

  return 0;
}
