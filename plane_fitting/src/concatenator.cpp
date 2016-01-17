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
	if((input->height*input->width)>0) pcl::fromROSMsg(*input,LC_cloud);
		
}

void CR_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if((input->height*input->width)>0) pcl::fromROSMsg(*input,CR_cloud);	
}

void LR_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if((input->height*input->width)>0) pcl::fromROSMsg(*input,LR_cloud);	
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
      pcl::PointCloud<pcl::PointXYZ> final_cloud;	

      final_cloud += LC_cloud;
      final_cloud += CR_cloud;
      final_cloud += LR_cloud;

      sensor_msgs::PointCloud2 finalCloud;
      pcl::toROSMsg(final_cloud, finalCloud);

      concatenated_visualizer.publish(finalCloud);

      LC_called = 0;
      CR_called = 0;
      LR_called = 0;				
    }
  }

  return 0;
}
