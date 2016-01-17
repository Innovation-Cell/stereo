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

ros::Publisher vis_pub;


/** Function Details
* Function Name     -- visualizePlanes
* Function Purpose  -- to publish marker to visualize the points belonging to the plane
* Function Input    -- plane id, pointCloud, inliers, point cloud frame, marker Publisher
* Function Output   -- marker message sent
*/
void visualizePlanesInliers(int i,pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::PointIndices &inliers,std::string &frame,ros::Publisher &visPub);//visualize plane inliers by publishing markers -- This is the function that you will need to change

void ProcessInputPointCloud2(const sensor_msgs::PointCloud2ConstPtr& input)
{
	//------------Done Initializing Segmenter-----//
	std::string frame;
	
	if((input->height*input->width)>0)
	{
		frame = input->header.frame_id;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		fromROSMsg (*input,cloud);
	}
	
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "planeSegmenter");
  ros::NodeHandle n;

  vis_pub = n.advertise<visualization_msgs::Marker>("/planesegmentation/planeVisualization", 1);
  ros::Publisher concatenated_visualizer = n.advertise<sensor_msgs::PointClouds2>

    std::string input_topic;    
	input_topic = "/camera/stereo_camera_LC/points2";
	ROS_INFO("Topic: %s", input_topic.c_str());     
	ros::Subscriber sub = n.subscribe(input_topic, 1, ProcessInputPointCloud2);

  	ros::spin();

  	return 0;
}

void visualizePlanesInliers(int i,pcl::PointCloud<pcl::PointXYZ> &ipCloud,pcl::PointIndices &inliers,std::string &frame,ros::Publisher &visPub)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time();
	marker.ns = "plane_fitting";
	marker.id = i;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 0.5;
	marker.lifetime = ros::Duration(0);
	geometry_msgs::Point p;
	
	
	for(unsigned int k=0; k<inliers.indices.size();k=k+10)
	{
		p.x = ipCloud.at(inliers.indices[k]).x; p.y = ipCloud.at(inliers.indices[k]).y; p.z = ipCloud.at(inliers.indices[k]).z;
		marker.points.push_back(p);
	}
	
	if (marker.id==0)
	{
		marker.color.r = 1.0;
	}
	else if (marker.id==1)
	{
		marker.color.g = 1.0;
	}
	else if (marker.id==2)
	{
		marker.color.b = 1.0;
	}	
	visPub.publish(marker);
	marker.points.clear();
	
};
