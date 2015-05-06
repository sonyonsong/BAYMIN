#include <ros/ros.h>
#include "gtest/gtest.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "perception_msgs/SegmentedObjectList.h"
#include "perception_msgs/SegmentedObject.h"
#include <ros/package.h>

namespace scene_segmenter_node_test
{
    class TestSceneSegmenterNode
    {
        private:
            ros::NodeHandle node_handle;


        public:
            TestSceneSegmenterNode();

            ros::Publisher scenePublisher;
            ros::Subscriber segmentedObjectsSubscriber;
            void sceneMessageCallback(const perception_msgs::SegmentedObjectList::ConstPtr &msg);

            bool hasReceivedMessage;
            int sizeOfReceivedSegmentedObjectList;
    };


    TestSceneSegmenterNode::TestSceneSegmenterNode(): node_handle("")
    {
        segmentedObjectsSubscriber = node_handle.subscribe("segmented_objects", 1000, &TestSceneSegmenterNode::sceneMessageCallback, this);

        scenePublisher = node_handle.advertise<sensor_msgs::PointCloud2>("scene",10);
        hasReceivedMessage = false;

        ROS_INFO("test_scene_segmenter_node ready");
    }


    void TestSceneSegmenterNode::sceneMessageCallback(const perception_msgs::SegmentedObjectList::ConstPtr &msg)
    {
        ROS_INFO("Received scene message");
        hasReceivedMessage = true;
        sizeOfReceivedSegmentedObjectList = msg.get()->segmentedObjects.size();
    }

}


scene_segmenter_node_test::TestSceneSegmenterNode buildTestNode()
{
    scene_segmenter_node_test::TestSceneSegmenterNode node;

    //give test node time to initialize VERY IMPORTANT
    ros::Duration(1).sleep();

    return node;
}

sensor_msgs::PointCloud2 buildTestScene()
{
    std::string fileName = ros::package::getPath("object_models") + "/models/pcl_example_scenes/table_scene_lms400.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }

    sensor_msgs::PointCloud2 sensorMessagePointCloud;
    pcl::toROSMsg(*cloud, sensorMessagePointCloud);

    return sensorMessagePointCloud;
}



TEST(SCENE_SEGMENTER_TEST_NODE, TestTableSceneSegmentation) {

  scene_segmenter_node_test::TestSceneSegmenterNode test_node = buildTestNode();
  sensor_msgs::PointCloud2 scene = buildTestScene();

  test_node.scenePublisher.publish(scene);

  double startTimeInSeconds =ros::Time::now().toSec();
  while(!test_node.hasReceivedMessage)
  {
      ros::spinOnce();
      double currentTimeInSeconds =ros::Time::now().toSec();

      //we should have received a message by now, something is wrong.
      if(currentTimeInSeconds-startTimeInSeconds > 5)
      {
          break;
      }
  }

  EXPECT_EQ(test_node.hasReceivedMessage, true);
  EXPECT_EQ(test_node.sizeOfReceivedSegmentedObjectList, 5);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_scene_segmenter_node");
  ros::NodeHandle nh;

  //give the test node time to start up.
  ros::Duration(1).sleep();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

