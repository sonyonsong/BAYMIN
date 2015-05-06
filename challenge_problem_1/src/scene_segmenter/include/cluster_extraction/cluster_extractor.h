#include <vector>


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


class ClusterExtractor
{
	protected:

		pcl::PCLPointCloud2::Ptr sceneCloud;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters;

	public:

		void computeClusters();

		void setCloud(pcl::PCLPointCloud2::Ptr scene_cloud)
		{
			 sceneCloud = scene_cloud;
		}

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getCloudClusters()
		{
			return cloudClusters;
		}

}; 