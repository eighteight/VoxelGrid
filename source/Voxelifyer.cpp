//
//  Voxelifier.cpp
//  voxelify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//


#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/make_shared.hpp>

#include <iostream>
#include "Voxelifyer.h"

using namespace boost;
using namespace pcl;
using namespace std;

template<typename T>
struct mallocDeleter
{
    void operator() (T*& ptr)
    {
        if (ptr)
        {
            free(ptr);
            ptr=NULL;
        }
    }
};


void computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& min, pcl::PointXYZ& max){
    pcl::PCA< PointXYZ > pca;
    
    pcl::PointCloud< PointXYZ > proj;

    pca.setInputCloud (cloud);
    pca.project (*cloud, proj);
    
    pcl::PointXYZ proj_min;
    pcl::PointXYZ proj_max;
    pcl::getMinMax3D (proj, proj_min, proj_max);
    
    pca.reconstruct (proj_min, min);
    pca.reconstruct (proj_max, max);
}

boost::shared_ptr<pcl::StatisticalOutlierRemoval<pcl::PointXYZ> > sor = make_shared<pcl::StatisticalOutlierRemoval<pcl::PointXYZ> >();

void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered){

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;
    
    // Create the filtering object
    sor->setInputCloud (cloud);
    sor->setMeanK (50);
    sor->setStddevMulThresh (0.5);

    sor->filter (*cloud_filtered);
    
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

}

    pcl::PCLPointCloud2::Ptr cloud2In = make_shared<pcl::PCLPointCloud2>();
    pcl::PCLPointCloud2::Ptr cloud2Out = make_shared<pcl::PCLPointCloud2> ();
    //boost::shared_ptr< pcl::PCLPointCloud2> cloud2Out( new pcl::PCLPointCloud2(), mallocDeleter<pcl::PCLPointCloud2>());
    VoxelGrid<PCLPointCloud2> voxelGrid;
VGrid Voxelifier::voxelify(const std::vector<std::vector<float> >& points, const float xLeaf, const float yLeaf, const float zLeaf){
    
    PointCloud<PointXYZ>::Ptr cloudInOut = make_shared<PointCloud<PointXYZ> >();
    
    cloudInOut->width  = points.size();
    cloudInOut->height = 1;
    cloudInOut->is_dense = false;
    cloudInOut->points.resize (points.size());
    
    for (size_t i = 0; i < cloudInOut->points.size (); ++i) {
        pcl::PointXYZ& pt = cloudInOut->points[i];
        pt.x = points[i][0];
        pt.y = points[i][1];
        pt.z = points[i][2];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    
    pcl::PointXYZ min, max;
    computeBoundingBox(cloudInOut, min, max);    
    //removeOutliers(cloudInOut, cloud_filtered);
    //cloudInOut = cloud_filtered;
    computeBoundingBox(cloudInOut, min, max);
    vector<float> bb (3);
    bb[0] = max.x - min.x;
    bb[1] = max.y - min.y;
    bb[2] = max.z - min.z;
    pcl::toPCLPointCloud2 (*cloudInOut,*cloud2In);

    voxelGrid.setInputCloud (cloud2In);
    voxelGrid.setLeafSize (xLeaf, yLeaf, zLeaf);

    voxelGrid.setSaveLeafLayout(true);
    voxelGrid.filter (*cloud2Out);
    
    cerr << cloudInOut->width * cloudInOut->height <<"->"<< cloud2Out->width * cloud2Out->height<<endl;
    
//    std::cerr << "Min box: " << endl<<voxelGrid.getMinBoxCoordinates()
//    << endl<<" Max Box " << voxelGrid.getMaxBoxCoordinates() << endl;

    pcl::fromPCLPointCloud2 (*cloud2Out, *cloudInOut);
    
    VGrid vGrid;
    vGrid.x_leaf = xLeaf;
    vGrid.y_leaf = yLeaf;
    vGrid.z_leaf = zLeaf;
    
    int data_cnt = 0;
    const Eigen::Vector3i v_ref = voxelGrid.getMinBoxCoordinates ();
    int nr0 = voxelGrid.getNrDivisions ()[0], nr1 = voxelGrid.getNrDivisions ()[1], nr2 = voxelGrid.getNrDivisions ()[2];
    vGrid.points = std::vector<vector<float> >(nr0*nr1*nr2, vector<float>(3,0.0));
    vGrid.indices = std::vector<int>(nr0*nr1*nr2, -1);
    for (int i = 0; i < nr0; i++) {
    for (int j = 0; j < nr1; j++) {
      Eigen::Vector3f p (0, 0, 0);

      for (int k = 0; k < nr2; k++) {
        Eigen::Vector3i v (i, j, k);
        v = v + v_ref;
        int index = voxelGrid.getCentroidIndexAt (v);

        if (index != -1){
            vGrid.indices[data_cnt] = 1;

            vGrid.points[data_cnt][0] = cloudInOut->points[index].x;
            vGrid.points[data_cnt][1] = cloudInOut->points[index].y;
            vGrid.points[data_cnt][2] = cloudInOut->points[index].z;

          Eigen::Vector3f grid_leaf_size = voxelGrid.getLeafSize ();
          p = p + Eigen::Vector3f (v[0] * grid_leaf_size[0], v[1] * grid_leaf_size[1], v[2] * grid_leaf_size[2]);

        }
        data_cnt++;
      }
    }
    }
    
    std::cout<<vGrid<<std::endl;
    return vGrid;
}

