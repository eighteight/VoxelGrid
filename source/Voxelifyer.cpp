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
#include <pcl/common/centroid.h>
#include <boost/make_shared.hpp>

#include <iostream>
#include "Voxelifyer.h"

using namespace boost;
using namespace pcl;
using namespace std;

template<typename T>
struct Deleter
{
    void operator() (T*& ptr)
    {
        if (ptr)
        {
//            free(ptr);
//            ptr=NULL;
        }
    }
};

boost::shared_ptr<pcl::StatisticalOutlierRemoval<pcl::PointXYZ> > sor = make_shared<pcl::StatisticalOutlierRemoval<pcl::PointXYZ> >();

void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, const int meanK, const float mulTrh){
    
    // Create the filtering object
    sor->setInputCloud (cloud);
    sor->setMeanK (meanK); //number of neighbors to consider
    sor->setStddevMulThresh (mulTrh);

    sor->filter (*cloud_filtered);
    
    //std::cerr << "Stat Outlier " << cloud->points.size()<<" "<<cloud_filtered->points.size() << std::endl;
}

vector<float> computeCentro(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);
    cout<<"CENTRO"<<centroid<<endl;
    vector<float> centro(3);
    centro[0] = centroid.x();
    centro[1] = centroid.y();
    centro[2] = centroid.z();
    
    return centro;
}
    pcl::PCLPointCloud2::Ptr cloud2In = make_shared<pcl::PCLPointCloud2>();
    pcl::PCLPointCloud2::Ptr cloud2Out = make_shared<pcl::PCLPointCloud2> ();
    //boost::shared_ptr< pcl::PCLPointCloud2> cloud2Out( new pcl::PCLPointCloud2(), mallocDeleter<pcl::PCLPointCloud2>());
    VoxelGrid<PCLPointCloud2> voxelGrid;

VGrid Voxelifier::voxelify(const std::vector<std::vector<float> >& points, const int gridSize, vector<float> &centro, int thre, float multi){
    
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

    removeOutliers(cloudInOut, cloud_filtered, thre, multi);
    pcl::PointXYZ min, max;

    pcl::getMinMax3D (*cloud_filtered, min, max);

    vector<float> bb (3);
    bb[0] = max.x - min.x;
    bb[1] = max.y - min.y;
    bb[2] = max.z - min.z;
    
    float xLeaf = abs(bb[0]/gridSize);
    float yLeaf = abs(bb[1]/gridSize);
    float zLeaf = abs(bb[2]/gridSize);

    pcl::toPCLPointCloud2 (*cloud_filtered,*cloud2In); 
    
    voxelGrid.setInputCloud (cloud2In);
    voxelGrid.setLeafSize (xLeaf, yLeaf, zLeaf);

    voxelGrid.setSaveLeafLayout(true);
    voxelGrid.filter (*cloud2Out);

    pcl::fromPCLPointCloud2 (*cloud2Out, *cloudInOut);
    
    VGrid vGrid(xLeaf,yLeaf,zLeaf);
    
    int data_cnt = 0;
    const Eigen::Vector3i v_min = voxelGrid.getMinBoxCoordinates ();
    
    int nr0 = voxelGrid.getNrDivisions ()[0], nr1 = voxelGrid.getNrDivisions ()[1], nr2 = voxelGrid.getNrDivisions ()[2];
    vGrid.points = vector<vector<float> >(nr0*nr1*nr2, vector<float>(3,0.0));
    vGrid.indices = vector<int>(nr0*nr1*nr2, -1);
    for (int i = 0; i < nr0; i++) {
        for (int j = 0; j < nr1; j++) {
            for (int k = 0; k < nr2; k++) {
                Eigen::Vector3i v (i, j, k);
                v = v + v_min;
                int index = voxelGrid.getCentroidIndexAt (v);
                if (index != -1){
                    vGrid.indices[data_cnt] = 1;
                    vGrid.points[data_cnt][0] = cloudInOut->points[index].x;
                    vGrid.points[data_cnt][1] = cloudInOut->points[index].y;
                    vGrid.points[data_cnt][2] = cloudInOut->points[index].z;
                }
            data_cnt++;
          }
        }
    }
    return vGrid;
}

