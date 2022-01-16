#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "GPF/GPF.h"
#include "dbscan/point.h"
// #include "dbscan/pclpoint.h"
#include "dbscan/dbscan.h"
#include "dbscan/cluster.h"
#include "dbscan/kdtree.h"
#include <chrono>
#include <thread>
#include <Eigen/Dense>


std::vector<dbscan::Point> read_file() {
    std::vector<dbscan::Point> points;
    std::ifstream newfile("/media/aldec/OS/Users/alvin/DATA/WAT/cpp_proj/RadarClustering/data/cluster_data.txt");
    std::string data;
    std::vector<std::string> coordinates;
    double x, y, z;
    while (std::getline(newfile, data)) {
        std::stringstream input(data);
        while (std::getline(input, data, ',')) {
            coordinates.push_back(data);
        }
    }

    for (int i = 0; i < coordinates.size(); i++) {
        if (i%4 == 0) {
            x = std::stof((coordinates[i]));
        } else if (i%4 == 1) {
            y = std::stof(coordinates[i]);
        } else if (i%4 == 2) {
            z = std::stof(coordinates[i]);
            points.push_back(dbscan::Point(x, y, z));
        }
    }

    return points;
}

int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_replaced (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/media/aldec/OS/Users/alvin/DATA/WAT/pc/pointcloud_1.pcd", *cloud) == -1) //* load the file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/aldec/DATA/WAT/FRUSTUM_SAMPLE_DATA/wato_data/all_data/perception_to_wloo/data/full_pc56.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    // GroundSegmentation ground_segment;
    // ground_segment.insertPoints(cloud);
    // ground_segment.lineFit();
    // cloud_replaced = ground_segment.object_output(cloud);
    // viewer.showCloud(cloud_replaced);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // DBSCAN START
    // std::vector<dbscan::Point> points = read_file();
    // int min_points = 7;
    // double epsilon = 1.2;

    // dbscan::Dbscan dbscan_object(points, min_points, epsilon);
    // dbscan_object.CreateClusters();

    // std::vector<dbscan::Cluster::BoundingBox> boundingBoxClusters; // vector to hold the bounding boxes that will come from the clusters

    // for (int i = 0; i < dbscan_object.clusters.size(); i++){
    //     dbscan::Cluster::BoundingBox box = dbscan_object.clusters[i].ConstructBoundingBox();
    //     Eigen::Vector3f translation(box.x_center, box.y_center, box.z_center);
    //     Eigen::Quaternionf rotation(0, 0, 0, 0);
    //     std::string cubeName = "cube" + std::to_string(i);
    //     viewer->addCube(translation, rotation, box.x_dist, box.y_dist, box.z_dist, cubeName, 0);
    //     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubeName);
    //     boundingBoxClusters.push_back(box); //populating vector with the constructed bounding boxes formed by the cluster information
    // }
    // DBSCAN END

    // KDTREES START
    std::vector<dbscan::Point> points = read_file();
    int min_points = 7;
    double epsilon = 1;

    std::vector<dbscan::Cluster::BoundingBox> boundingBoxClusters; 
    std::vector<dbscan::Cluster> clusters = kdtree::cluster_points(points, epsilon, min_points);
    for (int i = 0; i < clusters.size(); i++){
        dbscan::Cluster::BoundingBox box = clusters[i].ConstructBoundingBox();
        Eigen::Vector3f translation(box.x_center, box.y_center, box.z_center);
        Eigen::Quaternionf rotation(0, 0, 0, 0);
        std::string cubeName = "cube" + std::to_string(i);
        viewer->addCube(translation, rotation, box.x_dist, box.y_dist, box.z_dist, cubeName, 0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubeName);
        boundingBoxClusters.push_back(box); //populating vector with the constructed bounding boxes formed by the cluster information
    }
    // KDTREES END




    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<PointXYZV>::Ptr vradar_cloud (new pcl::PointCloud<PointXYZV>);
    for (dbscan::Point point : points){
        double x = point.x();
        double y = point.y();
        double z = point.z();
        radar_cloud->push_back(pcl::PointXYZ(x, y, z));
        // vradar_cloud->push_back(PointXYZV(point));
    }
    // pcl::copyPointCloud(*vradar_cloud, *radar_cloud);

    
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (radar_cloud, "radar cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "radar cloud");
    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }


	// pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // FloorSegment fs;
	// fs.Run(cloud, nofloor_cloud, onlyfloor_cloud);

    // viewer.showCloud(cloud);
    
    // while (!viewer.wasStopped ())
    // {
    // }
}