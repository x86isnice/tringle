#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>


using namespace std;

int user_data = 0;

void viewerOneOff(pcl::visualization::PCLVisualizer & viewer)
{
   viewer.setBackgroundColor(0.0,0.0,0.0);
   pcl::PointXYZ o;
   o.x = 0;
   o.y = 0;
   o.z = 0;
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
   static unsigned count = 0;
   std::stringstream ss;
   ss << "Onec per viewer loop:" << count++;

  viewer.addText(ss.str(),200, 300, "text" , 0);
  user_data++;
}

int main(int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  //std::string filename = "../blade";
  //std::string filename = "/home/x86isnice/0627_keypoints_with_filter/rem_data/p2at_rem";
  std::string filename = "/home/x86isnice/Match_Gen _PCL/myfairlady_o";
  pcl::io::loadPCDFile (filename + ".pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud
  std::cout << "step1" << std::endl;

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  std::cout << "step2" << std::endl;
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (1500);//600
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures
  std::cout << "step 3" << std::endl;
  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals
   std::cout << "step 4" << std::endl;
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;
  std::cout << "step 5" << std::endl;
  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (888); // 100

  // Set typical values for the parameters
  gp3.setMu (25); // 25
  gp3.setMaximumNearestNeighbors (1000); // 66
  gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
  gp3.setMinimumAngle(M_PI/20); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/2); // 120 degrees
  gp3.setNormalConsistency(false);
   std::cout << "step 7" << std::endl;
  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  std::cout << "step 8" << std::endl;
  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::io::saveVTKFile (filename+".vtk", triangles);
   std::cout << "-------------END------------------" << std::endl;
/*
   pcl::visualization::CloudViewer viewer("CloudViewer");

   viewer.showCloud(cloud);


   viewer.runOnVisualizationThread(viewerPsycho);

   while (!viewer.wasStopped())
   {
	   user_data++;
   }*/
  // Finish
  return (0);
}
