#include <iostream>
#include<vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//1.释放内存 *data 2.保存point_cloud为PCD，注意设定长和高
using namespace std;

int main()
{
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float* data = (float*)malloc(num * sizeof(float));// void *malloc(size_t size) 分配所需的内存空间，并返回一个指向它的指针。

    // pointers
    float* px = data + 0;
    float* py = data + 1;
    float* pz = data + 2;
    float* pr = data + 3;
    float* pg = data + 4;
    float* pb = data + 5;

    // load point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    FILE* stream;
    stream = fopen("00000021.bin", "rb");
    num = fread(data, sizeof(float), num, stream) / 6;
    point_cloud->width = num;//设定长
    point_cloud->height = 1;//设定高
    point_cloud->is_dense = false;//如果没有无效点（例如，具有NaN或Inf值），则为True
    int k = 0;
    for (int32_t i = 0; i < num; i++)
    {
        //vector<int32_t> point_cloud;
        pcl::PointXYZRGB point;
        point.x = *px;
        point.y = *py;
        point.z = *pz;
        point.r = *pr;
        point.g = *pg;
        point.b = *pb;
        if (k == 0) {
            cout << point.r << " " << point.g << " " << point.b << endl;
        }
        k++;
        point_cloud->points.push_back(point);
        px += 6; py += 6; pz += 6; pr += 6; pg += 6; pb += 6;
    }
    fclose(stream);
    free(data);//释放内存

    //可视化
    pcl::visualization::CloudViewer viewer("Cloud Viewer1");
    viewer.showCloud(point_cloud);
    while (!viewer.wasStopped());

    //写成PCD格式文件
    pcl::io::savePCDFileASCII("00000019.pcd", *point_cloud);
    std::cerr << "Saved " << point_cloud->size() << " data points to 000000.pcd." << std::endl;

    return 0;
}