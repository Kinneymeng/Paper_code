#include <iostream>           
#include <pcl/io/pcd_io.h>      
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>
//#include <dirent.h>
#include <io.h>
using namespace std;

//遍历文件夹获取文件夹下文件名
void getFileNames(string path, vector<string>& files)
{
	//文件句柄
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,递归查找
			//如果不是,把文件绝对路径存入vector中
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFileNames(p.assign(path).append("/").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


void pcd2bin(string in_file, string out_file)
{

	////Create a PointCloud value
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

	////Open the PCD file
	//if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1)
	//{
	//	PCL_ERROR("Couldn't read in_file\n");
	//}
	////Create & write .bin file
	//ofstream bin_file(out_file.c_str(), ios::out | ios::binary | ios::app);
	//if (!bin_file.good()) cout << "Couldn't open " << out_file << endl;

	////PCD 2 BIN 
	//for (size_t i = 0; i < cloud->points.size(); ++i)
	//{
	//	bin_file.write((char*)&cloud->points[i].x, 3 * sizeof(float));
	//	bin_file.write((char*)&cloud->points[i].intensity, sizeof(float));
	//	//bin_file.write(0, sizeof(float));
	//}

	//bin_file.close();

	//Create a PointCloud value
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//Open the PCD file
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(in_file, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read in_file\n");
	}
	//Create & write .bin file
	ofstream bin_file(out_file.c_str(), ios::out | ios::binary | ios::app);
	if (!bin_file.good()) cout << "Couldn't open " << out_file << endl;

	//PCD 2 BIN 
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		//const char* p = "A";
		bin_file.write((char*)&cloud->points[i].x, 3 * sizeof(float));
		//用x代替强度值
		bin_file.write((char*)&cloud->points[i].x, sizeof(float));
		//bin_file.write(p, sizeof(float));
	}

	bin_file.close();
}
void bin2pcd(string in_file, string out_file) {
	fstream input(in_file.c_str(), ios::in | ios::binary);
	if (!input.good()) {
		cerr << "Couldn't read in_file: " << in_file << endl;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

	int i;
	for (i = 0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char*)&point.x, 3 * sizeof(float));
		input.read((char*)&point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();

	pcl::io::savePCDFileASCII(out_file, *points);
}
int main(int argc, char** argv) {

	vector<string> fileNames;
	//将路径更改为要修改的点云的存储目录路径
	string path("E:/pointdata/Point2Bin/");
	getFileNames(path, fileNames);
	for (const auto& ph : fileNames) {
		std::cout << "ph: " << ph << "\n";
		//不带路径的文件名
		string::size_type iPos = ph.find_last_of("/") + 1;
		string filename = ph.substr(iPos, path.length() - iPos);
		cout << "filename: " << filename << endl;
		//不带后缀的文件名
		string name = filename.substr(0, filename.rfind("."));
		cout << "name: " << name << endl;
		//记得将这里的路径改为自己转换后的点云要存储的文件路径
		//pcd格式的点云转成bin格式的，对于不含强度信息的点云会提示缺少强度intensity信息，但还是可以转换成功
		pcd2bin(ph, "E:/pointdata/Point2Bin/" + name + ".bin");
		//bin格式的点云转成pcd格式的,使用方法注释上面那行，取消注释下面这行。
		//bin2pcd(ph, "E:/DataSet/PCD/" + name + ".pcd");
	}
	return 0;
}
