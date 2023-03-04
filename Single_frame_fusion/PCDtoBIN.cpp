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

//�����ļ��л�ȡ�ļ������ļ���
void getFileNames(string path, vector<string>& files)
{
	//�ļ����
	intptr_t hFile = 0;
	//�ļ���Ϣ
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//�����Ŀ¼,�ݹ����
			//�������,���ļ�����·������vector��
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
		//��x����ǿ��ֵ
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
	//��·������ΪҪ�޸ĵĵ��ƵĴ洢Ŀ¼·��
	string path("E:/pointdata/Point2Bin/");
	getFileNames(path, fileNames);
	for (const auto& ph : fileNames) {
		std::cout << "ph: " << ph << "\n";
		//����·�����ļ���
		string::size_type iPos = ph.find_last_of("/") + 1;
		string filename = ph.substr(iPos, path.length() - iPos);
		cout << "filename: " << filename << endl;
		//������׺���ļ���
		string name = filename.substr(0, filename.rfind("."));
		cout << "name: " << name << endl;
		//�ǵý������·����Ϊ�Լ�ת����ĵ���Ҫ�洢���ļ�·��
		//pcd��ʽ�ĵ���ת��bin��ʽ�ģ����ڲ���ǿ����Ϣ�ĵ��ƻ���ʾȱ��ǿ��intensity��Ϣ�������ǿ���ת���ɹ�
		pcd2bin(ph, "E:/pointdata/Point2Bin/" + name + ".bin");
		//bin��ʽ�ĵ���ת��pcd��ʽ��,ʹ�÷���ע���������У�ȡ��ע���������С�
		//bin2pcd(ph, "E:/DataSet/PCD/" + name + ".pcd");
	}
	return 0;
}
