#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/time.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

inline void PointXYZItoXYZRGB (const pcl::PointXYZI& in, pcl::PointXYZRGB& out){
    out.x = in.x; out.y = in.y; out.z = in.z;
    out.rgb = in.intensity;
}

inline void PointCloudItoRGB (const pcl::PointCloud<pcl::PointXYZI>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
    out.width   = in.width;
    out.height  = in.height;
    for (size_t i = 0; i < in.points.size (); i++){
        pcl::PointXYZRGB p;
        PointXYZItoXYZRGB (in.points[i], p);
        out.points.push_back (p);
    }
}

int compressXYZ(const char* inputFile)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (inputFile,"rb");
    num = fread(data,sizeof(float),num,stream)/4;

    for (int index = 0; index < num; index++) {
        pcl::PointXYZ point;
		point.x = *px;
		point.y = *py;
		point.z = *pz;
        // point.intensity = *pr;
        cloud->push_back(point);
        px+=4; py+=4; pz+=4; pr+=4;
	}

    fclose(stream);

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << inputFile << std::endl;

    FILE *fin = fopen("in.xyz", "w");
    for (int i = 0; i < cloud->points.size (); i++) {
        fprintf(fin, "%f %f %f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }
    fclose(fin);

    bool showStatistics = true;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;

    // instantiate point cloud compression for encoding and decoding
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ> (compressionProfile, showStatistics);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ> ();

    // stringstream to store compressed point cloud
    std::stringstream compressedData;
    // output pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::StopWatch* timer = new pcl::StopWatch();
    PointCloudEncoder->encodePointCloud (cloud, compressedData);
    std::cout << "encode time: " << timer->getTime() << std::endl;
    timer->reset();
    PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
    std::cout << "decode time: " << timer->getTime() << std::endl;
    std::cout << "Decode " << cloudOut->width * cloudOut->height << " data points." << std::endl;

    // delete point cloud compression instances
    delete (PointCloudEncoder);
    delete (PointCloudDecoder);

    FILE *fout = fopen("out.xyz", "w");
    for (int i = 0; i < cloudOut->points.size (); i++) {
        fprintf(fout, "%f %f %f\n", cloudOut->points[i].x, cloudOut->points[i].y, cloudOut->points[i].z);
    }
    fclose(fout);

    return (0);
}

int compressXYZRGB(const char* inputFile)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (inputFile,"rb");
    num = fread(data,sizeof(float),num,stream)/4;

    for (int index = 0; index < num; index++) {
        pcl::PointXYZRGB point;
		point.x = *px;
		point.y = *py;
		point.z = *pz;
        point.rgb = *pr;
        cloud->push_back(point);
        px+=4; py+=4; pz+=4; pr+=4;
	}

    fclose(stream);

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << inputFile << std::endl;

    FILE *fin = fopen("in.xyz", "w");
    for (int i = 0; i < cloud->points.size (); i++) {
        fprintf(fin, "%f %f %f %.f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, cloud->points[i].rgb);
    }
    fclose(fin);

    bool showStatistics = true;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR;

    // instantiate point cloud compression for encoding and decoding
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> (compressionProfile, showStatistics);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> ();

    // stringstream to store compressed point cloud
    std::stringstream compressedData;
    // output pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::StopWatch* timer = new pcl::StopWatch();
    PointCloudEncoder->encodePointCloud (cloud, compressedData);
    std::cout << "encode time: " << timer->getTime() << std::endl;
    timer->reset();
    PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
    std::cout << "decode time: " << timer->getTime() << std::endl;
    std::cout << "Decode " << cloudOut->width * cloudOut->height << " data points." << std::endl;

    // delete point cloud compression instances
    delete (PointCloudEncoder);
    delete (PointCloudDecoder);

    FILE *fout = fopen("out.xyz", "w");
    for (int i = 0; i < cloudOut->points.size (); i++) {
        fprintf(fout, "%f %f %f %.f\n", cloudOut->points[i].x, cloudOut->points[i].y, cloudOut->points[i].z, cloudOut->points[i].rgb);
    }
    fclose(fout);
    
    return (0);
}

int compressXYZI(const char* inputFile)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZI>);

    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (inputFile,"rb");
    num = fread(data,sizeof(float),num,stream)/4;

    for (int index = 0; index < num; index++) {
        pcl::PointXYZI point;
		point.x = *px;
		point.y = *py;
		point.z = *pz;
        point.intensity = *pr;
        cloud->push_back(point);
        px+=4; py+=4; pz+=4; pr+=4;
	}

    fclose(stream);

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << inputFile << std::endl;

    FILE *fin = fopen("in.xyz", "w");
    for (int i = 0; i < cloud->points.size (); i++) {
        fprintf(fin, "%f %f %f %.2f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, cloud->points[i].intensity);
    }
    fclose(fin);

    bool showStatistics = true;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_INTENSITY;
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITH_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_INTENSITY;

    // instantiate point cloud compression for encoding and decoding
    double pointResolution = 0.0001;
    const double octreeResolution = 0.01;
    bool doVoxelGridDownSampling = false;
    unsigned int iFrameRate = 30;
    const unsigned char colorBitResolution = 7;
    bool doColorEncoding = false;
    // double intensityResolution = 0.01;
    bool doIntensityEncoding = true;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZI>* PointCloudEncoder = 
        new pcl::io::OctreePointCloudCompression<pcl::PointXYZI> (pcl::io::MANUAL_CONFIGURATION, showStatistics, 
            pointResolution, octreeResolution, 
            doVoxelGridDownSampling, iFrameRate, doColorEncoding, 
            colorBitResolution, doIntensityEncoding);
    // pcl::io::OctreePointCloudCompression<pcl::PointXYZI>* PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZI> (compressionProfile, showStatistics);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZI>* PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZI> ();

    // stringstream to store compressed point cloud
    std::stringstream compressedData;
    // output pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI> ());

    pcl::StopWatch* timer = new pcl::StopWatch();
    PointCloudEncoder->encodePointCloud (cloud, compressedData);
    std::cout << "encode time: " << timer->getTime() << std::endl;
    timer->reset();
    PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
    std::cout << "decode time: " << timer->getTime() << std::endl;
    std::cout << "Decode " << cloudOut->width * cloudOut->height << " data points." << std::endl;

    // delete point cloud compression instances
    delete (PointCloudEncoder);
    delete (PointCloudDecoder);

    FILE *fout = fopen("out.xyz", "w");
    for (int i = 0; i < cloudOut->points.size (); i++) {
        fprintf(fout, "%f %f %f %.2f\n", cloudOut->points[i].x, cloudOut->points[i].y, cloudOut->points[i].z, cloudOut->points[i].intensity);
    }
    fclose(fout);
    
    return (0);
}

int main (int argc, char** argv)
{
    if(argc != 3) {
		std::cout << "usage: ./pcl_compression_test <xyz|xyzrgb|xyzi> input_file" << std::endl;
		return 1;
	}
    if(!strcmp(argv[1], "xyz")){
		compressXYZ(argv[2]);
	}
    else if(!strcmp(argv[1], "xyzrgb")){
		compressXYZRGB(argv[2]);
	}
    else if(!strcmp(argv[1], "xyzi")){
		compressXYZI(argv[2]);
	}
    else{
        std::cout << "unknown point cloud type" << std::endl;
    }
    return (0);
}