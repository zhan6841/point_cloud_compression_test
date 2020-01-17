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

struct Statistics{
    int count;
    double compression_ratio[2000];
    double cr_avg;
    double cr_stddev;
    double encoding_time[2000];
    double et_avg;
    double et_stddev;
    double decoding_time[2000];
    double dt_avg;
    double dt_stddev;
};

struct Statistics pcl_statistics;

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
    
    bool showStatistics = true;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_INTENSITY;
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITH_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_INTENSITY;
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_INTENSITY;

    // instantiate point cloud compression for encoding and decoding
    double pointResolution = 0.01;
    const double octreeResolution = 0.01;
    bool doVoxelGridDownSampling = true;
    unsigned int iFrameRate = 50;
    const unsigned char colorBitResolution = 4;
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
    double et = timer->getTime();

    pcl_statistics.compression_ratio[pcl_statistics.count] = compressedData.str().length()/(cloud->points.size() * (12.0f + 4.0f));
    pcl_statistics.cr_avg += compressedData.str().length()/(cloud->points.size() * (12.0f + 4.0f));
    pcl_statistics.encoding_time[pcl_statistics.count] = et;
    pcl_statistics.et_avg += et;

    std::cout << "encode time: " << et << std::endl;

    timer->reset();

    PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
    double dt = timer->getTime();
    pcl_statistics.decoding_time[pcl_statistics.count] = dt;
    pcl_statistics.dt_avg += dt;
    std::cout << "decode time: " << dt << std::endl;
    std::cout << "Decode " << cloudOut->width * cloudOut->height << " data points.\n" << std::endl;

    // delete point cloud compression instances
    delete (PointCloudEncoder);
    delete (PointCloudDecoder);

    pcl_statistics.count += 1;
    
    return (0);
}

int main (int argc, char** argv)
{
    if(argc != 1) {
		std::cout << "usage: ./pcl_evaluation " << std::endl;
		return 1;
	}

    pcl_statistics.count = 0;
    pcl_statistics.cr_avg = 0;
    pcl_statistics.cr_stddev = 0;
    pcl_statistics.et_avg = 0;
    pcl_statistics.et_stddev = 0;
    pcl_statistics.dt_avg = 0;
    pcl_statistics.dt_stddev = 0;

    // 2011_09_26_drive_0001_sync
    for(int frame_index = 0; frame_index < 108; frame_index++){
        char inputfile[256];
        if(frame_index < 10){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/000000000%d.bin", frame_index);
        }
        else if(frame_index >= 10 && frame_index < 100){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/00000000%d.bin", frame_index);
        }
        else{
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000%d.bin", frame_index);
        }
        compressXYZI(inputfile);
    }

    // 2011_09_26_drive_0002_sync
    for(int frame_index = 0; frame_index < 77; frame_index++){
        char inputfile[256];
        if(frame_index < 10){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/000000000%d.bin", frame_index);
        }
        else{
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/00000000%d.bin", frame_index);
        }
        compressXYZI(inputfile);
    }

    // 2011_09_26_drive_0005_sync
    for(int frame_index = 0; frame_index < 154; frame_index++){
        char inputfile[256];
        if(frame_index < 10){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/000000000%d.bin", frame_index);
        }
        else if(frame_index >= 10 && frame_index < 100){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/00000000%d.bin", frame_index);
        }
        else{
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/0000000%d.bin", frame_index);
        }
        compressXYZI(inputfile);
    }

    // 2011_09_26_drive_0009_sync
    for(int frame_index = 0; frame_index < 447; frame_index++){
        char inputfile[256];
        if(frame_index >= 177 && frame_index <= 180){
            continue;
        }
        if(frame_index < 10){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data/000000000%d.bin", frame_index);
        }
        else if(frame_index >= 10 && frame_index < 100){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data/00000000%d.bin", frame_index);
        }
        else{
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data/0000000%d.bin", frame_index);
        }
        compressXYZI(inputfile);
    }

    // 2011_09_26_drive_0011_sync
    for(int frame_index = 0; frame_index < 233; frame_index++){
        char inputfile[256];
        if(frame_index < 10){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0011_sync/velodyne_points/data/000000000%d.bin", frame_index);
        }
        else if(frame_index >= 10 && frame_index < 100){
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0011_sync/velodyne_points/data/00000000%d.bin", frame_index);
        }
        else{
            sprintf(inputfile, "../data/2011_09_26/2011_09_26_drive_0011_sync/velodyne_points/data/0000000%d.bin", frame_index);
        }
        compressXYZI(inputfile);
    }

    // calculate average
    pcl_statistics.cr_avg /= pcl_statistics.count;
    pcl_statistics.et_avg /= pcl_statistics.count;
    pcl_statistics.dt_avg /= pcl_statistics.count;

    // calculate stddev
    for(int i = 0; i < pcl_statistics.count; i++){
        pcl_statistics.cr_stddev += (pcl_statistics.compression_ratio[i] - pcl_statistics.cr_avg) * (pcl_statistics.compression_ratio[i] - pcl_statistics.cr_avg);
        pcl_statistics.et_stddev += (pcl_statistics.encoding_time[i] - pcl_statistics.et_avg) * (pcl_statistics.encoding_time[i] - pcl_statistics.et_avg);
        pcl_statistics.dt_stddev += (pcl_statistics.decoding_time[i] - pcl_statistics.dt_avg) * (pcl_statistics.decoding_time[i] - pcl_statistics.dt_avg);
    }
    pcl_statistics.cr_stddev = sqrt(pcl_statistics.cr_stddev / pcl_statistics.count);
    pcl_statistics.et_stddev = sqrt(pcl_statistics.et_stddev / pcl_statistics.count);
    pcl_statistics.dt_stddev = sqrt(pcl_statistics.dt_stddev / pcl_statistics.count);

	printf("Total Count: %d \
        \nOctree Avg. Compression ratio: %.6f, stddev: %.6f \
        \nOctree Avg. Encoding time: %.6fms, stddev: %.6f \
        \nOctree Avg. Decoding time: %.6fms, stddev: %.6f\n", \
        pcl_statistics.count, \
        pcl_statistics.cr_avg, pcl_statistics.cr_stddev, \
        pcl_statistics.et_avg, pcl_statistics.et_stddev, \
        pcl_statistics.dt_avg, pcl_statistics.dt_stddev);
    
    return (0);
}