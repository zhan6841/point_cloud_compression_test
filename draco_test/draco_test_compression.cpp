#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <dirent.h>

#include <vector>
#include <string>

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include "draco/point_cloud/point_cloud_builder.h"
#include "draco/compression/point_cloud/point_cloud_kd_tree_decoder.h"
#include "draco/compression/point_cloud/point_cloud_kd_tree_encoder.h"
#include "draco/compression/encode.h"

using namespace std;
using namespace draco;

#define MAX_LINE_SIZE 4096

inline void MyAssert(int x, int id) {
	if (!x) {
		printf("Assertion failed. ID=%d\n", id);
		exit(0);
	}
}

double NDKGetTime() {
    struct timespec res;
    clock_gettime(CLOCK_REALTIME, &res);
    double t = res.tv_sec + (double) res.tv_nsec / 1e9f;
    return t;
}

int main(int argc, char * * argv) {
	if (argc != 4) {
		printf("Usage: %s [input_file(.bin)] cl(compression level:0-10) qb(quantization bits:1-31)\n", argv[0]);
		return -1;
	}

    int cl = atoi(argv[2]);
    int qb = atoi(argv[3]);

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
    stream = fopen (argv[1],"rb");
    num = fread(data,sizeof(float),num,stream)/4;

    // point cloud builder
    PointCloudBuilder builder;
	builder.Start(num);
	const int pos_att_id = builder.AddAttribute(GeometryAttribute::POSITION, 3, DT_FLOAT32);
	const int color_att_id = builder.AddAttribute(GeometryAttribute::COLOR, 1, DT_FLOAT32);
    for (int index = 0; index < num; index++) {
		std::array<float, 3> point;
		point[0] = *px;
		point[1] = *py;
		point[2] = *pz;
		builder.SetAttributeValueForPoint(pos_att_id, PointIndex(index), &(point)[0]);

		std::array<float, 1> color;
		color[0] = *pr;
        // std::cout << color[0] << std::endl;
		builder.SetAttributeValueForPoint(color_att_id, PointIndex(index), &(color)[0]);

        // printf("%f %f %f %.2f\n", *px, *py, *pz, *pr);
        // std::cout << *px << " " << *py << " " << *pz << " " << *pr << std::endl;

        px+=4; py+=4; pz+=4; pr+=4;
	}
	std::unique_ptr<PointCloud> pc = builder.Finalize(false);

    fclose(stream);

    std::array<float, 3> position_array;
    std::array<float, 1> intensity_array;

    FILE *fin = fopen("in.xyz", "w");

    GeometryAttribute *pos = pc->attribute(pos_att_id);
    GeometryAttribute *intensity = pc->attribute(color_att_id);
    for (int index = 0; index < num; index++) {
        pos->GetValue(AttributeValueIndex(index), &position_array);
        intensity->GetValue(AttributeValueIndex(index), &intensity_array);
        fprintf(fin, "%f %f %f %.2f\n", position_array[0], position_array[1], position_array[2], intensity_array[0]);
    }

    fclose(fin);

    // encode/decode using kd-tree
    int compression_level = cl;
	EncoderBuffer buffer;
	PointCloudKdTreeEncoder encoder;
	EncoderOptions options = EncoderOptions::CreateDefaultOptions();
	options.SetGlobalInt("quantization_bits", qb);
	options.SetSpeed(10- compression_level, 10 - compression_level);
	encoder.SetPointCloud(*pc);
	double t1 = NDKGetTime();
	MyAssert(encoder.Encode(options, &buffer).ok(), 2001);
	double t2 = NDKGetTime();

	printf("Compression level: %d\n", compression_level);
	printf("KD-Tree Compression ratio: %.6f \
        \noriginal bytes:%d \
        \ncompressioned bytes:%d \
        \nKD-tree Encoding time:%.6f\n", \
        buffer.size()/(num * (12.0f + 4.0f)), num * (12 + 4), (int)buffer.size(), t2 - t1);
      
	DecoderBuffer dec_buffer;
	dec_buffer.Init(buffer.data(), buffer.size());
	PointCloudKdTreeDecoder decoder;

	std::unique_ptr<PointCloud> out_pc(new PointCloud());
	DecoderOptions dec_options;
	double t3 = NDKGetTime();
	MyAssert(decoder.Decode(dec_options, &dec_buffer, out_pc.get()).ok(), 2002);
	double t4 = NDKGetTime();

	printf("KD-Tree Decoding time: %.6f\n", t4 - t3);

    FILE *fout = fopen("out.xyz", "w");

    pos = out_pc->attribute(pos_att_id);
    intensity = out_pc->attribute(color_att_id);
    for (int index = 0; index < num; index++) {
        pos->GetValue(AttributeValueIndex(index), &position_array);
        intensity->GetValue(AttributeValueIndex(index), &intensity_array);
        fprintf(fout, "%f %f %f %.2f\n", position_array[0], position_array[1], position_array[2], intensity_array[0]);
    }

    fclose(fout);

	return 0;
}