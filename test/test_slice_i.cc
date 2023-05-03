#include "io/slice_io.h"
#include "patch/patch.h"
int main() {
	vvc::common::Slice slice;

	vvc::io::LoadSlice(slice, "./save_test.slice");
	vvc::common::PVVCParam_t::Ptr param(new vvc::common::PVVCParam_t());
	vvc::common::SetDefaultParams(param);

	std::cout << "Zstd geo : " << slice.geometry->size() << '\n';
	for (auto i : *slice.geometry) {
		printf("%02x ", i);
	}
	std::cout << "\nZstd color : " << slice.color->size() << '\n';
	for (auto i : *slice.color) {
		printf("%02x ", i);
	}

	vvc::common::ZstdDecoder zstd_dec;
	zstd_dec.Decode(slice.geometry);

	auto zstd_geo_result = zstd_dec.GetResult();

	std::cout << "\nGeo : " << zstd_geo_result->size() << '\n';
	for (auto i : *zstd_geo_result) {
		printf("%02x ", i);
	}

	zstd_dec.Decode(slice.color);
	auto zstd_color_result = zstd_dec.GetResult();

	std::cout << "\nRLGR : " << zstd_color_result->size() << '\n';
	for (auto i : *zstd_color_result) {
		printf("%02x ", i);
	}

	uint8_t temp[25];
	for (int i = 0; i < 25; ++i) {
		temp[i] = zstd_geo_result->at(i);
	}
	pcl::PointXYZ center, range;
	int           height;
	vvc::octree::LoadTreeCore(center, range, height, temp);

	std::cout << "\n" << center << '\n' << range << '\n' << height << '\n';

	vvc::common::RLGRDecoder rlgr_dec;
	rlgr_dec.Decode(zstd_color_result, 82);
	auto colors = rlgr_dec.GetResult();

	std::cout << "\nColors : " << colors->size() << '\n';
	for (auto i : *colors) {
		std::cout << i << ' ';
	}

	std::cout << '\n' << slice.timestamp << ' ' << slice.index << ' ';
	printf("%02x ", slice.type);
	std::cout << '\n' << slice.mv << '\n';
	return 0;
}
