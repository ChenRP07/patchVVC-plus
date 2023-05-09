#include "io/slice_io.h"
#include "patch/patch.h"
int main() {
	vvc::common::Slice slice;
	slice.timestamp = 100;
	slice.index     = 1023;
	slice.type      = 0b00011001;
	slice.mv(2, 3)  = 122.1827f;
	pcl::PointXYZ center{1.1f, 4.5f, 1.4f}, range{128.0f, 128.0f, 128.0f};

	vvc::common::PVVCParam_t::Ptr param;
	vvc::common::SetDefaultParams(param);
	auto geo = std::make_shared<std::vector<uint8_t>>();

	vvc::octree::SaveTreeCore(center, range, 8, geo);

	for (int i = 0; i < rand() % 20 + 10; ++i) {
		geo->emplace_back(rand() % 255);
	}

	auto color_source = std::make_shared<std::vector<vvc::common::FIX_DATA_INT>>();
	for (int i = 0; i < rand() % 60 + 30; ++i) {
		color_source->emplace_back(0);
	}
	color_source->emplace_back(-12);
	color_source->emplace_back(-25);
	color_source->emplace_back(13);
	color_source->emplace_back(53);
	color_source->emplace_back(53);
	color_source->emplace_back(-121);
	color_source->emplace_back(234);

	auto color_source_1 = std::make_shared<std::vector<vvc::common::FIX_DATA_INT>>(*color_source);
	for (auto i : *color_source_1) {
		color_source->emplace_back(i);
	}

	std::cout << slice.timestamp << ' ' << slice.index << ' ';
	printf("%02x ", slice.type);
	std::cout << '\n' << slice.mv << '\n';
	std::cout << center << '\n' << range << '\n' << 8 << '\n' << "Geometry : " << geo->size() << '\n';
	for (auto i : *geo) {
		printf("%02x ", i);
	}
	std::cout << '\n' << "Colors : " << color_source->size() << '\n';

	for (auto i : *color_source) {
		std::cout << i << ' ';
	}
	std::cout << '\n' << "RLGR : ";

	vvc::common::RLGREncoder rlgr_enc;
	rlgr_enc.Encode(color_source);

	auto color_rlgf = rlgr_enc.GetResult();

	std::cout << color_rlgf->size() << '\n';
	for (auto i : *color_rlgf) {
		printf("%02x ", i);
	}
	std::cout << '\n';
	vvc::common::ZstdEncoder zstd_enc;

	zstd_enc.SetParams(param);
	zstd_enc.Encode(geo);
	slice.geometry = zstd_enc.GetResult();

	std::cout << "Zstd geo : " << slice.geometry->size() << '\n';
	for (auto i : *slice.geometry) {
		printf("%02x ", i);
	}

	zstd_enc.Encode(color_rlgf);
	slice.color = zstd_enc.GetResult();
	std::cout << '\n' << "Zstd color : " << slice.color->size() << '\n';
	for (auto i : *slice.color) {
		printf("%02x ", i);
	}
	std::cout << '\n';

	std::cout << "Total size is : " << vvc::io::SaveSlice(slice, "./save_test.slice") << '\n';
	return 0;
}
