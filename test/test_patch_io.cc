#include "io/patch_io.h"

int main() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < rand() % 20 + 10; i++) {
		pcl::PointXYZRGB p;
		p.x = i + rand() % 10 / 10.0f;
		p.y = i + rand() % 10 / 10.0f;
		p.z = i + rand() % 10 / 10.0f;
		p.r = i + rand() % 20;
		p.g = i + rand() % 20;
		p.b = i + rand() % 20;
		cloud0->emplace_back(p);
	}
	vvc::common::Patch dat;
	dat.timestamp = rand() % 100;
	dat.index     = rand() % 1000;
	dat.cloud     = cloud0;
	dat.mv(2, 3)  = 2134.2f;

	std::cout << dat.timestamp << " / " << dat.index << '\n';
	std::cout << dat.mv << '\n';
	std::cout << dat.cloud->size() << '\n';
	for (auto i : *dat.cloud) {
		std::cout << i << '\n';
	}

	vvc::io::SavePatch(dat, "./save_test.patch");

	vvc::common::Patch res;
	vvc::io::LoadPatch(res, "./save_test.patch");

	std::cout << res.timestamp << " / " << dat.index << '\n';
	std::cout << res.mv << '\n';
	std::cout << res.cloud->size() << '\n';
	for (auto i : *res.cloud) {
		std::cout << i << '\n';
	}
	return 0;
}
