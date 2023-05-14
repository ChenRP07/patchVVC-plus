#include "codec/pvvc_encoder.h"
#include "io/patch_io.h"
#include "io/ply_io.h"
#include <fstream>
vvc::common::PVVCParam_t::Ptr param;
void                          SegmentCloud() {
    auto cloud = vvc::io::LoadColorPlyFile("./data/loot_vox10_1000.ply");

    vvc::segment::DenseSegment seg;
    seg.SetTimeStamp(1);
    seg.SetParams(param);
    seg.SetSourcePointCloud(cloud);
    seg.Segment();

    auto          patches = seg.GetResultPatches();
    boost::format fmt{"./data/result/result_%1$d/result_%1$d_%2$d.ply"};

    for (auto& p : patches) {
        fmt % p.timestamp % p.index;
        std::string name = fmt.str();
        vvc::io::SaveColorPlyFile(name, p.cloud);
        if (p.index == 100) {
            vvc::io::SavePatch(p, "./data/result_1_100.patch");
            std::ofstream out("./data/p1.txt");
            out << p.index << ' ' << p.timestamp << '\n';
            out << p.mv << '\n';
            for (auto i : *p.cloud) {
                out << i << '\n';
            }
        }
    }

    auto                           cloud_p = vvc::io::LoadColorPlyFile("./data/loot_vox10_1010.ply");
    vvc::registration::ParallelICP icp;
    icp.SetParams(param);
    icp.SetSourceClouds(patches);
    icp.SetTargetCloud(cloud_p);
    icp.Align();
    auto patches_p = icp.GetResultClouds();
    for (auto& p : patches_p) {
        fmt % p.timestamp % p.index;
        std::string name = fmt.str();
        vvc::io::SaveColorPlyFile(name, p.cloud);
        if (p.index == 100) {
            vvc::io::SavePatch(p, "./data/result_2_100.patch");
            std::ofstream out("./data/p2.txt");
            out << p.index << ' ' << p.timestamp << '\n';
            out << p.mv << '\n';
            for (auto i : *p.cloud) {
                out << i << '\n';
            }
        }
    }
}

vvc::common::Patch p1, p2;

void Read() {
	vvc::io::LoadPatch(p1, "./data/result_1_100.patch");
	vvc::io::LoadPatch(p2, "./data/result_2_100.patch");
}

void Encode() {
	vvc::patch::PatchFitting fit;
	fit.SetParams(param);
	fit.AddPatch(p1);
	fit.AddPatch(p2);
	auto ptr = fit.GetFittingCloud();
	auto pts = fit.GetSourcePatches();
	std::cout << ptr->size() << std::endl;
	vvc::common::MSE mse1, mse2;
	mse1.SetClouds(ptr, pts[0].cloud);
	mse1.Compute();
	auto m1 = mse1.GetGeoMSEs();
	std::cout << m1.first << " " << m1.second << '\n';
	mse2.SetClouds(ptr, pts[1].cloud);
	mse2.Compute();
	auto m2 = mse2.GetGeoMSEs();
	std::cout << m2.first << " " << m2.second << '\n';
	vvc::io::SaveColorPlyFile("./data/result_fit.ply", ptr);
	vvc::patch::GoPEncoding enc;
}

int main() {
	param = vvc::common::SetDefaultParams();
	// SegmentCloud();
	Read();
	Encode();
	return 0;
}
