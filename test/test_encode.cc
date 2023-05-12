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

void Read() {
	vvc::common::Patch p1, p2;
	vvc::io::LoadPatch(p1, "./data/result_1_100.patch");
	vvc::io::LoadPatch(p2, "./data/result_2_100.patch");
	std::ofstream out("./data/p11.txt");
	out << p1.index << ' ' << p1.timestamp << '\n';
	out << p1.mv << '\n';
	for (auto i : *p1.cloud) {
		out << i << '\n';
	}
	std::ofstream outt("./data/p22.txt");
	outt << p2.index << ' ' << p2.timestamp << '\n';
	outt << p2.mv << '\n';
	for (auto i : *p2.cloud) {
		outt << i << '\n';
	}
}

void Encode() {
	vvc::patch::PatchFitting fit;
	fit.SetParams(param);
	vvc::patch::GoPEncoding enc;
}

int main() {
	param = vvc::common::SetDefaultParams();
	// SegmentCloud();
    Read();
	return 0;
}
