#include "codec/pvvc_encoder.h"
#include "io/patch_io.h"
#include "io/ply_io.h"
#include "io/slice_io.h"
#include <fstream>
vvc::common::PVVCParam_t::Ptr param;
vvc::common::Patch            p1, p2;

void Decode() {
	vvc::common::Slice si, sp;
	vvc::io::LoadSlice(si, "./data/result_1_100.slice");
	vvc::io::LoadSlice(sp, "./data/result_2_100.slice");
	vvc::octree::InvertRAHTOctree dec;
	dec.SetParams(param);
	dec.SetSlice(si);
	auto resi = dec.GetPatch();
	dec.SetSlice(sp);
	auto resp = dec.GetPatch();
	auto             cloudi = vvc::io::LoadColorPlyFile("./data/result_fit_1_100.ply");
	auto             cloudp = vvc::io::LoadColorPlyFile("./data/result_fit_2_100.ply");
	vvc::common::MSE m1;
	m1.SetClouds(resi.cloud, cloudi);
	m1.Compute();
	auto gi = m1.GetGeoMSEs();
	std::cout << gi.first << " " << gi.second << '\n';
	auto gc = m1.GetYMSEs();
	std::cout << gc.first << " " << gc.second << '\n';
	vvc::common::MSE m2;
	m2.SetClouds(resp.cloud, cloudp);
	m2.Compute();
	auto gp = m2.GetGeoMSEs();
	std::cout << gp.first << " " << gp.second << '\n';
	auto gcp = m2.GetYMSEs();
	std::cout << gcp.first << " " << gcp.second << '\n';
    vvc::io::SavePatch(resi, "./data/decode_1_100.patch");
    vvc::io::SavePatch(resp, "./data/decode_2_100.patch");
}

int main() {
	vvc::common::ParameterLoader Loader;
	param = Loader.GetPVVCParam();
	Decode();
	return 0;
}
