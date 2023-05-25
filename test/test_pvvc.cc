#include "codec/pvvc_encoder.h"

int main() {
	vvc::common::ParameterLoader p_loader;

	auto param = p_loader.GetPVVCParam();

	char t;
	std::cout << "Select module \n\ts(segmentation)\n\td(deformation)\n\tc(compression)\n\tf(build-frame)\n\tr(decompression)\n\tother(exit)\nPlease input your selection: ";
	t = getchar();
	if (t == 's' || t == 'S') {
		vvc::codec::PVVCSegmentation seg;
		seg.SetParams(param);
		seg.LoadFrames();
		seg.Segmentation();
		seg.SavePatches();
	}
	else if (t == 'd' || t == 'D') {
		vvc::codec::PVVCDeformation def;
		def.SetParams(param);
		def.LoadPatches();
		def.Deformation();
		def.SaveDeformPatches();
	}
	else if (t == 'c' || t == 'C') {
		vvc::codec::PVVCCompression com;
		com.SetParams(param);
		com.LoadGoPs();
		com.Compression();
		com.SaveSlices();
	}
	else if (t == 'f' || t == 'F') {
		vvc::codec::BuildFrames(param);
	}
	else if (t == 'r' || t == 'R') {
		vvc::codec::PVVCDecompression dec;
		dec.SetParams(param);
		dec.LoadSlices();
		dec.Decompression();
	}
	else {
		std::cout << "Unknown type, patchVVC exit.\n";
	}
	return 0;
}
