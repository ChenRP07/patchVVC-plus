#include "codec/pvvc_encoder.h"

int main() {
	vvc::common::ParameterLoader p_loader;

	auto param = p_loader.GetPVVCParam();

	vvc::codec::PVVCCompensation enc;

	enc.SetParams(param);
	enc.LoadFrames();
}
