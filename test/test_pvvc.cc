#include "codec/pvvc_encoder.h"

int main() {
	vvc::common::ParameterLoader p_loader;

	auto param = p_loader.GetPVVCParam();

	vvc::codec::PVVCDeformation def;
	def.SetParams(param);
	def.LoadPatches();
	def.Deformation();
	def.SaveDeformPatches();
}
