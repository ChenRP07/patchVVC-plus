#include "common/parameter.h"
int main() {
	vvc::common::ParameterLoader loader;
	auto                         param = loader.GetPVVCParam();
	param->Log();
}
