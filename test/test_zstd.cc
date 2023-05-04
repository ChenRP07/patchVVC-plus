#include "common/entropy_codec.h"
int main() {
	using namespace std;
	auto p    = std::make_shared<std::vector<uint8_t>>(100, 12);
	p->at(23) = 100;
	auto pp   = std::make_shared<vvc::common::PVVCParam_t>();
	vvc::common::SetDefaultParams(pp);
	vvc::common::ZstdEncoder enc;
	enc.SetParams(pp);
	enc.Encode(p);
	auto res = enc.GetResult();

	cout << res->size() << '\n';
	for (auto i : *res) {
		printf("%02x ", i);
	}
	cout << '\n';

	vvc::common::ZstdDecoder dec;
	dec.Decode(res);
	auto d = dec.GetResult();

	for (int i = 0; i < 100; i++) {
		if (i % 10 == 0) {
			cout << '\n' << static_cast<int>(d->at(i));
		}
		cout << static_cast<int>(d->at(i));
	}
}
