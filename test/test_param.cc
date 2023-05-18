#include "cuda/base.cuh"
#include "io/slice_io.h"

void Write() {
	std::filesystem::path dir{"./data/test_frame"};
	std::string           out_name{"./data/result.frame"};
	vvc::io::ChangeSliceToFrame(dir, out_name);
}

void Read() {
	vvc::client::common::Frame_t f;
	std::string                  name{"./data/result.frame"};
	std::cout << vvc::client::io::LoadFrame(f, name) << std::endl;
	printf("Frame timestamp : %d\nTotal slice cnt : %d\n", f.timestamp, f.slice_cnt);
	printf("Index : ");
	for (int i = 0; i < f.slice_cnt; i++) {
		printf("%d ", f.index[i]);
	}
	printf("\nSize : ");

	for (int i = 0; i < f.slice_cnt; i++) {
		printf("%d ", f.size[i]);
	}
	printf("\nType : ");

	for (int i = 0; i < f.slice_cnt; i++) {
		printf("%02x ", f.type[i]);
	}
	printf("\nMV : \n");

	for (int i = 0; i < f.slice_cnt; i++) {
		for (int j = 0; j < 16; j++) {
			if (j % 4 == 0) {
				printf("\t");
			}
			printf("%.2f ", f.mv[i][j]);
		}
		printf("\n");
	}
	printf("QP : ");
	for (int i = 0; i < f.slice_cnt; i++) {
		printf("%d ", f.qp[i]);
	}
	printf("\nGEO size : ");

	for (int i = 0; i < f.slice_cnt; i++) {
		printf("%d ", f.geometry_size[i]);
	}
	printf("\nColor size : ");
	for (int i = 0; i < f.slice_cnt; i++) {
		printf("%d ", f.color_size[i]);
	}
    printf("\n");
}
int main() {
    int t;
    std::cin>>t;
    if (t) {
        Write();
    }
    else {
        Read();
    }
}
