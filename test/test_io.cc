#include "common/common.h"

int main() {
    FILE* fp;
    fp = fopen("./m.dat", "wb");
    Eigen::Matrix4f mm{Eigen::Matrix4f::Identity()};
    mm(2, 3) = 114514.0f;
    fwrite(&mm, sizeof(Eigen::Matrix4f), 1, fp);
    fclose(fp);

    FILE* ff;
    ff = fopen("./m.dat", "rb");
    float a[16];
    fread(a, sizeof(float), 16, ff);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f ", a[i * 4 + j]);
        }
        printf("\n");
    }
}
