#include "codec/pvvc_encoder.h"
int main() {
    vvc::common::Patch p;
    vvc::io::LoadPatch(p, "/mnt/data/pvvc_data/loot/gops/loot_0000/loot_patch_0015/loot_patch_0015_gop_0000/loot_patch_0015_gop_0000_time_0029.patch");
    std::cout << p.mv << std::endl;
    std::cout << p.mv.inverse() <<std::endl;
    FILE* fp = fopen("1.txt", "wb");
    fwrite(&p.mv, sizeof(Eigen::Matrix4f), 1, fp);
    fclose(fp);

    fp = fopen("1.txt", "rb");
    float a[16];
    fread(&a, sizeof(float), 16, fp);
    fclose(fp);

    for (auto i : a) {
        std::cout << i<< ' ';
    }
    std::cout << std::endl;
}
