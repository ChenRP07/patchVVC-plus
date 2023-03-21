/* Copyright Notice.
 * 
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 * 
 * Copyright: @ChenRP07, All Right Reserved.
 * 
 * Author        : ChenRP07
 * Description   : 
 * Create Time   : 2023/03/17 16:59
 * Last Modified : 2023/03/17 16:59
 * 
 */ 

#include <pcl/point_types.h>

namespace vvc {
namespace common {

    struct ColorYUV {
        float y, u, v;
        ColorYUV(float _y, float _u, float _v) {
            this->y = _y, this->u = _u, this->v = _v;
        }
        
        ColorYUV(const pcl::PointXYZRGB& _p) {
            this->y = _p.r * 0.299f + _p.g * 0.587f + _p.b * 0.114f;
            this->u = _p.r * -0.168736f - _p.g * 0.331264f + _p.b * 0.5f + 128;
            this->v = _p.r * 0.5f - _p.g * 0.418688f - _p.b * 0.081312f + 128;
        }
    };
}
}
