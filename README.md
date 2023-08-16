# A GPU-Enabled Real-Time Framework for Compressing and Rendering Volumetric Videos

## Install & Build

Clone this repo first.

`git clone git@github.com:ChenRP07/vvc.git`

Make sure you have already installed libpcl, libzstd and libboost.

* libpcl: [link](https://github.com/PointCloudLibrary/pcl)
* libzstd: [link](https://github.com/facebook/zstd)
* libboost: [link](https://www.boost.org/)

Note: libzstd and libboost are usually installed by default in linux distributions, at least in ubuntu.

### Linux & Unix

`cd vvc`

`mkdir build && cd build`

`cmake .. && make`

## Running 

## How to use

### Encoder

## TODO list 

* √ Add Optional Zstd encoding in `vvc::Patch::GoPEncoding::Encode()::152`  
* ~~Move Zstd and RLGR encoding from `vvc::Patch::GoPEncoding::Encode()` to `vvc::Octree::RAHTOctree`~~
* ~~Change interface of `vvc::Octree::RAHTOctree`, input `vvc::common::Patch` and output `vvc::common::Slice`~~
* √ Add `vvc::common::Slice::size, vvc::common::Slice::qp` to `vvc::io::SaveSlice(), vvc::io::LoadSlice`
* Implement `vvc::Octree::InverRAHTOctree`
* Test
