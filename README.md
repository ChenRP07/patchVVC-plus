# patchVVC: A Real-time Compression Framework for Streaming Volumetric Videos

## Install & Build

## Running 

## How to use

### Encoder

Module-IO: Read and write .ply point cloud file, supporting binary and ASCII format.
<h style="color:gold"> Completed. (Coding, formatting, checking)</h>

## TODO list 

* ~~Add Optional Zstd encoding in `vvc::Patch::GoPEncoding::Encode()::152`~~
* Move Zstd and RLGR encoding from `vvc::Patch::GoPEncoding::Encode()` to `vvc::Octree::RAHTOctree`
* Change interface of `vvc::Octree::RAHTOctree`, input `vvc::common::Patch` and output `vvc::common::Slice`
* Add `vvc::common::Slice::size, vvc::common::Slice::qp` to `vvc::io::SaveSlice(), vvc::io::LoadSlice`
* Implement `vvc::Octree::InverRAHTOctree`
* Test ``
