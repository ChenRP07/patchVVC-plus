# patchVVC: A Real-time Compression Framework for Streaming Volumetric Videos

## Install & Build

## Running 

## How to use

### Encoder

Module-IO: Read and write .ply point cloud file, supporting binary and ASCII format.
<h style="color:gold"> Completed. (Coding, formatting, checking)</h>


Read point cloud I-frame.(IO)
Generate initial patches.(segment--dense segment)
Read point cloud.(IO)
Search motion vectors.(registration)
Add patches to group, select new I-patch for deviate patches.(TODO)
Generate common patch.(TODO)
Compress geometry and colors.(TODO:RAHT, new entrpy codec)
Decompress.(TODO)
FoV.(Optional)
