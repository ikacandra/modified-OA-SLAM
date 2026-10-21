#!/bin/bash
set -e
cd "$(dirname "$0")"

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc)

cd ../../../

echo "Uncompress vocabulary ..."
cd Vocabulary
[ -f ORBvoc.txt ] || tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building OA-SLAM (ORB-SLAM3 backbone) ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
