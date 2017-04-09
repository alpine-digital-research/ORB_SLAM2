echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make VERBOSE=1 -j 1

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make VERBOSE=1 -j 1

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make VERBOSE=1 -j 1
