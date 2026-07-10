echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

<<<<<<< HEAD
cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."
=======
cd ../../elas

echo "Configuring and building Thirdparty/elas ..."
>>>>>>> origin/main

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

<<<<<<< HEAD
=======

>>>>>>> origin/main
cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
<<<<<<< HEAD
make -j4
=======
make -j 12
>>>>>>> origin/main
