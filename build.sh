echo "Fetching Pangolin ..."

cd Thirdparty
git clone https://github.com/stevenlovegrove/Pangolin.git
cd ..

echo "Configuring and building Thirdparty/Pangolin ..."

sudo apt-get install libglew-dev cmake libpython2.7-dev ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev -y
cd Thirdparty/Pangolin
mkdir build
cd build
cmake ..
cmake --build .
cd ../../..

echo "Fetching Eigen ..."

cd Thirdparty
git clone https://github.com/eigenteam/eigen-git-mirror.git
cd ..

echo "Configuring and building Thirdparty/Eigen ..."

cd Thirdparty/eigen-git-mirror
mkdir build
cd build
cmake ..
make install
cd ../../..





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

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Done"
