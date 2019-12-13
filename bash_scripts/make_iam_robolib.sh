# Get CPU core count
n_cores=$(grep ^cpu\\scores /proc/cpuinfo | uniq |  awk '{print $4}')

[ -d build ] || mkdir build
cd build
# make and ignore unused variable flags
cmake -j$n_cores -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-unused-variable -Wno-unused-parameter -Wno-maybe-uninitialized" .. && make
cd ..