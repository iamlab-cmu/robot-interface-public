cd robot-interface

mkdir -p /usr/local/lib/cmake/Poco/
cp cmake/FindPoco.cmake /usr/local/lib/cmake/FindPoco.cmake

bash ./bash_scripts/make_libfranka.sh
bash ./bash_scripts/make_iam_robolib.sh
