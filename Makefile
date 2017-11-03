all:
	g++ -I/usr/include/eigen3 -Wl,-rpath,"/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/lib" -I"/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/include/" main.cpp -L"/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/lib/" -lorocos-kdl -o main
	g++ -ggdb -O0 -c main.cpp -std=c++14

partial:
	g++ -I/usr/include/eigen3 -I"/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/include/" main.cpp -L"/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/lib/" -lorocos-kdl -o main
