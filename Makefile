all:
	g++ -I/usr/include/eigen3 -I"/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/include/" main.cpp -L"/home/djole/Downloads/Master/R_&_D/KDL_GIT/KDL_install_dir/lib/" -lorocos-kdl -o main 
