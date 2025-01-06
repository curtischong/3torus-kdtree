run:
	g++ -std=c++17 main.cc && ./a.out
test1d:
	g++ -std=c++17 kdtree_1d.cc && ./a.out
test3d:
	g++ -std=c++17 -I./eigen-3.4.0 kdtree_3d.cc && ./a.out
compile3d:
	g++ -O3 -Wall -shared -std=c++11 -fPIC \
	-I/Users/curtischong/Documents/dev/3torus-kdtree/venv/lib/python3.11/site-packages/pybind11/include \
	-I$(shell python3-config --includes) \
	-I./eigen-3.4.0 -I./eigen-3.4.0/Eigen \
	bind.cc -o example