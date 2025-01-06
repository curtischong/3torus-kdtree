run:
	g++ -std=c++17 main.cc && ./a.out
test1d:
	g++ -std=c++17 kdtree_1d.cc && ./a.out
test3d:
	g++ -std=c++17 -I./eigen-3.4.0 kdtree_3d.cc && ./a.out
compile3d:
	g++ -O3 -Wall -shared -std=c++11 -fPIC \
	-I./eigen-3.4.0 -I./eigen-3.4.0/Eigen \
	-I./venv/lib/python3.11/site-packages/pybind11/include \
	-I/opt/homebrew/Frameworks/Python.framework/Versions/3.11/include/python3.11 \
	-L/opt/homebrew/Frameworks/Python.framework/Versions/3.11/lib \
	-lpython3.11 \
	bind.cc -o kdtree_3torus.so
