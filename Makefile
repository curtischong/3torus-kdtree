run:
	g++ -std=c++17 main.cc && ./a.out
test1d:
	g++ -std=c++17 kdtree_1d.cc && ./a.out
test3d:
	g++ -std=c++17 -I./eigen-3.4.0 kdtree_3d.cc && ./a.out
compile3d:
# g++ -std=c++17 -I$(shell python3 -m pybind11 --includes) -I./eigen-3.4.0 bind.cc && ./a.out
	g++ -std=c++17 -I/opt/homebrew/opt/python@3.13/Frameworks/Python.framework/Versions/3.13/include/python3.13 -I/opt/homebrew/lib/python3.13/site-packages/pybind11/include -I./eigen-3.4.0 bind.cc && ./a.out