run:
	g++ -std=c++17 main.cc && ./a.out
test1d:
	g++ -std=c++17 kdtree_1d.cc && ./a.out
test3d:
	g++ -std=c++17 -I./eigen-3.4.0 kdtree_3d.cc && ./a.out
compile3d:
	c++ -O3 -Wall -std=c++11 -fPIC \
	$(python -m pybind11 --includes) \
	-I./eigen-3.4.0 -I./eigen-3.4.0/Eigen \
	-I./venv/lib/python3.11/site-packages/pybind11/include \
	-I/opt/homebrew/Frameworks/Python.framework/Versions/3.11/include/python3.11 \
	-L/opt/homebrew/Frameworks/Python.framework/Versions/3.11/lib \
	-lpython3.11 \
	bind.cc -o example 

compile3d2:
	g++ -O3 -Wall -shared -std=c++11 -fPIC \
	$(python3-config --includes) \
	$(python3-config --ldflags) \
	-I./eigen-3.4.0 -I./eigen-3.4.0/Eigen \
	-I./venv/lib/python3.11/site-packages/pybind11/include \
	-I/opt/homebrew/Frameworks/Python.framework/Versions/3.11/include/python3.11 \
	-L/opt/homebrew/Frameworks/Python.framework/Versions/3.11/lib \
	-lpython3.11 \
	bind.cc -o kdtree_3torus.so


simple:
	g++ -O3 -Wall -shared -std=c++11 -fPIC \
	-I./venv/lib/python3.11/site-packages/pybind11/include \
	-I./venv/include/python3.11 \
	-I./eigen-3.4.0 -I./eigen-3.4.0/Eigen \
	simple.cc -o example \
	-L./venv/lib \
	-lpython3.11

simple2:
	c++ -O3 -Wall -std=c++11 -fPIC \
	$(python -m pybind11 --includes) \
	-I./eigen-3.4.0 -I./eigen-3.4.0/Eigen \
	-I./venv/lib/python3.11/site-packages/pybind11/include \
	-I/opt/homebrew/Frameworks/Python.framework/Versions/3.11/include/python3.11 \
	-L/opt/homebrew/Frameworks/Python.framework/Versions/3.11/lib \
	-lpython3.11 \
	simple.cc -o example 
