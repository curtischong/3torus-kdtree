run:
	g++ -std=c++17 main.cc && ./a.out
test1d:
	g++ -std=c++17 kdtree_1d.cc && ./a.out
test3d:
	g++ -std=c++17 -I./eigen-3.4.0 kdtree_3d.cc && ./a.out