# 3torus-kdtree

### Install
git submodule update --init
python3.11 -m venv venv
pip install pybind11


The general algorithm:
- we use the periodic boundary distance metric to determine if we're going to recurse into the child branches
- Distance between two points in the kd tree is calculated like so:
  - take the cartesian distance. And make sure it's always within [-0.5, 0.5] fractional coord
  - I can do this without the inverse matrix. the solution is to just subtract or add the corresponding lattice vector