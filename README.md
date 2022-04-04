# pybound_mathlib
Bind [PX4](https://github.com/PX4/PX4-Autopilot) mathlib classes to python for class level testing and visualization.

## What's here?
Top-level directories:

`px4_platform_common/` contains copies of headers with some basic PX4 defines. Current upstream implementations found [here](https://github.com/PX4/PX4-Autopilot/tree/master/platforms/common/include/px4_platform_common/).

`filter/` contains copies of the `mathlib` filter headers. Current upstream implementations found [here](https://github.com/PX4/PX4-Autopilot/tree/master/src/lib/mathlib/math/filter).

`matrix/` contains copies of the `mathlib` matrix headers. Current upstream implementations found [here](https://github.com/PX4/PX4-Autopilot/tree/master/src/lib/mathlib/math/matrix/matrix).

`pybindings/` contains the python bindings for the C++ classes in the `filter/` directory. 

`tests/` contains python based tests and/or visualizations for the various python-bound classes.

## Prerequisites
C++11 support

Python 3.8

virtualenv 20.13.0

## Setup
Clone the repo. Note pybind11 is included as submodule.
```
git clone --recursive git@github.com:tstastny/pybound_mathlib.git
```
Create a virtual environment in the root directory of the cloned repo and activate it.
```
python3 -m venv venv
source venv/bin/activate
```
Install the requirements.
```
pip3 install -r requirements.txt 
```
 
## Build
```
mkdir build
cd build
cmake ..
make pybound_mathlib
cd ..
```

## Run the tests
e.g.
```
cd tests/
python3 second_order_reference_model_discretization_method_test.py
```