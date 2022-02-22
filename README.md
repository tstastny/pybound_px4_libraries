# pybound_mathlib
Bind [PX4](https://github.com/PX4/PX4-Autopilot) mathlib classes to python for class level testing and visualization.

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
make pybindings
cd ..
```

## Run the tests
```
python3 tests/second_order_reference_model_test.py
```

