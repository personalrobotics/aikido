# prtsr_py

This folder contains cpp files for generating `pr_tsrpy` using pybind11. All cpp files are generated using [chimera](https://github.com/personalrobotics/chimera.git).

## Installation
Install `python3-dev`
```
sudo apt-get install python3-dev
```

Install [pybind11](https://github.com/pybind/pybind11.git) from source (need version >= 2.2.0).
```
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build
cd build
cmake ..
make -j 4
sudo make install
```

Catkin build `pr_tsr` and source catkin.
```
cd ~/workspace-path/pr_tsr
catkin build pr_tsr
source ../devel/setup.bash
```

Create a separate build directory and build with python option.
```
mkdir build
cd build
cmake .. -DBUILD_PYTHON=ON
```

This by default builds for python3. You can specify the version by adding a flag `-DPR_TSRPY_PYTHON_VERSION=2.7` to cmake.

Make install.
```
make
sudo make install
```

Now you can import `pr_tsrpy`.
```
python3
>>> import pr_tsrpy
>>>canTSR = pr_tsrpy.getDefaultCanTSR()
```

You need to have `aikidopy` installed to be able to actually use the returned TSR.

