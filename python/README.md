# This folder contains manual bindings for aikidopy using pybind11.

To install, first catkin build aikido.
```
catkin build aikido
source ../../devel.setup.bash
```

Build and install the python binding
```
mkdir build
cd build
cmake ..
make -j
sudo make install
```

Try loading.
```
python3
>>> import aikidopy
```

