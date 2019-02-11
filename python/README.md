# This folder contains manual bindings for aikidopy using pybind11.

0. The installation guide have been tested on

- sudo apt installed python 2.7 and python 3.4.
- anaconda managed python 3.6

If you use system installed python, ensure you have `python2.7-dev` or `python3-dev` correspondingly, or run `sudo apt-get install python3-dev` to obtain it.


1. Install [pybind11](https://github.com/pybind/pybind11.git) **from source** following this [instruction](https://pybind11.readthedocs.io/en/master/basics.html#compiling-the-test-cases). (Need version >= 2.2.0).

```
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build
cd build
cmake ..
make -j 4
```

- For system managed python: `sudo make install`;
- For anaconda: `pip install -e .`.

You should be able to load `pybind11` in your python.

```
$ python
>> import pybind11
```

2. Build aikido and source the setup file.
```
catkin build aikido
source ../../devel.setup.bash
```

3. Create a folder `workspace/src/aikido/python/build` (instead of `workspace/src/aikido/build`). Build and install the python binding.

```
cd build
cmake .. -DAIKIDOPY_PYTHON_VERSION=3.4
make -j
sudo make install
```

Read the output of `sudo make install` and ensure that `aikidopy.so` get installed in desired location.

4. Try loading.
```
python
>>> import aikidopy
```

5. Tips for multiple python versions
- When you have multiple python environment, ensure that `which python` and the python version you passed to cmake refers to the same python.
