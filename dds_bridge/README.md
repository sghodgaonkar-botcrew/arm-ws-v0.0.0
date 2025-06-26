# CycloneDDS Communication

## CycloneDDS Installation

First we need to install the dependencies
```
sudo apt install libiceoryx-binding-c-dev libiceoryx-posh-dev
```
Now we need to clone and build the Core CycloneDDS
```
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
git checkout releases/0.10.x
rm -rf build
mkdir build && cd build
```
Now since we want CycloneDDS system-wide it is suggested to installed a user defined directory doing is safer as it does not ask for root privileges.
```
export CYCLONE_INSTALL=$HOME/dds-install         # core C library prefix
cmake -DCMAKE_INSTALL_PREFIX=$CYCLONE_INSTALL \
      -DBUILD_IDLC=ON \
      -DBUILD_EXAMPLES=ON \
      -DBUILD_DDSPERF=ON .. 

make -j$(nproc)
make install
```
Now we need to source the Path to our CycloneDDS so that it does use the one from ROS2
```
export PATH=$CYCLONE_INSTALL/bin:$PATH
export LD_LIBRARY_PATH=$CYCLONE_INSTALL/lib:$LD_LIBRARY_PATH
```
Check using
```
which idlc
```
Test installation as below(Requires the flag -DBUILD_EXAMPLES=ON )
```
cd ~/cyclonedds/build/bin
# In first terminal
./HelloworldPublisher

# In second terminal
./HellowworldSubscriber
```

Next we need to build the C++ wrapper for the CycloneDDS
```
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
cd cyclonedds-cxx
git checkout releases/0.10.x
mkdir build && cd build
```
Again we need the wrapper to be system wide but without messing with the root
```
export CXX_INSTALL=$HOME/dds-cxx-install         # C++ wrapper prefix
cmake -DCMAKE_INSTALL_PREFIX=$CXX_INSTALL \
      -DCMAKE_PREFIX_PATH="$CYCLONE_INSTALL" \
      -DBUILD_EXAMPLES=ON ..

make -j$(nproc)
make install
```

Set the library path so that correct idlc can be run
```
export LD_LIBRARY_PATH=$CXX_INSTALL/lib:$CYCLONE_INSTALL/lib:$LD_LIBRARY_PATH
```

Lets test if the wrapper is installed correctly
```
cd ~/cyclonedds-cxx/build/bin
# In first terminal
./ddscxxHelloworldPublisher

# In second terminal
./ddscxxHelloworldSubscriber
```
## Building Workspace
Now we create the header files from the idl message. Make sure you have exported the library path. For each ros package you need to build the header files from idl message
```
cd *_ros_pluggins
mkdir generated && cd generated
idlc -l cxx ../idl/<idl_file_name>
```
Here `-l cxx` tells the compiler will give cpp files for the headers and source files for the message. This will create the `.cpp` and `.hpp` files for the idl message in the current directory. 

Now we build the package after making sure that the paths to the core and c++ wrapper are set.

## Building the ROS packages after making changed
We need to export all the paths for CycloneDDS to find the libraries
```
# Assuming you followed the install paths in README
export CYCLONE_INSTALL=$HOME/dds-install
export CXX_INSTALL=$HOME/dds-cxx-install

# Add the paths to environment variables
export PATH=$CYCLONE_INSTALL/bin:$PATH
export LD_LIBRARY_PATH=$CXX_INSTALL/lib:$CYCLONE_INSTALL/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=$CXX_INSTALL:$CYCLONE_INSTALL:$CMAKE_PREFIX_PATH

```
Now `colcon build` works perfectly fine

## Building the CPP pluggins
Make sure you export all the paths for CycloneDDS as above.
We have multiple cpp packages and each needs to be built separately. Navigate to each *_cpp_pluggins and then create the build directory to build the executables

```
mkdir build && cd build
mkdir generated && cd generated
idlc -l cxx ../../idl/<idl_file_name>
cd ..
cmake .. -DCMAKE_INSTALL_PREFIX="$CYCLONE_INSTALL;$CXX_INSTALL"
make -j$(nproc)
```
Now all the executables are in the build folder inside the package.
