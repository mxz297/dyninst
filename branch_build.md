    
# Build Capstone
```
git clone -b access-fixes https://github.com/mxz297/capstone.git capstone
cd capstone
mkdir -p install build
cd build
cmake -DCMAKE_INSTALL_PREFIX=`pwd`/../install ..
make install -j4
cd ../..
```

# Build libunwind

```
git clone https://github.com/mxz297/libunwind.git libunwind
cd libunwind
mkdir install
./autogen.sh
./configure --prefix=`pwd`/install --enable-cxx-exceptions
make install -j4
cd ..
```

# Build asmjit
```
git clone https://github.com/asmjit/asmjit.git
cd asmjit
mkdir -p install build
cd build
cmake -DCMAKE_CXX_FLAGS="-fPIC" -DASMJIT_BUILD_X86=ON -DASMJIT_STATIC=ON -DCMAKE_INSTALL_PREFIX=`pwd`/../install ..
make install -j4
cd ../..
```
# Build Dyninst

```
git clone -b layout_opt https://github.com/mxz297/dyninst.git dyninst
cd dyninst
mkdir -p install build
cd build
cmake -DLibunwind_ROOT_DIR=`pwd`/../../libunwind/install -DCapstone_ROOT_DIR=`pwd`/../../capstone/install/ -DCMAKE_INSTALL_PREFIX=`pwd`/../install -G 'Unix Makefiles' ..
make install -j4
```

# To execute the rewritten binary

```
export LD_LIBRARY_PATH=<PATH_TO_LIBUNWIND_INSTALL>/lib:<PATH_TO_DYNINST_INSTALL>/lib:$LD_LIBRARY_PATH
export LD_PRELOAD=<PATH_TO_DYNINST_INSTALL>/lib/libdyninstAPI_RT.so:<PATH_TO_LIBUNWIND_INSTALL>/lib/libunwind.so
./rewritten
```
