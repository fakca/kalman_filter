# kalman_filter
vanilla kalman filter implementation

## Usage
### Build
```sh
mkdir build
cd build
cmake ..
make -j$(nproc)
```
### Execute and generate the plot
```sh
./build/test_kalman
./plot.sh
```
plot.sh generates plot.png under build directory.

![plot](https://user-images.githubusercontent.com/37161227/229395093-a836332a-9a64-4272-b36d-6f2cce13523f.png)
