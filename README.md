# Next Best View Computation 
### Using Occlussion Aware Voumetric Information
## Description

This algorithm reads a PLC file and prints the Next Best View (NBV) coordinates based on the Occlussion Aware VI method presented in [1].
The resulting coordinates are computed in 2D, futher updates will include 3D NBVs. The camera is set up as a kinect 1.

The object and camera center can be set up in the `float cloudCentroid[3]={1,0,0};` and `octomap::point3d sensorOrigin(0,0,0);` lines, the camera direction is assumed to be towards the centroid of the object.


[1] Delmerico, J., Isler, S., Sabzevari, R. et al. Auton Robot (2018) 42: 197. https://doi.org/10.1007/s10514-017-9634-0

## Installation 
The algorithm uses **Point Cloud Library** and **Octomap Library**.

* *Point Cloud Library* can be installed by running:
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```

To run * *Octomap* it needs to be downloaded or cloned with:
```
git clone git://github.com/OctoMap/octomap.git
```
Build the complete project with cmake:
```
mkdir build
cd build
cmake ..
```
Afterwards it can be installed system-wide:
```
make install
```

