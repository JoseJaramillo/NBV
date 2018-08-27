# Next Best View Computation 
### Using Occlussion Aware Voumetric Information

## Installation 
The algorithm uses **Point Cloud Library** and **Octomap Library**.

* *Point Cloud Library* * can be installed by running:
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```

To run * *Octomap* * it needs to be downloaded or cloned with:
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

