# Setup

- workspace
    - run `wstool update` using .rosinstall in this repository

- choreonoid
```
sudo choreonoid/misc/script/install-requisites-ubuntu-18.04.sh
```

- hairo plugin
```
cd choreonoid/ext/
git clone git@github.com:kirohy/hairo-world-plugin.git -b melodic
sudo hairo-world-plugin/misc/script/install-requisites-ubuntu-18.04.sh
```

- build
```
catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DUSE_PYTHON3=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_HAIRO_WORLD_PLUGIN=ON
catkin build
```
