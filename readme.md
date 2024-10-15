# CoHAN ATTR (attribute)

## The package consists of initial developement towards creating explainable features from human aware motion planning frameworks

## Installation
```
$ 
$ git clone https://github.com/hariharan20/cohan_attr
$ cd cohan_attr
$ cd custom_dockers
$ ./build_docker.sh #For first building of docker image
$ ./run_docker.sh 


```
## Execution
Once inside the docker image : (default working directory : catkin_ws)
```
$ catkin build
$ rosrun cohan_attr attr.py
```
For now, the script prints out the statements (TO BE converted to ROS messages

)