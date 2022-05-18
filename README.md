# 2022-group-04

## Name
DIT638 - Cyber Physical Systems and Systems of Systems
Group 4

## Introduction

This repository contains Group 4's the source files to a microservice used to generate steering wheel angle based on video feedback for DIT638 Cyber Physical Systems and Systems of Systems at the University of Gothenburg.

## License
[MIT](https://git.chalmers.se/courses/dit638/students/2022-group-04/-/blob/main/LICENSE)

## Technologies used 

- [OpenDLV](https://github.com/chalmers-revere/opendlv)
    - [Vehicle View](https://github.com/chalmers-revere/opendlv-vehicle-view)
    - [H264 Decoder](https://github.com/chalmers-revere/opendlv-video-h264-decoder)
- [OpenCV](https://opencv.org/)
- [Docker](https://www.docker.com/)
- [CMake](https://cmake.org/)
- [GCC](https://gcc.gnu.org/) 

## Prerequisites

- Linux environment (Ubuntu 20:04 is recommended)
- [Git](https://git-scm.com/downloads)
- [GitLab SSH Key set up](https://git.chalmers.se/-/profile/keys)
- [CMake](https://cmake.org/download/)
- [Docker](https://docs.docker.com/compose/install/)
- [Docker Compose](https://docs.docker.com/compose/install/)

NOTE: Click the respective prerequisite to see installation steps

## Getting started

1. To get started, create a folder and go to it 
```
mkdir group_4
cd group_4
```

2. Clone the repository:

```
git clone git@git.chalmers.se:courses/dit638/students/2022-group-04.git
```
3. Go to the source files: 
```
cd 2022-group-04/src
```
4. From there, run OpenDLV Vehicle View: 
```
docker run --rm --init --net=host --name=opendlv-vehicle-view -v $PWD:/opt/vehicle-view/recordings -v /var/run/docker.sock:/var/run/docker.sock -p 8081:8081 chalmersrevere/opendlv-vehicle-view-multi:v0.0.60
```
You can then go to http://localhost:8081/ to select and run any of the available videoclips

5. In another terminal, build the OpenDLV H264 decoder: 
```
docker build https://github.com/chalmers-revere/opendlv-video-h264-decoder.git#v0.0.4 -f Dockerfile.amd64 -t h264decoder:v0.0.4
```

6. Then run the following commands:
```
xhost + 
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp h264decoder:v0.0.4 --cid=253 --name=img
```

7. Via a third terminal, to start our microservice go to the 2022-group-04 folder and run the following commands:
```
docker build -f Dockerfile -t my-opencv-example .
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp my-opencv-example:latest --cid=253 --name=img --width=640 --height=480 --verbose 
```

**You should now be able to see the generated steering angle for a given timestamp whenever you run a video through OpenDLV Vehicle View.**

## New features process 
* Create an issue that addresses a requirement and assign it to one of the engineers.
* Implement the solution on our local machines.
* Create a new branch with an appropriate and descriptive name and description.
* Create a merge request with the associated issue and await a review.
* After the review process is complete and the merge request gets approved, we merge. 

## Bug fixing process 
* Create an issue with a bug tag and assign it to a developer. 
* When the bug has been fixed, we create a new branch and create a merge request. 
* After the review process is complete and the merge request gets approved, we merge. 
* Once an issue is found by a developer in a committed branch, the bug tag is selected to be seen by the group. 

## Commit message structure 
* The contents of the title shall contain a descriptive and short summary of the main changes using the imperative form (ie. Add counter to the home page). The title should not be longer than 50 characters. 
* The body of said comment shall contain a simple but descriptive technical description of modified code such that the changes can be easily understood. It should also include a description of any smaller changes or modifications. 

## Authors and acknowledgment
* [Anis Bourbia](https://git.chalmers.se/bourbia)
* [Erdem Halil](https://git.chalmers.se/erdemh)
* [Vlad Liteanu](https://git.chalmers.se/liteanu)
* [Gregory William Geraldi Sastrawidjaya](https://git.chalmers.se/geraldi)
* [Taofik Arnouk](https://git.chalmers.se/arnouk)
