---
sidebar_position: 2
---

# Getting started

## Requirements

- [Install Task](https://taskfile.dev/installation/#npm) (a task runner)
- [Configure Task autocompletion](https://taskfile.dev/installation/#setup-completions)

The commands needed to launch the people tracker are in the ```taskfile.dist.yml```.
It is possibile to override it by creating a ```taskfile.yml```, for local environment setup.


## Setup

To build all the docker images and download the necessary ML models, run:

```
task setup
```

## Run it
To run the rexasi-tracker alone just type:
```
task start-tracker
```

See the [example](./example) to run the tracker along with a detector (RGBD people detector) and RVIZ to visualize the output tracks.