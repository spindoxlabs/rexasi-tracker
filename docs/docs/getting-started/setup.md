---
sidebar_position: 1
---

# Setup

## Requirements

- [Install Task](https://taskfile.dev/installation/#npm) (a task runner)
- [Configure Task autocompletion](https://taskfile.dev/installation/#setup-completions)

## Taskfile
The commands needed to launch the people tracker are in the ```taskfile.dist.yml```.
It is possibile to override it by creating a ```taskfile.yml```, for local environment setup.


## Setup

It will build the docker image and download the models

```bash
task setup
```