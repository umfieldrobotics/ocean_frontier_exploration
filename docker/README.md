# Entended from the below template. this container builds isaacsim, installs oceansim, moves it to your bind directory for persistance, and installs ros2. Access the sim binaries via YOUR_WORK_DIR/built_isaacsim. Follow the below steps for inital workdir setup.


# Docker Template
This template is taken from [Seth's useful snippets](https://github.com/sethgi/useful-snippets). Give it a star!

## 1. Prerequisites
- **Docker** with the *nvidia‑container‑toolkit* installed  
- X server running on the host (probably not strictly required)  

## 2. One‑time setup
```bash
chmod +x build.sh run.sh entrypoint.sh
````

## 3. Minimal edits for a new task
These are the things you need to add to adapt this template. Each file listed contains some placeholders you must fill in.

| File                     | Placeholder        | Set to …                                                      |
| ------------------------ | ------------------ | ------------------------------------------------------------- |
| **build.sh**             | `<insert_tag>`     | Image tag, e.g. `myproj`                                      |
| **container.Dockerfile** | `<an_image>`       | Base image (e.g. `nvidia/cuda:12.4.1-devel-ubuntu22.04`)      |
|                          | `<whatever>`       | Linux user name inside the container (usually your host user) |
| **run.sh**               | `<image_tag>`      | Same value used in **build.sh**                               |
|                          | `<container_name>` | Unique container name, e.g. `myproj_dev`                      |
|                          | `<whatever>`       | Absolute path on host to mount as `/home/<user>/data`         |

### Optional tweaks

* Add whatever you need for your task in the dockerfile.
* Mount extra volumes by appending `-v host_path:container_path` lines in `run.sh`.
* Modify GPU, networking, or X11 options in `run.sh` to match your environment.

## 4. Build the image

```bash
./build.sh
```

Creates `<insert_tag>:latest`.

## 5. Run / resume / attach
The run script has some bells and whistles for managing containers.

```bash
./run.sh               # first time (creates container)
./run.sh               # later (attaches to running container)
./run.sh restart       # force‑restart
```

### Inside the container

The script drops you at `/home/<user>` with the project repo mounted, full sudo, and all NVIDIA driver capabilities.

## 6. Updating the image

After changing the Dockerfile:

```bash
./build.sh                # rebuild
./run.sh restart          # restart container with new image
```

## 7. Removing everything

```bash
docker rm -f <container_name>
docker rmi <image_tag>:latest
```

```
```
