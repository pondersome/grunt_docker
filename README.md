# grunt_docker
 Containers to support distributed grunt operations


## Docker Iffy

Docker has issues with ROS2 both under development and deployment scenarios. It's good to keep some of these issues in mind.

[ROS Docker; 6 reasons why they are not a good fit](https://ubuntu.com/blog/ros-docker) contains some very good points and I largely agree with them. But Docker is also extremely convenient as a way to get a common set of dependencies available in a dependable way. So while I don't use docker on robots directly where unfettered hardware i/o is desired, it is helpful for setting up distributed environments like simulations and remote operations stations where network communication is the only "hardware" needed.

Guidance for this configuration was taken from [this guide](https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/) - with emphasis on the dev containers section, [this related example](https://github.com/sea-bass/turtlebot3_behavior_demos/tree/main/docker) and [this repo](https://github.com/pondersome/leorover_gazebo_sim_docker)

## Layout
```
workspace/
│
├── grunt_docker/                       
│   ├── base/                     
│   │   └── Dockerfile            ← shared base (ROS 2 + core deps)
│   │
│   ├── bot/                    
│   │   └── Dockerfile            ← extends base, hardware independent robot‐specific code
│   │
│   ├── dev/                    
│   │   └── Dockerfile            ← extends osrf/ros:jazzy-desktop-full, adding development tools and user volumes
│   │
│   ├── ops/           
│   │   └── Dockerfile            ← extends dev with operator scripts/tools
│   │
│   ├── sim/        
│   │   └── Dockerfile            ← extends dev, adding simulation tools
│   │
│   ├── telemetry-aggregator/     
│   │   └── Dockerfile            ← extends base, databases/collectors
│   │
│   ├── bashrc_custom             ← .bashrc tweaks (copied by each image)
│   ├── dependencies.repos.yml    ← common source dependencies for base
│   ├── docker-compose.yml        ← orchestrates all profiles
```

The bot folder/image is tbd and might be developed later, but as right now I'm running a standard os install on the robot. Also for the time being a telemetry aggregator is not needed.

The layout here shows that the dev layer is used as the foundation for operational overlays which is probably not optimal, but for now the amount of experimentation is relatively high and I want to keep development capabilities in any image I might use. Specifically, I want the required repos to be built in a persistent mount that survives container restarts.

## WSL

The docker images are currently built with Docker Desktop on WSL2 and are meant to leverage WSLg instead of X11, though I may add X11 variants for pure Ubuntu hosts. With the exception of the robot, I'm mostly running these on Windows 11 machines. Networking may be more permissive in Windows containers.

## ZeroTier VPN

I'm using ZeroTier as my VPN for the robot network. On WSL2 with host networking enabled, this means that the VPN needs to be installed on the host machine, and then the participating containers get automatic access through the WSL VM. So there is no docker setup for this network.