# Docker-miljö

ROS 2 Humble + Pinocchio. Repot monteras som `/sot` i containern, så ändringar i Windows syns direkt.

## Starta (Git Bash)
```bash
cd /d/desktop/school/Robotics/project/git/Robotic-Stack-of-Tasks-Control/docker
bash build.sh   # bara första gången
bash run.sh
```

En till terminal i samma container: `bash attach.sh`

## Panda-modellen
`models/panda.urdf` är genererad från kursens `franka_gazebo`-xacro (7 leder, `panda_link0`–`panda_link7`).
Ladda den i C++ med `pinocchio::urdf::buildModel("/sot/models/panda.urdf", model);`
