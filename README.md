# ОТКРЫТИЕ ДВЕРИ МАНИПУЛЯТОРОМ 

## Требования
1. rospy
2. ur_rtde lib [here](https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html) ```pip3 install ur_rtde```

## Установка
1. Создать catkin_ws
```
mkdir catkin_ws
cd ./catkin_ws
mkdir src
catkin_make
```
2. Склонировать реп
```
cd ./src
git clone https://github.com/danissomo/door_open.git
```
3. Cборка
```
catkin_make
source ./devel/setup.bash
```

## Запуск
``` 
rosrun door_open door_open_node.py
```

## Заметка
roslaunch файл пока не работает, все настройки находятся в utils.py