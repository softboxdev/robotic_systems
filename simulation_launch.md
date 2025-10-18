# Руководство по запуску проекта в графической оболочке ROS

## 1. Установка необходимых инструментов

```bash
# Устанавливаем инструменты для визуализации
sudo apt-get update
sudo apt-get install ros-noetic-rviz ros-noetic-gazebo-ros ros-noetic-turtlebot3-simulations
```

## 2. Модификация проекта для визуализации

### scripts/line_follower_visual.py
```python
#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowerVisual:
    def __init__(self):
        rospy.init_node('line_follower_visual', anonymous=True)
        
        # Параметры
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        
        # Подписки и публикации
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.waypoints_pub = rospy.Publisher('/waypoints_markers', MarkerArray, queue_size=10)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Текущая позиция робота
        self.current_pose = None
        self.current_path = None
        self.current_waypoint_index = 0
        
        # Визуализация
        self.path_visual = Path()
        self.path_visual.header.frame_id = "odom"
        
    def publish_path_visualization(self):
        """Публикация пути для визуализации в Rviz"""
        if not self.current_path:
            return
            
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"
        
        for node in self.current_path:
            waypoint_x, waypoint_y = map(int, node.split('_'))
            world_x, world_y = self.pixel_to_world(waypoint_x, waypoint_y)
            
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_waypoints_markers(self):
        """Публикация маркеров для точек пути"""
        if not self.current_path:
            return
            
        marker_array = MarkerArray()
        
        for i, node in enumerate(self.current_path):
            waypoint_x, waypoint_y = map(int, node.split('_'))
            world_x, world_y = self.pixel_to_world(waypoint_x, waypoint_y)
            
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.waypoints_pub.publish(marker_array)
    
    def publish_robot_marker(self):
        """Публикация маркера текущей позиции робота"""
        if not self.current_pose:
            return
            
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose = self.current_pose
        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)
```

## 3. Launch файл с визуализацией

### launch/line_follower_visual.launch
```xml
<?xml version="1.0"?>
<launch>
    <!-- Запуск основного узла следования по линии -->
    <node name="line_follower" pkg="line_follower" type="line_follower_visual.py" output="screen">
        <param name="map_path" value="$(find line_follower)/maps/line_map.yaml"/>
    </node>

    <!-- Запуск Rviz с конфигурацией -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find line_follower)/config/line_follower.rviz"/>

    <!-- Запуск симуляции TurtleBot3 (опционально) -->
    <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_empty_world.launch"/>
    
    <!-- Публикация статического преобразования координат -->
    <node pkg="tf" type="static_transform_publisher" name="map_to_odom" args="0 0 0 0 0 0 map odom 100"/>
</launch>
```

## 4. Конфигурационный файл Rviz

### config/line_follower.rviz
```yaml
Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1
    Tree Height: 556
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /2D Nav Goal1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: odom
      Value: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Class: rviz/MarkerArray
      Enabled: true
      Name: Waypoints
      Namespaces:
        waypoints: true
      Topic: /waypoints_markers
      Value: true
    - Class: rviz/Marker
      Enabled: true
      Name: Robot Marker
      Namespaces:
        robot: true
      Topic: /visualization_marker
      Value: true
    - Class: rviz/Path
      Alpha: 1
      Color: 0; 255; 0
      Enabled: true
      Name: Planned Path
      Pose Style: Lines
      Buffer Length: 1
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Axes: Ray
      Pose Axes Length: 0.30000001192092896
      Pose Axes Radius: 0.029999999329447746
      Shaft Length: 0.10000000149011612
      Shaft Radius: 0.019999999552965164
      Head Length: 0.05999999865889549
      Head Radius: 0.03999999910593033
      Topic: /planned_path
      Unreliable: false
      Value: true
      Velocity Text:
        Color: 0; 0; 0
        Enabled: false
        Font Size: 12
        Horizontal Alignment: left
        Show Velocity: false
        Vertical Alignment: bottom
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/2D Nav Goal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0.35537123680114746
        Y: 0.037075042724609375
        Z: 0.019999999552965164
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1024
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500670065007400730100000041000000e60000000000000000fb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1280
  X: 100
  Y: 100
```

## 5. Запуск с различными симуляторами

### Для TurtleBot3:
```bash
# Установка TurtleBot3
sudo apt-get install ros-noetic-turtlebot3*

# Запуск с TurtleBot3 в Gazebo
export TURTLEBOT3_MODEL=waffle
roslaunch line_follower line_follower_turtlebot.launch
```

### launch/line_follower_turtlebot.launch
```xml
<?xml version="1.0"?>
<launch>
    <!-- Параметр модели TurtleBot3 -->
    <arg name="model" default="waffle" doc="model type [burger, waffle, waffle_pi]"/>
    
    <!-- Запуск Gazebo -->
    <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_empty_world.launch">
        <arg name="model" value="$(arg model)"/>
    </include>

    <!-- Запуск узла следования по линии -->
    <node name="line_follower" pkg="line_follower" type="line_follower_visual.py" output="screen">
        <param name="map_path" value="$(find line_follower)/maps/line_map.yaml"/>
    </node>

    <!-- Запуск Rviz -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find line_follower)/config/turtlebot3_line_follower.rviz"/>
</launch>
```

## 6. Интерактивный запуск с ручным заданием целей

### scripts/interactive_line_follower.py
```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import tf

class InteractiveLineFollower(LineFollowerVisual):
    def __init__(self):
        super().__init__()
        
        # Подписка на клики в Rviz
        self.click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.click_callback)
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("Готов к получению целей через Rviz (Tool: Publish Point)")
    
    def click_callback(self, msg):
        """Обработка клика в Rviz для установки новой цели"""
        try:
            # Преобразуем координаты в систему odom
            point_odom = self.tf_listener.transformPoint('odom', msg)
            
            goal_x = point_odom.point.x
            goal_y = point_odom.point.y
            
            rospy.loginfo(f"Получена новая цель: ({goal_x:.2f}, {goal_y:.2f})")
            
            # Устанавливаем новую цель
            self.set_goal(goal_x, goal_y)
            
        except Exception as e:
            rospy.logerr(f"Ошибка преобразования координат: {e}")
```

## 7. Полный процесс запуска

```bash
# 1. Открываем первый терминал
cd ~/line_follower_ws
source devel/setup.bash

# 2. Запускаем основной launch файл
roslaunch line_follower line_follower_visual.launch

# 3. Открываем второй терминал для мониторинга
rosrun rqt_graph rqt_graph

# 4. Открываем третий терминал для управления
rosrun rqt_console rqt_console

# 5. Для ручного управления (опционально)
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## 8. Просмотр топиков и отладка

```bash
# Просмотр всех активных топиков
rostopic list

# Просмотр пути
rostopic echo /planned_path

# Просмотр маркеров
rostopic echo /visualization_marker

# Просмотр позиции робота
rostopic echo /odom

# Граф вычислений
rqt_graph
```

## 9. Создание демонстрационной карты

Для тестирования создайте простую карту с линиями:

```bash
# Создаем простую PGM карту
convert -size 100x100 xc:white -fill black -draw "rectangle 10,10 90,90" -draw "line 10,50 90,50" -draw "line 50,10 50,90" test_map.pgm

# Создаем YAML файл
echo "image: test_map.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0" > test_map.yaml
```

## Особенности графического запуска:

1. **Rviz визуализация**: Путь, точки маршрута и позиция робота отображаются в реальном времени
2. **Интерактивное управление**: Возможность задавать цели кликом мыши в Rviz
3. **Мониторинг**: Использование rqt инструментов для отладки
4. **Совместимость**: Работа с различными симуляторами (Gazebo, TurtleBot3)

Такой подход позволяет наглядно видеть работу алгоритма и легко отлаживать систему.