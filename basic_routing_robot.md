# Руководство по созданию ROS проекта с роботом для следования по маршруту

## 1. Создание рабочего пространства ROS

```bash
# Создаем рабочее пространство
mkdir -p ~/line_follower_ws/src
cd ~/line_follower_ws/src

# Инициализируем рабочее пространство
catkin_init_workspace

# Создаем пакет для нашего проекта
catkin_create_pkg line_follower rospy geometry_msgs sensor_msgs nav_msgs std_msgs
```

## 2. Структура проекта

```
line_follower_ws/src/line_follower/
├── CMakeLists.txt
├── package.xml
├── scripts/
│   ├── line_follower.py
│   ├── path_planner.py
│   └── map_loader.py
├── launch/
│   └── line_follower.launch
├── maps/
│   ├── line_map.yaml
│   └── line_map.pgm
└── config/
    └── params.yaml
```

## 3. Конфигурационные файлы

### package.xml
```xml
<?xml version="1.0"?>
<package format="2">
  <name>line_follower</name>
  <version>0.0.0</version>
  <description>Line follower robot with path planning</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depends>catkin</buildtool_depends>
  <build_depend>rospy</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  
  <exec_depend>rospy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

### maps/line_map.yaml
```yaml
image: line_map.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

## 4. Основные скрипты

### scripts/map_loader.py
```python
#!/usr/bin/env python3

import rospy
import yaml
import numpy as np
from PIL import Image

class MapLoader:
    def __init__(self):
        self.map_data = None
        self.map_info = None
        self.graph = None
        
    def load_map(self, map_yaml_path):
        """Загрузка карты из YAML файла"""
        try:
            with open(map_yaml_path, 'r') as file:
                self.map_info = yaml.safe_load(file)
            
            # Загрузка изображения карты
            map_image_path = map_yaml_path.replace('.yaml', '.pgm')
            image = Image.open(map_image_path)
            self.map_data = np.array(image)
            
            rospy.loginfo("Карта успешно загружена")
            self._build_graph_from_map()
            return True
            
        except Exception as e:
            rospy.logerr(f"Ошибка загрузки карты: {e}")
            return False
    
    def _build_graph_from_map(self):
        """Построение графа из карты линий"""
        height, width = self.map_data.shape
        self.graph = {}
        
        # Пороговое значение для определения линии
        line_threshold = 128
        
        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] < line_threshold:  # Линия (темные пиксели)
                    node_id = f"{x}_{y}"
                    neighbors = []
                    
                    # Проверяем соседние пиксели
                    for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                        nx, ny = x + dx, y + dy
                        if (0 <= nx < width and 0 <= ny < height and 
                            self.map_data[ny, nx] < line_threshold):
                            neighbors.append(f"{nx}_{ny}")
                    
                    self.graph[node_id] = neighbors
        
        rospy.loginfo(f"Граф построен: {len(self.graph)} узлов")
    
    def get_graph(self):
        return self.graph
    
    def get_map_info(self):
        return self.map_info
    
    def pixel_to_world(self, pixel_x, pixel_y):
        """Преобразование координат пикселя в мировые координаты"""
        if not self.map_info:
            return pixel_x, pixel_y
            
        resolution = self.map_info['resolution']
        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]
        
        world_x = origin_x + (pixel_x * resolution)
        world_y = origin_y + ((self.map_data.shape[0] - pixel_y) * resolution)
        
        return world_x, world_y
```

### scripts/path_planner.py
```python
#!/usr/bin/env python3

import rospy
import heapq
from collections import deque

class PathPlanner:
    def __init__(self, graph):
        self.graph = graph
    
    def dijkstra(self, start, goal):
        """Алгоритм Дейкстры для поиска кратчайшего пути"""
        if start not in self.graph or goal not in self.graph:
            rospy.logwarn("Стартовая или целевая точка не найдена в графе")
            return None
        
        # Инициализация
        distances = {node: float('inf') for node in self.graph}
        distances[start] = 0
        previous = {node: None for node in self.graph}
        queue = [(0, start)]
        
        while queue:
            current_distance, current_node = heapq.heappop(queue)
            
            if current_node == goal:
                break
                
            if current_distance > distances[current_node]:
                continue
                
            for neighbor in self.graph[current_node]:
                distance = current_distance + 1  # Все ребра имеют вес 1
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(queue, (distance, neighbor))
        
        # Восстановление пути
        path = []
        current = goal
        
        while current is not None:
            path.append(current)
            current = previous[current]
        
        path.reverse()
        
        if path[0] == start:
            rospy.loginfo(f"Путь найден: {len(path)} точек")
            return path
        else:
            rospy.logwarn("Путь не найден")
            return None
    
    def find_nearest_node(self, x, y):
        """Поиск ближайшего узла графа к заданным координатам"""
        nearest_node = None
        min_distance = float('inf')
        
        target_coords = (x, y)
        
        for node in self.graph:
            node_x, node_y = map(int, node.split('_'))
            distance = abs(node_x - x) + abs(node_y - y)  # Манхэттенское расстояние
            
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        return nearest_node
```

### scripts/line_follower.py
```python
#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        
        # Параметры
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        
        # Подписки и публикации
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Текущая позиция робота
        self.current_pose = None
        self.current_path = None
        self.current_waypoint_index = 0
        
        # Загрузка карты и планировщика
        map_loader = MapLoader()
        map_path = rospy.get_param('~map_path', 'maps/line_map.yaml')
        
        if map_loader.load_map(map_path):
            self.graph = map_loader.get_graph()
            self.map_loader = map_loader
            self.path_planner = PathPlanner(self.graph)
        else:
            rospy.logerr("Не удалось загрузить карту")
            return
        
        # Параметры управления
        self.linear_speed = 0.1
        self.angular_speed = 0.3
        self.waypoint_tolerance = 0.1  # метры
        
    def odom_callback(self, msg):
        """Обработка одометрии"""
        self.current_pose = msg.pose.pose
        
    def pixel_to_world(self, pixel_x, pixel_y):
        """Преобразование координат пикселя в мировые координаты"""
        return self.map_loader.pixel_to_world(pixel_x, pixel_y)
    
    def set_goal(self, goal_x, goal_y):
        """Установка целевой точки"""
        # Преобразуем мировые координаты в пиксели карты
        # (упрощенная реализация - в реальном проекте нужна обратная трансформация)
        pixel_goal_x = int(goal_x / self.map_loader.map_info['resolution'])
        pixel_goal_y = int(goal_y / self.map_loader.map_info['resolution'])
        
        # Находим ближайшие узлы графа
        start_node = self.path_planner.find_nearest_node(
            int(self.current_pose.position.x / self.map_loader.map_info['resolution']),
            int(self.current_pose.position.y / self.map_loader.map_info['resolution'])
        )
        goal_node = self.path_planner.find_nearest_node(pixel_goal_x, pixel_goal_y)
        
        # Планируем путь
        self.current_path = self.path_planner.dijkstra(start_node, goal_node)
        self.current_waypoint_index = 0
        
        if self.current_path:
            rospy.loginfo(f"Путь к цели планируется через {len(self.current_path)} точек")
        else:
            rospy.logwarn("Не удалось найти путь к цели")
    
    def follow_path(self):
        """Следование по запланированному пути"""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            self.stop_robot()
            return
        
        # Получаем текущую целевую точку
        current_waypoint = self.current_path[self.current_waypoint_index]
        waypoint_x, waypoint_y = map(int, current_waypoint.split('_'))
        world_x, world_y = self.pixel_to_world(waypoint_x, waypoint_y)
        
        # Вычисляем угол к целевой точке
        dx = world_x - self.current_pose.position.x
        dy = world_y - self.current_pose.position.y
        target_angle = math.atan2(dy, dx)
        
        # Получаем текущую ориентацию робота
        current_orientation = self.current_pose.orientation
        _, _, current_yaw = self.quaternion_to_euler(
            current_orientation.x,
            current_orientation.y, 
            current_orientation.z,
            current_orientation.w
        )
        
        # Вычисляем разницу углов
        angle_error = self.normalize_angle(target_angle - current_yaw)
        
        # Управление роботом
        twist_msg = Twist()
        
        if abs(angle_error) > 0.1:  # Поворачиваем к цели
            twist_msg.angular.z = self.angular_speed * angle_error
        else:  # Двигаемся вперед
            twist_msg.linear.x = self.linear_speed
        
        self.cmd_vel_pub.publish(twist_msg)
        
        # Проверяем достижение точки
        distance_to_waypoint = math.sqrt(dx**2 + dy**2)
        if distance_to_waypoint < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            rospy.loginfo(f"Достигнута точка {self.current_waypoint_index}/{len(self.current_path)}")
    
    def quaternion_to_euler(self, x, y, z, w):
        """Преобразование кватерниона в углы Эйлера"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z
    
    def normalize_angle(self, angle):
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def stop_robot(self):
        """Остановка робота"""
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)
    
    def run(self):
        """Основной цикл"""
        # Пример: установка цели в определенные координаты
        self.set_goal(2.0, 2.0)  # x=2m, y=2m
        
        while not rospy.is_shutdown():
            if self.current_pose and self.current_path:
                self.follow_path()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = LineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
```

## 5. Launch файл

### launch/line_follower.launch
```xml
<?xml version="1.0"?>
<launch>
    <node name="line_follower" pkg="line_follower" type="line_follower.py" output="screen">
        <param name="map_path" value="$(find line_follower)/maps/line_map.yaml"/>
    </node>
</launch>
```

## 6. Сборка и запуск

```bash
# Переходим в корень рабочего пространства
cd ~/line_follower_ws

# Собираем проект
catkin_make

# Добавляем рабочее пространство в environment
source devel/setup.bash

# Делаем скрипты исполняемыми
chmod +x src/line_follower/scripts/*.py

# Запускаем проект
roslaunch line_follower line_follower.launch
```

## 7. Создание тестовой карты

Для создания тестовой карты линий можно использовать любой графический редактор:
- Создайте черно-белое изображение (PGM формат)
- Черные пиксели - линии для следования
- Белые пиксели - свободное пространство
- Сохраните как `line_map.pgm` и создайте соответствующий YAML файл

## Особенности реализации:

1. **Графовая модель**: Карта линий преобразуется в граф, где узлы - точки на линии, ребра - соединения между соседними точками

2. **Алгоритм Дейкстры**: Используется для поиска кратчайшего пути на графе с единичными весами

3. **Преобразование координат**: Перевод между пикселями карты и мировыми координатами

4. **Управление движением**: PD-регулятор для следования по точкам маршрута

Этот проект предоставляет основу для создания робота, способного следовать по заранее известной разметке с использованием алгоритмов поиска кратчайшего пути.