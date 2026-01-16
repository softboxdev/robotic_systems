# Настройка и запуск move_base для конкретного робота в ROS

## **Общая схема работы move_base**

```
      [Целевая точка]
            |
            v
    [Global Planner (A*, Dijkstra)]
            |
            v
    [Global Costmap (карта)]
            |
            v
    [Local Planner (DWA, TEB)]
            |
            v
    [Local Costmap (карта)]
            |
            v
[Контроллер скорости → Робот]
```

---

## **Шаг 1: Базовая структура конфигурационных файлов**

Для каждого робота создаем пакет с конфигурацией:
```
~/catkin_ws/src/
└── my_robot_navigation/
    ├── launch/
    │   ├── move_base.launch       # Главный launch-файл
    │   └── ...                    # Другие launch-файлы
    ├── config/
    │   ├── costmap_common_params.yaml  # ОБЩИЕ параметры карт
    │   ├── global_costmap_params.yaml  # ГЛОБАЛЬНАЯ карта
    │   ├── local_costmap_params.yaml   # ЛОКАЛЬНАЯ карта
    │   ├── global_planner_params.yaml  # Глобальный планировщик
    │   └── local_planner_params.yaml   # Локальный планировщик (DWA/TEB)
    └── CMakeLists.txt, package.xml
```

---

## **Шаг 2: Обязательные параметры и их настройка**

### **1. `costmap_common_params.yaml` - ОБЯЗАТЕЛЬНО**

**Что это:** Общие настройки для обеих карт (глобальной и локальной)

**Почему важно:** Определяет, как робот "видит" мир и препятствия

```yaml
# costmap_common_params.yaml
obstacle_layer:
  enabled: true
  observation_sources: scan
  scan:
    data_type: LaserScan
    topic: /scan                 # Топик лидара/лазера
    marking: true                # Отмечать препятствия
    clearing: true               # Очищать свободное пространство
    expected_update_rate: 0.5    # Как часто ждем данные (сек)
    
  obstacle_range: 2.5            # Макс. расстояние для обнаружения препятствий (м)
  raytrace_range: 3.0            # Макс. расстояние для очистки пространства (м)
  
  # КРИТИЧЕСКИЙ ПАРАМЕТР:
  inflation_radius: 0.55         # Радиус "раздутия" препятствий (должен быть ≥ радиуса робота)
  
  cost_scaling_factor: 10.0      # Как быстро падает стоимость от центра препятствия
  inflation_decay_distance: 0.58 # На каком расстоянии стоимость становится минимальной

# Размеры робота (КРИТИЧЕСКИ!)
robot_base_frame: base_footprint # Фрейм робота в TF
robot_radius: 0.45               # Радиус круглого робота (м)
# ИЛИ для прямоугольного робота:
# footprint: [[-0.5, -0.3], [-0.5, 0.3], [0.5, 0.3], [0.5, -0.3]]

# Преобразования стоимости в OccupancyGrid (0-255)
map_type: costmap
# Стоимости:
# 0: свободно, 255: занято, -1: неизвестно
# 253: смертельное препятствие (inflation)
# 128-252: постепенно дешевле
```

**Обязательные проверки:**
1. `robot_radius` или `footprint` должны соответствовать реальным размерам робота
2. `inflation_radius` ≥ `robot_radius` + запас безопасности
3. Топики датчиков (`/scan`) должны совпадать с реальными

---

### **2. `global_costmap_params.yaml` - ОБЯЗАТЕЛЬНО**

**Что это:** Настройки глобальной карты (используется для глобального планирования)

**Почему важно:** Планирует путь по всей карте, игнорируя динамические препятствия

```yaml
# global_costmap_params.yaml
global_costmap:
  global_frame: map              # Система координат карты
  robot_base_frame: base_footprint
  update_frequency: 1.0          # Частота обновления (Гц) - можно реже
  publish_frequency: 0.5         # Частота публикации визуализации
  static_map: true               # Используем статическую карту (из map_server)
  rolling_window: false          # НЕ использовать скользящее окно (для глобальной карты)
  transform_tolerance: 0.5       # Допуск по времени для TF (сек)
  
  # Размеры карты
  width: 30.0                    # Ширина в метрах
  height: 30.0                   # Высота в метрах
  resolution: 0.05               # Разрешение (м/пиксель). 0.05 = 5 см
  
  # Источники данных (наследуются из common)
  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

**Ключевые параметры:**
- `static_map: true` — используем загруженную карту
- Разрешение `resolution` — компромисс между точностью и производительностью
- `transform_tolerance` — особенно важно при плохой локализации

---

### **3. `local_costmap_params.yaml` - ОБЯЗАТЕЛЬНО**

**Что это:** Настройки локальной карты (используется для локального планирования)

**Почему важно:** Для объезда динамических препятствий в реальном времени

```yaml
# local_costmap_params.yaml
local_costmap:
  global_frame: odom             # Лучше использовать odom для стабильности
  robot_base_frame: base_footprint
  update_frequency: 5.0          # Частота обновления (Гц) - часто!
  publish_frequency: 2.0
  static_map: false              # НЕ используем статическую карту
  rolling_window: true           # Скользящее окно (следует за роботом)
  rolling_window_size: 6.0       # Размер окна в метрах (должно быть > 2×robot_radius)
  transform_tolerance: 0.5
  
  # Размеры локальной карты (скользящее окно)
  width: 6.0                     # Ширина окна в метрах
  height: 6.0                    # Высота окна в метрах
  resolution: 0.05               # Должно совпадать с глобальной картой
  
  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

**Важные отличия от глобальной:**
- `rolling_window: true` — карта движется вместе с роботом
- `update_frequency: 5.0` — обновляется чаще (для динамических препятствий)
- `global_frame: odom` — лучше для локальной навигации

---

### **4. `global_planner_params.yaml` - ОБЯЗАТЕЛЬНО**

**Что это:** Настройки глобального планировщика (A*, Dijkstra)

**Почему важно:** Находит путь от текущей позиции к цели

```yaml
# global_planner_params.yaml
NavfnROS:  # Или GlobalPlanner
  # Основные параметры
  use_dijkstra: false            # true = Dijkstra, false = A*
  allow_unknown: true            # Разрешать путь через неизвестные области
  default_tolerance: 0.5         # Допуск при достижении цели (м)
  visualize_potential: false     # Визуализировать потенциалы (для отладки)
  
  # КРИТИЧЕСКИЕ параметры:
  cost_factor: 3.0               # Множитель стоимости (высокий = избегать препятствий)
  neutral_cost: 50               # Стоимость нейтральной клетки
  lethal_cost: 253               # Смертельная стоимость (совпадает с inflation)
  
  # Если робот застрял:
  planner_window_x: 0.0          # Размер окна для планирования
  planner_window_y: 0.0
  planner_window_size: 0.0       # 0 = использовать всю карту
  
  # Старый планировщик (navfn):
  old_navfn_behavior: false
```

**Выбор планировщика:**
- **A***: быстрее, находит "достаточно хороший" путь
- **Dijkstra**: гарантированно оптимальный, но медленнее
- **Theta***: находит более прямые пути

---

### **5. `local_planner_params.yaml` - САМЫЙ ВАЖНЫЙ!**

**Что это:** Настройки локального планировщика (DWA, TEB)

**Почему важно:** Отвечает за реальное движение и объезд препятствий

#### **Вариант А: DWA (Dynamic Window Approach)**
```yaml
# local_planner_params.yaml (DWA)
DWAPlannerROS:
  # Ограничения робота (ДОЛЖНЫ совпадать с реальностью!)
  max_vel_x: 0.5                 # Макс. скорость вперед/назад (м/с)
  min_vel_x: -0.1                # Мин. скорость назад (отрицательная)
  
  max_vel_theta: 1.0             # Макс. угловая скорость (рад/с)
  min_vel_theta: -1.0            # Мин. угловая скорость
  
  acc_lim_x: 0.5                 # Ускорение по X (м/с²)
  acc_lim_theta: 1.0             # Угловое ускорение (рад/с²)
  
  # Окно скоростей для планирования
  vx_samples: 6                  # Количество пробных линейных скоростей
  vtheta_samples: 20             # Количество пробных угловых скоростей
  sim_time: 1.5                  # Время симуляции траекторий (сек)
  sim_granularity: 0.025         # Шаг симуляции (сек)
  
  # Веса для оценки траекторий
  path_distance_bias: 32.0       # Вес следования глобальному пути
  goal_distance_bias: 24.0       # Вес движения к цели
  occdist_scale: 0.01            # Вес избегания препятствий
  
  # КРИТИЧЕСКИЕ параметры:
  dwa: true                      # Использовать DWA (true) или не использовать (false)
  oscillation_reset_dist: 0.05   # Расстояние для сброса флага осцилляции (м)
  
  # Настройки для достижения цели
  xy_goal_tolerance: 0.15        # Допуск по позиции (м)
  yaw_goal_tolerance: 0.15       # Допуск по ориентации (рад)
  latch_xy_goal_tolerance: false
  
  # Вращение на месте
  meter_scoring: true
  oscillation_reset_time: 2.0    # Время для сброса осцилляции (сек)
```

#### **Вариант Б: TEB (Timed Elastic Band) - для дифференциальных роботов**
```yaml
# local_planner_params.yaml (TEB)
TebLocalPlannerROS:
  # Модель робота
  odom_topic: odom
  footprint_model:
    type: "circular"
    radius: 0.45  # Совпадает с robot_radius из common_params
  
  # Ограничения
  max_vel_x: 0.5
  max_vel_x_backwards: 0.2
  max_vel_theta: 1.0
  acc_lim_x: 0.5
  acc_lim_theta: 1.0
  
  # Оптимизация
  no_inner_iterations: 5
  no_outer_iterations: 4
  
  # Веса
  weight_kinematics_forward_drive: 10
  penalty_epsilon: 0.1
```

---

## **Шаг 3: Launch-файл для запуска**

```xml
<!-- move_base.launch -->
<launch>
  <!-- Загружаем карту (если используется) -->
  <arg name="map_file" default="$(find my_robot_navigation)/maps/map.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)"/>
  
  <!-- Запускаем AMCL для локализации (если нужно) -->
  <include file="$(find my_robot_navigation)/launch/amcl.launch"/>
  
  <!-- Основной узел move_base -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    
    <!-- Загружаем все конфигурационные файлы -->
    <rosparam file="$(find my_robot_navigation)/config/costmap_common_params.yaml" 
              command="load" ns="global_costmap"/>
    <rosparam file="$(find my_robot_navigation)/config/costmap_common_params.yaml" 
              command="load" ns="local_costmap"/>
    
    <rosparam file="$(find my_robot_navigation)/config/global_costmap_params.yaml" 
              command="load"/>
    <rosparam file="$(find my_robot_navigation)/config/local_costmap_params.yaml" 
              command="load"/>
    
    <rosparam file="$(find my_robot_navigation)/config/global_planner_params.yaml" 
              command="load"/>
    <rosparam file="$(find my_robot_navigation)/config/local_planner_params.yaml" 
              command="load"/>
    
    <!-- Планировщики (можно менять) -->
    <param name="base_global_planner" value="global_planner/GlobalPlanner"/>
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS"/>
    <!-- Или для TEB: <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/> -->
    
    <!-- Настройки восстановления при застревании -->
    <param name="clearing_rotation_allowed" value="true"/>
    <param name="recovery_behavior_enabled" value="true"/>
    <param name="controller_frequency" value="5.0"/>  # Частота контроллера (Гц)
    
  </node>
  
  <!-- Визуализация в RVIZ (опционально) -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_robot_navigation)/rviz/navigation.rviz"/>
</launch>
```

---

## **Шаг 4: Процесс запуска и отладки**

### **Последовательность запуска:**
```bash
# 1. Запускаем ROS мастер
roscore

# 2. Запускаем драйверы робота (лидар, одометрия, моторы)
roslaunch my_robot_driver bringup.launch

# 3. Проверяем TF-дерево
rosrun tf view_frames
# Должно быть: map → odom → base_footprint → base_link → sensor_frames

# 4. Запускаем навигацию
roslaunch my_robot_navigation move_base.launch

# 5. Отправляем цель (в RViz или через командную строку)
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...
```

### **Типичные проблемы и их решение:**

**Проблема 1: Робот не двигается**
```
1. Проверить: rostopic echo /cmd_vel — есть ли команды?
2. Проверить: rostopic echo /scan — есть ли данные лидара?
3. Проверить TF: rosrun tf tf_echo map base_footprint
```

**Проблема 2: Робот вращается на месте**
```
1. Увеличить xy_goal_tolerance
2. Проверить параметры локального планировщика
3. Увеличить weight_kinematics_forward_drive (для TEB)
```

**Проблема 3: Робот врезается в препятствия**
```
1. Увеличить inflation_radius (должен быть > robot_radius)
2. Проверить footprint робота
3. Увеличить occdist_scale в DWA
```

**Проблема 4: Планировщик не находит путь**
```
1. Проверить allow_unknown: true
2. Увеличить cost_factor (робот слишком "боится" препятствий)
3. Проверить карту: нет ли полосок препятствий?
```

---

## **Шаг 5: Тестирование и тонкая настройка**

### **Тест 1: Базовое движение**
```bash
# Отправляем простую цель
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
"header:
  seq: 0
  stamp:
    secs: $(date +%s)
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

### **Тест 2: Мониторинг состояния**
```bash
# Смотреть состояние move_base
rostopic echo /move_base/status

# Смотреть глобальный план
rostopic echo /move_base/GlobalPlanner/plan

# Смотреть локальные траектории
rostopic echo /move_base/DWAPlannerROS/local_plan
```

### **Параметры для тонкой настройки:**

**Для плавности движения (DWA):**
```yaml
# Уменьшить рывки:
acc_lim_x: 0.3           # Меньше ускорение = плавнее
sim_time: 2.0            # Дальше смотрим вперед
path_distance_bias: 20   # Меньше = свободнее отклоняться от пути
goal_distance_bias: 15   # Меньше = не так стремиться к цели
```

**Для агрессивного вождения:**
```yaml
max_vel_x: 1.0           # Быстрее
acc_lim_x: 1.0           # Быстрее разгоняться
sim_time: 1.0            # Короче горизонт планирования
path_distance_bias: 40   # Сильнее следовать пути
```

---

## **Шаг 6: Чеклист обязательных проверок**

Перед запуском на реальном роботе проверьте:

### **✓ Обязательные проверки:**
1. [ ] `robot_radius` ≥ реального размера робота + 10%
2. [ ] `inflation_radius` ≥ `robot_radius` + 0.1 м
3. [ ] Топики датчиков совпадают (`/scan`, `/odom`)
4. [ ] TF-дерево правильно настроено
5. [ ] `footprint` соответствует форме робота
6. [ ] `max_vel_x` ≤ максимальной скорости робота
7. [ ] `acc_lim_x` ≤ реальному ускорению робота

### **✓ Проверки для первого запуска:**
```bash
# 1. Робот на открытом пространстве
# 2. Нет людей и хрупких предметов вокруг
# 3. Есть кнопка экстренной остановки
# 4. Скорость снижена: max_vel_x: 0.3
# 5. inflation_radius увеличен: +50%
```

---

## **Итог: минимальный рабочий конфиг**

**Абсолютный минимум для запуска:**
1. **costmap_common_params.yaml** — `robot_radius`, `inflation_radius`, топик лидара
2. **global_costmap_params.yaml** — `static_map: true`, разрешение, размер
3. **local_costmap_params.yaml** — `rolling_window: true`, размер окна
4. **local_planner_params.yaml** — `max_vel_x`, `acc_lim_x`, `xy_goal_tolerance`

**Запуск:** `roslaunch my_robot_navigation move_base.launch`

**После запуска:** Начинать с низких скоростей и постепенно увеличивать, тестируя в безопасных условиях.