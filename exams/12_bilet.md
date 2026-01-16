# Разбор билета 12

---

## **Теория 1: Проблемы интеграции и отладки сложных робототехнических систем. Инструменты ROS для их решения**

### **Проблемы интеграции сложных робототехнических систем:**

#### **1. Проблема синхронизации и временных задержек:**
- **Симптом:** Команды приходят с разной задержкой от разных датчиков
- **Пример:** Лидар (20 Гц) + камера (30 Гц) + IMU (100 Гц)
- **Последствия:** Неточная оценка состояния, артефакты в данных

#### **2. Проблема несовместимости интерфейсов:**
- **Симптом:** Разные форматы данных, единицы измерения, системы координат
- **Пример:** Лидар в мм, камера в пикселях, одометрия в м
- **Последствия:** Ошибки преобразования, некорректное слияние данных

#### **3. Проблема распределенной отладки:**
- **Симптом:** Ошибка проявляется в одном узле, но причина в другом
- **Пример:** Робот не двигается, потому что TF трансформ не публикуется
- **Последствия:** Трудно локализовать корневую причину

#### **4. Проблема воспроизводимости:**
- **Симптом:** На реальном роботе работает, в симуляции нет (или наоборот)
- **Пример:** Разные задержки, шумы, поведение датчиков
- **Последствия:** Трудно переносить решения между системами

#### **5. Проблема "цепочки зависимостей":**
- **Симптом:** Отказ одного компонента останавливает всю систему
- **Пример:** Падает драйвер лидара → падает SLAM → падает навигация
- **Последствия:** Хрупкость системы, низкая отказоустойчивость

---

### **Инструменты ROS для решения этих проблем:**

#### **1. rqt_graph - визуализация графа вычислений**

**Что решает:** Проблему понимания структуры системы и потоков данных

```bash
# Запуск
rosrun rqt_graph rqt_graph
# Или
rqt
# затем Plugins → Introspection → Node Graph
```

**Пример использования:**
```bash
# 1. Обнаружение отсутствующих связей
# Если expected connection отсутствует на графе

# 2. Обнаружение лишних/дублирующихся нод
# Несколько нод публикуют в один топик

# 3. Понимание топологии системы
# Кто кого слушает, какие данные куда идут
```

**Визуализация:**
```
┌─────────┐     /scan      ┌─────────┐     /map       ┌─────────┐
│ urg_node├────────────────►gmapping ├────────────────►map_server│
└─────────┘                └─────────┘                └─────────┘
                                   │
                             /amcl_pose│
                                   ▼
                            ┌─────────┐     /cmd_vel
                            │ move_base├────────────────►
                            └─────────┘
```

**Практическое применение:**
- **Диагностика:** Почему не приходят данные на /cmd_vel?
- **Интеграция:** Правильно ли подключен новый узел восприятия?
- **Оптимизация:** Какие ноды можно объединить для снижения задержек?

#### **2. RViz - 3D визуализация и отладка в реальном времени**

**Что решает:** Проблему пространственного понимания состояния системы

```bash
# Запуск
rosrun rviz rviz
# Или с конфигурацией
rosrun rviz rviz -d $(find my_robot)/config/navigation.rviz
```

**Ключевые дисплеи для отладки:**

**А) TF (Transformation Framework):**
```xml
<!-- В RViz -->
<Display type="rviz/TF">
  <Topic>/tf</Topic>
  <Marker Scale>0.1</MarkerScale>
</Display>
```
**Что показывает:** Правильность иерархии систем координат, наличие всех фреймов

**Б) LaserScan:**
```xml
<Display type="rviz/LaserScan">
  <Topic>/scan</Topic>
  <Size>0.05</Size>
</Display>
```
**Что показывает:** Корректность данных лидара, наличие артефактов, слепых зон

**В) Costmap (локальная и глобальная):**
```xml
<Display type="rviz/Map">
  <Topic>/move_base/local_costmap/costmap</Topic>
  <Alpha>0.7</Alpha>
  <Color Scheme>costmap</ColorScheme>
</Display>
```
**Что показывает:** Как система "видит" препятствия, правильно ли работает inflation

**Г) Path (глобальный и локальный план):**
```xml
<Display type="rviz/Path">
  <Topic>/move_base/GlobalPlanner/plan</Topic>
  <Color>0;0;1</Color> <!-- Синий -->
</Display>
<Display type="rviz/Path">
  <Topic>/move_base/DWAPlannerROS/local_plan</Topic>
  <Color>0;1;0</Color> <!-- Зеленый -->
</Display>
```
**Что показывает:** Соответствие планов, проблемы локального планирования

**Пример отладки проблемы:**
```
Проблема: Робот не движется к цели
Шаги в RViz:
1. Проверить TF: есть ли transform map→base_link?
2. Проверить LaserScan: видит ли робот препятствия?
3. Проверить Costmap: отображаются ли препятствия правильно?
4. Проверить Path: есть ли глобальный план? Локальный план?
5. Проверить Goal: правильно ли задана цель (система координат)?
```

#### **3. rosbag - запись и воспроизведение данных**

**Что решает:** Проблему воспроизводимости и постфактум анализа

```bash
# Запись данных
rosbag record -a -O my_experiment.bag  # Все топики
rosbag record /scan /odom /cmd_vel -O navigation.bag  # Выборочно

# Воспроизведение
rosbag play my_experiment.bag
rosbag play --rate=0.5 navigation.bag  # В замедленном темпе

# Анализ
rosbag info my_experiment.bag  # Информация о записи
rosbag filter input.bag output.bag "topic == '/scan'"  # Фильтрация
```

**Сценарии использования:**

**А) Воспроизведение проблемных ситуаций:**
```bash
# 1. Записать ситуацию, когда робот "застревает"
rosbag record /scan /odom /amcl_pose /move_base/status -O stuck_robot.bag

# 2. Воспроизвести и проанализировать
rosbag play stuck_robot.bag &
rosrun rviz rviz  # Одновременно смотреть в RViz
```

**Б) Тестирование алгоритмов на одинаковых данных:**
```bash
# Разработчики получают одинаковый dataset
rosbag play dataset.bag --clock  # Публикует /clock для синхронизации

# Все тестируют свои алгоритмы на одних данных
# Сравниваем результаты
```

**В) Создание synthetic данных для тестирования:**
```python
# Генерация тестовых данных
import rospy
from sensor_msgs.msg import LaserScan
import rosbag

bag = rosbag.Bag('test_data.bag', 'w')

scan = LaserScan()
scan.header.stamp = rospy.Time.now()
scan.ranges = [1.0] * 360  # Все расстояния = 1 м

bag.write('/scan', scan)
bag.close()
```

**Г) Анализ производительности:**
```bash
# Измерение частоты топиков в записи
rostopic hz /scan --bagfile my_experiment.bag
# Результат: average rate: 19.987

# Поиск временных задержек
# В rosbag play можно ускорить/замедлить для анализа timing issues
```

#### **4. Другие инструменты ROS для отладки:**

**rqt_console - централизованный просмотр логов:**
```bash
rosrun rqt_console rqt_console
```
- Фильтрация по severity (DEBUG, INFO, WARN, ERROR, FATAL)
- Поиск по ключевым словам
- Экспорт логов

**rqt_plot - графики данных в реальном времени:**
```bash
rosrun rqt_plot rqt_plot
```
- Построение графиков числовых данных
- Пример: `/cmd_vel/linear/x`, `/odom/twist/twist/linear/x`
- Выявление осцилляций, дрейфов

**rostopic - инспекция топиков:**
```bash
# Просмотр сообщений
rostopic echo /scan | head -5

# Информация о топике
rostopic info /scan
# Type: sensor_msgs/LaserScan
# Publishers: /urg_node
# Subscribers: /gmapping, /move_base

# Измерение частоты
rostopic hz /scan
```

**rosnode - управление нодами:**
```bash
# Список нод
rosnode list

# Информация о ноде
rosnode info /move_base
# Publications: /move_base/status, /cmd_vel
# Subscriptions: /scan, /odom, /map

# Проверка живости
rosnode ping /move_base
```

**rosservice - вызов сервисов:**
```bash
# Список сервисов
rosservice list

# Вызов сервиса
rosservice call /gazebo/reset_world "{}"

# Информация о сервисе
rosservice type /set_parameters | rossrv show
```

### **Интеграционный пайплайн с ROS инструментами:**

```
Разработка → Тестирование → Интеграция → Отладка
    │           │           │          │
    ↓           ↓           ↓          ↓
  rqt_     rosbag     rqt_      rviz
console   record     graph
           │          │
           ↓          ↓
        Воспроиз-  Понимание
        ведение    архитектуры
```

### **Пример полной сессии отладки:**

```bash
# Ситуация: Робот крутится на месте вместо движения к цели

# 1. Быстрая проверка
rosnode list | grep move_base  # move_base запущен?
rostopic echo /move_base/status  # Какое состояние?

# 2. Анализ графа
rqt_graph  # Есть ли связь move_base с одометрией?

# 3. Визуальная проверка
rviz  # Добавить TF, LaserScan, Path, Goal

# 4. Запись для детального анализа
rosbag record /scan /odom /amcl_pose /tf /move_base/* -O debug.bag

# 5. Воспроизведение и анализ
rosbag play debug.bag --rate=0.5 &
rqt_plot /amcl_pose/pose/pose/position/x /amcl_pose/pose/pose/position/y

# 6. Выявление проблемы: TF transform не публикуется
rosrun tf tf_echo map base_link  # Ошибка: "No transform available"

# 7. Решение: перезапустить AMCL или задать начальную позу
```

### **Передовые практики отладки в ROS:**

1. **Инструментализация кода:**
```python
# Добавление диагностических сообщений
rospy.logdebug("Processing scan with %d points", len(scan.ranges))
rospy.loginfo("Goal received at (%f, %f)", goal.x, goal.y)
rospy.logwarn("Low battery: %d%%", battery_level)
rospy.logerr("Failed to reach goal after %d attempts", attempts)
```

2. **Диагностический агрегатор:**
```xml
<!-- Публикация диагностической информации -->
<node pkg="diagnostic_aggregator" type="aggregator_node" 
      name="diagnostic_aggregator">
  <rosparam command="load" 
            file="$(find my_robot)/config/diagnostics.yaml"/>
</node>
```

3. **Профилирование производительности:**
```bash
# Измерение загрузки CPU нодами
top -p $(pidof roslaunch)  # Или конкретной ноды

# Профилирование с rosprof
rosrun --prefix 'valgrind --tool=callgrind' my_node my_node
```

---

## **Теория 2: Методы RRT и RRT***

### **Контекст:** Планирование движения в сложных пространствах

### **Классическая проблема планирования:**
- **Конфигурационное пространство (C-space):** Все возможные состояния робота
- **Для манипулятора с N сочленениями:** C-space = ℝ^N (N-мерное пространство)
- **Пример:** 7-осевой манипулятор → 7-мерное пространство
- **Проблема:** Традиционные методы (A*, grid-based) не масштабируются

---

### **Алгоритм RRT (Rapidly-exploring Random Tree)**

**Идея:** Постепенное исследование пространства случайными выборками

#### **Основной алгоритм RRT:**
```python
def rrt_plan(start, goal, space, max_iter=1000):
    # Инициализация дерева
    tree = Tree()
    tree.add_node(start)
    
    for i in range(max_iter):
        # 1. Случайная выборка
        q_rand = random_sample(space)
        
        # 2. Найти ближайший узел в дереве
        q_near = nearest_neighbor(tree, q_rand)
        
        # 3. Попытаться продвинуться от q_near к q_rand
        q_new = steer(q_near, q_rand, step_size)
        
        # 4. Проверить, что ребро q_near→q_new свободно
        if collision_free(q_near, q_new, obstacles):
            # 5. Добавить новый узел в дерево
            tree.add_node(q_new)
            tree.add_edge(q_near, q_new)
            
            # 6. Проверить, достигли ли цели
            if distance(q_new, goal) < threshold:
                # Построить путь от start до goal
                return extract_path(tree, start, q_new)
    
    return None  # Путь не найден
```

#### **Ключевые компоненты RRT:**

**1. random_sample():** Генерация случайной точки в C-space
```python
def random_sample(space):
    # Для 2D: (x, y)
    # Для манипулятора: (θ₁, θ₂, ..., θₙ)
    if random() < goal_bias:
        return goal  # С вероятностью p вернуть цель (ускорение сходимости)
    else:
        return uniform_sample(space)
```

**2. nearest_neighbor():** Поиск ближайшего узла
```python
def nearest_neighbor(tree, q_rand):
    # Для высоких размерностей используем KD-tree
    min_dist = INF
    nearest = None
    for node in tree.nodes:
        d = distance(node, q_rand)
        if d < min_dist:
            min_dist = d
            nearest = node
    return nearest
```

**3. steer():** Движение от q_near к q_rand
```python
def steer(q_near, q_rand, step_size):
    # Линейная интерполяция
    direction = normalize(q_rand - q_near)
    q_new = q_near + direction * min(step_size, distance(q_near, q_rand))
    return q_new
```

#### **Преимущества RRT для пространств высокой размерности:**

**1. Эффективность в высоких размерностях:**
- **Grid-based методы:** O(k^N), где N - размерность
- **RRT:** O(N log M), где M - число узлов в дереве

**2. Не требует дискретизации пространства:**
- Работает в непрерывном C-space
- Не страдает от "проклятия размерности"

**3. Быстрое исследование:**
- Фокусируется на неисследованных областях
- Exponential growth coverage

**4. Учет ограничений:**
```python
def collision_free(q1, q2, obstacles):
    # Проверка на пути между конфигурациями
    # Для манипулятора: forward kinematics + проверка столкновений
    for alpha in linspace(0, 1, num_check_points):
        q = interpolate(q1, q2, alpha)
        if in_collision(q, obstacles):
            return False
    return True
```

#### **Недостатки базового RRT:**

1. **Не оптимальность:** Путь далек от оптимального
2. **Детерминированность:** Разные запуски дают разные результаты
3. **Медленная сходимость к цели:** Особенно в узких проходах

---

### **RRT* (Optimal RRT)**

**Идея:** Постепенная оптимизация пути при добавлении новых узлов

#### **Улучшения RRT* по сравнению с RRT:**

**1. Выбор родителя не по близости, а по оптимальности:**
```python
def rrt_star_choose_parent(q_new, tree, radius):
    # Найти все узлы в окрестности q_new
    neighbors = find_neighbors(tree, q_new, radius)
    
    # Среди них найти того, от которого путь к q_new самый короткий
    best_parent = None
    min_cost = INF
    
    for q_near in neighbors:
        # Стоимость до q_near + стоимость от q_near до q_new
        cost = tree.cost[q_near] + distance(q_near, q_new)
        
        if cost < min_cost and collision_free(q_near, q_new):
            min_cost = cost
            best_parent = q_near
    
    return best_parent, min_cost
```

**2. Переподключение (rewiring):**
```python
def rrt_star_rewire(tree, q_new, radius):
    # Для всех соседей q_new
    neighbors = find_neighbors(tree, q_new, radius)
    
    for q_near in neighbors:
        # Если путь через q_new короче текущего пути к q_near
        new_cost = tree.cost[q_new] + distance(q_new, q_near)
        old_cost = tree.cost[q_near]
        
        if new_cost < old_cost and collision_free(q_new, q_near):
            # Переподключить q_near к q_new
            tree.remove_edge(q_near.parent, q_near)
            tree.add_edge(q_new, q_near)
            tree.cost[q_near] = new_cost
            
            # Обновить стоимости всех потомков q_near
            update_costs(tree, q_near)
```

#### **Полный алгоритм RRT*:**
```python
def rrt_star(start, goal, space, max_iter):
    tree = Tree()
    tree.add_node(start)
    tree.cost[start] = 0
    
    for i in range(max_iter):
        # 1. Случайная выборка
        q_rand = random_sample(space)
        
        # 2. Ближайший узел
        q_nearest = nearest_neighbor(tree, q_rand)
        
        # 3. Steer к новой точке
        q_new = steer(q_nearest, q_rand, step_size)
        
        if collision_free(q_nearest, q_new):
            # 4. Найти лучшего родителя в окрестности
            radius = gamma * (log(len(tree.nodes)) / len(tree.nodes)) ** (1/dim)
            q_parent, cost = choose_parent(q_new, tree, radius)
            
            # 5. Добавить узел
            tree.add_node(q_new)
            tree.add_edge(q_parent, q_new)
            tree.cost[q_new] = cost
            
            # 6. Переподключение соседей
            rewire(tree, q_new, radius)
    
    return tree
```

#### **Свойства RRT*:**

1. **Асимптотическая оптимальность:** При бесконечном времени сходится к оптимальному пути
2. **Incremental optimization:** Путь улучшается со временем
3. **Любые допустимые пути:** Находит путь, если он существует

---

### **Варианты и расширения RRT/RRT*:**

#### **1. Informed RRT*:**
```python
def informed_rrt_star(start, goal, space):
    # После нахождения первого пути
    best_path = initial_rrt_star(start, goal)
    best_cost = path_cost(best_path)
    
    # Ограничить выборку эллипсоидом
    c_min = distance(start, goal)  # Прямое расстояние
    c_best = best_cost
    
    # Эллипсоид: фокусы = start, goal, сумма расстояний = c_best
    while time_remaining:
        q_rand = sample_from_ellipse(start, goal, c_best, c_min)
        # ... остальной алгоритм как в RRT*
    
    return best_path
```
**Преимущество:** Фокусирует поиск на перспективных областях

#### **2. RRT-Connect:**
```python
def rrt_connect(start, goal):
    # Два дерева: от start и от goal
    tree_a = Tree(start)
    tree_b = Tree(goal)
    
    while True:
        # Расширяем tree_a
        q_rand = random_sample()
        q_near_a = nearest_neighbor(tree_a, q_rand)
        q_new_a = steer(q_near_a, q_rand)
        
        if collision_free(q_near_a, q_new_a):
            tree_a.add(q_new_a)
            
            # Пытаемся соединить с tree_b
            q_near_b = nearest_neighbor(tree_b, q_new_a)
            if collision_free(q_new_a, q_near_b):
                # Нашли соединение!
                return connect_trees(tree_a, tree_b)
        
        # Меняем деревья местами
        tree_a, tree_b = tree_b, tree_a
```
**Преимущество:** Быстрее находит путь в простых случаях

#### **3. Kinodynamic RRT*:**
```python
def kinodynamic_rrt_star(state_start, state_goal):
    # Учитывает динамику: не только позиции, но и скорости/ускорения
    # steer() генерирует траектории с учетом динамических ограничений
    
    # Пример для дрона: (x, y, z, vx, vy, vz, ...)
    # steer() решает двухточечную краевую задачу
```
**Преимущество:** Учитывает динамические ограничения робота

#### **4. Multi-directional RRT:**
```python
def multi_rrt(start, goals):
    # Несколько целей или несколько стартов
    trees = [Tree(start)] + [Tree(goal) for goal in goals]
    
    # Параллельное расширение всех деревьев
    # Соединение при контакте
```
**Преимущество:** Для multi-goal planning

---

### **Сравнительная таблица:**

| Алгоритм | Оптимальность | Скорость | Память | Применение |
|----------|---------------|----------|--------|------------|
| **RRT** | Нет | Быстро | Мало | Быстрый поиск любого пути |
| **RRT*** | Асимптотически оптимальный | Медленнее | Больше | Качественные траектории |
| **RRT-Connect** | Нет | Очень быстро | Мало | Простые среды |
| **Informed RRT*** | Асимптотически оптимальный | Быстрее чем RRT* | Средняя | Когда известна оценка оптимального пути |

### **Применение для манипуляторов:**

#### **Пример: 7-осевой промышленный робот**
```python
class ManipulatorPlanner:
    def __init__(self, robot_model):
        self.dof = 7  # Degrees of freedom
        self.joint_limits = [
            (-3.14, 3.14),  # θ₁
            (-2.09, 2.09),  # θ₂
            # ... для всех 7 суставов
        ]
    
    def plan_path(self, q_start, q_goal):
        # C-space = [θ₁, θ₂, ..., θ₇]
        space = CartesianProduct(self.joint_limits)
        
        # Планирование в 7-мерном пространстве
        path = rrt_star(q_start, q_goal, space, max_iter=5000)
        
        return path
    
    def collision_check(self, q1, q2):
        # Forward kinematics для проверки столкновений
        for alpha in linspace(0, 1, 10):
            q = interpolate(q1, q2, alpha)
            
            # Вычислить позицию всех звеньев
            positions = forward_kinematics(q)
            
            # Проверить столкновения с окружением
            for link_pos in positions:
                if in_collision(link_pos, obstacles):
                    return False
        
        return True
```

### **Преимущества RRT/RRT* для сложных ограничений:**

#### **1. Произвольные ограничения:**
```python
def constrained_rrt(q_near, q_rand):
    # Ограничения могут быть:
    # - Кинематические: joint limits
    # - Динамические: скорости, ускорения
    # - Задачи: ориентация инструмента, избегание singularities
    
    # RRT может учитывать их в функции steer() и collision_check()
    q_new = steer_with_constraints(q_near, q_rand, constraints)
    
    return q_new
```

#### **2. Работа в task space и configuration space:**
```python
def task_space_rrt(x_start, x_goal):
    # x_start, x_goal в task space (например, позиция захвата)
    # Обратная кинематика может иметь несколько решений
    
    # Алгоритм:
    # 1. Выборка в task space
    # 2. Для каждой выборки находим обратную кинематику
    # 3. Выбираем решение, ближайшее к текущей конфигурации
    # 4. Планируем в configuration space между решениями
```

#### **3. Multi-modal planning:**
```python
def multi_modal_rrt(start, goal):
    # Когда есть несколько способов достичь цели
    # Например, манипулятор может достичь точки разными конфигурациями
    
    # RRT естественным образом находит разные пути
    # Можно затем выбрать лучший по критерию (энергия, время, плавность)
```

### **Интеграция с ROS:**

#### **Пакет MoveIt! использует RRT для манипуляторов:**
```bash
# MoveIt! конфигурация для планирования
roslaunch panda_moveit_config demo.launch
```

#### **OMPL (Open Motion Planning Library) в ROS:**
```xml
<!-- В MoveIt! конфигурации -->
<param name="planning_plugin" value="ompl_interface/OMPLPlanner"/>
<rosparam command="load" file="$(find my_robot)/config/ompl_planning.yaml"/>
```

```yaml
# ompl_planning.yaml
planning_adapters:
  - default_planner_request_adapters/AddTimeParameterization
  
planner_configs:
  RRT:
    type: geometric::RRT
  RRTConnect:
    type: geometric::RRTConnect
  RRTstar:
    type: geometric::RRTstar
  PRM:
    type: geometric::PRM
```

### **Ограничения RRT/RRT*:**

1. **Потребление памяти:** Дерево растет с числом итераций
2. **Стохастичность:** Нет гарантий на конечное время
3. **"Червоточины":** В узких проходах медленная сходимость
4. **Не подходит для динамических сред:** Требует перепланирования

---

## **Задача: Настройка и запуск move_base для конкретного робота**

### **Обзор:** move_base - центральный узел навигационного стека ROS

### **Шаг 1: Структура конфигурационных файлов**

```
my_robot_navigation/
├── launch/
│   ├── move_base.launch          # Главный launch-файл
│   └── amcl.launch              # Локализация
├── config/
│   ├── costmap_common_params.yaml    # ОБЩИЕ параметры карт
│   ├── global_costmap_params.yaml    # Глобальная карта
│   ├── local_costmap_params.yaml     # Локальная карта
│   ├── global_planner_params.yaml    # Глобальный планировщик
│   ├── dwa_local_planner_params.yaml # Локальный планировщик (DWA)
│   └── teb_local_planner_params.yaml # Альтернатива: TEB
└── maps/
    ├── map.yaml                  # Статическая карта
    └── map.pgm
```

### **Шаг 2: Обязательные параметры для настройки**

#### **1. costmap_common_params.yaml - САМЫЙ ВАЖНЫЙ!**

**Почему важен:** Определяет, как робот воспринимает мир

```yaml
# === РАЗМЕРЫ РОБОТА (КРИТИЧЕСКИ!) ===
robot_base_frame: base_footprint  # Фрейм робота в TF

# Вариант А: Круглый робот
robot_radius: 0.45  # В метрах, должен быть ≥ реального радиуса + запас

# Вариант Б: Прямоугольный/сложной формы робот
footprint: [[-0.3, -0.25], [-0.3, 0.25], [0.3, 0.25], [0.3, -0.25]]
# Координаты углов относительно robot_base_frame

# === ИНФЛЯЦИЯ ПРЕПЯТСТВИЙ ===
inflation_radius: 0.55  # ДОЛЖЕН БЫТЬ > robot_radius на 10-20%
inflation_decay_distance: 0.58  # Расстояние, на котором стоимость становится минимальной
cost_scaling_factor: 10.0  # Как быстро падает стоимость от препятствия

# === НАСТРОЙКИ ДАТЧИКОВ ===
obstacle_layer:
  enabled: true
  observation_sources: scan
  scan:
    data_type: LaserScan
    topic: /scan  # ДОЛЖЕН СОВПАДАТЬ с реальным топиком лидара!
    marking: true    # Отмечать препятствия
    clearing: true   # Очищать свободное пространство
    expected_update_rate: 0.5  # Как часто ожидаем данные (сек)
    
  obstacle_range: 2.5    # Макс. расстояние обнаружения препятствий (м)
  raytrace_range: 3.0    # Макс. расстояние очистки пространства

# === ПРЕОБРАЗОВАНИЕ В ОCCUPANCY GRID ===
map_type: costmap
# Стоимости: 0-255, где:
# 0: свободно, 255: занято, -1: неизвестно
# 253-254: inflation зона (постепенно дешевле)
```

**Проверки:**
```bash
# 1. Проверить реальные размеры робота
# 2. Измерить расстояние от центра до самой дальней точки
# 3. Добавить запас безопасности (10-20%)
# 4. Проверить топик лидара: rostopic list | grep scan
```

#### **2. global_costmap_params.yaml - для глобального планирования**

**Почему важен:** Определяет, как робот планирует путь по всей карте

```yaml
global_costmap:
  global_frame: map              # Система координат карты
  robot_base_frame: base_footprint
  update_frequency: 1.0          # Можно реже (1-2 Гц)
  publish_frequency: 0.5
  static_map: true               # Используем загруженную карту!
  rolling_window: false          # НЕ использовать для глобальной карты
  transform_tolerance: 0.5       # Допуск для TF (сек)
  
  # РАЗМЕРЫ КАРТЫ
  width: 30.0      # Ширина в метрах (должна покрывать рабочую зону)
  height: 30.0     # Высота в метрах
  resolution: 0.05 # Разрешение (м/пиксель). 0.05 = 5 см
  
  # ИСТОЧНИКИ ДАННЫХ (наследуются из common)
  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

**Ключевые параметры:**
- `static_map: true` - обязательно для глобальной карты
- `resolution` - компромисс: меньше = точнее, но больше памяти
- Размеры должны покрывать всю рабочую область

#### **3. local_costmap_params.yaml - для локального планирования**

**Почему важен:** Для избегания динамических препятствий в реальном времени

```yaml
local_costmap:
  global_frame: odom             # Лучше odom для стабильности
  robot_base_frame: base_footprint
  update_frequency: 5.0          # Часто! 5-10 Гц для динамических препятствий
  publish_frequency: 2.0
  static_map: false              # НЕ использовать статическую карту!
  rolling_window: true           # Скользящее окно (следит за роботом)
  rolling_window_size: 6.0       # Размер окна (должен быть > 2×robot_radius)
  transform_tolerance: 0.5
  
  # РАЗМЕРЫ ЛОКАЛЬНОЙ КАРТЫ
  width: 6.0      # Ширина окна в метрах
  height: 6.0     # Высота окна
  resolution: 0.05 # ДОЛЖНО СОВПАДАТЬ с глобальной картой!
  
  # ИСТОЧНИКИ
  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

**Ключевые отличия от глобальной:**
- `rolling_window: true` - карта движется с роботом
- `update_frequency: 5.0` - обновляется чаще
- `global_frame: odom` - лучше для локальной стабильности

#### **4. global_planner_params.yaml - глобальный планировщик**

**Почему важен:** Находит путь от текущей позиции к цели

```yaml
# Для NavfnROS (старый) или GlobalPlanner (новый)
NavfnROS:
  use_dijkstra: false            # false = A*, true = Dijkstra
  allow_unknown: true            # Разрешить путь через неизвестные области
  default_tolerance: 0.5         # Допуск при достижении цели (м)
  
  # КРИТИЧЕСКИЕ ПАРАМЕТРЫ:
  cost_factor: 3.0               # Множитель стоимости (выше = избегать препятствий)
  neutral_cost: 50               # Стоимость нейтральной клетки
  lethal_cost: 253               # Смертельная стоимость (совпадает с inflation)
  
  # Для больших карт:
  planner_window_x: 0.0          # Окно планирования (0 = вся карта)
  planner_window_y: 0.0
  planner_window_size: 0.0

# Альтернатива: GlobalPlanner
GlobalPlanner:
  use_dijkstra: false
  use_grid_path: false           # true = grid path, false = gradient
  allow_unknown: true
  use_quadratic: true
```

**Выбор планировщика:**
- **A*:** Быстрее, "достаточно хороший" путь
- **Dijkstra:** Гарантированно оптимальный, но медленнее
- **Theta*:** Более прямые пути (если поддерживается)

#### **5. local_planner_params.yaml - локальный планировщик (DWA)**

**Почему важен:** Отвечает за реальное движение и объезд препятствий

```yaml
DWAPlannerROS:
  # === ОГРАНИЧЕНИЯ РОБОТА (ДОЛЖНЫ СОВПАДАТЬ С РЕАЛЬНОСТЬЮ!) ===
  max_vel_x: 0.5                 # Макс. скорость вперед (м/с)
  min_vel_x: -0.2                # Мин. скорость (ОТРИЦАТЕЛЬНАЯ для движения назад!)
  max_vel_theta: 1.0             # Макс. угловая скорость (рад/с)
  min_vel_theta: -1.0            # Мин. угловая скорость
  
  acc_lim_x: 0.5                 # Линейное ускорение (м/с²)
  acc_lim_theta: 1.0             # Угловое ускорение (рад/с²)
  
  # === ОКНО ПЛАНИРОВАНИЯ ===
  vx_samples: 6                  # Количество пробных линейных скоростей
  vtheta_samples: 20             # Количество пробных угловых скоростей
  sim_time: 1.5                  # Время симуляции траекторий (сек)
  sim_granularity: 0.025         # Шаг симуляции (сек)
  
  # === ВЕСА ДЛЯ ОЦЕНКИ ТРАЕКТОРИЙ ===
  path_distance_bias: 32.0       # Вес следования глобальному пути
  goal_distance_bias: 24.0       # Вес движения к цели
  occdist_scale: 0.01            # Вес избегания препятствий (меньше = смелее)
  
  # === КРИТИЧЕСКИЕ ПАРАМЕТРЫ ===
  dwa: true                      # Использовать DWA
  oscillation_reset_dist: 0.05   # Расстояние для сброса осцилляции (м)
  
  # === ДОСТИЖЕНИЕ ЦЕЛИ ===
  xy_goal_tolerance: 0.15        # Допуск по позиции (м)
  yaw_goal_tolerance: 0.15       # Допуск по ориентации (рад)
  latch_xy_goal_tolerance: false
  
  # === ВРАЩЕНИЕ НА МЕСТЕ ===
  meter_scoring: true
  oscillation_reset_time: 2.0    # Время сброса осцилляции (сек)
```

**Критические проверки:**
1. `min_vel_x` должен быть отрицательным для движения назад
2. `robot_radius`/`footprint` должен совпадать с `costmap_common_params`
3. `xy_goal_tolerance` должен быть больше погрешности локализации

### **Шаг 3: Launch-файл для запуска**

```xml
<!-- move_base.launch -->
<launch>
  <!-- 1. Загружаем карту (если используется) -->
  <arg name="map_file" default="$(find my_robot_navigation)/maps/map.yaml"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)"/>
  
  <!-- 2. Запускаем AMCL для локализации -->
  <include file="$(find my_robot_navigation)/launch/amcl.launch"/>
  
  <!-- 3. Основной узел move_base -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    
    <!-- Загружаем ВСЕ конфигурационные файлы -->
    <!-- Общие параметры (для обеих карт) -->
    <rosparam file="$(find my_robot_navigation)/config/costmap_common_params.yaml" 
              command="load" ns="global_costmap"/>
    <rosparam file="$(find my_robot_navigation)/config/costmap_common_params.yaml" 
              command="load" ns="local_costmap"/>
    
    <!-- Глобальная карта -->
    <rosparam file="$(find my_robot_navigation)/config/global_costmap_params.yaml" 
              command="load"/>
    
    <!-- Локальная карта -->
    <rosparam file="$(find my_robot_navigation)/config/local_costmap_params.yaml" 
              command="load"/>
    
    <!-- Глобальный планировщик -->
    <rosparam file="$(find my_robot_navigation)/config/global_planner_params.yaml" 
              command="load"/>
    
    <!-- Локальный планировщик -->
    <rosparam file="$(find my_robot_navigation)/config/dwa_local_planner_params.yaml" 
              command="load"/>
    
    <!-- ВЫБОР ПЛАНИРОВЩИКОВ -->
    <param name="base_global_planner" value="global_planner/GlobalPlanner"/>
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS"/>
    <!-- Альтернатива для дифференциальных роботов: -->
    <!-- <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/> -->
    
    <!-- НАСТРОЙКИ MOVE_BASE -->
    <param name="controller_frequency" value="5.0"/>  # Частота контроллера (Гц)
    <param name="planner_frequency" value="1.0"/>    # Частота перепланирования
    <param name="recovery_behavior_enabled" value="true"/>
    <param name="clearing_rotation_allowed" value="true"/>
    
  </node>
  
  <!-- 4. Визуализация (опционально, но рекомендуется) -->
  <node name="rviz" pkg="rviz" type="rviz" 
        args="-d $(find my_robot_navigation)/rviz/navigation.rviz"/>
</launch>
```

### **Шаг 4: Процесс запуска и проверки**

#### **Последовательность запуска:**
```bash
# 1. ROS мастер
roscore

# 2. Драйверы робота (в отдельном терминале или launch-файле)
roslaunch my_robot_driver bringup.launch

# 3. Проверка TF дерева
rosrun tf view_frames  # Должно быть: map → odom → base_footprint → base_link → ...

# 4. Запуск навигации
roslaunch my_robot_navigation move_base.launch

# 5. Задание цели (в RViz кликом или через командную строку)
```

#### **Проверки после запуска:**

**1. Проверить все ноды запущены:**
```bash
rosnode list
# Должны быть: /map_server, /amcl, /move_base, (возможно /rviz)
```

**2. Проверить топики:**
```bash
rostopic list | grep -E "(scan|odom|cmd_vel|map|amcl_pose)"
# /scan - данные лидара
# /odom - одометрия
# /cmd_vel - команды скорости (должны публиковаться при движении)
# /map - карта
# /amcl_pose - оценка позиции
```

**3. Проверить TF:**
```bash
rosrun tf tf_echo map base_link
# Должна быть трансформация
# Если "No transform available" - проблема с локализацией (AMCL)
```

**4. Визуальная проверка в RViz:**
```
Добавить дисплеи:
1. Map: topic=/map
2. LaserScan: topic=/scan
3. TF
4. Path (глобальный): topic=/move_base/GlobalPlanner/plan
5. Path (локальный): topic=/move_base/DWAPlannerROS/local_plan
6. Costmap: topic=/move_base/global_costmap/costmap
```

### **Шаг 5: Решение типичных проблем**

#### **Проблема 1: Робот не двигается**
```bash
# 1. Проверить команды скорости
rostopic echo /cmd_vel  # Должны быть ненулевые при наличии цели

# 2. Проверить глобальный план
rostopic echo /move_base/GlobalPlanner/plan  # Должен существовать

# 3. Проверить локализацию
rostopic echo /amcl_pose  # Должна быть разумная позиция на карте

# 4. Проверить параметр min_vel_x
rosparam get /move_base/DWAPlannerROS/min_vel_x
# Должен быть отрицательным для возможности движения назад
```

#### **Проблема 2: Робот врезается в препятствия**
```bash
# 1. Проверить robot_radius/inflation_radius
rosparam get /move_base/global_costmap/inflation_radius
# Должен быть ≥ robot_radius + 0.1 м

# 2. Проверить данные лидара в RViz
# Видит ли робот препятствия?

# 3. Увеличить occdist_scale в DWA
rosparam set /move_base/DWAPlannerROS/occdist_scale 0.05
```

#### **Проблема 3: Робот осциллирует или не может достичь цели**
```bash
# 1. Увеличить допуски
rosparam set /move_base/DWAPlannerROS/xy_goal_tolerance 0.3
rosparam set /move_base/DWAPlannerROS/yaw_goal_tolerance 0.5

# 2. Увеличить goal_distance_bias
rosparam set /move_base/DWAPlannerROS/goal_distance_bias 50.0
```

### **Шаг 6: Тонкая настройка для конкретного робота**

#### **Для медленных/тяжелых роботов:**
```yaml
# Уменьшить скорости и ускорения
max_vel_x: 0.3
acc_lim_x: 0.3
sim_time: 2.0  # Смотреть дальше вперед
```

#### **Для быстрых/маневренных роботов:**
```yaml
# Увеличить скорости
max_vel_x: 1.0
acc_lim_x: 1.0
vx_samples: 10  # Больше вариантов скоростей
vtheta_samples: 30
```

#### **Для роботов с плохой одометрией:**
```yaml
# В local_costmap_params.yaml
global_frame: map  # Вместо odom (но может быть менее стабильно)
transform_tolerance: 1.0  # Увеличить допуск
```

### **Минимальный рабочий конфиг для тестирования:**

```yaml
# Минимальный costmap_common_params.yaml
robot_base_frame: base_link
robot_radius: 0.3
inflation_radius: 0.4

obstacle_layer:
  observation_sources: scan
  scan: {topic: /scan, data_type: LaserScan}

# Минимальный dwa_local_planner_params.yaml
DWAPlannerROS:
  max_vel_x: 0.5
  min_vel_x: -0.2
  max_vel_theta: 1.0
  xy_goal_tolerance: 0.3
  yaw_goal_tolerance: 0.5
```

### **Заключение:**

Настройка move_base требует внимания к **четырем ключевым аспектам**:
1. **Размеры робота** (robot_radius/footprint) - критично для безопасности
2. **Сенсорная конфигурация** (топики лидара) - чтобы робот "видел" мир
3. **Кинематические ограничения** (скорости, ускорения) - должны соответствовать реальности
4. **Параметры безопасности** (inflation, occdist_scale) - баланс между агрессивностью и безопасностью

**Рекомендуемый процесс:** Начинать с консервативных параметров (низкие скорости, большие допуски), тестировать в безопасных условиях, постепенно оптимизировать под конкретные требования задачи.
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