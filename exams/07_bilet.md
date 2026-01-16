# Разбор билета 7

---

## **Теория 1: Основные принципы построения ROS и их реализация**

### **1. Принцип слабой связности (Loose Coupling)**

**Определение:** Компоненты системы минимально зависят друг от друга и взаимодействуют через четко определенные интерфейсы.

**Реализация в ROS:**

#### **А) Публикация/подписка на сообщения (Publish/Subscribe)**
```python
# Узел-издатель (не знает о подписчиках)
pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
# Узел-подписчик (не знает об издателях)
rospy.Subscriber('/scan', LaserScan, callback_function)
```
**Преимущество:** Издатель и подписчик не знают о существовании друг друга, могут запускаться/останавливаться независимо.

#### **Б) Сервисы (Services) - синхронный RPC**
```python
# Сервер (предоставляет услугу)
s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
# Клиент (вызывает услугу)
rospy.wait_for_service('add_two_ints')
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
result = add_two_ints(5, 3)
```
**Преимущество:** Клиент и сервер независимы, сервис может быть переписан без изменения клиента.

#### **В) Действия (Actions) - асинхронные задачи**
```python
# Сервер действия
action_server = actionlib.SimpleActionServer(
    'move_base', MoveBaseAction, execute_cb, auto_start=False)
# Клиент действия
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.send_goal(goal)
```
**Преимущество:** Длительные операции с обратной связью, прерыванием.

#### **Г) Параметрический сервер (Parameter Server)**
```python
# Запись параметра
rospy.set_param('/robot_speed', 0.5)
# Чтение параметра (из любого узла)
speed = rospy.get_param('/robot_speed', 0.3)
```
**Преимущество:** Централизованное хранение конфигурации, узлы не связаны жестко.

### **2. Принцип повторного использования кода (Code Reusability)**

**Определение:** Компоненты разрабатываются как независимые, переиспользуемые модули.

**Реализация в ROS:**

#### **А) Пакеты (Packages)**
```bash
# Структура пакета
my_robot_navigation/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── move_base.launch
├── config/
│   └── dwa_local_planner_params.yaml
├── src/
│   └── navigation_node.cpp
└── scripts/
    └── teleop_script.py
```
**Преимущество:** Пакет - самодостаточная единица, может быть скопирован в другой проект.

#### **Б) Ноды (Nodes) как независимые процессы**
```bash
# Каждая нода - отдельный процесс
rosrun turtlesim turtlesim_node  # Нода 1
rosrun turtlesim turtle_teleop_key  # Нода 2
```
**Преимущество:** Ноды можно перезапускать независимо, разрабатывать на разных языках.

#### **В) Библиотеки (Libraries)**
```cmake
# CMakeLists.txt - сборка библиотеки
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_navigation_lib
  CATKIN_DEPENDS roscpp rospy std_msgs
)

# Другие пакеты могут ссылаться на библиотеку
find_package(catkin REQUIRED COMPONENTS my_robot_navigation)
target_link_libraries(my_node ${catkin_LIBRARIES})
```

#### **Г) Драйверы как отдельные пакеты**
```
# Драйвер Hokuyo лидара работает с любым роботом
rosrun urg_node urg_node _ip_address:=192.168.0.10
# Подключается к любому роботу через топик /scan
```

### **3. Принцип языковой независимости (Language Neutrality)**

**Определение:** Компоненты могут быть написаны на разных языках программирования.

**Реализация в ROS:**

#### **А) Интерфейс сообщений (Message Interface)**
```bash
# Определение сообщения (языково-независимое)
# LaserScan.msg
Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32[] ranges

# Генерация кода для разных языков
rosmsg show LaserScan  # Показывает структуру на любом языке
```

#### **Б) Клиентские библиотеки (Client Libraries)**
```python
# Python нода
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo("Received scan with %d ranges", len(data.ranges))

rospy.init_node('python_listener')
rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
```

```cpp
// C++ нода (та же функциональность)
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Received scan with %lu ranges", msg->ranges.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cpp_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 10, callback);
    ros::spin();
    return 0;
}
```

#### **В) Инструменты командной строки (языково-независимые)**
```bash
# Те же команды работают независимо от языка реализации нод
rostopic echo /scan  # Выводит сообщения от любой ноды
rosnode list  # Показывает все ноды независимо от языка
rosservice call /reset  # Вызывает сервис на любом языке
```

#### **Г) RPC-подобная коммуникация**
```xml
<!-- XML-RPC для управления нодами -->
<methodCall>
  <methodName>getBusStats</methodName>
  <params>
    <param><value><string>/node_name</string></value></param>
  </params>
</methodCall>
```
**Преимущество:** Мастер ROS использует XML-RPC для управления нодами, не зависит от языка реализации нод.

### **Дополнительные принципы ROS:**

#### **4. Принцип инструментальности (Tool-Based)**
```bash
# Богатый набор инструментов
rqt_graph  # Визуализация графа вычислений
rosbag record -a  # Запись данных
rviz  # 3D визуализация
rqt_console  # Просмотр логов
```

#### **5. Принцип многоуровневости (Multi-Level)**
```
Уровень 1: ОС (Linux) + драйверы
Уровень 2: ROS ядро (roscore, roslaunch)
Уровень 3: Драйверы устройств (лидары, камеры)
Уровень 4: Алгоритмы (навигация, восприятие)
Уровень 5: Приложения
```

### **Пример полной системы, демонстрирующей все принципы:**

```bash
# Слабая связность: независимые ноды
rosrun urg_node urg_node _ip_address:=192.168.0.10  # Драйвер лидара (C++)
rosrun gmapping slam_gmapping  # SLAM (C++)
rosrun rviz rviz  # Визуализация (C++/Qt)
rosrun my_navigation navigation_node.py  # Навигация (Python)

# Повторное использование: стандартные сообщения
# Все ноды используют sensor_msgs/LaserScan, nav_msgs/Odometry

# Языковая независимость:
# urg_node (C++) → gmapping (C++) → navigation_node (Python) → rviz (C++/Qt)

# Коммуникация через топики:
/scan (LaserScan) ← urg_node → gmapping → /map → navigation_node → /cmd_vel
```

---

## **Теория 2: Алгоритм D* (D-Star) и D* Lite**

### **Контекст:** Планирование пути в неизвестных или динамически изменяющихся средах

### **Алгоритм A* (база для сравнения):**
```
1. Планирует от start до goal
2. Использует эвристику h(n) (обычно манхэттенское или евклидово расстояние)
3. Одноразовое вычисление: при изменении среды перепланирует с нуля
4. Эффективен для статических сред
```

### **Алгоритм D* (Dynamic A*):**
**Разработан:** Anthony Stentz, 1994
**Идея:** Инкрементальное перепланирование при изменении стоимости ребер

#### **Ключевые особенности:**

**1. Обратный поиск (backward search):**
- Планирует **от цели к старту**
- Вычисляет оптимальную стоимость до цели для каждой вершины
- При движении робота используется прямой поиск по уже вычисленным стоимостям

**2. Приоритетная очередь с двумя состояниями:**
- **RAISE:** Стоимость вершины увеличилась (стала хуже)
- **LOWER:** Стоимость вершины уменьшилась (улучшилась)

**3. Локальность изменений:**
- При изменении стоимости одного ребра пересчитываются только затронутые вершины
- Не нужно перепланировать весь путь

#### **Псевдокод D*:**
```python
class DStar:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.open_list = PriorityQueue()
        self.cost = {}  # g-значения (стоимость до цели)
        self.backpointers = {}  # Родительские вершины
        
    def main(self):
        # Инициализация
        self.initialize()
        
        # Основной цикл
        while True:
            # Обработка изменений среды
            changes = self.detect_environment_changes()
            if changes:
                self.modify_costs(changes)
                self.process_state()  # Инкрементальное перепланирование
            
            # Движение робота
            if self.current != self.goal:
                next_node = self.backpointers[self.current]
                self.move_to(next_node)
                self.current = next_node
    
    def process_state(self):
        """Обработка вершины с наивысшим приоритетом"""
        X = open_list.pop()
        
        if X is None:
            return -1
        
        k_old = open_list.get_priority(X)
        
        if k_old < self.cost[X]:
            # RAISE состояние
            for neighbor in X.neighbors:
                # Пытаемся улучшить соседей
                new_cost = self.cost[neighbor] + cost(neighbor, X)
                if new_cost < self.cost[X]:
                    self.backpointers[X] = neighbor
                    self.cost[X] = new_cost
        elif k_old == self.cost[X]:
            # LOWER состояние
            for neighbor in X.neighbors:
                # Распространение улучшений
                if self.backpointers[neighbor] == X:
                    self.propagate_improvements(neighbor)
                else:
                    new_cost = self.cost[X] + cost(X, neighbor)
                    if new_cost < self.cost[neighbor]:
                        self.backpointers[neighbor] = X
                        self.cost[neighbor] = new_cost
                        open_list.insert(neighbor, new_cost)
```

### **D* Lite (упрощенная и более эффективная версия):**
**Разработан:** Sven Koenig и Maxim Likhachev, 2002

#### **Улучшения по сравнению с D*:**

**1. Более простая реализация:**
- Использует алгоритм, похожий на A*, но с инкрементальными обновлениями
- Меньше состояний, проще для понимания

**2. Эффективное обновление эвристики:**
```python
def calculate_key(node):
    """Ключ для приоритетной очереди в D* Lite"""
    g = cost_so_far[node]  # Текущая стоимость
    h = heuristic(node, start)  # Эвристика до старта (обратный поиск!)
    return (min(g, rhs(node)) + h, min(g, rhs(node)))

def rhs(node):
    """Right-hand side - одностep lookahead стоимость"""
    if node == goal:
        return 0
    # Минимальная стоимость через соседей
    return min(cost(node, neighbor) + cost_so_far[neighbor] 
               for neighbor in node.neighbors)
```

**3. Два основных этапа:**

**Этап A: Инициализация (похоже на A* но от цели)**
```python
def compute_shortest_path():
    while open_list.not_empty() and 
          (open_list.min_key() < calculate_key(start) or 
           rhs(start) != cost_so_far[start]):
        
        node = open_list.pop()
        
        if cost_so_far[node] > rhs(node):
            # Overconsistent - улучшаем вершину
            cost_so_far[node] = rhs(node)
            update_neighbors(node)
        else:
            # Underconsistent - ухудшаем вершину
            cost_so_far[node] = INF
            update_neighbors(node)
            update_node(node)
```

**Этап B: Инкрементальное обновление**
```python
def update_cell(changed_cell, new_cost):
    # Обновляем стоимость ребра
    old_cost = edge_costs[changed_cell]
    edge_costs[changed_cell] = new_cost
    
    # Если стоимость увеличилась (препятствие появилось)
    if new_cost > old_cost:
        # Помечаем вершины как needing update
        affected_nodes = get_affected_nodes(changed_cell)
        for node in affected_nodes:
            update_node(node)
    
    # Перепланирование
    compute_shortest_path()
```

### **Ключевые отличия D* от A*:**

| Характеристика | A* | D* / D* Lite |
|----------------|-----|--------------|
| **Направление поиска** | От старта к цели | От цели к старту (обратный) |
| **Реакция на изменения** | Полное перепланирование | Инкрементальное обновление |
| **Эффективность при изменениях** | O(n log n) каждый раз | O(k log n), где k ≪ n |
| **Использование эвристики** | Постоянная | Адаптивная, обновляется |
| **Память** | Только для текущего пути | Хранит всю карту стоимостей |
| **Реальное время** | Медленное при частых изменениях | Быстрое, локальные обновления |

### **Почему D* эффективен для неизвестных/изменяющихся сред:**

#### **1. Инкрементальность:**
```python
# Пример: Робот движется, появляется препятствие
# A*: перепланирует весь путь от текущей позиции
# D*: обновляет только вершины в области препятствия

def handle_new_obstacle(obstacle_cell):
    # D* Lite обновление
    update_vertex(obstacle_cell)
    for neighbor in obstacle_cell.neighbors:
        update_vertex(neighbor)
    compute_shortest_path()  # Быстро, т.к. мало изменений
```

#### **2. Локальность:**
- Изменение стоимости одного ребра влияет только на локальную область
- Не нужно пересчитывать весь граф

#### **3. Обратный поиск:**
- Поскольку планирование от цели, робот может начать движение сразу после частичного планирования
- По мере движения достраивает/уточняет путь

#### **4. Эффективность при неизвестной среде:**
```python
# Сценарий: исследование неизвестной территории
robot_path = []
while not at_goal:
    # Двигаемся по текущему лучшему пути
    move_one_step()
    
    # Сканируем новую область
    new_scan = lidar.scan()
    
    # Обновляем карту
    for cell in new_scan.obstacles:
        if cell not in known_map:
            update_cell(cell, INF)  # Новое препятствие
            # D* быстро перепланирует
    
    # Если путь заблокирован, D* найдет обход
```

### **Пример использования в робототехнике:**

#### **Mars Rover (реальный пример использования D*):**
```python
class MarsRoverNavigation:
    def __init__(self):
        self.planner = DStarLite()
        self.known_map = OccupancyGrid()
        
    def navigate_to(self, goal):
        # Начальное планирование с известной информацией
        self.planner.initialize(self.current_pose, goal, self.known_map)
        
        while not self.at_goal():
            # Получить следующий шаг
            next_step = self.planner.get_next_step()
            
            # Двигаться
            self.move(next_step)
            
            # Обновлять карту по мере движения
            new_obstacles = self.cameras.detect_obstacles()
            for obstacle in new_obstacles:
                # D* Lite быстро перепланирует
                self.planner.update_cost(obstacle.cell, INF)
                
            # Если обнаружили более короткий путь (например, проход)
            new_free_cells = self.cameras.detect_free_space()
            for cell in new_free_cells:
                self.planner.update_cost(cell, 1)  # Низкая стоимость
```

#### **Сравнительная эффективность:**
```
Среда: 100×100 grid, 10% препятствий, 1 изменение в секунду

A*:
- Первое планирование: 50 мс
- Каждое изменение: 50 мс
- Всего за 60 сек: 50 + 59×50 = 3000 мс простоя

D* Lite:
- Первое планирование: 80 мс (медленнее из-за инициализации)
- Каждое изменение: 5 мс (в среднем)
- Всего за 60 сек: 80 + 59×5 = 375 мс простоя
```

### **Ограничения D*:**
1. **Потребление памяти:** Хранит информацию о всех вершинах
2. **Сложность реализации:** Сложнее чем A*
3. **Не оптимален для полностью динамических сред:** Если меняется много ячеек одновременно

### **Модификации и улучшения:**
- **Field D*:** Для непрерывных пространств, плавные пути
- **Anytime D*:** Постепенное улучшение пути при наличии времени
- **Multi-resolution D*:** Иерархическое планирование для больших карт

---

## **Задача: Блок-схема алгоритма поведения для объезда внезапного препятствия с использованием DWA**

### **Контекст:**
- Робот движется по глобальному плану
- Внезапно появляется препятствие (динамическое или неучтенное)
- Используется DWA (Dynamic Window Approach) в ROS move_base

### **Блок-схема алгоритма:**

```
┌─────────────────────────────────────────────────────────────────────┐
│                    НАЧАЛО: Движение по глобальному плану            │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Цикл навигации (каждый шаг контроллера, обычно 5-10 Гц)            │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│  1. ПОЛУЧИТЬ ТЕКУЩЕЕ СОСТОЯНИЕ                                      │
│     - Поза робота из одометрии/локализации                          │
│     - Скорость (v, ω) из энкодеров                                  │
│     - Глобальный план от планировщика                               │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│  2. ПОЛУЧИТЬ ДАННЫЕ О ПРЕПЯТСТВИЯХ                                  │
│     - Локальная costmap из лидара/сонаров                           │
│     - Обнаружение внезапных препятствий                             │
│     - Динамические объекты из трекера                               │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│  3. ОБНАРУЖЕНИЕ ВНЕЗАПНОГО ПРЕПЯТСТВИЯ                              │
│     IF (новое препятствие на пути ИЛИ                               │
│         стоимость ячеек на пути > порога) THEN                      │
│         │  Препятствие обнаружено                                   │
│         ELSE                                                         │
│         │  Продолжить движение по плану                             │
│         │  ▼                                                        │
│         └──────────────────────┐                                    │
└──────────────────────────────┬─┼────────────────────────────────────┘
                               │ │
                    Обнаружено ▼ │ Не обнаружено
                               │ │
                               │ ▼
                ┌──────────────┴─┴─────────────────────────┐
                │                                          │
                ▼                                          ▼
┌──────────────────────────────┐           ┌──────────────────────────────┐
│  4. АКТИВАЦИЯ РЕЖИМА ОБЪЕЗДА │           │  7. ВЫБОР ОПТИМАЛЬНОЙ        │
│     - Увеличить occdist_scale│           │     ТРАЕКТОРИИ ИЗ ОКНА       │
│     - Уменьшить sim_time     │           │     - Оценить каждую траект. │
│     - Включить резерв.повед. │           │     - Выбрать лучшую по      │
│     - Обновить лок. costmap  │           │       взвеш. функции         │
└──────────────────────────────┘           └──────────────┬───────────────┘
                │                                          │
                ▼                                          │
┌──────────────────────────────┐                          │
│  5. ГЕНЕРАЦИЯ ДИНАМИЧЕСКОГО  │                          │
│     ОКНА СКОРОСТЕЙ (DWA)     │                          │
│     - Вычислить допустимые   │                          │
│       скорости (v, ω)        │                          │
│       с учетом ускорений     │                          │
│     - Создать окно:          │                          │
│       v_min ... v_max        │                          │
│       ω_min ... ω_max        │                          │
└──────────────────────────────┘                          │
                │                                          │
                ▼                                          │
┌──────────────────────────────┐                          │
│  6. СИМУЛЯЦИЯ ТРАЕКТОРИЙ     │──────────────────────────┘
│     Для каждой пары (v, ω):  │
│     - Просимулировать траект.│
│       на время sim_time      │
│     - Оценить:               │
│       ① Близость к цели      │
│       ② Следование глоб.плану│
│       ③ Расст. до препятствий│
│       ④ Скорость             │
└──────────────────────────────┘
                │
                ▼
┌──────────────────────────────┐
│  8. ИСПОЛНЕНИЕ КОМАНДЫ       │
│     - Отправить выбранные    │
│       (v_best, ω_best) в     │
│       /cmd_vel               │
│     - Обновить одометрию     │
└──────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────────────────┐
│  9. ПРОВЕРКА УСПЕШНОСТИ ОБЪЕЗДА                                      │
│     IF (препятствие осталось позади И                               │
│         робот вернулся на глобальный план) THEN                     │
│         │  Возврат к нормальному режиму                             │
│         │  ▼                                                        │
│         └──────────────────────┐                                    │
│     ELSE                        │                                    │
│         │  Продолжить объезд    │                                    │
│         ▼                       │                                    │
┌─────────┴──────────────────────┴──────────────────────────────────┐│
│  10. АКТИВАЦИЯ RECOVERY BEHAVIORS (при необходимости)             ││
│      IF (робот заблокирован > timeout) THEN                       ││
│      │  Очистить локальную costmap                                ││
│      │  Выполнить вращение на месте                               ││
│      │  Попробовать движение назад                                ││
│      └────────────────────────────────────────────────────────────┘│
│                                                                    │
└──────────────────────────────────┬─────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────────┐
│  КОНЕЦ ЦИКЛА → переход к шагу 1 для следующей итерации              │
└─────────────────────────────────────────────────────────────────────┘
```

### **Детализация ключевых блоков:**

#### **Блок 2: Получение данных о препятствиях**
```
┌─────────────────────────────────────┐
│        Локальная costmap            │
├─────────────────────────────────────┤
│ Источники:                          │
│ 1. Лидар (LaserScan → ObstacleLayer)│
│ 2. Сонары (Range → ObstacleLayer)   │
│ 3. Статическая карта (StaticLayer)  │
│ 4. Визуальное восприятие (камеры)   │
└─────────────────────────────────────┘
```

#### **Блок 3: Алгоритм обнаружения внезапного препятствия**
```python
def detect_sudden_obstacle(global_plan, local_costmap):
    """
    Обнаружение внезапных препятствий на пути
    """
    for waypoint in global_plan[0:lookahead_points]:
        # Преобразовать waypoint в координаты costmap
        mx, my = world_to_map(waypoint.pose.position, costmap.info)
        
        # Проверить стоимость ячейки
        cost = local_costmap.data[my * costmap.info.width + mx]
        
        if cost > OBSTACLE_THRESHOLD:
            # Препятствие обнаружено!
            return True, (mx, my), cost
    
    return False, None, 0

# Пороги (примерные значения)
FREE_THRESHOLD = 10      # Свободно
OBSTACLE_THRESHOLD = 50  # Возможно препятствие
LETHAL_THRESHOLD = 100   # Смертельное препятствие
```

#### **Блок 5-7: Ядро DWA алгоритма**
```python
class DWA_Obstacle_Avoidance:
    def dynamic_window(self, current_vel, dt=0.1):
        """
        Вычисление динамического окна скоростей
        """
        # Текущие скорости
        v_current, w_current = current_vel
        
        # Ограничения робота
        v_min, v_max = self.config['min_vel_x'], self.config['max_vel_x']
        w_min, w_max = self.config['min_vel_theta'], self.config['max_vel_theta']
        
        # Ограничения по ускорению
        v_acc = self.config['acc_lim_x'] * dt
        w_acc = self.config['acc_lim_theta'] * dt
        
        # Динамическое окно
        v_window = [
            max(v_min, v_current - v_acc),
            min(v_max, v_current + v_acc)
        ]
        w_window = [
            max(w_min, w_current - w_acc),
            min(w_max, w_current + w_acc)
        ]
        
        return v_window, w_window
    
    def simulate_trajectories(self, v_window, w_window, current_pose):
        """
        Симуляция траекторий для всех пар (v, ω)
        """
        trajectories = []
        
        # Дискретизация окна
        v_samples = linspace(v_window[0], v_window[1], self.config['vx_samples'])
        w_samples = linspace(w_window[0], w_window[1], self.config['vtheta_samples'])
        
        for v in v_samples:
            for w in w_samples:
                # Симуляция траектории
                traj = self.simulate_trajectory(
                    current_pose, v, w, 
                    self.config['sim_time'],
                    self.config['sim_granularity']
                )
                
                # Оценка траектории
                score = self.evaluate_trajectory(traj)
                trajectories.append((traj, v, w, score))
        
        return trajectories
    
    def evaluate_trajectory(self, trajectory):
        """
        Оценка траектории по 4 критериям
        """
        # 1. Heading: ориентация на цель
        heading_score = self.heading_cost(trajectory)
        
        # 2. Clearance: расстояние до препятствий
        clearance_score = self.clearance_cost(trajectory)
        
        # 3. Velocity: предпочтение более высоким скоростям
        velocity_score = self.velocity_cost(trajectory)
        
        # 4. Path: следование глобальному плану
        path_score = self.path_cost(trajectory)
        
        # Взвешенная сумма
        total_score = (
            self.config['heading_weight'] * heading_score +
            self.config['clearance_weight'] * clearance_score +
            self.config['velocity_weight'] * velocity_score +
            self.config['path_weight'] * path_score
        )
        
        return total_score
```

#### **Блок 9: Критерии успешного объезда**
```python
def is_obstacle_bypassed(robot_pose, obstacle_position, global_plan):
    """
    Проверка, успешно ли робот объехал препятствие
    """
    # 1. Препятствие должно быть позади робота
    robot_to_obstacle = obstacle_position - robot_pose.position
    robot_heading = get_yaw(robot_pose.orientation)
    
    # Угол между направлением робота и вектором к препятствию
    angle = angle_between(robot_heading, robot_to_obstacle)
    
    # Если препятствие сзади (угол > 90 градусов)
    obstacle_is_behind = abs(angle) > math.pi/2
    
    # 2. Робот должен вернуться на глобальный план
    distance_to_plan = distance_to_global_plan(robot_pose, global_plan)
    on_plan = distance_to_plan < ON_PLAN_THRESHOLD
    
    return obstacle_is_behind and on_plan
```

#### **Блок 10: Recovery Behaviors**
```python
def execute_recovery_behaviors(blocked_time):
    """
    Иерархия recovery behaviors при длительной блокировке
    """
    if blocked_time < 2.0:
        # Небольшая задержка - продолжаем пытаться
        return "CONTINUE"
    
    elif blocked_time < 5.0:
        # 1. Очистка costmap в локальной области
        clear_costmap_around_robot(radius=1.0)
        return "CLEAR_COSTMAP"
    
    elif blocked_time < 10.0:
        # 2. Вращение на месте для поиска выхода
        execute_in_place_rotation(angle=math.pi/2)
        return "ROTATE"
    
    elif blocked_time < 15.0:
        # 3. Движение назад
        move_backward(distance=0.5)
        return "MOVE_BACKWARD"
    
    else:
        # 4. Перепланирование глобального пути
        request_global_replanning()
        return "GLOBAL_REPLAN"
```

### **Параметры настройки для режима объезда:**

```yaml
# dwa_local_planner_params.yaml
# Нормальный режим:
DWAPlannerROS:
  max_vel_x: 0.5
  path_distance_bias: 32.0
  goal_distance_bias: 20.0
  occdist_scale: 0.01
  sim_time: 1.5

# Режим объезда препятствий (активируется при обнаружении):
ObstacleAvoidanceMode:
  max_vel_x: 0.3                    # Медленнее для безопасности
  path_distance_bias: 16.0          # Меньше следование плану
  goal_distance_bias: 10.0          # Меньше стремление к цели
  occdist_scale: 0.05               # Сильнее избегание препятствий
  sim_time: 2.0                     # Дальше смотрим вперед
  vx_samples: 12                    # Больше вариантов скоростей
  vtheta_samples: 40                # Больше вариантов поворотов
  oscillation_reset_dist: 0.3       # Больше допуск для осцилляций
```

### **Интеграция с ROS move_base:**

```xml
<!-- В launch-файле -->
<node pkg="move_base" type="move_base" name="move_base" output="screen">
  
  <!-- Параметры DWA -->
  <rosparam file="$(find my_robot)/config/dwa_local_planner_params.yaml" 
            command="load"/>
  
  <!-- Динамическая реконфигурация -->
  <node pkg="rqt_reconfigure" type="rqt_reconfigure" name="rqt_reconfigure"/>
  
</node>

<!-- Узел мониторинга препятствий -->
<node pkg="obstacle_monitor" type="obstacle_monitor_node" name="obstacle_monitor">
  <remap from="scan" to="/base_scan"/>
  <remap from="cmd_vel" to="/move_base/cmd_vel"/>
  <param name="obstacle_threshold" value="0.5"/>  # м
  <param name="reaction_distance" value="1.0"/>   # м
</node>
```

### **Визуализация в RViz для отладки:**

```
Дисплеи для мониторинга объезда препятствий:
1. LaserScan: topic=/scan (красные точки)
2. Local Costmap: topic=/move_base/local_costmap/costmap (тепловая карта)
3. Global Plan: topic=/move_base/GlobalPlanner/plan (синяя линия)
4. Local Plan: topic=/move_base/DWAPlannerROS/local_plan (зеленая линия)
5. Trajectories: topic=/move_base/DWAPlannerROS/trajectories (разноцветные)
6. Obstacle Markers: topic=/obstacle_monitor/markers (обнаруженные препятствия)
```

### **Последовательность действий при объезде:**

1. **Обнаружение:** Лидар видит препятствие на расстоянии 1.0 м
2. **Активация:** DWA переходит в режим объезда (более консервативные параметры)
3. **Генерация траекторий:** DWA оценивает варианты: объезд слева/справа, остановка
4. **Выбор:** Выбирается траектория с лучшим балансом безопасности и прогресса
5. **Исполнение:** Робот начинает маневр объезда
6. **Мониторинг:** Постоянная проверка, не появились ли новые препятствия
7. **Завершение:** Когда препятствие остается позади, возврат к нормальным параметрам

Этот алгоритм обеспечивает безопасный и плавный объезд внезапных препятствий, интегрируясь с существующей архитектурой ROS navigation stack.