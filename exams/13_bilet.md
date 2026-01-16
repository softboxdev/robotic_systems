# Разбор билета 13

---

## **Теория 1: Анализ требований и проектирование архитектуры ПО в жизненном цикле робототехнических систем**

### **Жизненный цикл ПО для робота:**

```
Анализ требований → Проектирование архитектуры → Реализация → Тестирование → Развертывание → Поддержка
```

### **1. Анализ требований**

**Что включает:**
- **Функциональные требования:** Что должен делать робот?
  - "Робот должен автономно патрулировать территорию"
  - "Должен избегать статических и динамических препятствий"
  - "Должен заряжаться самостоятельно при низком заряде"
- **Нефункциональные требования:**
  - **Производительность:** Частота обновления планировщика (10 Гц)
  - **Надежность:** Доступность 99.9%, время наработки на отказ
  - **Безопасность:** Аварийная остановка за < 0.1 сек
  - **Энергоэффективность:** Работа от батареи 8+ часов
- **Ограничения:**
  - Вычислительные ресурсы (CPU, RAM)
  - Сенсорные возможности
  - Бюджет проекта

**Методы анализа:**
- **Use-case диаграммы:** Сценарии использования
- **User stories:** С точки зрения конечного пользователя
- **Матрица трассируемости:** Связь требований с компонентами

### **2. Проектирование архитектуры**

**Уровни архитектуры:**

#### **А) Концептуальная архитектура (Logical View):**
```
[Восприятие] → [Принятие решений] → [Управление] → [Актуаторы]
```
- **Компоненты и их ответственность**
- **Потоки данных:** Кто что кому передает?
- **Интерфейсы:** Как компоненты взаимодействуют?

#### **Б) Физическая архитектура (Deployment View):**
- **Распределение по вычислительным узлам:**
  - NVIDIA Jetson: Компьютерное зрение
  - Raspberry Pi: Низкоуровневый контроль
  - Облако: Аналитика, ML-модели
- **Сеть и коммуникация:**
  - ROS over Wi-Fi/Ethernet
  - Real-time протоколы для критичных компонентов

#### **В) Процессная архитектура (Process View):**
- **Потоки выполнения (threads):**
  - Поток управления двигателями (высокий приоритет, real-time)
  - Поток планирования (средний приоритет)
  - Поток логирования (низкий приоритет)
- **Временные характеристики:**
  - Частота обновления контроллера: 100 Гц
  - Задержка реакции на препятствие: < 100 мс

### **Как выбор архитектуры на основе ROS влияет на последующие этапы:**

#### **Положительное влияние:**

**1. На этапе реализации:**
- **Повторное использование компонентов:** Готовые пакеты (navigation, perception)
- **Стандартизация:** Единые интерфейсы сообщений
- **Параллельная разработка:** Разные команды работают над разными нодами
- **Языковая независимость:** C++ для производительности, Python для прототипирования

**2. На этапе тестирования:**
- **Модульное тестирование:** Каждую ноду можно тестировать отдельно
- **Интеграционное тестирование:** `roslaunch` для запуска подсистем
- **Репродуцируемость:** `rosbag` для записи и воспроизведения данных
- **Симуляция:** Gazebo + ROS = тестирование без железа

**3. На этапе развертывания:**
- **Контейнеризация:** Docker + ROS = воспроизводимые среды
- **Управление зависимостями:** `rosdep` автоматически устанавливает зависимости
- **Конфигурационное управление:** Параметры через `rosparam`, конфиги в YAML

**4. На этапе поддержки:**
- **Диагностика:** `rqt_graph`, `rostopic`, `rosnode`
- **Мониторинг:** `ros_monitor`, `diagnostic_aggregator`
- **Обновление:** Обновлять можно отдельные пакеты

#### **Отрицательное влияние/ограничения:**

**1. Производительность:**
- **Накладные расходы:** Сериализация/десериализация сообщений
- **Задержки:** Не подходит для hard real-time (> 1 мс)
- **Решение:** Использовать ROS 2 с DDS, real-time расширения

**2. Сложность отладки распределенных систем:**
- **Распределенная отладка:** Трассировка запроса через несколько нод
- **Синхронизация времени:** NTP обязателен
- **Решение:** Использовать `ros_tracing`, логирование с correlation ID

**3. Безопасность:**
- **ROS 1:** Нет встроенной безопасности
- **Решение:** ROS 2 с DDS Security, сетевые firewall правила

**4. Ресурсоемкость:**
- **Память:** Каждая нода — отдельный процесс
- **Решение:** Component nodes в ROS 2, микросервисная архитектура

### **Пример проектирования для автономного погрузчика:**

#### **Требования:**
- Перевозить грузы до 100 кг
- Автономная навигация на складе 100×50 м
- Избегать людей и техники
- Работать 16 часов без подзарядки

#### **Архитектура на ROS:**

```
Физический слой:
- Лидар 2D (навигация) → /scan
- Камера глубины (обнаружение людей) → /rgbd
- IMU + одометрия → /odom
- Контроллер двигателей → /cmd_vel

Программный слой:
1. Драйверы (C++): lidar_node, camera_node, motor_driver
2. Восприятие (Python+OpenCV): people_detector, obstacle_detector
3. Навигация (C++): move_base (глобальный + локальный планировщик)
4. Управление заданиями (Python): task_manager (SMACH)
5. Мониторинг (Python): battery_monitor, health_checker

Коммуникация:
- Real-time: /cmd_vel, /emergency_stop (100 Гц)
- Near real-time: /scan, /odom (10-20 Гц)
- Асинхронно: /goals, /tasks (1 Гц)
```

#### **Влияние на жизненный цикл:**
- **Реализация:** 3 команды параллельно: драйверы, навигация, бизнес-логика
- **Тестирование:** Gazebo для алгоритмов, хардвар-in-the-loop для драйверов
- **Развертывание:** Docker образы для каждой подсистемы
- **Масштабирование:** Добавить робота = запустить еще одну копию ПО

---

## **Теория 2: Планирование пути в динамической среде**

### **Динамическая среда:**
Среда, где присутствуют **движущиеся препятствия** (люди, другие роботы, транспорт).

### **Подход 1: Реактивный репланинг (Reactive Replanning)**

**Суть:** "Вижу препятствие → перепланирую"

#### **Алгоритм:**
```
1. Имеем глобальный план (статический)
2. На каждом шаге:
  3. Проверяем локальную область на наличие динамических препятствий
  4. Если препятствие на пути:
    5. Запускаем локального планировщика для объезда
    6. После объезда возвращаемся к глобальному плану
```

#### **Реализация в ROS (DWA Planner):**
```python
# Локальный планировщик пересчитывает траекторию на каждом шаге
while not at_goal:
    # Получаем текущие препятствия из costmap
    obstacles = local_costmap.get_obstacles()
    
    # Генерируем траектории в пространстве скоростей
    trajectories = generate_trajectories(current_vel, obstacles)
    
    # Выбираем лучшую траекторию
    best_traj = evaluate_trajectories(trajectories, global_plan)
    
    # Отправляем команду
    publish_cmd_vel(best_traj.velocity)
```

#### **Преимущества:**
- **Простота реализации:** Стандартный подход в ROS navigation stack
- **Низкие вычислительные затраты:** Планируем только локально
- **Устойчивость к неожиданностям:** Реагирует на внезапные препятствия

#### **Недостатки:**
- **Короткий горизонт планирования:** Видит только ближайшие препятствия
- **"Туннельное зрение":** Может попасть в ловушки
- **Неэффективность:** Частые маневры, неоптимальные траектории
- **Осцилляции:** "Маятниковое" поведение при встречном движении

### **Подход 2: Предсказание траекторий (Trajectory Prediction)**

**Суть:** "Предсказываю движение препятствий → планирую с учетом будущего"

#### **Алгоритм:**
```
1. Обнаруживаем динамические объекты
2. Для каждого объекта:
  3. Оцениваем текущее состояние (позиция, скорость, ускорение)
  4. Строим вероятностную модель движения
  5. Предсказываем будущие позиции на горизонте T
3. Планируем путь, избегая предсказанных траекторий
```

#### **Методы предсказания:**

**1. Линейная экстраполяция (простая):**
```python
def predict_linear(obj, horizon, dt):
    """Предсказание с постоянной скоростью"""
    predictions = []
    for t in range(horizon):
        time = t * dt
        pos = obj.position + obj.velocity * time
        predictions.append(pos)
    return predictions
```

**2. Кинематические модели:**
- **Constant Velocity (CV):** `xₜ₊₁ = xₜ + vₜ·Δt`
- **Constant Acceleration (CA):** `xₜ₊₁ = xₜ + vₜ·Δt + ½a·Δt²`
- **Constant Turn Rate and Velocity (CTRV):** Для маневрирующих объектов

**3. Машинное обучение:**
- **LSTM:** Для пешеходов с социальным поведением
- **GAN:** Генерация множества возможных траекторий
- **Трансформеры:** Контекстно-зависимое предсказание

**4. Интерактивное предсказание:**
```python
def predict_interactive(robot, humans, horizon):
    """Учитывает взаимодействие робот-человек"""
    # Человек реагирует на движение робота
    human_pred = predict_human_trajectory(human, robot.planned_path)
    # Робот планирует с учетом реакции человека
    robot_plan = plan_with_prediction(human_pred)
    return robot_plan
```

#### **Реализация в ROS:**

**Архитектура:**
```
[Датчики] → [Трекинг объектов] → [Предсказание траекторий] → [Планировщик]
       ↓           ↓                   ↓                       ↓
   /scan      /detected_objects   /predicted_trajectories  /cmd_vel
   /camera    (custom_msg)        (custom_msg)
```

**Сообщение для предсказанных траекторий:**
```yaml
# predicted_trajectories.msg
Header header
PredictedTrajectory[] trajectories

PredictedTrajectory:
  uint32 object_id
  float32 confidence
  geometry_msgs/PoseWithCovariance[] predicted_poses
  geometry_msgs/TwistWithCovariance[] predicted_velocities
```

#### **Преимущества:**
- **Длинный горизонт планирования:** Избегаем "туннельного зрения"
- **Проактивное поведение:** Предугадываем конфликты
- **Более плавные траектории:** Меньше резких маневров
- **Социальная приемлемость:** Учитываем ожидания людей

#### **Недостатки:**
- **Вычислительная сложность:** Предсказание + планирование
- **Неточность предсказаний:** Модели неидеальны
- **Сложность реализации:** Нет стандартных решений в ROS
- **Переобучение на конкретные сценарии**

### **Сравнительная таблица:**

| Критерий | Реактивный репланинг | Предсказание траекторий |
|----------|----------------------|-------------------------|
| **Горизонт** | Короткий (1-2 сек) | Длинный (5-10 сек) |
| **Вычисления** | Низкие | Высокие |
| **Поведение** | Реактивное | Проактивное |
| **Устойчивость** | Высокая | Зависит от точности предсказаний |
| **Реализация** | Стандартная (ROS nav) | Кастомная |
| **Подходит для** | Простые динамические препятствия | Сложные взаимодействия |

### **Гибридные подходы:**

#### **1. Иерархическое планирование:**
```
Уровень 1: Глобальный планировщик с предсказанием (низкая частота)
Уровень 2: Локальный планировщик реактивный (высокая частота)
```

#### **2. Множество гипотез (Multiple Hypothesis):**
```python
# Генерируем несколько вариантов предсказаний
predictions = generate_multiple_hypotheses(objects)

# Для каждой гипотезы планируем
plans = []
for pred in predictions:
    plan = plan_with_prediction(pred)
    plans.append((plan, pred.confidence))

# Выбираем наиболее надежный план
best_plan = select_best_plan(plans)
```

#### **3. Model Predictive Control (MPC):**
```
minimize J(x,u) = cost_to_goal + comfort_penalty + safety_margin
subject to:
    x_{t+1} = f(x_t, u_t)  # Динамика робота
    g(x_t) ≤ 0            # Ограничения (препятствия)
    h(x_t, o_t) ≥ d_safe  # Безопасное расстояние
```
где `o_t` — предсказанные позиции объектов.

### **Практические рекомендации:**

**Использовать реактивный подход когда:**
- Препятствия движутся медленно/предсказуемо
- Вычислительные ресурсы ограничены
- Среда относительно простая

**Использовать предсказание когда:**
- Работаем с людьми (непредсказуемые)
- Высокие скорости движения
- Критична плавность и предсказуемость траектории
- Есть вычислительные ресурсы

---

## **Задача: Чек-лист диагностики неподвижного робота в Gazebo**

### **Общая схема диагностики:**
```
[Алгоритм] → [ROS] → [Контроллер Gazebo] → [Физическая модель] → [Движение]
```

### **Чек-лист по категориям:**

#### **Категория 1: TF (Transformation) проблемы**

**Признаки:** Робот не понимает где он, цели в неправильных координатах

**Диагностика:**
```bash
# 1. Проверить наличие всех TF фреймов
rosrun tf view_frames  # Создает PDF с графом трансформ

# 2. Проверить конкретные трансформы
rosrun tf tf_echo map base_link
# Должно быть: transform: (x, y, z), rotation: (x, y, z, w)
# Если "No transform available" → проблема с локализацией

# 3. Мониторить TF в реальном времени
rosrun tf tf_monitor map base_link

# 4. Проверить частоту обновления TF
rostopic hz /tf
# Должно быть ≥ 10 Гц

# 5. Визуализировать TF в RViz
# Добавить display "TF", должны видеть все оси
```

**Распространенные проблемы:**
- **Отсутствует `odom` → `base_link`:** AMCL не запущен или не публикует
- **Статические трансформы не настроены:** Лидар/камера не привязаны к роботу
- **Частота:** Контроллеры требуют стабильных трансформ

#### **Категория 2: Move_base и планирование**

**Признаки:** `/move_base/status` показывает активную цель, но нет команд скорости

**Диагностика:**
```bash
# 1. Проверить статус move_base
rostopic echo /move_base/status
# Смотреть status: 1=ACTIVE, 3=SUCCEEDED, 4=ABORTED

# 2. Проверить публикует ли планировщик команды
rostopic echo /move_base/DWAPlannerROS/cmd_vel
# ИЛИ для другого планировщика
rostopic echo /cmd_vel

# 3. Проверить глобальный план
rostopic echo /move_base/GlobalPlanner/plan
# Путь должен быть от текущей позиции к цели

# 4. Проверить локальный план
rostopic echo /move_base/DWAPlannerROS/local_plan

# 5. Проверить costmaps
rostopic echo /move_base/global_costmap/costmap
rostopic echo /move_base/local_costmap/costmap
```

**Распространенные проблемы:**
- **Локальный минимум:** Робот "застрял", force re-planning
- **Costmap пустой:** Препятствия не видны
- **Цель недостижима:** Проверить allow_unknown в планировщике
- **Параметры DWA слишком консервативные:** Уменьшить occdist_scale

#### **Категория 3: URDF и контроллеры Gazebo**

**Признаки:** Команды `/cmd_vel` публикуются, но робот не движется в Gazebo

**Диагностика:**
```bash
# 1. Проверить контроллеры в Gazebo
rosservice call /gazebo/get_joint_properties '{joint_name: "left_wheel_joint"}'
# Должны видеть velocity ≠ 0

# 2. Проверить публикуется ли odometry
rostopic echo /odom
# position должно меняться

# 3. Проверить топик команд
rostopic echo /cmd_vel
# linear.x и angular.z должны быть ненулевые

# 4. Проверить наличие контроллера
rosservice call /controller_manager/list_controllers
# Должен быть активен diff_drive_controller или аналогичный

# 5. Проверить настройки контроллера
rosparam get /diff_drive_controller
```

**Критичные места в URDF:**
```xml
<!-- 1. Правильные имена суставов -->
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <axis xyz="0 1 0"/>  <!-- Ось вращения -->
</joint>

<!-- 2. Transmission для Gazebo -->
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<!-- 3. Gazebo ROS control plugin -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

#### **Категория 4: Контроллеры ROS control**

**Признаки:** Контроллеры есть, но не реагируют на команды

**Диагностика:**
```bash
# 1. Проверить состояние контроллера
rosservice call /controller_manager/switch_controller \
  "start_controllers: ['diff_drive_controller'] \
   stop_controllers: [] \
   strictness: 2"

# 2. Перезапустить контроллер если завис
rosservice call /controller_manager/load_controller \
  "{name: 'diff_drive_controller'}"

# 3. Проверить публикует ли контроллер команды в Gazebo
rostopic echo /left_wheel_velocity_controller/command
rostopic echo /right_wheel_velocity_controller/command

# 4. Проверить параметры контроллера
rosparam get /diff_drive_controller/wheel_radius           # Должен совпадать с URDF
rosparam get /diff_drive_controller/wheel_separation       # Расстояние между колесами
rosparam get /diff_drive_controller/pose_covariance_diagonal  # Для одометрии
```

**Конфигурация YAML для контроллера:**
```yaml
# diff_drive_controller.yaml
diff_drive_controller:
  type: "diff_drive_controller/DiffDriveController"
  left_wheel: ['left_wheel_joint']
  right_wheel: ['right_wheel_joint']
  
  wheel_separation: 0.5      # Должно совпадать с URDF!
  wheel_radius: 0.1         # Должно совпадать с URDF!
  
  pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
  twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
  
  publish_rate: 50.0        # Частота публикации odom
  odom_frame_id: odom
  base_frame_id: base_footprint
  
  # Ограничения скорости
  linear:
    x:
      has_velocity_limits: true
      max_velocity: 1.0
      min_velocity: -1.0
      has_acceleration_limits: true
      max_acceleration: 0.5
  angular:
    z:
      has_velocity_limits: true
      max_velocity: 1.0
      has_acceleration_limits: true
      max_acceleration: 0.5
```

#### **Категория 5: Физика Gazebo**

**Признаки:** Колеса крутятся, но робот не едет

**Диагностика:**
```bash
# 1. Проверить контакт колес с поверхностью
# В Gazebo GUI: View -> Contacts
# Должны видеть контакты между wheels и ground_plane

# 2. Проверить физические параметры колес в URDF
<gazebo reference="left_wheel">
  <mu1>1.0</mu1>      # Коэффициент трения
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>  # Жесткость пружины контакта
  <kd>1.0</kd>        # Демпфирование
</gazebo>

# 3. Проверить массу и инерцию
<inertial>
  <mass value="5.0"/>  # Колесо не должно быть слишком легким
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

**Распространенные проблемы:**
- **Слишком низкий mu1/mu2:** Колеса проскальзывают
- **Слишком низкая масса:** Недостаточное сцепление
- **Неправильный axis в joint:** Колесо вращается вокруг неправильной оси

#### **Категория 6: Локализация (AMCL)**

**Признаки:** Робот "не знает где он", цели в странных местах

**Диагностика:**
```bash
# 1. Проверить публикует ли AMCL позу
rostopic echo /amcl_pose
# Должны видеть разумные координаты в системе map

# 2. Проверить TF от AMCL
rosrun tf tf_echo map odom
# Должна быть трансформа

# 3. Проверить параметры AMCL
rosparam get /amcl
# min_particles, max_particles, update_min_d, update_min_a

# 4. Проверить входные данные AMCL
rostopic echo /scan  # Лидар данные
rostopic echo /initialpose  # Начальная поза задана?
```

#### **Полный диагностический скрипт:**

```bash
#!/bin/bash
# diagnostic_robot.sh

echo "=== 1. Проверка ROS мастер ==="
roscore_running=$(ps aux | grep roscore | grep -v grep | wc -l)
if [ $roscore_running -eq 0 ]; then
    echo "❌ ROS мастер не запущен"
else
    echo "✓ ROS мастер запущен"
fi

echo -e "\n=== 2. Проверка TF дерева ==="
rosrun tf view_frames 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ TF дерево сгенерировано, смотрите frames.pdf"
    
    # Проверка критичных трансформ
    echo "Проверяем критические трансформы..."
    timeout 2 rosrun tf tf_echo map base_link 2>&1 | grep -q "Translation"
    if [ $? -eq 0 ]; then
        echo "✓ Трансформ map->base_link существует"
    else
        echo "❌ Нет трансформа map->base_link"
    fi
else
    echo "❌ Ошибка генерации TF дерева"
fi

echo -e "\n=== 3. Проверка move_base ==="
rostopic list | grep -q "/move_base/status"
if [ $? -eq 0 ]; then
    echo "✓ move_base запущен"
    # Смотрим статус последней цели
    echo "Статус move_base:"
    rostopic echo -n 1 /move_base/status/status_list 2>/dev/null | grep status | tail -1
else
    echo "❌ move_base не запущен"
fi

echo -e "\n=== 4. Проверка команд скорости ==="
rostopic hz /cmd_vel 2>/dev/null &
CMD_VEL_PID=$!
sleep 2
kill $CMD_VEL_PID 2>/dev/null

echo -e "\n=== 5. Проверка одометрии ==="
rostopic echo -n 1 /odom/pose/pose/position 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ Одометрия публикуется"
else
    echo "❌ Нет одометрии"
fi

echo -e "\n=== 6. Проверка контроллеров ROS control ==="
rosservice call /controller_manager/list_controllers 2>/dev/null | grep -A5 "diff_drive"
if [ $? -eq 0 ]; then
    echo "✓ Контроллер дифференциального привода найден"
else
    echo "❌ Контроллер не найден"
fi

echo -e "\n=== 7. Проверка лидара/сканов ==="
rostopic hz /scan 2>/dev/null &
SCAN_PID=$!
sleep 2
kill $SCAN_PID 2>/dev/null

echo -e "\n=== 8. Проверка RViz (если запущен) ==="
ps aux | grep -q "rviz"
if [ $? -eq 0 ]; then
    echo "✓ RViz запущен"
    echo "Проверьте в RViz:"
    echo "  1. TF отображаются правильно?"
    echo "  2. Карта загружена?"
    echo "  3. Робот виден в правильном месте?"
    echo "  4. Лазерные сканы отображаются?"
else
    echo "ℹ RViz не запущен, рекомендуется запустить для визуальной диагностики"
fi

echo -e "\n=== Диагностика завершена ==="
```

### **Методика систематического поиска проблемы:**

**Шаг 1: Проверить самый низкий уровень**
```bash
# Команды доходят до Gazebo?
rostopic echo /left_wheel_velocity_controller/command
```

**Шаг 2: Если нет, поднимаемся выше**
```bash
# Контроллеры работают?
rosservice call /controller_manager/list_controllers
```

**Шаг 3: Если контроллеры есть, но команд нет**
```bash
# Move_base публикует команды?
rostopic echo /cmd_vel
```

**Шаг 4: Если move_base не публикует**
```bash
# Планировщик находит путь?
rostopic echo /move_base/DWAPlannerROS/local_plan
```

**Шаг 5: Если планировщик не работает**
```bash
# Локализация работает?
rostopic echo /amcl_pose
rosrun tf tf_echo map base_link
```

**Шаг 6: Если локализация не работает**
```bash
# Лидар работает? Карта загружена?
rostopic echo /scan | head -5
rosparam get /map_server/topic
```

### **Быстрые исправления (quick fixes):**

1. **Перезапустить move_base:**
   ```bash
   rosnode kill /move_base
   roslaunch my_robot move_base.launch
   ```

2. **Сбросить локализацию:**
   ```bash
   # В RViz: 2D Pose Estimate
   # ИЛИ через командную строку
   rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped ...
   ```

3. **Перезапустить контроллеры:**
   ```bash
   rosservice call /controller_manager/switch_controller \
     "start_controllers: []
      stop_controllers: ['diff_drive_controller']
      strictness: 2"
   rosservice call /controller_manager/switch_controller \
     "start_controllers: ['diff_drive_controller']
      stop_controllers: []
      strictness: 2"
   ```

4. **Проверить параметры физики Gazebo:**
   ```xml
   <!-- В URDF, увеличить трение -->
   <gazebo reference="wheel">
     <mu1>10.0</mu1>
     <mu2>10.0</mu2>
   </gazebo>
   ```

### **Профилактика проблем:**

1. **Использовать готовые шаблоны URDF** (turtlebot, pioneer)
2. **Тестировать по отдельности:**
   - Сначала контроллеры с ручным управлением
   - Потом move_base с заданными целями
   - Потом полный стек
3. **Логировать всё:**
   ```bash
   rosbag record -a -O diagnosis.bag
   ```
4. **Использовать симуляцию с разной сложностью:**
   - Пустая комната → простой лабиринт → динамическая среда

**Золотое правило:** Всегда тестируйте компоненты по отдельности перед интеграцией!