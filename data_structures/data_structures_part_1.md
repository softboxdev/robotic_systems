# Руководство по структурам данных в Python

## Оглавление
1. [Введение](#введение)
2. [Базовые структуры данных](#базовые-структуры-данных)
3. [Списки (Lists)](#списки-lists)
4. [Кортежи (Tuples)](#кортежи-tuples)
5. [Словари (Dictionaries)](#словари-dictionaries)
6. [Множества (Sets)](#множества-sets)
7. [Строки (Strings)](#строки-strings)
8. [Коллекции из модуля collections](#коллекции-из-модуля-collections)
9. [Практические примеры](#практические-примеры)

## Введение

Структуры данных - это способы организации и хранения данных для эффективного доступа и модификации. Python предоставляет богатый набор встроенных структур данных.

## Базовые структуры данных

### Списки (Lists)
```python
# Создание списка
my_list = [1, 2, 3, 4, 5]
mixed_list = [1, "hello", 3.14, True]

# Основные операции
my_list.append(6)           # Добавление элемента
my_list.insert(0, 0)        # Вставка по индексу
my_list.remove(3)           # Удаление элемента
popped = my_list.pop()      # Удаление и возврат последнего элемента
length = len(my_list)       # Длина списка

# Срезы (slicing)
sublist = my_list[1:4]      # Элементы с индексом 1, 2, 3
reversed_list = my_list[::-1] # Разворот списка

# Списковые включения (list comprehensions)
squares = [x**2 for x in range(10)]
even_squares = [x**2 for x in range(10) if x % 2 == 0]
```

### Кортежи (Tuples)
```python
# Создание кортежа
my_tuple = (1, 2, 3, "hello")
single_element = (42,)      # Кортеж с одним элементом

# Особенности
# Кортежи неизменяемы
# my_tuple[0] = 10  # Вызовет ошибку

# Распаковка кортежа
a, b, c, d = my_tuple

# Использование в функциях
def get_coordinates():
    return 10, 20

x, y = get_coordinates()
```

### Словари (Dictionaries)
```python
# Создание словаря
my_dict = {"name": "Alice", "age": 30, "city": "New York"}
empty_dict = {}
dict_from_list = dict([("a", 1), ("b", 2)])

# Основные операции
my_dict["email"] = "alice@example.com"  # Добавление
value = my_dict["name"]                 # Получение значения
age = my_dict.get("age")                # Безопасное получение
removed = my_dict.pop("city")           # Удаление

# Итерация по словарю
for key in my_dict:
    print(key, my_dict[key])

for key, value in my_dict.items():
    print(f"{key}: {value}")

# Словарные включения
squares_dict = {x: x**2 for x in range(5)}
```

### Множества (Sets)
```python
# Создание множества
my_set = {1, 2, 3, 4, 5}
empty_set = set()
from_list = set([1, 2, 2, 3, 4])  # {1, 2, 3, 4}

# Основные операции
my_set.add(6)           # Добавление элемента
my_set.remove(3)        # Удаление элемента
my_set.discard(10)      # Удаление без ошибки, если элемента нет

# Операции с множествами
set1 = {1, 2, 3}
set2 = {3, 4, 5}

union = set1 | set2        # Объединение: {1, 2, 3, 4, 5}
intersection = set1 & set2 # Пересечение: {3}
difference = set1 - set2   # Разность: {1, 2}
symmetric_diff = set1 ^ set2 # Симметрическая разность: {1, 2, 4, 5}

# Множественные включения
even_squares = {x**2 for x in range(10) if x % 2 == 0}
```

### Строки (Strings)
```python
# Создание и базовые операции
my_string = "Hello, World!"
multiline = """Это
многострочная
строка"""

# Методы строк
lowercase = my_string.lower()
uppercase = my_string.upper()
words = my_string.split(",")        # Разделение по разделителю
joined = "-".join(words)           # Объединение
stripped = "  hello  ".strip()     # Удаление пробелов
replaced = my_string.replace("World", "Python")

# Проверки
is_alpha = "abc".isalpha()
is_digit = "123".isdigit()
starts_with = my_string.startswith("Hello")

# Форматирование
name = "Alice"
age = 30
formatted = f"Name: {name}, Age: {age}"
old_style = "Name: {}, Age: {}".format(name, age)
```

## Коллекции из модуля collections

```python
from collections import defaultdict, Counter, deque, namedtuple

# defaultdict - словарь с значением по умолчанию
dd = defaultdict(int)
dd['a'] += 1  # Не вызовет KeyError

# Counter - подсчет элементов
words = ['apple', 'banana', 'apple', 'orange', 'banana', 'apple']
word_count = Counter(words)
print(word_count.most_common(2))  # [('apple', 3), ('banana', 2)]

# deque - двусторонняя очередь
dq = deque([1, 2, 3])
dq.append(4)           # Добавление справа
dq.appendleft(0)       # Добавление слева
right = dq.pop()       # Удаление справа
left = dq.popleft()    # Удаление слева

# namedtuple - именованный кортеж
Point = namedtuple('Point', ['x', 'y'])
p = Point(10, 20)
print(p.x, p.y)  # 10 20
```

## Практические примеры

### Пример 1: Анализ текста
```python
def analyze_text(text):
    """Анализирует текст и возвращает статистику"""
    words = text.lower().split()
    word_count = Counter(words)
    
    return {
        'total_words': len(words),
        'unique_words': len(word_count),
        'most_common': word_count.most_common(5),
        'word_frequency': dict(word_count)
    }

text = "hello world hello python world programming python"
stats = analyze_text(text)
print(stats)
```

### Пример 2: Управление задачами
```python
class TaskManager:
    def __init__(self):
        self.tasks = []
        self.completed = set()
    
    def add_task(self, task, priority=1):
        self.tasks.append({'task': task, 'priority': priority})
        self.tasks.sort(key=lambda x: x['priority'])
    
    def complete_task(self, task):
        if task in [t['task'] for t in self.tasks]:
            self.completed.add(task)
            self.tasks = [t for t in self.tasks if t['task'] != task]
    
    def get_pending_tasks(self):
        return [t['task'] for t in self.tasks]

# Использование
manager = TaskManager()
manager.add_task("Купить продукты", 2)
manager.add_task("Сделать домашку", 1)
manager.complete_task("Сделать домашку")
print(manager.get_pending_tasks())
```

### Пример 3: Кэширование результатов
```python
from functools import lru_cache

@lru_cache(maxsize=128)
def fibonacci(n):
    """Вычисление чисел Фибоначчи с кэшированием"""
    if n < 2:
        return n
    return fibonacci(n-1) + fibonacci(n-2)

# Без декоратора вычисления были бы значительно медленнее
print(fibonacci(50))
```

## Заключение

Python предоставляет мощные и гибкие структуры данных, которые покрывают большинство потребностей программирования. Ключевые моменты:

- **Списки** для упорядоченных изменяемых коллекций
- **Кортежи** для неизменяемых последовательностей
- **Словари** для хранения пар ключ-значение
- **Множества** для уникальных элементов и операций с множествами
- **Строки** для работы с текстом

Используйте подходящую структуру данных для каждой задачи, и ваш код станет более эффективным и читаемым.

