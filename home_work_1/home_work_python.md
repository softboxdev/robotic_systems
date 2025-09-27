
## 🐍 Data Types (Типы данных)

### Задача 1: Анализ данных пользователя
```python
def analyze_user_data():
    """
    Задача: Проанализировать данные пользователя и определить типы данных
    """
    # Данные пользователя (имитация ввода)
    name = "Анна Петрова"          # str - строка, текстовые данные
    age = 25                       # int - целое число
    height = 1.75                  # float - число с плавающей точкой
    is_student = True              # bool - логическое значение (True/False)
    courses = ["Math", "Physics"]  # list - список, изменяемая последовательность
    
    # Выводим типы данных
    print("Типы данных переменных:")
    print(f"name: {type(name)}")        # <class 'str'>
    print(f"age: {type(age)}")          # <class 'int'>
    print(f"height: {type(height)}")    # <class 'float'>
    print(f"is_student: {type(is_student)}")  # <class 'bool'>
    print(f"courses: {type(courses)}")  # <class 'list'>
    
    # Преобразование типов
    age_str = str(age)              # Конвертируем int в str
    height_int = int(height)        # Конвертируем float в int (отбрасывает дробную часть)
    
    print(f"\nПреобразованные данные:")
    print(f"age как строка: '{age_str}'")
    print(f"height как целое: {height_int}")

# analyze_user_data()
```

### Задача 2: Работа со строками
```python
def string_operations():
    """
    Задача: Продемонстрировать основные операции со строками
    """
    text = "Python Programming Language"
    
    # Методы работы со строками
    print("Оригинальная строка:", text)
    print("В верхнем регистре:", text.upper())      # PYTHON PROGRAMMING LANGUAGE
    print("В нижнем регистре:", text.lower())       # python programming language
    print("Заглавные буквы:", text.title())         # Python Programming Language
    print("Длина строки:", len(text))               # 26 символов
    
    # Срезы (slicing)
    print("Первые 6 символов:", text[:6])           # Python
    print("С 7 по 18 символ:", text[7:18])          # Programming
    print("Последние 8 символов:", text[-8:])       # Language
    
    # Поиск и замена
    print("Начинается с 'Python':", text.startswith("Python"))  # True
    print("Содержит 'Java':", "Java" in text)                   # False
    print("Замена 'Programming' на 'Scripting':", 
          text.replace("Programming", "Scripting"))

# string_operations()
```

### Задача 3: Операции со списками
```python
def list_operations():
    """
    Задача: Показать основные операции со списками
    """
    # Создание списка
    fruits = ["apple", "banana", "orange"]  # list - упорядоченная коллекция
    numbers = [1, 2, 3, 4, 5]              # Список чисел
    
    print("Исходный список фруктов:", fruits)
    
    # Добавление элементов
    fruits.append("grape")           # Добавить в конец
    fruits.insert(1, "kiwi")         # Вставить на позицию 1
    print("После добавления:", fruits)
    
    # Удаление элементов
    removed_fruit = fruits.pop(2)    # Удалить элемент с индексом 2
    print(f"Удален фрукт: {removed_fruit}")
    print("После удаления:", fruits)
    
    # Сортировка
    fruits.sort()                    # Сортировка по алфавиту
    print("Отсортированный список:", fruits)
    
    # Объединение списков
    combined = fruits + numbers
    print("Объединенный список:", combined)

# list_operations()
```

### Задача 4: Словари (dictionaries)
```python
def dictionary_operations():
    """
    Задача: Работа со словарями - структурами ключ-значение
    """
    # Создание словаря с информацией о студенте
    student = {
        "name": "Иван Иванов",      # ключ: значение
        "age": 20,                  # int как значение
        "grades": [4, 5, 4, 5],     # list как значение
        "is_active": True           # bool как значение
    }
    
    print("Словарь студента:", student)
    
    # Доступ к значениям по ключам
    print("Имя студента:", student["name"])        # Иван Иванов
    print("Возраст:", student.get("age"))          # 20
    
    # Добавление новой пары ключ-значение
    student["faculty"] = "Computer Science"
    print("После добавления факультета:", student)
    
    # Перебор словаря
    print("\nИнформация о студенте:")
    for key, value in student.items():
        print(f"{key}: {value}")
    
    # Проверка существования ключа
    print("Есть ли ключ 'email':", "email" in student)  # False

# dictionary_operations()
```

### Задача 5: Кортежи и множества
```python
def tuple_set_operations():
    """
    Задача: Показать разницу между кортежами и множествами
    """
    # Кортеж (tuple) - неизменяемый упорядоченный набор
    coordinates = (10, 20)           # Координаты x, y
    rgb_colors = (255, 128, 0)       # Цвет в формате RGB
    
    print("Координаты:", coordinates)
    print("RGB цвета:", rgb_colors)
    
    # Множество (set) - неупорядоченная коллекция уникальных элементов
    unique_numbers = {1, 2, 3, 2, 1, 4, 5}  # Дубликаты автоматически удаляются
    print("Уникальные числа:", unique_numbers)  # {1, 2, 3, 4, 5}
    
    # Операции с множествами
    set_a = {1, 2, 3, 4}
    set_b = {3, 4, 5, 6}
    
    print("Объединение:", set_a | set_b)        # {1, 2, 3, 4, 5, 6}
    print("Пересечение:", set_a & set_b)        # {3, 4}
    print("Разность A-B:", set_a - set_b)       # {1, 2}
    print("Симметричная разность:", set_a ^ set_b)  # {1, 2, 5, 6}

# tuple_set_operations()
```

## 🔢 Quiz: Data Types, Numbers, Boolean

### Задача 1: Калькулятор BMI
```python
def calculate_bmi():
    """
    Задача: Рассчитать индекс массы тела (BMI) с проверкой типов данных
    """
    try:
        # Ввод данных с преобразованием типов
        weight = float(input("Введите вес в кг: "))    # float для точности
        height = float(input("Введите рост в метрах: "))
        
        # Проверка валидности данных
        if weight <= 0 or height <= 0:
            print("Ошибка: вес и рост должны быть положительными числами")
            return
        
        # Расчет BMI
        bmi = weight / (height ** 2)  # Формула: вес / рост²
        
        # Логические проверки
        is_underweight = bmi < 18.5
        is_normal = 18.5 <= bmi < 25
        is_overweight = 25 <= bmi < 30
        is_obese = bmi >= 30
        
        print(f"\nВаш BMI: {bmi:.2f}")  # :.2f - округление до 2 знаков после запятой
        print("Диагноз:")
        print(f"Недостаточный вес: {is_underweight}")
        print(f"Нормальный вес: {is_normal}")
        print(f"Избыточный вес: {is_overweight}")
        print(f"Ожирение: {is_obese}")
        
    except ValueError:
        print("Ошибка: введите числовые значения!")

# calculate_bmi()
```

### Задача 2: Конвертер валют
```python
def currency_converter():
    """
    Задача: Конвертировать валюту с использованием разных числовых операций
    """
    # Курсы валют (float для точности)
    usd_to_rub = 91.50    # 1 USD = 91.50 RUB
    eur_to_rub = 98.25    # 1 EUR = 98.25 RUB
    
    print("Конвертер валют")
    print("1. USD to RUB")
    print("2. EUR to RUB")
    print("3. RUB to USD")
    print("4. RUB to EUR")
    
    choice = input("Выберите операцию (1-4): ")
    amount = float(input("Введите сумму: "))
    
    # Проверка на положительное число
    if amount <= 0:
        print("Сумма должна быть положительной!")
        return
    
    # Выполнение конвертации
    if choice == "1":
        result = amount * usd_to_rub
        print(f"{amount} USD = {result:.2f} RUB")  # Округление до копеек
    elif choice == "2":
        result = amount * eur_to_rub
        print(f"{amount} EUR = {result:.2f} RUB")
    elif choice == "3":
        result = amount / usd_to_rub
        print(f"{amount} RUB = {result:.2f} USD")
    elif choice == "4":
        result = amount / eur_to_rub
        print(f"{amount} RUB = {result:.2f} EUR")
    else:
        print("Неверный выбор операции!")

# currency_converter()
```

### Задача 3: Проверка простого числа
```python
def is_prime_number():
    """
    Задача: Проверить, является ли число простым
    """
    try:
        number = int(input("Введите целое число: "))
        
        # Проверка граничных случаев
        if number < 2:
            print(f"{number} не является простым числом")
            return
        
        # Проверка на простоту
        is_prime = True  # Булева переменная-флаг
        
        for i in range(2, int(number ** 0.5) + 1):  # Проверяем до корня из числа
            if number % i == 0:  # Если делится без остатка
                is_prime = False
                break
        
        # Вывод результата с использованием булевых значений
        print(f"Число {number} является простым: {is_prime}")
        
        # Дополнительная информация
        if is_prime:
            print("Простое число делится только на 1 и на себя")
        else:
            print("Составное число имеет делители кроме 1 и себя")
            
    except ValueError:
        print("Ошибка: введите целое число!")

# is_prime_number()
```

### Задача 4: Битовые операции
```python
def bitwise_operations():
    """
    Задача: Показать работу с битовыми операциями
    """
    a = 10  # 1010 в двоичной системе
    b = 6   # 0110 в двоичной системе
    
    print("Битовые операции:")
    print(f"a = {a} (binary: {bin(a)})")
    print(f"b = {b} (binary: {bin(b)})")
    
    # Битовые операции
    and_result = a & b   # 1010 & 0110 = 0010 (2)
    or_result = a | b    # 1010 | 0110 = 1110 (14)
    xor_result = a ^ b   # 1010 ^ 0110 = 1100 (12)
    not_result = ~a      # Инверсия битов
    shift_left = a << 1  # 1010 << 1 = 10100 (20)
    shift_right = a >> 1 # 1010 >> 1 = 0101 (5)
    
    print(f"a AND b: {and_result} ({bin(and_result)})")
    print(f"a OR b: {or_result} ({bin(or_result)})")
    print(f"a XOR b: {xor_result} ({bin(xor_result)})")
    print(f"NOT a: {not_result} ({bin(not_result)})")
    print(f"a << 1: {shift_left} ({bin(shift_left)})")
    print(f"a >> 1: {shift_right} ({bin(shift_right)})")

# bitwise_operations()
```

### Задача 5: Логические операции
```python
def logical_operations():
    """
    Задача: Демонстрация логических операций и таблиц истинности
    """
    # Тестовые булевы значения
    x = True
    y = False
    
    print("Логические операции:")
    print(f"x = {x}, y = {y}")
    print()
    
    # Таблица истинности для AND
    print("Логическое И (AND):")
    print(f"x AND y: {x and y}")    # True AND False = False
    print(f"x AND x: {x and x}")    # True AND True = True
    print(f"y AND y: {y and y}")    # False AND False = False
    
    print("\nЛогическое ИЛИ (OR):")
    print(f"x OR y: {x or y}")      # True OR False = True
    print(f"x OR x: {x or x}")      # True OR True = True
    print(f"y OR y: {y or y}")      # False OR False = False
    
    print("\nЛогическое НЕ (NOT):")
    print(f"NOT x: {not x}")        # NOT True = False
    print(f"NOT y: {not y}")        # NOT False = True
    
    # Практический пример
    age = 25
    has_license = True
    has_experience = False
    
    can_drive = age >= 18 and has_license
    needs_supervisor = age >= 16 and age < 18 and has_license
    cannot_drive = age < 16 or not has_license
    
    print(f"\nПрактический пример (возраст: {age}, права: {has_license}):")
    print(f"Может водить: {can_drive}")
    print(f"Нужен сопровождающий: {needs_supervisor}")
    print(f"Не может водить: {cannot_drive}")

# logical_operations()
```

## ⚖️ Conditional Statements (Условные операторы)

### Задача 1: Калькулятор с меню
```python
def calculator_with_menu():
    """
    Задача: Создать калькулятор с меню выбора операции
    """
    print("=== КАЛЬКУЛЯТОР ===")
    print("1. Сложение")
    print("2. Вычитание")
    print("3. Умножение")
    print("4. Деление")
    print("5. Возведение в степень")
    
    try:
        choice = input("Выберите операцию (1-5): ")
        num1 = float(input("Введите первое число: "))
        num2 = float(input("Введите второе число: "))
        
        # Простое if-elif-else
        if choice == "1":
            result = num1 + num2
            operation = "+"
        elif choice == "2":
            result = num1 - num2
            operation = "-"
        elif choice == "3":
            result = num1 * num2
            operation = "*"
        elif choice == "4":
            # Проверка деления на ноль
            if num2 == 0:
                print("Ошибка: деление на ноль!")
                return
            result = num1 / num2
            operation = "/"
        elif choice == "5":
            result = num1 ** num2
            operation = "^"
        else:
            print("Неверный выбор операции!")
            return
        
        print(f"Результат: {num1} {operation} {num2} = {result}")
        
    except ValueError:
        print("Ошибка: введите числовые значения!")

# calculator_with_menu()
```

### Задача 2: Определение времени суток
```python
def time_of_day():
    """
    Задача: Определить время суток по введенному часу
    """
    try:
        hour = int(input("Введите текущий час (0-23): "))
        
        # Проверка валидности ввода
        if hour < 0 or hour > 23:
            print("Ошибка: час должен быть от 0 до 23")
            return
        
        # Вложенные условные операторы
        if 0 <= hour < 6:
            time_period = "Ночь"
            greeting = "Доброй ночи! 🌙"
        elif 6 <= hour < 12:
            time_period = "Утро"
            greeting = "Доброе утро! ☀️"
        elif 12 <= hour < 18:
            time_period = "День"
            greeting = "Добрый день! 🌞"
        else:  # 18 <= hour <= 23
            time_period = "Вечер"
            greeting = "Добрый вечер! 🌆"
        
        print(f"Сейчас {time_period}")
        print(greeting)
        
        # Дополнительная информация с использованием логических операторов
        is_working_hours = 9 <= hour < 18
        is_sleep_time = hour < 6 or hour > 23
        is_weekend_time = hour >= 23 or hour < 6
        
        print(f"Рабочее время: {is_working_hours}")
        print(f"Время сна: {is_sleep_time}")
        print(f"Подходит для выходных: {is_weekend_time}")
        
    except ValueError:
        print("Ошибка: введите целое число!")

# time_of_day()
```

### Задача 3: Проверка пароля
```python
def password_validator():
    """
    Задача: Проверить сложность пароля по нескольким критериям
    """
    password = input("Введите пароль для проверки: ")
    
    # Булевы флаги для проверки критериев
    has_upper = False    # Есть заглавные буквы
    has_lower = False    # Есть строчные буквы  
    has_digit = False    # Есть цифры
    has_special = False  # Есть специальные символы
    is_long_enough = len(password) >= 8  # Достаточная длина
    
    # Проверка каждого символа
    for char in password:
        if char.isupper():
            has_upper = True
        elif char.islower():
            has_lower = True
        elif char.isdigit():
            has_digit = True
        elif char in "!@#$%^&*()_+-=[]{}|;:,.<>?":
            has_special = True
    
    # Оценка сложности пароля
    score = 0
    if has_upper: score += 1
    if has_lower: score += 1  
    if has_digit: score += 1
    if has_special: score += 1
    if is_long_enough: score += 1
    
    # Определение уровня безопасности
    if score == 5:
        security_level = "Очень надежный 🔒"
    elif score >= 3:
        security_level = "Надежный ✅"
    elif score >= 2:
        security_level = "Средний ⚠️"
    else:
        security_level = "Слабый ❌"
    
    # Вывод результатов
    print("\n=== АНАЛИЗ ПАРОЛЯ ===")
    print(f"Длина ≥ 8 символов: {is_long_enough}")
    print(f"Заглавные буквы: {has_upper}")
    print(f"Строчные буквы: {has_lower}")
    print(f"Цифры: {has_digit}")
    print(f"Спецсимволы: {has_special}")
    print(f"Общий счет: {score}/5")
    print(f"Уровень безопасности: {security_level}")
    
    # Рекомендации
    if not is_long_enough:
        print("Рекомендация: увеличьте длину пароля")
    if not has_upper:
        print("Рекомендация: добавьте заглавные буквы")
    if not has_special:
        print("Рекомендация: добавьте специальные символы")

# password_validator()
```

### Задача 4: Игра "Камень, ножницы, бумага"
```python
def rock_paper_scissors():
    """
    Задача: Реализовать игру против компьютера
    """
    import random
    
    choices = ["камень", "ножницы", "бумага"]
    
    print("=== КАМЕНЬ-НОЖНИЦЫ-БУМАГА ===")
    print("1. Камень")
    print("2. Ножницы") 
    print("3. Бумага")
    
    try:
        user_choice = int(input("Выберите вариант (1-3): ")) - 1
        
        if user_choice < 0 or user_choice > 2:
            print("Неверный выбор!")
            return
        
        computer_choice = random.randint(0, 2)
        
        print(f"\nВы выбрали: {choices[user_choice]}")
        print(f"Компьютер выбрал: {choices[computer_choice]}")
        
        # Логика определения победителя
        if user_choice == computer_choice:
            result = "Ничья! 🤝"
        elif (user_choice == 0 and computer_choice == 1) or \
             (user_choice == 1 and computer_choice == 2) or \
             (user_choice == 2 and computer_choice == 0):
            result = "Вы победили! 🎉"
        else:
            result = "Компьютер победил! 💻"
        
        print(result)
        
        # Статистика (можно расширить для нескольких игр)
        wins = 0
        losses = 0
        draws = 0
        
        if "победили" in result:
            wins += 1
        elif "Компьютер" in result:
            losses += 1
        else:
            draws += 1
            
        print(f"Статистика: Победы: {wins}, Поражения: {losses}, Ничьи: {draws}")
        
    except ValueError:
        print("Ошибка: введите число от 1 до 3!")

# rock_paper_scissors()
```

### Задача 5: Система оценок
```python
def grade_system():
    """
    Задача: Конвертировать процент в буквенную оценку
    """
    try:
        percentage = float(input("Введите процент выполнения (0-100): "))
        
        if percentage < 0 or percentage > 100:
            print("Ошибка: процент должен быть от 0 до 100")
            return
        
        # Определение оценки по диапазонам
        if percentage >= 90:
            grade = "A"
            description = "Отлично"
        elif percentage >= 80:
            grade = "B" 
            description = "Очень хорошо"
        elif percentage >= 70:
            grade = "C"
            description = "Хорошо"
        elif percentage >= 60:
            grade = "D"
            description = "Удовлетворительно"
        else:
            grade = "F"
            description = "Неудовлетворительно"
        
        # Дополнительная информация
        passed = percentage >= 60  # Прошел ли студент
        honors = percentage >= 90  # С отличием
        
        print(f"\n=== РЕЗУЛЬТАТЫ ===")
        print(f"Процент: {percentage}%")
        print(f"Оценка: {grade} ({description})")
        print(f"Статус: {'Сдал' if passed else 'Не сдал'}")
        print(f"С отличием: {'Да' if honors else 'Нет'}")
        
        # Рекомендации
        if not passed:
            print("Рекомендация: необходимо пересдать экзамен")
        elif percentage < 70:
            print("Рекомендация: нужно улучшить результаты")
        else:
            print("Рекомендация: продолжайте в том же духе!")
            
    except ValueError:
        print("Ошибка: введите числовое значение!")

# grade_system()
```

## 🔁 Loops (Циклы)

### Задача 1: Таблица умножения
```python
def multiplication_table():
    """
    Задача: Вывести таблицу умножения с использованием вложенных циклов
    """
    print("=== ТАБЛИЦА УМНОЖЕНИЯ ===")
    
    # Внешний цикл для строк (множители от 1 до 10)
    for i in range(1, 11):
        # Внутренний цикл для столбцов (множители от 1 до 10)
        for j in range(1, 11):
            result = i * j
            # Форматирование вывода для ровной таблицы
            print(f"{i} × {j} = {result:2d}", end="   ")
        print()  # Переход на новую строку после каждой строки таблицы
    
    # Альтернативный компактный вариант
    print("\n=== КОМПАКТНАЯ ВЕРСИЯ ===")
    for i in range(1, 11):
        row = []
        for j in range(1, 11):
            row.append(i * j)
        # Вывод строки с числами
        print(" ".join(f"{num:3d}" for num in row))

# multiplication_table()
```

### Задача 2: Поиск простых чисел
```python
def find_prime_numbers():
    """
    Задача: Найти все простые числа в заданном диапазоне
    """
    try:
        start = int(input("Введите начало диапазона: "))
        end = int(input("Введите конец диапазона: "))
        
        if start < 2:
            start = 2  # Простые числа начинаются с 2
        
        primes = []  # Список для хранения простых чисел
        
        # Цикл по всем числам в диапазоне
        for number in range(start, end + 1):
            is_prime = True  # Предполагаем, что число простое
            
            # Проверка делителей от 2 до корня из числа
            for divisor in range(2, int(number ** 0.5) + 1):
                if number % divisor == 0:
                    is_prime = False  # Нашли делитель - число не простое
                    break  # Выходим из внутреннего цикла
            
            # Если число простое, добавляем в список
            if is_prime:
                primes.append(number)
        
        # Вывод результатов
        print(f"\nПростые числа в диапазоне от {start} до {end}:")
        if primes:
            # Выводим по 10 чисел в строку для удобства чтения
            for i in range(0, len(primes), 10):
                print(" ".join(map(str, primes[i:i+10])))
            print(f"\nВсего найдено: {len(primes)} простых чисел")
        else:
            print("Простых чисел в заданном диапазоне не найдено")
            
    except ValueError:
        print("Ошибка: введите целые числа!")

# find_prime_numbers()
```

### Задача 3: Обработка списка чисел
```python
def process_number_list():
    """
    Задача: Обработать список чисел разными способами с использованием циклов
    """
    # Создаем список случайных чисел
    import random
    numbers = [random.randint(1, 100) for _ in range(20)]
    
    print("Исходный список чисел:", numbers)
    
    # 1. Сумма всех чисел
    total_sum = 0
    for num in numbers:
        total_sum += num
    print(f"Сумма всех чисел: {total_sum}")
    
    # 2. Максимальное и минимальное значение
    max_num = numbers[0]
    min_num = numbers[0]
    
    for num in numbers:
        if num > max_num:
            max_num = num
        if num < min_num:
            min_num = num
    print(f"Максимальное число: {max_num}")
    print(f"Минимальное число: {min_num}")
    
    # 3. Среднее арифметическое
    average = total_sum / len(numbers)
    print(f"Среднее арифметическое: {average:.2f}")
    
    # 4. Четные и нечетные числа
    even_numbers = []
    odd_numbers = []
    
    for num in numbers:
        if num % 2 == 0:
            even_numbers.append(num)
        else:
            odd_numbers.append(num)
    
    print(f"Четные числа: {even_numbers}")
    print(f"Нечетные числа: {odd_numbers}")
    
    # 5. Числа больше среднего
    above_average = []
    for num in numbers:
        if num > average:
            above_average.append(num)
    print(f"Числа выше среднего: {above_average}")

# process_number_list()
```

### Задача 4: Рисование фигур
```python
def draw_shapes():
    """
    Задача: Нарисовать различные фигуры с помощью циклов
    """
    print("=== ПРЯМОУГОЛЬНИК ===")
    width = 8
    height = 4
    
    # Прямоугольник
    for i in range(height):
        for j in range(width):
            print("*", end="")
        print()  # Новая строка
    
    print("\n=== ТРЕУГОЛЬНИК ===")
    # Прямоугольный треугольник
    for i in range(1, 6):
        for j in range(i):
            print("*", end="")
        print()
    
    print("\n=== ПИРАМИДА ===")
    # Пирамида
    rows = 5
    for i in range(1, rows + 1):
        # Пробелы перед звездочками
        print(" " * (rows - i), end="")
        # Звездочки
        print("*" * (2 * i - 1))
    
    print("\n=== ШАХМАТНАЯ ДОСКА ===")
    # Шахматная доска 8x8
    for i in range(8):
        for j in range(8):
            if (i + j) % 2 == 0:
                print("█", end="")
            else:
                print(" ", end="")
        print()

# draw_shapes()
```

### Задача 5: Игра "Угадай число"
```python
def guess_the_number():
    """
    Задача: Реализовать игру, где компьютер загадывает число, а пользователь угадывает
    """
    import random
    
    print("=== ИГРА 'УГАДАЙ ЧИСЛО' ===")
    print("Компьютер загадал число от 1 до 100. Попробуйте угадать!")
    
    # Компьютер загадывает число
    secret_number = random.randint(1, 100)
    attempts = 0
    max_attempts = 10
    
    # Основной игровой цикл
    while attempts < max_attempts:
        try:
            guess = int(input(f"\nПопытка {attempts + 1}/{max_attempts}. Ваша догадка: "))
            attempts += 1
            
            # Проверка догадки
            if guess < secret_number:
                print("Загаданное число БОЛЬШЕ")
            elif guess > secret_number:
                print("Загаданное число МЕНЬШЕ")
            else:
                print(f"🎉 Поздравляем! Вы угадали число {secret_number}!")
                print(f"Количество попыток: {attempts}")
                break
            
            # Подсказка после нескольких попыток
            if attempts == 5:
                # Подсказка: четное/нечетное
                if secret_number % 2 == 0:
                    print("💡 Подсказка: число четное")
                else:
                    print("💡 Подсказка: число нечетное")
            
            # Последняя попытка
            if attempts == max_attempts - 1:
                print("⚠️  Осталась последняя попытка!")
                
        except ValueError:
            print("Ошибка: введите целое число!")
            continue
    
    # Если закончились попытки
    if attempts == max_attempts and guess != secret_number:
        print(f"\n💔 Игра окончена! Загаданное число было: {secret_number}")
        print("Попробуйте еще раз!")

# guess_the_number()
```

Отличная идея! Вот упражнения на закрепление для каждой темы, созданные по аналогии с предыдущими задачами.

## 📝 Упражнения на закрепление

### 🔤 Data Types (Типы данных)

**Упражнение 1: Конвертер температур**
```python
def temperature_converter():
    """
    Задача: Написать конвертер температур между Цельсием, Фаренгейтом и Кельвином
    """
    # TODO: Получить от пользователя значение температуры и единицы измерения
    # TODO: Реализовать преобразование между всеми тремя системами
    # TODO: Вывести результаты преобразования
    
    # Пример вывода:
    # "25°C = 77°F = 298K"
    pass

# temperature_converter()
```

**Упражнение 2: Анализатор текста**
```python
def text_analyzer():
    """
    Задача: Проанализировать введенный текст и вывести статистику
    """
    # TODO: Подсчитать количество символов, слов, предложений
    # TODO: Найти самое длинное слово
    # TODO: Определить процент заглавных/строчных букв
    # TODO: Посчитать количество цифр в тексте
    
    # Пример вывода:
    # "Символов: 150, Слов: 25, Предложений: 3"
    # "Самое длинное слово: 'программирование' (14 символов)"
    pass

# text_analyzer()
```

**Упражнение 3: Калькулятор списков**
```python
def list_calculator():
    """
    Задача: Выполнить различные операции со списком чисел
    """
    numbers = [12, 45, 23, 67, 34, 89, 56]
    
    # TODO: Найти сумму всех элементов
    # TODO: Найти среднее значение
    # TODO: Создать новый список с квадратами чисел
    # TODO: Отфильтровать числа больше 30
    # TODO: Отсортировать список по убыванию
    
    print("Исходный список:", numbers)
    # Вывести все результаты
    pass

# list_calculator()
```

### 🔢 Numbers & Boolean

**Упражнение 4: Финансовый калькулятор**
```python
def financial_calculator():
    """
    Задача: Рассчитать сложные проценты по вкладу
    """
    # TODO: Запросить начальную сумму, процентную ставку, срок в годах
    # TODO: Рассчитать конечную сумму по формуле сложных процентов
    # TODO: Определить, превысит ли сумма целевое значение
    # TODO: Рассчитать, через сколько лет сумма удвоится
    
    # Пример вывода:
    # "Через 5 лет: 1276.28 руб."
    # "Цель 1500 руб. достигнута: False"
    # "Сумма удвоится через 7 лет"
    pass

# financial_calculator()
```

**Упражнение 5: Система проверки пароля**
```python
def advanced_password_checker():
    """
    Задача: Усовершенствованная проверка сложности пароля
    """
    # TODO: Проверить длину (мин. 12 символов)
    # TODO: Наличие букв в верхнем и нижнем регистре
    # TODO: Наличие цифр и специальных символов
    # TODO: Отсутствие популярных паролей ("123456", "password" и т.д.)
    # TODO: Присвоить оценку от 1 до 5
    
    # Пример вывода:
    # "Оценка безопасности: 4/5"
    # "Рекомендации: увеличьте длину пароля"
    pass

# advanced_password_checker()
```

### ⚖️ Conditional Statements

**Упражнение 6: Калькулятор налога**
```python
def tax_calculator():
    """
    Задача: Рассчитать налог в зависимости от дохода
    """
    # TODO: Запросить годовой доход
    # TODO: Применить прогрессивную шкалу налогообложения:
    #       - До 15,000: 0%
    #       - 15,001-50,000: 15%
    #       - 50,001-100,000: 25%
    #       - Свыше 100,000: 30%
    # TODO: Рассчитать чистый доход
    
    # Пример вывода:
    # "Налог: 12,500 руб."
    # "Чистый доход: 87,500 руб."
    pass

# tax_calculator()
```

**Упражнение 7: Определение сезона и погоды**
```python
def weather_advisor():
    """
    Задача: Дать рекомендации по одежде в зависимости от сезона и температуры
    """
    # TODO: Запросить месяц и текущую температуру
    # TODO: Определить сезон (зима, весна, лето, осень)
    # TODO: Дать рекомендации по одежде
    # TODO: Предупредить об экстремальных температурах
    
    # Пример вывода:
    # "Сейчас зима, -5°C"
    # "Рекомендация: теплая куртка, шапка, перчатки"
    # "Осторожно! Возможен гололед"
    pass

# weather_advisor()
```

### 🔁 Loops

**Упражнение 8: Генератор математических примеров**
```python
def math_examples_generator():
    """
    Задача: Сгенерировать и решить случайные математические примеры
    """
    import random
    
    # TODO: Сгенерировать 10 случайных примеров (+, -, *, /)
    # TODO: Предложить пользователю решить их
    # TODO: Проверить ответы и подсчитать баллы
    # TODO: Вывести статистику: правильные/неправильные ответы
    
    # Пример вывода:
    # "1. 15 + 27 = ? Ответ: 42 ✅"
    # "Результат: 8/10 правильных ответов (80%)"
    pass

# math_examples_generator()
```

**Упражнение 9: Анализ последовательности чисел**
```python
def number_sequence_analyzer():
    """
    Задача: Проанализировать последовательность чисел
    """
    # TODO: Запросить у пользователя числа до ввода 'stop'
    # TODO: Найти максимальное, минимальное, среднее
    # TODO: Определить, есть ли повторяющиеся числа
    # TODO: Отсортировать числа и найти медиану
    # TODO: Построить простой график с помощью символов
    
    # Пример вывода:
    # "Чисел введено: 7"
    # "Максимум: 95, Минимум: 12, Среднее: 47.3"
    # "Медиана: 45"
    # "График: ███▇▅▂"
    pass

# number_sequence_analyzer()
```

**Упражнение 10: Игра "Быки и коровы"**
```python
def bulls_and_cows():
    """
    Задача: Реализовать логическую игру "Быки и коровы"
    """
    import random
    
    # TODO: Компьютер загадывает 4-значное число без повторяющихся цифр
    # TODO: Игрок пытается угадать число
    # TODO: За каждую попытку выводить:
    #       - "Быки": цифры на своих местах
    #       - "Коровы": правильные цифры не на своих местах
    # TODO: Ограничить количество попыток (10)
    
    # Пример вывода:
    # "Попытка 1: 1234 -> Быки: 1, Коровы: 2"
    # "Поздравляем! Вы угадали число 1928 за 5 попыток!"
    pass

# bulls_and_cows()
```

## 🎯 Комплексные упражнения

**Упражнение 11: Система управления студентами**
```python
def student_management_system():
    """
    Задача: Мини-система для управления данными студентов
    """
    students = []
    
    while True:
        print("\n=== СИСТЕМА УПРАВЛЕНИЯ СТУДЕНТАМИ ===")
        print("1. Добавить студента")
        print("2. Показать всех студентов")
        print("3. Найти студента по имени")
        print("4. Рассчитать средний балл")
        print("5. Выход")
        
        choice = input("Выберите действие: ")
        
        # TODO: Реализовать функционал каждого пункта меню
        # 1. Добавление: имя, возраст, оценки (список)
        # 2. Вывод в табличном формате
        # 3. Поиск и вывод информации
        # 4. Расчет статистики по группе
        # 5. Выход из программы
        
        if choice == "5":
            break

# student_management_system()
```

**Упражнение 12: Генератор отчетов**
```python
def report_generator():
    """
    Задача: Сгенерировать отчет по продажам
    """
    # Пример данных о продажах
    sales_data = [
        {"product": "Ноутбук", "price": 50000, "quantity": 3},
        {"product": "Мышь", "price": 1500, "quantity": 10},
        {"product": "Клавиатура", "price": 3000, "quantity": 5},
        {"product": "Монитор", "price": 20000, "quantity": 2}
    ]
    
    # TODO: Рассчитать общую выручку
    # TODO: Найти самый продаваемый товар
    # TODO: Определить товар с максимальной выручкой
    # TODO: Рассчитать средний чек
    # TODO: Сгенерировать красивый отчет
    
    # Пример вывода:
    # "=== ОТЧЕТ О ПРОДАЖАХ ==="
    # "Общая выручка: 255,000 руб."
    # "Самый продаваемый товар: Мышь (10 шт.)"
    # "Товар с макс. выручкой: Ноутбук (150,000 руб.)"
    pass

# report_generator()
```

## 📊 Дополнительные задания повышенной сложности

**Упражнение 13: Шифратор/дешифратор Цезаря**
```python
def caesar_cipher():
    """
    Задача: Реализовать шифр Цезаря с возможностью шифрования и дешифрования
    """
    # TODO: Запросить у пользователя текст и сдвиг
    # TODO: Реализовать шифрование (сдвиг букв вперед)
    # TODO: Реализовать дешифрование (сдвиг букв назад)
    # TODO: Обрабатывать разные регистры и сохранять пробелы/знаки препинания
    
    # Пример:
    # Текст: "Hello World!", сдвиг: 3
    # Зашифровано: "Khoor Zruog!"
    pass

# caesar_cipher()
```

**Упражнение 14: Симулятор банкомата**
```python
def atm_simulator():
    """
    Задача: Реализовать упрощенную симуляцию работы банкомата
    """
    balance = 10000  # Начальный баланс
    
    while True:
        print(f"\n=== БАНКОМАТ ===")
        print(f"Текущий баланс: {balance} руб.")
        print("1. Показать баланс")
        print("2. Снять наличные")
        print("3. Пополнить счет")
        print("4. Выход")
        
        # TODO: Реализовать все операции с проверками:
        # - Нельзя снять больше, чем на счете
        # - Суммы должны быть положительными
        # - Снятие/пополнение только кратно 100
        # - Лимит на снятие за одну операцию
        
        pass

# atm_simulator()
```

# Дополнительные материалы

https://www.geeksforgeeks.org/python/python-programming-language-tutorial
https://www.geeksforgeeks.org/python/python-web-development