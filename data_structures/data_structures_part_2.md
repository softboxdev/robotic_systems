# Деревья в Python: полное руководство

## Оглавление
1. [Что такое дерево?](#что-такое-дерево)
2. [Бинарные деревья](#бинарные-деревья)
3. [Бинарные деревья поиска](#бинарные-деревья-поиска)
4. [Сбалансированные деревья](#сбалансированные-деревья)
5. [Обход деревьев](#обход-деревьев)
6. [Практические применения](#практические-применения)
7. [Деревья в стандартной библиотеке](#деревья-в-стандартной-библиотеке)

## Что такое дерево?

**Дерево** - это иерархическая структура данных, состоящая из узлов, где:
- Каждый узел содержит значение и ссылки на дочерние узлы
- Есть один корневой узел (не имеет родителя)
- Узлы без дочерних элементов называются листьями
- Узлы с дочерними элементами называются внутренними узлами

### Базовый класс узла дерева
```python
class TreeNode:
    def __init__(self, value):
        self.value = value
        self.children = []
    
    def add_child(self, child_node):
        self.children.append(child_node)
    
    def __str__(self, level=0):
        ret = "  " * level + str(self.value) + "\n"
        for child in self.children:
            ret += child.__str__(level + 1)
        return ret

# Пример использования
root = TreeNode("A")
b = TreeNode("B")
c = TreeNode("C")
d = TreeNode("D")

root.add_child(b)
root.add_child(c)
b.add_child(d)

print(root)
# A
#   B
#     D
#   C
```

## Бинарные деревья

**Бинарное дерево** - дерево, где каждый узел имеет не более двух дочерних элементов (левый и правый).

```python
class BinaryTreeNode:
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None
    
    def __str__(self):
        return f"Node({self.value})"

class BinaryTree:
    def __init__(self, root_value):
        self.root = BinaryTreeNode(root_value)
    
    def insert_left(self, parent, value):
        """Вставка в левое поддерево"""
        if parent.left is None:
            parent.left = BinaryTreeNode(value)
        else:
            new_node = BinaryTreeNode(value)
            new_node.left = parent.left
            parent.left = new_node
    
    def insert_right(self, parent, value):
        """Вставка в правое поддерево"""
        if parent.right is None:
            parent.right = BinaryTreeNode(value)
        else:
            new_node = BinaryTreeNode(value)
            new_node.right = parent.right
            parent.right = new_node
    
    def display(self, node=None, level=0, prefix="Root: "):
        if node is None:
            node = self.root
        
        print(" " * (level * 4) + prefix + str(node.value))
        if node.left:
            self.display(node.left, level + 1, "L--- ")
        if node.right:
            self.display(node.right, level + 1, "R--- ")

# Пример
tree = BinaryTree(1)
tree.insert_left(tree.root, 2)
tree.insert_right(tree.root, 3)
tree.insert_left(tree.root.left, 4)
tree.insert_right(tree.root.left, 5)

tree.display()
# Root: 1
#     L--- 2
#         L--- 4
#         R--- 5
#     R--- 3
```

## Бинарные деревья поиска (BST)

**BST** - бинарное дерево, где для каждого узла:
- Все значения в левом поддереве меньше значения узла
- Все значения в правом поддереве больше значения узла

```python
class BSTNode:
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None
        self.parent = None

class BinarySearchTree:
    def __init__(self):
        self.root = None
    
    def insert(self, value):
        """Вставка значения в дерево"""
        if self.root is None:
            self.root = BSTNode(value)
        else:
            self._insert_recursive(self.root, value)
    
    def _insert_recursive(self, current, value):
        if value < current.value:
            if current.left is None:
                current.left = BSTNode(value)
                current.left.parent = current
            else:
                self._insert_recursive(current.left, value)
        else:
            if current.right is None:
                current.right = BSTNode(value)
                current.right.parent = current
            else:
                self._insert_recursive(current.right, value)
    
    def search(self, value):
        """Поиск значения в дереве"""
        return self._search_recursive(self.root, value)
    
    def _search_recursive(self, current, value):
        if current is None:
            return False
        if current.value == value:
            return True
        elif value < current.value:
            return self._search_recursive(current.left, value)
        else:
            return self._search_recursive(current.right, value)
    
    def find_min(self, node=None):
        """Нахождение минимального значения"""
        if node is None:
            node = self.root
        while node.left is not None:
            node = node.left
        return node.value
    
    def find_max(self, node=None):
        """Нахождение максимального значения"""
        if node is None:
            node = self.root
        while node.right is not None:
            node = node.right
        return node.value
    
    def delete(self, value):
        """Удаление значения из дерева"""
        self.root = self._delete_recursive(self.root, value)
    
    def _delete_recursive(self, current, value):
        if current is None:
            return current
        
        if value < current.value:
            current.left = self._delete_recursive(current.left, value)
        elif value > current.value:
            current.right = self._delete_recursive(current.right, value)
        else:
            # Узел с одним или без детей
            if current.left is None:
                return current.right
            elif current.right is None:
                return current.left
            
            # Узел с двумя детьми: находим минимальный в правом поддереве
            min_node = self._find_min_node(current.right)
            current.value = min_node.value
            current.right = self._delete_recursive(current.right, min_node.value)
        
        return current
    
    def _find_min_node(self, node):
        while node.left is not None:
            node = node.left
        return node
    
    def display(self):
        """Визуализация дерева"""
        if self.root:
            self._display_recursive(self.root)
    
    def _display_recursive(self, node, level=0, prefix="Root: "):
        if node:
            print(" " * (level * 4) + prefix + str(node.value))
            if node.left or node.right:
                if node.left:
                    self._display_recursive(node.left, level + 1, "L--- ")
                else:
                    print(" " * ((level + 1) * 4) + "L--- None")
                if node.right:
                    self._display_recursive(node.right, level + 1, "R--- ")
                else:
                    print(" " * ((level + 1) * 4) + "R--- None")

# Пример использования
bst = BinarySearchTree()
values = [50, 30, 70, 20, 40, 60, 80]

for value in values:
    bst.insert(value)

print("Дерево:")
bst.display()
print(f"Минимальное значение: {bst.find_min()}")
print(f"Максимальное значение: {bst.find_max()}")
print(f"Поиск 40: {bst.search(40)}")
print(f"Поиск 100: {bst.search(100)}")
```

## Сбалансированные деревья

### AVL-дерево
```python
class AVLNode:
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None
        self.height = 1

class AVLTree:
    def __init__(self):
        self.root = None
    
    def _get_height(self, node):
        if not node:
            return 0
        return node.height
    
    def _get_balance(self, node):
        if not node:
            return 0
        return self._get_height(node.left) - self._get_height(node.right)
    
    def _update_height(self, node):
        if node:
            node.height = 1 + max(self._get_height(node.left), 
                                self._get_height(node.right))
    
    def _rotate_right(self, y):
        x = y.left
        T2 = x.right
        
        x.right = y
        y.left = T2
        
        self._update_height(y)
        self._update_height(x)
        
        return x
    
    def _rotate_left(self, x):
        y = x.right
        T2 = y.left
        
        y.left = x
        x.right = T2
        
        self._update_height(x)
        self._update_height(y)
        
        return y
    
    def insert(self, value):
        self.root = self._insert_recursive(self.root, value)
    
    def _insert_recursive(self, node, value):
        # Обычная вставка BST
        if not node:
            return AVLNode(value)
        
        if value < node.value:
            node.left = self._insert_recursive(node.left, value)
        else:
            node.right = self._insert_recursive(node.right, value)
        
        # Обновление высоты
        self._update_height(node)
        
        # Балансировка
        balance = self._get_balance(node)
        
        # Левый левый случай
        if balance > 1 and value < node.left.value:
            return self._rotate_right(node)
        
        # Правый правый случай
        if balance < -1 and value > node.right.value:
            return self._rotate_left(node)
        
        # Левый правый случай
        if balance > 1 and value > node.left.value:
            node.left = self._rotate_left(node.left)
            return self._rotate_right(node)
        
        # Правый левый случай
        if balance < -1 and value < node.right.value:
            node.right = self._rotate_right(node.right)
            return self._rotate_left(node)
        
        return node
```

## Обход деревьев

```python
class TreeTraversal:
    @staticmethod
    def inorder(node):
        """Центрированный обход (левый-корень-правый)"""
        result = []
        if node:
            result.extend(TreeTraversal.inorder(node.left))
            result.append(node.value)
            result.extend(TreeTraversal.inorder(node.right))
        return result
    
    @staticmethod
    def preorder(node):
        """Прямой обход (корень-левый-правый)"""
        result = []
        if node:
            result.append(node.value)
            result.extend(TreeTraversal.preorder(node.left))
            result.extend(TreeTraversal.preorder(node.right))
        return result
    
    @staticmethod
    def postorder(node):
        """Обратный обход (левый-правый-корень)"""
        result = []
        if node:
            result.extend(TreeTraversal.postorder(node.left))
            result.extend(TreeTraversal.postorder(node.right))
            result.append(node.value)
        return result
    
    @staticmethod
    def level_order(node):
        """Обход в ширину (по уровням)"""
        if not node:
            return []
        
        result = []
        queue = [node]
        
        while queue:
            current = queue.pop(0)
            result.append(current.value)
            
            if current.left:
                queue.append(current.left)
            if current.right:
                queue.append(current.right)
        
        return result

# Пример использования обходов
bst = BinarySearchTree()
for value in [50, 30, 70, 20, 40, 60, 80]:
    bst.insert(value)

print("Inorder:", TreeTraversal.inorder(bst.root))
print("Preorder:", TreeTraversal.preorder(bst.root))
print("Postorder:", TreeTraversal.postorder(bst.root))
print("Level order:", TreeTraversal.level_order(bst.root))
```

## Практические применения

### Пример 1: Дерево выражений
```python
class ExpressionTree:
    def __init__(self, value):
        self.value = value
        self.left = None
        self.right = None
    
    def evaluate(self):
        """Вычисление значения выражения"""
        if self.value.isdigit():
            return int(self.value)
        
        left_val = self.left.evaluate()
        right_val = self.right.evaluate()
        
        if self.value == '+':
            return left_val + right_val
        elif self.value == '-':
            return left_val - right_val
        elif self.value == '*':
            return left_val * right_val
        elif self.value == '/':
            return left_val / right_val
    
    @staticmethod
    def build_from_postfix(expression):
        """Построение дерева из постфиксной записи"""
        stack = []
        tokens = expression.split()
        
        for token in tokens:
            if token.isdigit():
                stack.append(ExpressionTree(token))
            else:
                right = stack.pop()
                left = stack.pop()
                node = ExpressionTree(token)
                node.left = left
                node.right = right
                stack.append(node)
        
        return stack.pop()

# Пример: (3 + 4) * 5 в постфиксной записи: 3 4 + 5 *
expr_tree = ExpressionTree.build_from_postfix("3 4 + 5 *")
print(f"Результат: {expr_tree.evaluate()}")  # 35
```

### Пример 2: Дерево файловой системы
```python
import os

class FileSystemNode:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.children = []
        self.is_file = os.path.isfile(path)
        self.size = os.path.getsize(path) if self.is_file else 0
    
    def build_tree(self, max_depth=3, current_depth=0):
        """Рекурсивное построение дерева файловой системы"""
        if current_depth >= max_depth or self.is_file:
            return
        
        try:
            for item in os.listdir(self.path):
                item_path = os.path.join(self.path, item)
                child = FileSystemNode(item_path)
                self.children.append(child)
                child.build_tree(max_depth, current_depth + 1)
        except PermissionError:
            pass
    
    def display(self, level=0):
        """Отображение структуры"""
        indent = "  " * level
        node_type = "📄" if self.is_file else "📁"
        size_info = f" ({self.size} bytes)" if self.is_file else ""
        print(f"{indent}{node_type} {self.name}{size_info}")
        
        for child in self.children:
            child.display(level + 1)
    
    def total_size(self):
        """Общий размер директории"""
        if self.is_file:
            return self.size
        
        total = 0
        for child in self.children:
            total += child.total_size()
        return total

# Пример использования
# root = FileSystemNode("/path/to/directory")
# root.build_tree(max_depth=2)
# root.display()
# print(f"Общий размер: {root.total_size()} bytes")
```

### Пример 3: Дерево решений для классификации
```python
class DecisionTreeNode:
    def __init__(self, feature=None, threshold=None, value=None, left=None, right=None):
        self.feature = feature      # Индекс признака для разделения
        self.threshold = threshold  # Пороговое значение
        self.value = value          # Значение для листового узла
        self.left = left           # Левое поддерево (<= threshold)
        self.right = right         # Правое поддерево (> threshold)
    
    def is_leaf(self):
        return self.value is not None

class SimpleDecisionTree:
    def __init__(self, max_depth=5):
        self.max_depth = max_depth
        self.root = None
    
    def fit(self, X, y):
        """Обучение дерева"""
        self.root = self._build_tree(X, y, depth=0)
    
    def _build_tree(self, X, y, depth):
        # Базовые случаи для остановки
        if depth >= self.max_depth or len(set(y)) == 1:
            return DecisionTreeNode(value=self._most_common_label(y))
        
        # Находим лучшее разделение
        best_feature, best_threshold = self._best_split(X, y)
        
        if best_feature is None:
            return DecisionTreeNode(value=self._most_common_label(y))
        
        # Разделяем данные
        left_mask = X[:, best_feature] <= best_threshold
        right_mask = ~left_mask
        
        # Рекурсивно строим поддеревья
        left = self._build_tree(X[left_mask], y[left_mask], depth + 1)
        right = self._build_tree(X[right_mask], y[right_mask], depth + 1)
        
        return DecisionTreeNode(
            feature=best_feature,
            threshold=best_threshold,
            left=left,
            right=right
        )
    
    def predict(self, X):
        """Предсказание для нескольких образцов"""
        return [self._predict_sample(x, self.root) for x in X]
    
    def _predict_sample(self, x, node):
        if node.is_leaf():
            return node.value
        
        if x[node.feature] <= node.threshold:
            return self._predict_sample(x, node.left)
        else:
            return self._predict_sample(x, node.right)
    
    def _best_split(self, X, y):
        # Упрощенная реализация поиска лучшего разделения
        best_gain = -1
        best_feature = None
        best_threshold = None
        
        for feature in range(X.shape[1]):
            thresholds = np.unique(X[:, feature])
            for threshold in thresholds:
                gain = self._information_gain(y, X[:, feature], threshold)
                
                if gain > best_gain:
                    best_gain = gain
                    best_feature = feature
                    best_threshold = threshold
        
        return best_feature, best_threshold
    
    def _information_gain(self, y, feature, threshold):
        # Упрощенный расчет информационного прироста
        left_mask = feature <= threshold
        right_mask = ~left_mask
        
        if sum(left_mask) == 0 or sum(right_mask) == 0:
            return 0
        
        n = len(y)
        n_left, n_right = sum(left_mask), sum(right_mask)
        
        # Энтропия родителя
        parent_entropy = self._entropy(y)
        
        # Взвешенная энтропия детей
        left_entropy = self._entropy(y[left_mask])
        right_entropy = self._entropy(y[right_mask])
        child_entropy = (n_left / n) * left_entropy + (n_right / n) * right_entropy
        
        return parent_entropy - child_entropy
    
    def _entropy(self, y):
        """Расчет энтропии"""
        proportions = np.bincount(y) / len(y)
        return -np.sum([p * np.log2(p) for p in proportions if p > 0])
    
    def _most_common_label(self, y):
        """Наиболее частый класс"""
        return np.bincount(y).argmax()

# Пример использования
# import numpy as np
# from sklearn.datasets import make_classification
# 
# X, y = make_classification(n_samples=100, n_features=4, random_state=42)
# tree = SimpleDecisionTree(max_depth=3)
# tree.fit(X, y)
# predictions = tree.predict(X)
```

## Деревья в стандартной библиотеке

### Модуль anytree
```python
from anytree import Node, RenderTree

# Создание дерева
root = Node("A")
b = Node("B", parent=root)
c = Node("C", parent=root)
d = Node("D", parent=b)
e = Node("E", parent=b)

# Визуализация
for pre, fill, node in RenderTree(root):
    print(f"{pre}{node.name}")

# A
# ├── B
# │   ├── D
# │   └── E
# └── C
```

## Заключение

Деревья - фундаментальная структура данных с множеством применений:

- **Бинарные деревья поиска** для эффективного поиска (O(log n) в сбалансированном случае)
- **AVL-деревья** для автоматической балансировки
- **Деревья выражений** для вычисления математических выражений
- **Деревья решений** для машинного обучения
- **Файловые деревья** для организации иерархических данных

Ключевые преимущества деревьев:
- Эффективный поиск (в сбалансированных деревьях)
- Естественное представление иерархических данных
- Гибкость в организации информации

Выбор конкретного типа дерева зависит от решаемой задачи и требований к производительности.