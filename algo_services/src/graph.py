# Добавить логгирование каждого действия log()

class Graph:
    def __init__(self):
        """
        Инициализация пустого графа.
        Граф представлен в виде словаря, где:
        - ключи: вершины графа,
        - значения: словари соседей с весами ребер
        """
        self.graph = {}

    def add_vertex(self, vertex):
        """
        Добавления вершины в граф.
        Параметры:
             vertex: название вершины (строка или число)
        """
        if vertex not in self.graph:
            self.graph[vertex] = {}

    def add_edge(self, vertex1, vertex2, weight=1, directed=False):
        """
        Добавление ребра между двумя вершинами.
        Параметра: 
             vertex1: первая вершина
             vertex2:  вторая вершина
             weight: вес ребра (по умолчанию 1)
             directed: ориентация ребра, по умолчанию отсуствует
        """
        self.add_vertex(vertex1)
        self.add_vertex(vertex2)
        self.graph[vertex1][vertex2] = weight # nested dictionary двухуровневая структура данных в Питон
        """
        self.graph = {
        'Вершина A': {Б:3, В:2, К:1, ...}, # А -(>) Б ребро с весом 3
        'Вершина Б': {...},
        ...,
        'Вершина N': {...}
        }
        """
        if not directed:
            self.graph[vertex2][vertex1] = weight

    def remove_edge(self, vertex1, vertex2, directed=False):
        """
        Удаление ребра между двумя вершинами.
        Параметры: 
            vertex1: первая вершина,
            vertex2: вторая вершина,
            directed: ориентация (по умолчанию отсуствует)
        """
        if vertex1 in self.graph and vertex2 in self.graph[vertex1]:
            del self.graph[vertex1, vertex2]

            # Если граф неориентировнный, удаляем обратно ребро
            if not directed and vertex2 in self.graph and vertex1 in self.graph[vertex2]:
                del self.graph[vertex2][vertex1]
    def get_vertices(self):
        """
        Получаем список всех вершин графа.
        Возвращает: 
            list: список вершин
        """
        return list(self.graph.keys)
    def get_edges(self, directed=False):
        edges = []
        visited = set()
        # Переделать обход через while
        for vertex1 in self.graph:
            for vertex2, weight in self.graph[vertex1].items():
                if not directed: # для неорентированного графа избегаем дублирования
                    edge = tuple(sorted((vertex1, vertex2))) # неизменяемый кортеж, предназначеный для хранения упорядоченных последовательностей
                    # 
                    if edge not in visited:
                        edges.append((vertex1, vertex2, weight))
                        visited.add(edge)
                    else:
                        edges.append((vertex1, vertex2, weight))


        return edges
    def get_neighbors(self, vertex):
        """
        Получение соседей вершины.
        Параметры: 
            vertex : вершина, для которой ищем соседей

        Возвращаем: 
            dict: словарь соседей с весами ребер
        """
        return self.graph.get(vertex, {})
    
    def get_weight(self, vertex1, vertex2):
        """
        Получение веса ребра между двумя вершинами.
        Параметры:
            vertex1: первая вершина
            vertex2: вторая вершина
        Возвращаем:
            number: вес ребра или None, если ребра нет
        """
        if vertex1 in self.graph and vertex2 in self.graph[vertex1]:
            return self.graph[vertex1][vertex2]
        return None
    def is_connected(self, vertex1, vertex2):
        """
        Проверка наличия ребра между двумя вершинами
        Параметры:
            vertex1: первая вершина
            vertex2: вторая вершина
        Возвращаем:
            bool: True , если есть ребро, иначе False
        """
        return vertex1 in self.graph and vertex2 in self.graph[vertex1]
    
    def __str__(self):
        """
        Строковое представление графа
        """
        result = "Graph:\n"
        for vertex in self.graph:
            result += f"{vertex}: {self.graph[vertex]}\n"
        return result
    # Дописать метод def() визуализацию результата (попробовать самостоятельно)




    def __contains__(self, vertex):
        """
        Проверка наличия вершины в графе.
        """
        return vertex in self.graph

                    


