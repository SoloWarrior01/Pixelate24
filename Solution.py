import cv2
import numpy as np
import math
from collections import deque, namedtuple
import cv2.aruco as aruco

from aruco_detection import aruco_location
from publisher import move_straight, rotate_to_final_location
from camera_feed import capture, color_identification


def get_node_pairs(n1, n2, both_ends=True):
    if both_ends:
        node_pairs = [[n1, n2], [n2, n1]]
    else:
        node_pairs = [[n1, n2]]
    return node_pairs


class Dijkstra:
    def __init__(self, edges):
        self.edges = [make_edge(*edge) for edge in edges]

    @property
    def vertices(self):
        return set(

            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'

        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:

            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])

            if distances[current_vertex] == inf:
                break

            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost

                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

            vertices.remove(current_vertex)

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path, distances[dest]


Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
    return Edge(start, end, cost)


if __name__ == "__main__":
    capture()
    img = cv2.imread("image_for_color_identification.jpg")
    inf = float('inf')
    ordered_2D, colorwise_list = color_identification()

    value_of_colors = [1, 1, 1, 2, 3, 4, 200]  # white, red, purple, green, yellow, blue, orange


    for i in range(len(ordered_2D)):
        j = 0
        while j < len(ordered_2D[i]) - 1:
            if ordered_2D[i][j] != (-1, -1):
                no_of_black_tiles = (ordered_2D[i][j + 1][0] - ordered_2D[i][j][0]) // 44 - 1
                for k in range(no_of_black_tiles):
                    ordered_2D[i].insert(j + 1, (-1, -1))
            j += 1

    connections = []
    for i in range(len(ordered_2D)):
        for j in range(len(ordered_2D[i])):
            if(ordered_2D[i][j] != (-1, -1)):
                neighbour_nodes = []
                if(i > 0):
                    neighbour_nodes.append((i-1, j))
                elif(i < len(ordered_2D[i])-1):
                    neighbour_nodes.append((i+1, j))
                elif(j > 0):
                    neighbour_nodes.append((i, j-1))
                elif(j < len(ordered_2D)-1):
                    neighbour_nodes.append((i, j+1))

                for node in neighbour_nodes:
                    try:
                        if ordered_2D[node[0]][node[1]] != (-1, -1):
                            for k in range(7):
                                if ordered_2D[node[0]][node[1]] in colorwise_list[k]:
                                    connections.append(((i, j), node, value_of_colors[k]))
                    except IndexError:
                        pass

    algorithm = Dijkstra(connections)
        
