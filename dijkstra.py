# implementation of dijkstra's shortest path search algorithm

# see wikipedia https://en.wikipedia.org/wiki/Dijkstra's_algorithm


def dijkstra(graph, start, end):
    """
    a function that searches through the given graph to find the
    shortest path from the start node to the end node
    --assumes a connected graph (all nodes have min 1 path to all other nodes)
    --assumes edge weights are positive integers
    :param graph: a connected graph object, with two fields: nodes (list of strings) and edges (list of Edge)
    :param start: the starting node (string name of node)
    :param end: the ending node (string name of node)
    :return: a list of nodes enumerating the steps through the graph
    """

    edges = graph.edges
    # initialize path to empty list (this is will be returned, list of nodes that represents the path through the graph
    #       from start to end)
    path = []

    # initialize visited list of node-prevNode-distance lists (empty at beginning)
    visited = []

    # initialize current node-prevNode-distance list to start
    current = [start, False, 0]

    # initialize unvisited list of node-prevNode-distance lists, with distance infinity and prevNode empty
    # excluding the starting node
    unvisited = []
    for node in graph.nodes:
        if node != start:
            unvisited.append([node, False, float('inf')])

    # LOOP
    # continuation condition is that we haven't yet reached the end node
    while current[0] != end:

        # update distances and prev_node in unvisited
        # find nearest node in unvisited
        # copy current to visited
        # copy nearest in unvisited to current
        # remove nearest in unvisited from unvisited

        # iterate over edges to update known distances in unvisited
        for edge in edges:
            for node in unvisited:
                # if the given edge goes from the current node to the given unvisited node
                if ((edge.node1 == node) | (edge.node2 == node)) & ((edge.node1 == current) | (edge.node2 == current)):
                    # calculate the distance to the unvisited node by traversing current node and current edge
                    dist = current[2] + edge.weight
                    # if this distance is less than the current known distance to the unvisited node, then
                    # update the unvisited node's distance and prev_node
                    if dist < node[2]:
                        node[2] = dist
                        node[1] = current[0]
                    # remove the edge from edges since it has been applied
                    edges.remove(edge)
                    break

        # find the nearest (shortest distance) node in unvisited
        unvisited_min_node = ['', False, float('inf')]
        for node in unvisited:
            if node[2] < unvisited_min_node[2]:
                unvisited_min_node = node

        # add the current node visited, if not empty, to account for the first iteration
        if current[0] != '':
            visited.append(current)

        # set current node to the next nearest node
        current = unvisited_min_node

        # remove current from visited
        unvisited.remove(current)

    # END LOOP

    # append end (current) to path
    path.append(current)

    # LOOP
    # trace 'path' from 'visited'
    while True:
        # get prev_node from last node in 'path'
        prev_node = path[len(path) - 1]

        # if the last node in 'path' is the start node, then the path has been completed, so break
        if prev_node[0] == start:
            break

        # otherwise we are still midway through re-tracing the path
        else:
            # find prev_node in 'visited'
            for node in visited:
                if node[0] == prev_node[0]:
                    # add prev_node to path, and break this for loop
                    path.append(node)
                    break

    # END LOOP

    # prepare return values:
    # --get the total distance
    distance = path[0][2]
    # --reverse path and strip prev_node and distance values
    path2 = []
    for node in path:
        path2.append(node[0])
    path2.reverse()

    return distance, path2


class Edge:
    def __init__(self, node1, node2, weight):
        self.node1 = node1  # this is a string
        self.node2 = node2  # this is a string
        self.weight = weight  # this is a positive integer


class Graph:
    def __init__(self, nodes, edges):
        self.nodes = nodes  # this is a list of strings
        self.edges = edges  # this is a list of Edge
