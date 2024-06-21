# an implementation of dijkstra's shortest path search algorithm

# see wikipedia https://en.wikipedia.org/wiki/Dijkstra's_algorithm

# consider doing this using dataframes
# write function
# build test graphs


def dijkstra(graph, start, end):
    """
    a function that searches through the given graph to find the
    shortest path from the start node to the end node
    --assumes a connected graph (all nodes have min 1 path to all other nodes)
    --assumes edge weights are positive integers
    --assumes an edge is a 3-tuple of two node names and a weight
    :param graph: a connected graph, with two fields: nodes and edges
    :param start: the starting node
    :param end: the ending node
    :return: a list of nodes enumerating the steps through the graph
    """

    # initialize path to empty list (this is will be returned, list of nodes that represents the path through the graph
    #       from start to end)
    path = []

    # initialize visited set of node-prevNode-distance 3-tuples (empty at beginning)
    visited = []

    # initialize current node-prevNode-distance tuple
    current = (start, False, 0)

    # initialize unvisited set of node-prevNode-distance 3-tuples, with distance infinity and prevNode empty
    unvisited = []

    # set distance of start node to 0 in the unvisited node set

    # LOOP
    #
    # search for min distance in the unvisited set of nodes, and set that node to current node
    # --if current node is end, stop LOOP
    #
    # iterate over list of edges, and for each edge that contains current node:
    # --search (in the unvisited set of nodes) for the  other node in the edge
    # --calculate distance through current node (current node distance plus distance of edge)
    # --if distance through current node is less than distance on thee next node, overwite the distance, and set the next-hop's prevNode to the current node
    #
    # add the current node to the list of visited nodes
    #
    # END LOOP


    # in list of visited nodes, find end, add to path
    # LOOP
    # add prevNode of end to path
    # lookup prevNode in visited nodes
    # END LOOP

    # reverse path
    # return path

    return


class Node:
    def __init__(self, name, prev_node):
        self.name = name
        self.prev_node = prev_node

