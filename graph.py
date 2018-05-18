"""
Implimentation of Graph class for Python
"""

class Graph(object):
    def __init__(self):
        self.nodes = {}
    def addNode(self,newNode):
        self.nodes[newNode.name] = newNode
    def getNode(self,nodeName):
        if nodeName in self.nodes:
            return self.nodes[nodeName]
        else:
            return None
    def addEdge(self, node1, node2, weight):
        self.nodes[node1.name].addEdge(self.nodes[node2.name],weight)
        self.nodes[node2.name].addEdge(self.nodes[node1.name],weight)

    def __str__(self):
        print("Node names")
        for i in self.nodes:
            print i
        print("\nNode edges")
        for i in self.nodes:
            print self.nodes[i]
        return ""


class Node(object):
    def __init__(self,name):
        self.name = name
        self.edges = {}
    def addEdge(self,node,weight):
        self.edges[str(node.name)] = weight
    def addEdgeManual(self,name,weight):
        self.edges[str(name)] = weight
    def __str__(self):
        totalStr = ""
        totalStr += "Node Name: " + self.name + "\n"
        totalStr += "Edges: \n"
        for i in self.edges:
            totalStr += i + ": "+str(self.edges[i]) + "\n"

        return totalStr
