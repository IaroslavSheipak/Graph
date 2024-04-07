#include <iostream>
#include <unordered_map>
#include <vector>
#include <list>
#include <memory>
#include <queue>
#include <stack>
#include <unordered_set>
#include <limits>
#include <algorithm>

class Node;

class Edge {
public:
    std::shared_ptr<Node> adjacentNode;
    int weight;

    Edge(std::shared_ptr<Node> adjacentNode, int weight) : adjacentNode(adjacentNode), weight(weight) {}
};

class Node {
public:
    int value;
    std::list<Edge> edges;
    std::unordered_map<Node*, Edge> parents;

    Node(int value) : value(value) {}
};

class Graph {
public:

    std::shared_ptr<Node> addOrGetNode(int value);
    void createGraph(const std::vector<std::vector<int>>& graphData);

    void printGraph();

    void DFSUtil(int v, std::unordered_set<int>& visited);

    void DFS(int startValue);

    void BFS(int startValue);

    void Dijkstra(int startValue);
    void AStar(int startValue, int goalValue);

    int EdmondsKarp(int sourceValue, int sinkValue);

    bool BellmanFord(int startValue, std::unordered_map<int, int>& distances);

    private:
        std::vector<int> BFS_find_path(int sourceValue, int sinkValue, std::unordered_map<int, std::unordered_map<int, int>>& residual);
        std::unordered_map<int, std::shared_ptr<Node>> nodes;
};
