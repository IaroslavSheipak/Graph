#include <iostream>
#include "Graph.hpp"  // This assumes your Graph class is defined in Graph.hpp.

int main() {
    Graph g;

    // Create a simple graph:
    // (1) -2-> (2) -3-> (3)
    //  |       |       |
    //  4       2       1
    //  V       V       V
    // (4) -3-> (5) -1-> (6)

    g.createGraph({{1, 2, 2}, {2, 3, 3}, {1, 4, 4}, {2, 5, 2}, {3, 6, 1}, {4, 5, 3}, {5, 6, 1}});

    // Print the graph
    std::cout << "Graph:" << std::endl;
    g.printGraph();
    std::cout << std::endl;

    // Test DFS from node 1
    std::cout << "DFS starting from node 1:" << std::endl;
    g.DFS(1);
    std::cout << std::endl;

    // Test BFS from node 1
    std::cout << "BFS starting from node 1:" << std::endl;
    g.BFS(1);
    std::cout << std::endl;

    // Test Dijkstra's algorithm from node 1
    std::cout << "Dijkstra starting from node 1:" << std::endl;
    g.Dijkstra(1);
    std::cout << std::endl;

    // Test A* from node 1 to node 6
    std::cout << "A* from node 1 to node 6:" << std::endl;
    g.AStar(1, 6);
    std::cout << std::endl;

     // Test Bellman-Ford algorithm from node 1
    std::cout << "Bellman-Ford starting from node 1:" << std::endl;
    std::unordered_map<int, int> distances;
    if (g.BellmanFord(1, distances)) {
        for (const auto& distance : distances) {
            std::cout << "Node " << distance.first << " has minimum distance " << distance.second << std::endl;
        }
    } else {
        std::cout << "Negative weight cycle detected in the graph." << std::endl;
    }
    std::cout << std::endl;

    // Test Edmonds-Karp algorithm from node 1 to node 6
    std::cout << "Edmonds-Karp maximum flow from node 1 to node 6:" << std::endl;
    int maxFlow = g.EdmondsKarp(1, 6);
    std::cout << "Maximum flow: " << maxFlow << std::endl;

    return 0;
}
