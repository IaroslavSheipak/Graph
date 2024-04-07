#include "Graph.hpp"

std::shared_ptr<Node> Graph::addOrGetNode(int value) {
        if (value == -1) return nullptr;
        auto it = nodes.find(value);
        if (it != nodes.end()) {
            return it->second;
        }
        auto node = std::make_shared<Node>(value);
        nodes[value] = node;
        return node;
}

void Graph::createGraph(const std::vector<std::vector<int>>& graphData) {
        for (const auto& row : graphData) {
            auto node = addOrGetNode(row[0]);
            auto adjacentNode = addOrGetNode(row[1]);
            if (!adjacentNode) continue;
            Edge edge(adjacentNode, row[2]);
            node->edges.push_back(edge);
            
            adjacentNode->parents.emplace(std::piecewise_construct,
                              std::forward_as_tuple(node.get()),
                              std::forward_as_tuple(adjacentNode, row[2]));
        }
    }

void Graph::printGraph() {
    for (const auto& pair : nodes) {
        std::cout << "Node " << pair.first << " has edges to: ";
        for (const Edge& edge : pair.second->edges) {
            std::cout << "(" << edge.adjacentNode->value << ", weight: " << edge.weight << ") ";
        }
        std::cout << std::endl;
    }
}

void Graph::DFSUtil(int v, std::unordered_set<int>& visited) {
    visited.insert(v);
    std::cout << v << " ";

    for (const Edge& edge : nodes[v]->edges) {
        if (visited.find(edge.adjacentNode->value) == visited.end()) {
            DFSUtil(edge.adjacentNode->value, visited);
        }
    }
}

void Graph::DFS(int startValue) {
    std::unordered_set<int> visited;
    DFSUtil(startValue, visited);
    std::cout << std::endl;
}

// Breadth-First Search
void Graph::BFS(int startValue) {
    std::unordered_set<int> visited;
    std::queue<int> queue;

    visited.insert(startValue);
    queue.push(startValue);

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

        std::cout << current << " ";

        for (const Edge& edge : nodes[current]->edges) {
            if (visited.find(edge.adjacentNode->value) == visited.end()) {
                visited.insert(edge.adjacentNode->value);
                queue.push(edge.adjacentNode->value);
            }
        }
    }
    std::cout << std::endl;
}

void Graph::Dijkstra(int startValue) {
    std::unordered_map<int, int> distances;
    for (const auto& pair : nodes) {
        distances[pair.first] = std::numeric_limits<int>::max();
    }
    distances[startValue] = 0;

    auto compare = [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return a.second > b.second;
    };
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(compare)> queue(compare);

    queue.push(std::make_pair(startValue, 0));

    while (!queue.empty()) {
        int current = queue.top().first;
        queue.pop();

        for (const Edge& edge : nodes[current]->edges) {
            int alt = distances[current] + edge.weight;
            if (alt < distances[edge.adjacentNode->value]) {
                distances[edge.adjacentNode->value] = alt;
                queue.push(std::make_pair(edge.adjacentNode->value, alt));
            }
        }
    }

    for (const auto& distance : distances) {
        std::cout << "Node " << distance.first << " has minimum distance " << distance.second << std::endl;
    }
}
    void Graph::AStar(int startValue, int goalValue) {
    auto heuristic = [goalValue](int nodeValue) -> int {
        // A more complex heuristic should be used in practice
        // For now, it's a dummy heuristic (equivalent to Dijkstra's algorithm)
        return 0;
    };

    std::unordered_map<int, int> gScore;
    for (const auto& node : nodes) {
        gScore[node.first] = std::numeric_limits<int>::max();
    }
    gScore[startValue] = 0;

    std::unordered_map<int, int> cameFrom;

    std::unordered_set<int> openSetNodes;

    auto compare = [&](const int lhs, const int rhs) {
        int fScore_lhs = gScore[lhs] + heuristic(lhs);
        int fScore_rhs = gScore[rhs] + heuristic(rhs);
        return fScore_lhs > fScore_rhs;
    };
    std::priority_queue<int, std::vector<int>, decltype(compare)> openSet(compare);

    openSet.push(startValue);
    openSetNodes.insert(startValue);

    while (!openSet.empty()) {
        int current = openSet.top();
        openSet.pop();
        openSetNodes.erase(current);

        // If it's the goal, reconstruct the path and return it
        if (current == goalValue) {
            std::vector<int> path;
            while (current != startValue) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(startValue);
            std::reverse(path.begin(), path.end());

            std::cout << "Path: ";
            for (int node : path) {
                std::cout << node << " ";
            }
            std::cout << std::endl;
            return;
        }

        for (const Edge& edge : nodes[current]->edges) {
            int neighbor = edge.adjacentNode->value;
            int tentative_gScore = gScore[current] + edge.weight;

            if (tentative_gScore < gScore[neighbor]) {
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;

                if (openSetNodes.count(neighbor) == 0) {
                    openSet.push(neighbor);
                    openSetNodes.insert(neighbor);
                }
            }
        }
    }

    std::cout << "No path found." << std::endl;
}


std::vector<int> Graph::BFS_find_path(int sourceValue, int sinkValue, std::unordered_map<int, std::unordered_map<int, int>>& residual) {
    std::unordered_map<int, int> parent;
    std::unordered_set<int> visited;
    std::queue<int> queue;
    queue.push(sourceValue);
    visited.insert(sourceValue);
    parent[sourceValue] = -1; // Source has no parent

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

        for (const Edge& edge : nodes[current]->edges) {
            int adjValue = edge.adjacentNode->value;
            if (visited.find(adjValue) == visited.end() && residual[current][adjValue] > 0) {
                parent[adjValue] = current;
                visited.insert(adjValue);
                queue.push(adjValue);
                if (adjValue == sinkValue) { // Reached sink, path found
                    std::vector<int> path;
                    int crawl = sinkValue;
                    while (parent[crawl] != -1) {
                        path.push_back(crawl);
                        crawl = parent[crawl];
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }
            }
        }
    }

    return std::vector<int>(); // No path found
}

int Graph::EdmondsKarp(int sourceValue, int sinkValue) {
    // Initialize residual capacities
    std::unordered_map<int, std::unordered_map<int, int>> residual;
    for (const auto& node : nodes) {
        for (const Edge& edge : node.second->edges) {
            residual[node.first][edge.adjacentNode->value] = edge.weight;
            // Initialize reverse edge in residual graph
            residual[edge.adjacentNode->value][node.first] = 0;
        }
    }

    int max_flow = 0;
    std::vector<int> path = BFS_find_path(sourceValue, sinkValue, residual);

    while (!path.empty()) {
        // Find minimum residual capacity of the edges along the path
        int path_flow = std::numeric_limits<int>::max();
        for (size_t i = 0; i < path.size() - 1; i++) {
            int u = path[i];
            int v = path[i + 1];
            path_flow = std::min(path_flow, residual[u][v]);
        }

        // Update residual capacities of the edges and reverse edges
        for (size_t i = 0; i < path.size() - 1; i++) {
            int u = path[i];
            int v = path[i + 1];
            residual[u][v] -= path_flow;
            residual[v][u] += path_flow;
        }

        max_flow += path_flow;
        path = BFS_find_path(sourceValue, sinkValue, residual);
    }

    return max_flow;
}

bool Graph::BellmanFord(int startValue, std::unordered_map<int, int>& distances) {
    // Initialize distances
    for (auto& node : nodes) {
        distances[node.first] = std::numeric_limits<int>::max();
    }
    distances[startValue] = 0;

    // Relax edges repeatedly
    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        for (const auto& node : nodes) {
            for (const Edge& edge : node.second->edges) {
                int u = node.first;
                int v = edge.adjacentNode->value;
                int weight = edge.weight;
                if (distances[u] != std::numeric_limits<int>::max() && distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                }
            }
        }
    }

    // Check for negative weight cycles
    for (const auto& node : nodes) {
        for (const Edge& edge : node.second->edges) {
            int u = node.first;
            int v = edge.adjacentNode->value;
            int weight = edge.weight;
            if (distances[u] != std::numeric_limits<int>::max() && distances[u] + weight < distances[v]) {
                std::cout << "Graph contains a negative weight cycle" << std::endl;
                return false; // Negative weight cycle detected
            }
        }
    }

    return true; // No negative weight cycles found
}
