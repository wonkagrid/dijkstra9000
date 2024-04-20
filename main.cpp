#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

// вершина графа
struct Node {
    int id; // номер вершины
    int weight;
    vector<Node*> adj;
    Node(int _id,int _weight){
        this->id = _id;
        this->weight = _weight;
    }
    void addNeighbour(Node* nd) //смежные узлы
    {
        nd->adj.push_back(this);
        adj.push_back(nd);
    }
};

class WeightedGraph {
public:
    vector<Node*> nodes;

    // добавление узла в граф
    void addNode(Node *nd) {
        nodes.push_back(nd);
    }
    // собственно сам алгоритм дейкстры с использованием встроенной в stl кучи
    vector<int> dijkstra(int start) {
        vector<int> distance(nodes.size(), numeric_limits<int>::max());
        distance[start] = nodes[start]->weight;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
        pq.emplace(0, start);

        // основной цикл
        while (!pq.empty()) {
            int u = pq.top().second; // извлекаем вершину с минимальным расстоянием
            pq.pop();

            // обходим все смежные с u вершины и обновляем расстояния
            for (Node* v : nodes[u]->adj) {
                int weight = v->weight;
                // если найден более короткий путь к вершине v через вершину u
                if (distance[v->id] > distance[u] + weight) {
                    distance[v->id] = distance[u] + weight; // обновляем расстояние
                    pq.emplace(distance[v->id], v->id); // добавляем вершину v в очередь
                }
            }
        }
        return distance;
    }
};

int main() {
    system("chcp 65001");
    WeightedGraph graph;
    Node* nd0 = new Node(0, 6);
    Node* nd1 = new Node(1, 10);
    Node* nd2 = new Node(2, 3);
    Node* nd3 = new Node(3, 4);
    Node* nd4 = new Node(4, 2);

    graph.addNode(nd0);
    graph.addNode(nd1);
    graph.addNode(nd2);
    graph.addNode(nd3);
    graph.addNode(nd4);

    nd0->addNeighbour(nd1);
    nd0->addNeighbour(nd2);
    nd0->addNeighbour(nd4);
    nd1->addNeighbour(nd3);
    nd2->addNeighbour(nd3);
    nd3->addNeighbour(nd4);

    vector<int> distances = graph.dijkstra(1);
    cout << "Кратчайшие расстояния от вершины 0:" << endl;
    for (int i = 0; i < distances.size(); ++i) {
        cout << "Вершина " << i << ": " << distances[i] << endl;
    }
    return 0;
}
