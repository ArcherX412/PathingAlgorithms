#include <iostream>
#include <string>
#include "utilities.hpp"
#include <vector>
#include <list>
#include <utility>
#include <math.h>
#include <limits>
#include <queue>
#include <random>


using namespace std;


class Graph {

public:
    Graph(int verticies);
    void addEdge(int a, int b, int weight);
    void printGraph() const;
    void bellmandFord(int start) const;
    void aStar(int start, int goal, const vector<int>& heuristic) const;

    const int V;  //liczba wierzcholkow
    vector<list<pair<int, int>>> adjacentVerticies;     //lista krawedzi wychodzaca z kazdego wierzcholka
};

Graph::Graph(int vertices) : V(vertices), adjacentVerticies(vertices) {}

void::Graph::addEdge(int a, int b, int weight)
{
    bool edgeExists = false;

    for (const auto& edge : adjacentVerticies[a]) {
        if (edge.first == b) {
            edgeExists = true;
            break;
        }
    }
    for (const auto& edge : adjacentVerticies[b]) {
        if (edge.first == a) {
            edgeExists = true;
            break;
        }
    }

    if (!edgeExists) {
        adjacentVerticies[a].push_back(make_pair(b, weight));
        adjacentVerticies[b].push_back(make_pair(a, weight));
    }


    //cout << "g.addEdge(" << a << ", " << b << ", " << weight << ");" << endl;
}

void::Graph::printGraph() const
{
    for (int v = 0; v < V; v++)
    {
        cout << "Wierzchoek " << v << ":";
        for (const auto& neighbor : adjacentVerticies[v])
        {
            cout << " -> (" << neighbor.first << ", waga: " << neighbor.second << ")";
        }
        cout << endl;
    }
}

void::Graph::bellmandFord(int start) const 
{
    vector<int> distance(V, numeric_limits<int>::max());
    distance[start] = 0;

    for (int i = 1; i <= V - 1; ++i) {
        for (int u = 0; u < V; ++u) {
            for (const auto& neighbor : adjacentVerticies[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;
                if (distance[u] != numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                }
            }
        }
    }

    // Sprawdzanie cykli o ujemnej wadze
    for (int u = 0; u < V; ++u) {
        for (const auto& neighbor : adjacentVerticies[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;
            if (distance[u] != numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
                cout << "Graf zawiera cykl o ujemnej wadze" << endl;
                return;
            }
        }
    }

    // Wypisanie wyników
    cout << "Wierzchołek startowy: " << start << endl;
    for (int i = 0; i < V; ++i) {
        cout << "Odległość do wierzchołka " << i << " wynosi " << distance[i] << endl;
    }
}

void Graph::aStar(int start, int goal, const vector<int>& heuristic) const {
    vector<int> distance(V, numeric_limits<int>::max());
    distance[start] = 0;
    vector<int> previous(V, -1);
    vector<bool> closedSet(V, false);

    auto compare = [&](int lhs, int rhs) {
        return distance[lhs] + heuristic[lhs] > distance[rhs] + heuristic[rhs];
        };

    priority_queue<int, vector<int>, decltype(compare)> openSet(compare);
    openSet.push(start);

    while (!openSet.empty()) {
        int current = openSet.top();
        openSet.pop();

        if (current == goal) {
            cout << "Ścieżka znaleziona: ";
            vector<int> path;
            for (int at = goal; at != -1; at = previous[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());
            for (int vertex : path) {
                cout << vertex << " ";
            }
            cout <<endl<<distance[goal]<<endl;
            return;
        }

        closedSet[current] = true;

        for (const auto& neighbor : adjacentVerticies[current]) {
            int neighborNode = neighbor.first;
            int weight = neighbor.second;

            if (closedSet[neighborNode]) {
                continue;
            }

            int tentative_gScore = distance[current] + weight;

            if (tentative_gScore < distance[neighborNode]) {
                previous[neighborNode] = current;
                distance[neighborNode] = tentative_gScore;

                openSet.push(neighborNode);
            }
        }
    }

    cout << "Ścieżka nie została znaleziona." << endl;
}

int findCheapestNode_Djikstra(Graph g, int* weights,bool* explored)
{
    int node = 0;
    int cost = 100000;
    for (int v = 0; v < g.V; v++)
    {
        if (weights[v] < cost && explored[v] == false)
        {
            cost = weights[v];
            node = v;
        }
    }
    return node;
    
}

void Djikstra(Graph g, int start, int end)
{
    int* weights = new int[g.V];
    bool* explored = new bool[g.V];
    int* parent = new int[g.V];

    for (int v = 0; v < g.V; v++)
    {
        weights[v] = 100000;
        explored[v] = false;
    }

    weights[start] = 0;
    parent[start] = -1;

    int currentNode = 0;

    for (int v = 0; v < g.V; v++)
    {
        currentNode = findCheapestNode_Djikstra(g, weights, explored);
        explored[currentNode] = true;
        
        int cost = weights[currentNode];
        for (const auto& neighbor : g.adjacentVerticies[currentNode])
        {
            int newCost = cost + neighbor.second;
            if (newCost < weights[neighbor.first])
            {
                weights[neighbor.first] = newCost;
                parent[neighbor.first] = currentNode;
            }
        }
    }

    cout << "Odleglosc od startu do celu: "<< weights[end]<<endl;

    int n = end;
    cout << "Droga do celu: ";
    cout << n;
    do
    {
        n = parent[n];

        cout << "<-" << n;
        
    } while (n != -1);

}

Graph getGraph(int size)
{
    Graph g(size);
    random_device randomGenerator;
    mt19937 gen(randomGenerator());
    int a, b, cost;

    for (int i = 0; i < size; i++)
    {
        if (i < 5)
        {
            for (int j = 0; j < 1; j++)
            {
                uniform_int_distribution<> distr(0, 4);
                a = i;
                b = a;
                while (b == a)
                {
                    b = distr(gen);
                }
                uniform_int_distribution<> sraka(1, 50);
                cost = sraka(gen);
                g.addEdge(a, b, cost);
            }           

        }
        else if (i < 15)
        {
            for (int j = 0; j < 3; j++)
            {
                uniform_int_distribution<> distr(0, 14);
                a = i;
                b = a;
                while (b == a)
                {
                    b = distr(gen);
                }
                uniform_int_distribution<> sraka(1, 50);
                cost = sraka(gen);
                g.addEdge(a, b, cost);
            }
        }
        else
        {
            for (int j = 0; j < 4; j++)
            {
                uniform_int_distribution<> distr(0, size-1);
                a = i;
                b = a;
                while (b == a)
                {
                    b = distr(gen);
                }
                uniform_int_distribution<> sraka(1, 50);
                cost = sraka(gen);
                g.addEdge(a, b, cost);
            }
        }

    }

    return g;
}


int main()
{
    setlocale(LC_ALL, "polish");
    int testSize = 1000;
    int target = 768;
    /*Graph g(6);
    g.addEdge(0, 1, 7);
    g.addEdge(0, 2, 9);
    g.addEdge(0, 5, 14);
    g.addEdge(1, 2, 10);
    g.addEdge(1, 3, 15);
    g.addEdge(2, 3, 11);
    g.addEdge(2, 5, 2);
    g.addEdge(3, 4, 6);
    g.addEdge(4, 5, 9);*/

    Graph g = getGraph(testSize);
    //g.printGraph();

    DisplayingText("DJIKSTRA");
    Djikstra(g, 0, target);

    /*DisplayingText("BELLMAN-FORD");
    g.bellmandFord(0);*/

    DisplayingText("A*");
    vector<int> heuristic = {};
    g.aStar(0, target, heuristic);

    return 0;
}


