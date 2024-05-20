#include <iostream>
#include <string>
#include "utilities.hpp"
#include <vector>
#include <list>
#include <utility>
#include <math.h>

using namespace std;


class Graph {

public:
    Graph(int verticies);
    void addEdge(int a, int b, int weight);
    void printGraph() const;


    const int V;  //liczba wierzcholkow
    vector<list<pair<int, int>>> adjacentVerticies;     //lista krawedzi wychodzaca z kazdego wierzcholka
};

Graph::Graph(int vertices) : V(vertices), adjacentVerticies(vertices) {}

void::Graph::addEdge(int a, int b, int weight)
{
    adjacentVerticies[a].push_back(make_pair(b, weight));
    adjacentVerticies[b].push_back(make_pair(a, weight));
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

void BellmanFord(Graph g, int start, int end)
{

}

int main()
{
    Graph g(6);
    g.addEdge(0, 1, 7);
    g.addEdge(0, 2, 9);
    g.addEdge(0, 5, 14);
    g.addEdge(1, 2, 10);
    g.addEdge(1, 3, 15);
    g.addEdge(2, 3, 11);
    g.addEdge(2, 5, 2);
    g.addEdge(3, 4, 6);
    g.addEdge(4, 5, 9);


    g.printGraph();


    Djikstra(g, 0, 4);
}


