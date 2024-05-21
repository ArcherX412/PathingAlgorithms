#include <iostream>
#include "utilities.hpp"
#include <vector>
#include <list>
#include <limits>
#include <queue>
#include <random>
#include <chrono>
#include <iomanip>


using namespace std;

class Timer {
private:
    using clock_t = std::chrono::high_resolution_clock;
    using second_t = std::chrono::duration<double, std::ratio<1>>;
    std::chrono::time_point<clock_t> m_start_time;

public:
    Timer() : m_start_time(clock_t::now()) {}

    void reset() {
        m_start_time = clock_t::now();
    }

    double elapsed() const {
        return std::chrono::duration_cast<second_t>(clock_t::now() - m_start_time).count();
    }
};

class Graph {

public:
    Graph(int verticies);
    void addEdge(int a, int b, int weight);
    void printGraph() const;
    void bellmanFord(int start,int target, vector<int>& heuristic) const;
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

void::Graph::bellmanFord(int start, int target, vector<int>& heuristic) const {

    Timer timer;

    vector<int> distance(V, numeric_limits<int>::max());
    vector<int> predecessor(V, -1); // Wektor poprzedników
    distance[start] = 0;

    for (int i = 1; i <= V - 1; ++i) {
        for (int u = 0; u < V; ++u) {
            for (const auto& neighbor : adjacentVerticies[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;
                if (distance[u] != numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                    predecessor[v] = u; // Aktualizacja poprzednika
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
    double time = timer.elapsed();
    // Wypisanie wyników
    for (int i = 0; i < V; ++i) {
        heuristic.push_back(distance[i]);
        //cout << "Odległość do wierzchołka " << i << " wynosi " << distance[i] << endl;
    }

    // Wypisanie ścieżki do celu
    if (distance[target] == numeric_limits<int>::max()) {
        cout << "Nie ma ścieżki do wierzchołka " << target << endl;
    }
    else {
        cout << "Droga do celu: ";
        vector<int> path;
        for (int at = target; at != -1; at = predecessor[at]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        for (size_t i = 0; i < path.size(); ++i) {
            cout << path[i];
            if (i < path.size() - 1) cout << " -> ";
        }
        cout << endl;
    }

    cout << "Odleglosc od startu do celu: " << distance[target]<<endl;
    cout <<"Czas wykonania: " << time << " sekund\n";
}

void Graph::aStar(int start, int goal, const vector<int>& heuristic) const {

    Timer timer;

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
            double time = timer.elapsed();

            cout << "Droga do celu: ";
            vector<int> path;
            for (int at = goal; at != -1; at = previous[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());
            for (int i = 0; i < path.size(); i++)
            {
                cout << path[i];
                if (i < path.size() - 1) cout << " -> ";
            }
            cout <<endl<<"Odleglosc od startu do celu: "<< distance[goal] << endl;

            
            cout <<"Czas wykonania: " << time << " sekund\n";

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
    Timer timer;

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

    cout << "Droga do celu: ";
    vector<int> path;
    for (int at = end; at != -1; at = parent[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());
    for (size_t i = 0; i < path.size(); ++i) {
        cout << path[i];
        if (i < path.size() - 1) cout << " -> ";
    }
    cout << endl;


    cout <<endl<< "Odleglosc od startu do celu: " << weights[end] << endl;

    double time = timer.elapsed();
    cout << "Czas wykonania: " << time << " sekund\n";
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

void Test(int size)
{
    vector<int> heuristic;
    heuristic.reserve(size);

    Graph g = getGraph(size);
    
    DisplayingText(to_string(size));

    DisplayingText("BELLMAN-FORD");
    g.bellmanFord(0, size/2, heuristic);

    DisplayingText("A*");

    g.aStar(0, size / 2, heuristic);

    DisplayingText("DJIKSTRA");
    Djikstra(g, 0, size / 2);
}

int main()
{
    setlocale(LC_ALL, "polish");

    Test(100);
    Test(500);
    Test(1000);

    return 0;
}


