#include <iostream>
#include <vector>
#include <limits.h>
#include <string.h>
#include <queue>
#include <algorithm>

using namespace std;


// Замутил страшный класс графов с устрашающим кол-вом методов для удобства использования, но не чтения)
class Graph {
public:
    vector < vector < vector < int > > > Edges;
    int VERTICES_NUM;


    Graph(vector < vector < vector < int > > > input_edges) {
        Edges = input_edges;
        VERTICES_NUM = input_edges.size();
    }


    void print_graph() {
        for (int i = 0; i < Edges.size(); i++)
            for (int j = 1; j < Edges[i].size(); j++)
                cout << i << " - " << Edges[i][j][0] << endl;
    }


    // Затяпал перегрузку функции weight: в первом случае функция возвращает текущий вес ребра,
    // а во втором - меняет вес ребра.
    int weight(int src, int dir) {
        for (int i = 1; i < Edges[src].size(); i++) {
            if (Edges[src][i][0] == dir) {
                return Edges[src][i][1];
            }
        }
        return -1;
    }

    void weight(int src, int dir, int new_weight) {
        for (int i = 1; i < Edges[src].size(); i++) {
            if (Edges[src][i][0] == dir) {
                Edges[src][i][1] = new_weight;
                break;
            }
        }
    }


    void delete_edge(int src, int dir) {
        for (int i = 1; i < Edges[src].size(); i++) {
            if (Edges[src][i][0] == dir) {
                auto iter = Edges[src].cbegin();
                Edges[src].erase(iter + i);
                break;
            }
        }
    }


    void append_edge(int src, int dir, int weight) {
        Edges[src].push_back({ dir, weight });
    }


    vector<int> Dijkstra(int src, int dir) {
        vector < vector < vector < int > > > edges = Edges;
        vector <int> dist(VERTICES_NUM, INT_MAX);
        dist[src] = 0;

        priority_queue < pair<int, int> > q;
        q.push(make_pair(src, 0));

        vector <int> parent(VERTICES_NUM, -1);

        while (!q.empty()) {
            int u = q.top().first;
            int l = q.top().second;
            q.pop();
            if (l > dist[u]) continue;
            for (int i = 0; i < edges[u].size(); i++) {
                int v = edges[u][i][0];
                int w = edges[u][i][1];
                if (dist[v] > dist[u] + w && dist[u] != INT_MAX) {
                    dist[v] = dist[u] + w;
                    q.push(make_pair(v, dist[v]));

                    parent[v] = u;
                }
            }
        }

        // В векторе parent хранятся все кратчайшие пути от заданной нами точки src до любой другой. 
        // При помощи рекурсии достанем нужный нам путь от src до dir в нормальном виде и вернем его.
        vector < int > way = fill_way(parent, dir, way);
        return way;
    }


    vector<int> fill_way(vector<int> parent, int vertex, vector <int> way) {
        if (vertex < 0) {
            if (way.size() == 1) return {};
            else return way;
        }

        way.insert(way.begin(), vertex);
        return fill_way(parent, parent[vertex], way);
    }
};




// Поиск максимального потока.
int Ford_Fulkerson(Graph& graph) {
    int max_flow = 0;
    vector < int > way = graph.Dijkstra(0, graph.VERTICES_NUM - 1);
    while (!way.empty()) {
        int min = INT_MAX;
        // Находим минимальное ребро в кратчайшем пути.
        for (int i = 1; i < way.size(); i++) {
            if (min > graph.weight(way[i - 1], way[i]) && graph.weight(way[i - 1], way[i]) != -1)
                min = graph.weight(way[i - 1], way[i]);
        }

        // Проворачиваем алгоритм.
        for (int i = 1; i < way.size(); i++) {
            int forward_weight = graph.weight(way[i - 1], way[i]);
            int back_weight = graph.weight(way[i], way[i - 1]);

            if (forward_weight - min != 0)
                graph.weight(way[i - 1], way[i], forward_weight - min);
            else
                graph.delete_edge(way[i - 1], way[i]);


            if (back_weight != -1)
                graph.weight(way[i], way[i - 1], back_weight + min);
            else
                graph.append_edge(way[i], way[i - 1], min);

        }

        max_flow += min;

        // Находим новый кратчайший путь, и если он не существует, то заканчиваем цикл.
        way = graph.Dijkstra(0, graph.VERTICES_NUM - 1);
    }

    return max_flow;
}

// Поиск минимального разреза.
void check_vertices(Graph graph, int s, vector <bool>& visited) {
    visited[s] = true;
    for (int i = 1; i < graph.Edges[s].size(); i++)
        if (!visited[graph.Edges[s][i][0]])
            check_vertices(graph, graph.Edges[s][i][0], visited);
}


void minCut(Graph graph, int s, int t)
{
    Graph rGraph(graph.Edges);
    // Прогоняем алгоритм Форда-Фалкерсона и работаем с измененным графом.
    Ford_Fulkerson(rGraph);

    // Проверяем все задействованные вершины.
    vector <bool> visited(rGraph.VERTICES_NUM, false);
    check_vertices(rGraph, s, visited);

    for (int i = 0; i < graph.Edges.size(); i++) {
        for (int j = 1; j < graph.Edges[i].size(); j++) {
            int u = i;
            int v = graph.Edges[u][j][0];

            if (visited[u] && !visited[v]) cout << u << " - " << v << endl;
        }
    }
}


int main()
{
    Graph graph({
        { {0, 0}, {1, 16}, {2, 13}},
        { {1, 0}, {2, 10}, {3, 12} },
        { {2, 0}, {1, 4}, {4, 14} },
        { {3, 0}, {2, 9}, {5, 20} },
        { {4, 0}, {3, 7}, {5, 4} },
        { {5, 0} }
        });

    minCut(graph, 0, 5);
}
