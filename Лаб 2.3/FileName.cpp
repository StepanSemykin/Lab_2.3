#include <iostream>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <iterator>
#include <string>
#include <queue>
#include <limits>

using namespace std;

template<typename Vertex, typename Distance = double>
struct Edge
{
    Vertex from, to;
    Distance len;
};

template<typename Vertex, typename Distance = double>
class Graph
{
public:
    struct Edge
    {
        Vertex from, to;
        Distance len;
    };

private:
    unordered_map<Vertex, vector<Edge>> _data;

public:
    Graph() {};

    ~Graph()
    {
        _data.clear();
    }

    bool has_vertex(const Vertex& v) const
    {
        if (_data.find(v) == _data.end()) return false;
        return true;
    }

    void add_vertex(const Vertex& v)
    {
        if (has_vertex(v)) return;
        _data.insert({ v, {} });
    }

    bool remove_vertex(const Vertex& v)
    {
        if (!has_vertex(v)) return false;
        vector<Vertex> connected_vertices_from;
        vector<Vertex> connected_vertices_to;
        for (const auto& e : _data.find(v)->second)
        {
            if (e.from == v && e.to == v) continue;
            if (e.from == v) connected_vertices_from.push_back(e.to);
            else if (e.to == v) connected_vertices_to.push_back(e.from);
        }
        for (const auto& vertex : connected_vertices_from)
        {
            remove_edge(v, vertex);
        }
        for (const auto& vertex : connected_vertices_to)
        {
            remove_edge(vertex, v);
        }
        _data.erase(v);
        return true;
    }

    unique_ptr<vector<Vertex>> get_vertices() const
    {
        vector<Vertex> vertices;
        vertices.reserve(_data.size());
        for (const auto& [vertex, edge] : _data)
        {
            vertices.push_back(vertex);
        }
        return make_unique<vector<Vertex>>(vertices);
    }

    bool has_edge(const Vertex& from, const Vertex& to) const
    {
        if (!has_vertex(from) || !has_vertex(to)) throw invalid_argument("Graph does not have a given vertex");
        auto& edges = _data.find(from)->second;
        auto res = find_if(edges.begin(), edges.end(), [&](const Edge& edge) { return edge.to == to && edge.from == from; });
        if (res == edges.end()) return false;
        return true;
    }

    bool has_edge(const Edge& e) const
    {
        if (!has_vertex(e.from) || !has_vertex(e.to)) throw invalid_argument("Graph does not have a given vertex");
        auto& edges = _data.find(e.from)->second;
        auto res = find_if(edges.begin(), edges.end(), [&](const Edge& edge) { return edge.to == e.to && edge.from == e.from && edge.len == e.len; });
        if (res == edges.end()) return false;
        return true;
    }

    void add_edge(const Vertex& from, const Vertex& to, const Distance& d)
    {
        if (!has_vertex(from) || !has_vertex(to)) throw invalid_argument("Graph does not have a given vertex");
        if (from == to) _data[from].push_back({ from, to, d });
        else
        {
            _data[from].push_back({ from, to, d });
            _data[to].push_back({ from, to, d });
        }
    }

    bool remove_edge(const Vertex& from, const Vertex& to)
    {
        if (!has_vertex(from) || !has_vertex(to)) throw invalid_argument("Graph does not have a given vertex");
        if (!has_edge(from, to)) return false;
        auto& edges_from = _data.find(from)->second;
        auto& edges_to = _data.find(to)->second;
        remove_if(edges_from.begin(), edges_from.end(), [&](const auto& e) { return e.from == from && e.to == to; });
        remove_if(edges_to.begin(), edges_to.end(), [&](const auto& e) { return e.from == from && e.to == to; });
        return true;
    }

    bool remove_edge(const Edge& e)
    {
        if (!has_vertex(e.from) || !has_vertex(e.to)) throw invalid_argument("Graph does not have a given vertex");
        if (!has_edge(e)) return false;
        auto& edges_from = _data.find(e.from)->second;
        auto& edges_to = _data.find(e.to)->second;
        remove_if(edges_from.begin(), edges_from.end(), [&](const auto& edge) { return edge.from == e.from && edge.to == e.to && edge.len == e.len; });
        remove_if(edges_to.begin(), edges_to.end(), [&](const auto& edge) { return edge.from == e.from && edge.to == e.to && edge.len == e.len; });
        return true;
    }

    unique_ptr<vector<Edge>> get_edges(const Vertex& vertex) const
    {
        if (!has_vertex(vertex)) throw invalid_argument("Graph does not have a given vertex");
        vector<Edge> edges;
        edges.reserve(_data.find(vertex)->second.size());
        for (const auto& edge : _data.find(vertex)->second)
        {
            edges.push_back(edge);
        }
        return make_unique<vector<Edge>>(edges);
    }

    size_t order() const
    {
        return _data.size();
    }

    size_t degree(const Vertex& vertex) const
    {
        if (!has_vertex(vertex)) throw invalid_argument("Graph does not have a given vertex");
        return _data[vertex].size();
    }

    Distance get_len(const Vertex& from, const Vertex& to) const
    {
        for (const auto& e : _data.find(from)->second)
        {
            if (e.from == from && e.to == to) return e.len;
        }
        return numeric_limits<Distance>::infinity();
    }

    unique_ptr<vector<Edge>> shortest_path(const Vertex& from, const Vertex& to) const
    {
        if (!has_vertex(from) || !has_vertex(to)) throw invalid_argument("Graph does not have a given vertex");
        unordered_map<Vertex, Distance> d;
        for (const auto& [vertex, edge] : _data)
        {
            d.insert({ vertex, {numeric_limits<Distance>::infinity()} });
        }
        d[from] = 0;
        unordered_map<Vertex, Vertex> prevs;
        priority_queue<pair<Distance, Vertex>, vector<pair<Distance, Vertex>>, greater<pair<Distance, Vertex>>> q;
        q.push(make_pair(0, from));
        while (!q.empty())
        {
            Vertex v = q.top().second;
            q.pop();
            for (const auto& [vertex, dist] : d)
            {
                if (d[vertex] > d[v] + get_len(v, vertex))
                {
                    d[vertex] = d[v] + get_len(v, vertex);
                    q.push(make_pair(dist, vertex));
                    prevs[vertex] = v;
                }
            }
        }
        vector<Vertex> path_vert;
        Vertex curr = to;
        while (curr != from)
        {
            path_vert.push_back(curr);
            curr = prevs[curr];
        }
        path_vert.push_back(from);
        reverse(path_vert.begin(), path_vert.end());
        //for (const auto& v : path_vert)
        //{
        //    cout << v << " ";
        //}
        vector<Edge> path_edges;
        for (auto it_1 = path_vert.begin(); (it_1 + 1) != path_vert.end(); it_1++)
        {
            auto& edges = _data.find(*it_1)->second;
            for (const auto& e : edges)
            {
                if (e.from == *it_1 && e.to == *(it_1 + 1)) path_edges.push_back(e);
            }
        }
        //for (const auto& p : path_edges)
        //{
        //    cout << "from: " << p.from << " to: " << p.to << " len: " << p.len << ";  ";
        //}
        //cout << endl;
        //for (const auto& [vertex, edge] : d)
        //{
        //    cout << vertex << ": " << edge << endl;
        //}
        return make_unique<vector<Edge>>(path_edges);
    }

    Vertex storage()
    {
        priority_queue<pair<Distance, Vertex>, vector<pair<Distance, Vertex>>, greater<pair<Distance, Vertex>>> res;;
        for (const auto& [vertex_1, edge_1] : _data)
        {
            Vertex from = vertex_1;
            unordered_map<Vertex, Distance> d;
            for (const auto& [vertex_2, edge_2] : _data)
            {
                d.insert({ vertex_2, {numeric_limits<Distance>::infinity()} });
            }
            d[from] = 0;
            priority_queue< pair<Distance, Vertex>, vector<pair<Distance, Vertex>>, greater<pair<Distance, Vertex>> > q;
            q.push(make_pair(0, from));
            while (!q.empty())
            {
                Vertex v = q.top().second;
                q.pop();
                for (const auto& [vertex, dist] : d)
                {
                    if (d[vertex] > d[v] + get_len(v, vertex))
                    {
                        d[vertex] = d[v] + get_len(v, vertex);
                        q.push(make_pair(dist, vertex));
                    }
                }
            }
            Distance average = 0;
            for (const auto& [vertex, dist] : d)
            {
                average += dist;
            }
            average /= (d.size() - 1);
            cout << "average: " << average;
            res.push(make_pair(average, from));
            cout << endl;
            for (const auto& [vertex, dist] : d)
            {
                cout << vertex << ": " << dist << endl;
            }
        }
        Vertex v = res.top().second;
        cout << "Storage: " << v << endl;
        return v;
    }

    unique_ptr<vector<Vertex>> walk(const Vertex& start_vertex) 
    {
        unordered_map<Vertex, bool> visited;
        vector<Vertex> result;
        result.reserve(_data.size());
        dfs(visited, start_vertex, result);
        for (const auto& [vertex, edge] : _data)
        {
            if (find(result.begin(), result.end(), vertex) == result.end())
            {
                dfs(visited, vertex, result);
            }
        }
        return make_unique<vector<Vertex>>(result);
    }

    void dfs(unordered_map<Vertex, bool>& visited, const Vertex& current_vertex, vector<Vertex>& result) 
    {
        visited[current_vertex] = true;
        result.push_back(current_vertex);
        for (auto& edge : _data[current_vertex])
        {
            const auto& next_vertex = edge.to;
            if (!visited[next_vertex]) dfs(visited, next_vertex, result);
        }
    }

    friend ostream& operator << (ostream& out, const Graph& g)
    {
        for (const auto& [vertex, edge] : g._data)
        {
            out << vertex << ": ";
            for (const auto& e : edge)
            {
                out << "from: " << e.from << " to: " << e.to << " len: " << e.len << "; ";
            }
            out << endl;
        }
        return out;
    }
};

int main()
{
    Graph<size_t, double> g;
    g.add_vertex(1);
    g.add_vertex(2);
    g.add_vertex(3);
    g.add_vertex(4);
    g.add_edge(1, 2, 2);
    g.add_edge(4, 4, 1);
    g.add_edge(3, 1, 2);
    g.add_edge(2, 3, 5);
    g.add_edge(3, 4, 5);
    g.add_edge(2, 4, 20);

    cout << g << endl << endl;

    g.remove_edge(3, 1);

    cout << g << endl << endl;

    cout << "Has edge 3->1: " << g.has_edge(3, 1) << endl;
    cout << "Has vertex 5: " << g.has_vertex(5) << endl;
    cout << "Len 2->1: " << g.get_len(2, 1) << endl;
    
    auto path = g.shortest_path(1, 4);
    cout << "Shortest path 1->4: ";
    for (auto& i : *path)
    {
        cout << "from: " << i.from << " to: " << i.to << " len: " << i.len << "; ";
    }
    cout << endl;

    unique_ptr<vector<size_t>> result = g.walk(1);
    cout << "Walk: ";
    for (auto i : *result) 
    {
        cout << i << "\t";
    }
    cout << endl;
    
    auto result_vert = g.get_vertices();
    cout << "Vertex: ";
    for (auto& i : *result_vert) 
    {
        cout << i << "\t";
    }
    cout << endl;

    auto result_edges = g.get_edges(2);
    cout << "Edges: ";
    for (auto& i : *result_edges) 
    {
        cout << "from: " << i.from << " to: " << i.to << " len: " << i.len << "; ";
    }
    cout << endl;

    Graph<size_t, double> gr;
    gr.add_vertex(1);
    gr.add_vertex(2);
    gr.add_vertex(3);
    gr.add_edge(1, 2, 1);
    gr.add_edge(1, 3, 7);
    gr.add_edge(2, 1, 2);
    gr.add_edge(2, 3, 3);
    gr.add_edge(3, 1, 1);
    gr.add_edge(3, 2, 5);

    gr.storage();

    return 0;
}

