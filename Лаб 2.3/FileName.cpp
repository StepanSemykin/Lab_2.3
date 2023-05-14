#include<iostream>
#include <unordered_map>
#include <string>

using namespace std;

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
        for (const auto& [vertex, edge] : _data)
        {
            if (vertex == v) return true;
        }
        return false;
    }

    void add_vertex(const Vertex& v)
    {
        if (has_vertex(v)) return;
        _data.insert({ v, {} });
    }

    bool remove_vertex(const Vertex& v)
    {
        if (!has_vertex(v)) return false;
        _data.erase(v);
        return true;
    }

    vector<Vertex> get_vertices() const;


    //проверка-добавление-удаление ребер
    void add_edge(const Vertex& from, const Vertex& to,
        const Distance& d);
    bool remove_edge(const Vertex& from, const Vertex& to);
    bool remove_edge(const Edge& e); //c учетом расстояния
    bool has_edge(const Vertex& from, const Vertex& to) const;
    bool has_edge(const Edge& e); //c учетом расстояния в Edge

    //получение всех ребер, выходящих из вершины
    vector<Edge> get_edges(const Vertex& vertex);

    size_t order() const; //порядок
    size_t degree() const; //степень


    //поиск кратчайшего пути
    vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const;
    //обход
    vector<Vertex> walk(const Vertex& start_vertex) const;
};

int main()
{
    //unordered_map<string, vector<int>> freqs;
    //string word;
    //vector<int> a, b, c;
    //a.push_back(1);
    //a.push_back(2);
    //b.push_back(3);
    //b.push_back(4);
    //b.push_back(5);
    //c.push_back(6);
    //c.push_back(7);
    //freqs["GeeksforGeeks"] = a;
    //freqs["Practice"] = b;
    //freqs["Contribute"] = c;
    //for (const auto& [word, freq] : freqs) 
    //{
    //    cout << word << "\t" << freq[0] << "\n";
    //}

    return 0;
}
