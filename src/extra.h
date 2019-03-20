#ifndef SEG_GRAPH_H
#define SEG_GRAPH_H
#include <vector>
class Area {
public:
    Area(int cols) {
        columm = cols;
    }

    void addPixel(int x_axis, int y_axis) {
        int pixelId = x_axis * columm + y_axis;
        pixels.insert(pixelId);
    }

    std::set <int> getPixels() {
        return pixels;
    }

    int getImageColumns() {
        return columm;
    }

    long getPixelsCount() {
        return pixels.size();
    }

private:
    int columm;
    std::set <int> pixels;
};

class Edge {
public:
    int vertices;
    double flow;
    double weight;
    int rev;
    int pixelX;
    int pixelY;
};

class Graph {
public:
    Graph(int vert) {
        vertices = vert;
        adj = new std::vector<Edge>[vertices];
    }

    void addEdge(int u, int v, double edge, int x, int y) {
        Edge a;
        a.vertices = v;
        a.flow = 0;
        a.weight = edge;
        a.rev = adj[v].size();
        a.pixelX = x;
        a.pixelY = y;

        Edge b;
        b.vertices = u;
        b.flow = 0;
        b.weight = 0;
        b.rev = adj[v].size();
        b.pixelX = x;
        b.pixelY = y + 1;

        adj[u].push_back(a);
        adj[v].push_back(b); 
    }

    std::vector<Edge> &getNearbyNodes(int pixel) {
        return adj[pixel];
    }
    void enterNeigbouringNodes(std::vector<Edge> nodes, int index)
    {
        adj[index] = nodes;
    }
    void updateEdge(int pixel, int nextPixel, double pathFlow) {
        adj[pixel][nextPixel].flow -= pathFlow;
    }

    int getVertices() {
        return vertices;
    }
private:
    int vertices;
    std::vector<Edge> *adj;
};
#endif
