#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <string>
#include <algorithm>
#include <limits>
#include <memory>
#include <cmath>
#include <cassert>

using namespace std;

class Node; 

class Edge {
private:
    int id;
    Node* source;
    Node* target;
    double distance;
    double baseTime;
    double capacity;
    double currentFlow;
    string roadType;
    unordered_map<int, double> timeFactors;

public:
    Edge(int id, Node* src, Node* tgt,
         double dist, double time, double cap,
         const string& type = "street")
        : id(id), source(src), target(tgt), distance(dist),
          baseTime(time), capacity(cap), currentFlow(0.0),
          roadType(type)
    {
        timeFactors[-1] = 1.0;
    }

    ~Edge() = default;

    int getId() const { return id; }
    Node* getSource() const { return source; }
    Node* getTarget() const { return target; }
    double getDistance() const { return distance; }
    double getBaseTime() const { return baseTime; }
    double getCapacity() const { return capacity; }
    const string& getRoadType() const { return roadType; }

    void setCurrentFlow(double flow) {
        currentFlow = std::max(0.0, std::min(flow, capacity));
    }

    double getCurrentFlow() const { return currentFlow; }

    void setTimeFactor(int timeSlot, double factor) {
        timeFactors[timeSlot] = max(factor, 1.0);
    }

    double getTimeFactor(int timeSlot) const {
        auto it = timeFactors.find(timeSlot);
        if (it != timeFactors.end()) return it->second;
        return timeFactors.at(-1);
    }

    double getTravelTime(int currentTimeSlot = -1) const {
        double tf = getTimeFactor(currentTimeSlot);
        double cong = 1.0;
        if (capacity > 0.0) {
            double ratio = currentFlow / capacity;
            cong = 1.0 + 0.15 * pow(ratio, 4);
        }
        return baseTime * tf * cong;
    }

    double getCongestionLevel() const {
        if (capacity <= 0.0) return 0.0;
        return min(currentFlow / capacity, 1.0);
    }
};

class Node {
private:
    int id;
    string name;
    double latitude;
    double longitude;
    unordered_map<int, Edge*> outgoingEdges;

public:
    Node(int id, const string& name = "", double lat = 0.0, double lon = 0.0)
        : id(id), name(name), latitude(lat), longitude(lon)
    {}

    ~Node() = default;

    int getId() const { return id; }
    const string& getName() const { return name; }
    pair<double, double> getCoordinates() const { return {latitude, longitude}; }

    void addOutgoingEdge(Edge* e) {
        if (e->getSource()->getId() == id) {
            outgoingEdges[e->getTarget()->getId()] = e;
        }
    }

    void removeOutgoingEdge(int targetNodeId) {
        outgoingEdges.erase(targetNodeId);
    }

    Edge* getEdgeTo(int targetNodeId) const {
        auto it = outgoingEdges.find(targetNodeId);
        return (it != outgoingEdges.end()) ? it->second : nullptr;
    }

    vector<Edge*> getAllOutgoingEdges() const {
        vector<Edge*> v;
        v.reserve(outgoingEdges.size());
        for (auto& kv : outgoingEdges) v.push_back(kv.second);
        return v;
    }

    bool operator==(const Node& other) const {
        return id == other.id;
    }
};

class Graph {
private:
    unordered_map<int, unique_ptr<Node>> nodes;
    vector<unique_ptr<Edge>> edges;

public:
    Graph() = default;

    ~Graph() = default;

    void addNode(int id, const string& name = "", double lat = 0.0, double lon = 0.0) {
        if (nodes.count(id)) throw runtime_error("Node with ID exists");
        nodes[id] = make_unique<Node>(id, name, lat, lon);
    }

    Node* getNode(int id) const {
        auto it = nodes.find(id);
        return (it != nodes.end()) ? it->second.get() : nullptr;
    }

    void addEdge(int edgeId, int srcId, int tgtId,
                 double dist, double time, double cap,
                 const string& type = "street") {
        Node* s = getNode(srcId);
        Node* t = getNode(tgtId);
        if (!s || !t) throw runtime_error("Invalid node IDs");
        edges.emplace_back(make_unique<Edge>(edgeId, s, t, dist, time, cap, type));
        s->addOutgoingEdge(edges.back().get());
    }

    vector<Node*> dijkstraShortestPath(int startId, int endId) {
        auto cmp = [](auto& a, auto& b){ return a.first > b.first; };
        priority_queue<pair<double, Node*>, vector<pair<double, Node*>>, decltype(cmp)> pq(cmp);
        unordered_map<int, double> dist;
        unordered_map<int, Node*> prev;

        for (auto& kv : nodes) dist[kv.first] = numeric_limits<double>::infinity();
        dist[startId] = 0.0;
        pq.push({0.0, getNode(startId)});

        while (!pq.empty()) {
            auto top = pq.top(); pq.pop();
            double d = top.first;
            Node* u = top.second;

            if (u->getId() == endId) break;
            if (d > dist[u->getId()]) continue;

            for (auto* e : u->getAllOutgoingEdges()) {
                Node* v = e->getTarget();
                double nd = d + e->getDistance();
                if (nd < dist[v->getId()]) {
                    dist[v->getId()] = nd;
                    prev[v->getId()] = u;
                    pq.push({nd, v});
                }
            }
        }

        vector<Node*> path;
        if (!prev.count(endId) && startId != endId) return path;
        for (Node* at = getNode(endId); at; at = (prev.count(at->getId()) ? prev[at->getId()] : nullptr)) {
            path.push_back(at);
            if (at->getId() == startId) break;
        }
        reverse(path.begin(), path.end());
        return path;
    }

    size_t getNodeCount() const { return nodes.size(); }
    size_t getEdgeCount() const { return edges.size(); }
};

int main() {
    try {
        Graph transportNetwork;
        transportNetwork.addNode(1, "CBD", 40.7128, -74.0060);
        transportNetwork.addNode(2, "ROngai", 40.7282, -73.7949);
        transportNetwork.addNode(3, "Industrial Area", 40.7465, -73.9887);

        transportNetwork.addEdge(1, 1, 2, 15.5, 25.0, 100, "highway");
        transportNetwork.addEdge(2, 1, 3, 10.2, 20.0, 80, "main street");
        transportNetwork.addEdge(3, 2, 3, 8.7, 15.0, 60, "local road");

        auto shortestPath = transportNetwork.dijkstraShortestPath(1, 3);
        cout << "Shortest Path:" << endl;
        for (auto* node : shortestPath) {
            cout << "Node ID: " << node->getId()
                 << " Name: " << node->getName() << endl;
        }
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
    }
    return 0;
}
