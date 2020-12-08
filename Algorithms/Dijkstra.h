#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "graphAlgorithms.h"

template<typename TV, typename TE>
class Dijkstra{
private:
    std::unordered_map<std::string, Vertex<TV, TE>*> vertexes;
    void DijkstraAlgorithm(returnDijkstraType &result, std::string id);
    #define INF std::numeric_limits<TE>::max();
public:
    Dijkstra(Graph<TV, TE>* graph);
    returnDijkstraType apply(std::string id);
};

template<typename TV, typename TE>
Dijkstra<TV, TE>::Dijkstra(Graph<TV, TE>* graph){
    DirectedGraph<TV, TE>* dg = dynamic_cast<DirectedGraph<TV, TE>*>(graph);
    if(dg){
        this->vertexes = dg->vertexes;
        return;
    }
    UnDirectedGraph<TV, TE>* ug = dynamic_cast<UnDirectedGraph<TV, TE>*>(graph);
    if(ug){
        this->vertexes = ug->vertexes;
        return;
    }
    throw std::runtime_error("RUNTIME ERROR: No graph.");
}

template<typename TV, typename TE>
void Dijkstra<TV, TE>::DijkstraAlgorithm(returnDijkstraType &result, std::string id){
    std::priority_queue<
        std::pair<Vertex<TV, TE>*, TE>, 
        std::vector<std::pair<Vertex<TV, TE>*, TE> >, 
        ComparePairVertexTE<TV, TE>> pq;
    
    std::unordered_map<Vertex<TV, TE>*, bool> visited;
    for(auto p : this->vertexes){
        result[p.second].first = nullptr;
        result[p.second].second = INF;
        visited[p.second] = false;
    }   
    result[this->vertexes[id]].first = this->vertexes[id];
    result[this->vertexes[id]].second = 0;
    
    pq.push(std::make_pair(this->vertexes[id], 0));

    while(!pq.empty()){
        std::pair<Vertex<TV, TE>*, TE> cur = pq.top();
        pq.pop();
        if(visited[cur.first])
            continue;
        
        visited[cur.first] = true;

        for(Edge<TV, TE>* edge : cur.first->edges){
            if(!visited[edge->vertexes[1]]){
                if(result[cur.first].second + edge->weight < result[edge->vertexes[1]].second){
                    result[edge->vertexes[1]].second = result[cur.first].second + edge->weight;
                    pq.push(std::make_pair(edge->vertexes[1], result[edge->vertexes[1]].second));
                    result[edge->vertexes[1]].first = cur.first;
                }
            }
        }
    }
}

template<typename TV, typename TE>
returnDijkstraType Dijkstra<TV, TE>::apply(std::string id){
    returnDijkstraType result;
    
    if(!this->vertexes.count(id)){
        std::cout << "ERROR: ID not found.\n";
        return result;
    }

    std::cout << "\nDijkstra from vertex: " << this->vertexes[id]->data << "\n";
    
    DijkstraAlgorithm(result, id);

    return result;
}

#endif