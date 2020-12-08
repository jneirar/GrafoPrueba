#ifndef FLOYDWARSHALL_H
#define FLOYDWARSHALL_H

#include "graphAlgorithms.h"

template<typename TV, typename TE>
class FloydWarshall{
private:
    std::unordered_map<std::string, Vertex<TV, TE>*> vertexes;
    void FloydWarshallAlgorithm(distanceMatrixType &distances, pathMatrixType &paths);
    #define INF std::numeric_limits<TE>::max();
public:
    FloydWarshall(Graph<TV, TE>* graph);
    returnFloydWarshallType apply();
};

template<typename TV, typename TE>
FloydWarshall<TV, TE>::FloydWarshall(Graph<TV, TE>* graph){
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
void FloydWarshall<TV, TE>::FloydWarshallAlgorithm(distanceMatrixType &distances, pathMatrixType &paths){
    TE maxVal = INF;
    for(auto k : this->vertexes){
        //k.second es el vertice
        for(auto i : this->vertexes){
            for(auto j : this->vertexes){
                if(i.second->id != k.second->id && j.second->id != k.second->id){
                    if(distances[i.second][k.second] != maxVal && distances[k.second][j.second] != maxVal){
                        if(distances[i.second][k.second] + distances[k.second][j.second] < distances[i.second][j.second]){
                            distances[i.second][j.second] = distances[i.second][k.second] + distances[k.second][j.second];
                            paths[i.second][j.second] = k.second;
                        }
                    }
                }
            }
        }
    }
}

template<typename TV, typename TE>
returnFloydWarshallType FloydWarshall<TV, TE>::apply(){
    distanceMatrixType distances;
    pathMatrixType paths;
    for(auto i : this->vertexes){
        for(auto j : this->vertexes){
            if(i.second == j.second)  distances[i.second][j.second] = 0;
            else    distances[i.second][j.second] = INF;
            if(i.second == j.second)  paths[i.second][j.second] = nullptr;
            else    paths[i.second][j.second] = i.second;
        }
    }

    for(auto p : this->vertexes)    
        for(Edge<TV, TE>* edge : p.second->edges)   
            distances[edge->vertexes[0]][edge->vertexes[1]] = edge->weight;

    FloydWarshallAlgorithm(distances, paths);

    return std::make_pair(distances, paths);
}

#endif