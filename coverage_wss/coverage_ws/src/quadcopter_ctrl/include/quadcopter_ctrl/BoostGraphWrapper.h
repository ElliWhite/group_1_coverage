#ifndef BOOST_GRAPH_WRAPPER_H_
#define BOOST_GRAPH_WRAPPER_H_

#include <cstring>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include "boost/graph/graph_traits.hpp"
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "graphStructs.h"

#define TC_RED          "\033[1;31m"        /* 0 -> normal ;  31 -> red */
#define TC_CYAN         "\033[1;36m"        /* 1 -> bold ;  36 -> cyan */
#define TC_GREEN        "\033[1;32m"        /* 4 -> underline ;  32 -> green */
#define TC_BLUE         "\033[1;34m"        /* 9 -> strike ;  34 -> blue */
#define TC_YELLOW       "\033[1;33m"
#define TC_BLACK        "\033[0;30m"
#define TC_BROWN        "\033[0;33m"
#define TC_MAGENTA      "\033[1;35m"
#define TC_GRAY         "\033[1;37m"
#define TC_NONE         "\033[0m"

using namespace boost;
typedef property<edge_weight_t, int> EdgeWeightProperty;
typedef property<vertex_index2_t, int> VertexProperty;

typedef adjacency_list<vecS,vecS,undirectedS,VertexProperty,EdgeWeightProperty> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef graph_traits<Graph>::edge_descriptor Edge;
typedef property_map<Graph, vertex_index2_t>::type VertexIndexMap;
typedef property_map<Graph, edge_weight_t>::type EdgeDistance;
typedef graph_traits<Graph>::edge_iterator edge_iterator;
typedef graph_traits<Graph>::adjacency_iterator n_iterator;
typedef graph_traits<Graph>::vertex_iterator vertex_iterator;



class BoostGraphWrapper
{
public:
    BoostGraphWrapper(std::string filename);
    virtual ~BoostGraphWrapper();
    
    int gridSizeX;
	int gridSizeY;
	
    void printGraph();
    void printVertexes();
    void printPath(std::vector<int> path);
    bool findVertex(int i, Vertex& vertex);
    void computeShortestPath(int i, int j, std::vector<int>& path);
    
protected:

	std::vector<int> access_vec;
    Graph* graph_ptr;
    VertexIndexMap index;
    EdgeDistance distances;

    std::string get_selfpath();
    void loadMatrixFile(std::ifstream &access_mat);
    
    
};

#endif
