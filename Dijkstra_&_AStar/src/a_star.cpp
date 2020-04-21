/*
* Find shortest paths using A* algorithm
* The MIT License
*
* Copyright 2020 Kostas Bourantas.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/
#include <iostream>
#include <stdlib.h>

#include <map>
#include <limits>
#include <vector>
#include <algorithm>
#include <math.h>
#include <ctime>
#include <set>

#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/graph/graphviz.hpp>

using namespace std;

#include "dijkstra_sp.h"

typedef boost::grid_graph<2> Graph;
typedef boost::graph_traits<Graph> Traits;
typedef boost::property_map<Graph, boost::vertex_index_t>::const_type IdMap;
typedef boost::property_map<Graph, boost::vertex_index_t>::const_type indexMapType;

struct position
{
    int x;
    int y;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ASSIGN POSITIONS
boost::vector_property_map<position, indexMapType> assign_positions_to_vertices(Graph graph, int n_cols)
{
    //create index map for grid graph
    indexMapType indexMap(get(boost::vertex_index, graph));

    //create structure position for every vertex of the graph in order to save its coordinates
    boost::vector_property_map<position, indexMapType> dataMap(num_vertices(graph), indexMap);

    //set for every vertex of the graph its coordinates
    for (Traits::vertices_size_type v_index = 0;
         v_index < num_vertices(graph); ++v_index)
    {

        put(dataMap, vertex(v_index, graph), position{vertex(v_index, graph)[1], vertex(v_index, graph)[0]});
        // Get the data at the node at position (0,1) in the grid
        position retrieved = get(dataMap, vertex(v_index, graph));

    }
    return dataMap;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRINT SHORTEST PATH
void printShortestPath_forAstar(map<Traits::vertices_size_type, int> parent, Traits::vertex_descriptor finish_vertex, Graph graph)
{
    cout << "Shortest Path: " << get(boost::vertex_index, graph, finish_vertex);
    while (parent[get(boost::vertex_index, graph, finish_vertex)] != -1)
    {
        cout << " <- " << parent[get(boost::vertex_index, graph, finish_vertex)];
        finish_vertex = vertex(parent[get(boost::vertex_index, graph, finish_vertex)], graph);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FIND EUCLIDEAN distance
int find_euclidean_distance(boost::vector_property_map<position, indexMapType> PositionsMap, Traits::vertices_size_type vertex_1, Traits::vertices_size_type vertex_2, Graph graph)
{
    double euc;

    //find the coordinates for the two vertices from their positionsmap
    position pos_vertex_1 = get(PositionsMap, vertex(vertex_1, graph));
    position pos_vertex_2 = get(PositionsMap, vertex(vertex_2, graph));

    //calculate their euclidean distance and return it
    euc = sqrt(pow(pos_vertex_1.x - pos_vertex_2.x, 2) + pow(pos_vertex_1.y - pos_vertex_2.y, 2));
    //cout<<"distance: "<<euc<<endl;
    return (int)euc;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//A_Star FUNCTION
std::pair<double, int> A_Star(Graph graph, map<Traits::edges_size_type, int> edge_weight, Traits::vertex_descriptor s, Traits::vertex_descriptor t, int cols)
{   
    boost::vector_property_map<position, indexMapType> PositionsMap = assign_positions_to_vertices(graph, cols);

    //save the distance of the vertex with its id
    map<Traits::vertices_size_type, int> distance;
    map<Traits::vertices_size_type, int>::iterator distance_iterator = distance.begin();

    //parent: structure for saving parent of a vertex in the SP
    map<Traits::vertices_size_type, int> parent;
    map<Traits::vertices_size_type, int>::iterator parent_iterator = parent.begin();
    
    
    Traits::vertices_size_type starting_vertex = get(boost::vertex_index, graph, s);

    //set distance of the vertex s = 0 and its parent vertex = -1
    distance.insert(pair<Traits::vertices_size_type, int>(starting_vertex, 0));
    parent.insert(pair<Traits::vertices_size_type, int>(starting_vertex, -1));

    //change the weight of every edge based on the euclidean distance
    for (Traits::edges_size_type e_index = 0;
         e_index < num_edges(graph); ++e_index)
    {

        edge_weight[e_index] = edge_weight[e_index]
         + find_euclidean_distance(PositionsMap, get(boost::vertex_index, graph, source(edge_at(e_index, graph), graph)), get(boost::vertex_index, graph, t), graph)
         - find_euclidean_distance(PositionsMap, get(boost::vertex_index, graph, target(edge_at(e_index, graph), graph)), get(boost::vertex_index, graph, t), graph);
    }

    //set distance of every other vertex to infinity and parent -1
    for (Traits::vertices_size_type index_vertex = 1;
         index_vertex < num_vertices(graph); ++index_vertex)
    {
        distance.insert(distance_iterator, pair<Traits::vertices_size_type, int>(index_vertex, numeric_limits<int>::max()));
        parent.insert(parent_iterator, pair<Traits::vertices_size_type, Traits::vertices_size_type>(index_vertex, -1));
    }

    //priority queue that saves the distances together with the vertices , return vertex with the shortest distance
    priority_queue<pair<int, Traits::vertices_size_type>, vector<pair<int, Traits::vertices_size_type> >, greater<pair<int, Traits::vertices_size_type> > > pq;

    //put the first vertex s in pq with distance 0
    pq.push(pair<int, Traits::vertices_size_type>(0, starting_vertex));

    Traits::vertex_descriptor finish_vertex = t;
    Traits::out_edge_iterator edge_it, edge_it_end;
    int cost_with_heuristic, visited_vertices = 0;

    clock_t begin = clock();

   
    while (!pq.empty())
    {

        visited_vertices++;

        Traits::vertex_descriptor vertex_to_be_examined = vertex(pq.top().second, graph);
        
        if (distance.find(get(boost::vertex_index, graph, vertex_to_be_examined))->first == get(boost::vertex_index, graph, finish_vertex))
        {
            
            clock_t end = clock();
            double time_elapsed = double(end - begin) / CLOCKS_PER_SEC;

            //printShortestPath_forAstar(parent, finish_vertex, graph);
            return std::make_pair(time_elapsed, visited_vertices);
        }
        
        pq.pop();

        // for every adj edge 
        for (boost::tie(edge_it, edge_it_end) = out_edges(vertex_to_be_examined, graph); edge_it != edge_it_end; ++edge_it)
        {
            Traits::vertex_descriptor adj_vertex = target(*edge_it, graph);

            cost_with_heuristic = distance[get(boost::vertex_index, graph, vertex_to_be_examined)] + edge_weight[get(boost::edge_index, graph, *edge_it)];

          
            if (cost_with_heuristic < distance[get(boost::vertex_index, graph, adj_vertex)])
            {
               

                distance[get(boost::vertex_index, graph, adj_vertex)] = cost_with_heuristic;

                pq.push(pair<int, Traits::vertices_size_type>(distance[get(boost::vertex_index, graph, adj_vertex)], get(boost::vertex_index, graph, adj_vertex)));

                parent[get(boost::vertex_index, graph, adj_vertex)] = get(boost::vertex_index, graph, vertex_to_be_examined);
            }
          
        }

       
    } //while (!pq.empty())
}
