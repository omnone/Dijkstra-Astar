/*
* Find shortest paths using Dijkstra's algorithm
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PRINT SHORTEST PATH
void  printShortestPath_forDijkstra(map<Traits::vertices_size_type, int> parent, Traits::vertex_descriptor finish_vertex, Graph graph)
{
    cout << "Shortest Path: " << get(boost::vertex_index, graph, finish_vertex);
    while (parent[get(boost::vertex_index, graph, finish_vertex)] != -1)
    {
        cout << " <- " << parent[get(boost::vertex_index, graph, finish_vertex)];
        finish_vertex = vertex(parent[get(boost::vertex_index, graph, finish_vertex)], graph);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dijkstra_SP FUNCTION
std::pair<double, int> Dijkstra_SP(Graph graph, map<Traits::edges_size_type, int> edge_weight, Traits::vertex_descriptor s, Traits::vertex_descriptor t)
{

    map<Traits::vertices_size_type, int> distance;
    map<Traits::vertices_size_type, int>::iterator distance_iterator = distance.begin();

    map<Traits::vertices_size_type, int> parent;
    map<Traits::vertices_size_type, int>::iterator parent_iterator = parent.begin();

    Traits::vertices_size_type starting_vertex = get(boost::vertex_index, graph, s);

    distance.insert(pair<Traits::vertices_size_type, int>(starting_vertex, 0));
    parent.insert(pair<Traits::vertices_size_type, int>(starting_vertex, -1));

    for (Traits::vertices_size_type index_vertex = 1;
         index_vertex < num_vertices(graph); ++index_vertex)
    {
        distance.insert(distance_iterator, pair<Traits::vertices_size_type, int>(index_vertex, numeric_limits<int>::max()));
        parent.insert(parent_iterator, pair<Traits::vertices_size_type, Traits::vertices_size_type>(index_vertex, -1));
        
    }

    priority_queue<pair<int, Traits::vertices_size_type>, vector<pair<int, Traits::vertices_size_type> >, greater<pair<int, Traits::vertices_size_type> > > pq;

    pq.push(pair<int, Traits::vertices_size_type>(0, starting_vertex));

    Traits::vertex_descriptor finish_vertex = t;
    Traits::out_edge_iterator edge_it, edge_it_end;
    int visited_vertices = 0;

    clock_t begin = clock();

    while (!pq.empty())
    {

        visited_vertices++;

        Traits::vertex_descriptor vertex_to_be_examined = vertex(pq.top().second, graph);
        
        if (distance.find(get(boost::vertex_index, graph, vertex_to_be_examined))->first == get(boost::vertex_index, graph, finish_vertex))
        {
          
            clock_t end = clock();
            double time_elapsed = double(end - begin) / CLOCKS_PER_SEC;
            
            return std::make_pair(time_elapsed, visited_vertices);
        }
        
        pq.pop();
        

        for (boost::tie(edge_it, edge_it_end) = out_edges(vertex_to_be_examined, graph); edge_it != edge_it_end; ++edge_it)
        {
            Traits::vertex_descriptor adj_vertex = target(*edge_it, graph);

           
            if (distance[get(boost::vertex_index, graph, vertex_to_be_examined)] + edge_weight[get(boost::edge_index, graph, *edge_it)] < distance[get(boost::vertex_index, graph, adj_vertex)])
            {
               
                distance[get(boost::vertex_index, graph, adj_vertex)] = distance[get(boost::vertex_index, graph, vertex_to_be_examined)] + edge_weight[get(boost::edge_index, graph, *edge_it)];

                pq.push(pair<int, Traits::vertices_size_type>(distance[get(boost::vertex_index, graph, adj_vertex)], get(boost::vertex_index, graph, adj_vertex)));

                parent[get(boost::vertex_index, graph, adj_vertex)] = get(boost::vertex_index, graph, vertex_to_be_examined);
            }
          
        }

       
    } //while (!pq.empty())
}
