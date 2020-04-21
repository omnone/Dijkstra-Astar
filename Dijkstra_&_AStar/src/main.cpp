   
/*
* Find shortest paths using Dijkstra's and A* algorithm and compare their results.
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

#include <typeinfo>

#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graphviz.hpp>

using namespace std;

#include "dijkstra_sp.h"
#include "a_star.h"

typedef boost::grid_graph<2> Graph;
typedef boost::graph_traits<Graph> Traits;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HELPER FUNCTIONS
void print_graph_with_graphviz(Graph graph)
{
    //write file with graphviz
    ofstream gout;
    gout.open("test.dot");
    boost::write_graphviz(cout, graph);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    
    srand(time(0));

    //graph dimensions for grid graph
    int rows = 1000;
    int cols[] = {30, 60, 80};
    int range[] = {10, 100};
    int range_option = 0;

    //choose how many times you wish to run the algorithms
    int times_to_execute;
    cout << "\x1B[33mPlease enter the number of times you wish to execute each experiment:\033[0m";
    cin >> times_to_execute;

    while (range_option <= 1)
    {
        cout << "Test for range of weigths [1," << range[range_option] << "]\n------------------------------------------------" << endl;

        //for every dimension
        for (int i = 0; i < 3; ++i)
        {

            //create grid graph
            boost::array<std::size_t, 2> lengths = {{cols[i], rows}};
            Graph graph(lengths);

            map<Traits::edges_size_type, int> edge_weight;

            //set weight for every edge of the graph
            for (Traits::edges_size_type e_index = 0;
                 e_index < num_edges(graph); ++e_index)
            {
                edge_weight[e_index] = 1 + (rand() % (range[range_option] - 1 + 1));
            }

            int x = 0, y = 1;

            //find randomly vertices s and t
            Traits::vertex_descriptor s;
            s[x] = rand() % cols[i]; //column
            s[y] = 0;                //row

            Traits::vertex_descriptor t;
            t[x] = rand() % cols[i]; //column
            t[y] = rows - 1;         //row

            cout << "[*]Searching for the shortest path beetween vertex (" << get(boost::vertex_index, graph, s) << ") and vertex (" << get(boost::vertex_index, graph, t) << ")" << endl;

            //total runtime for alg. Dijkstra-sp
            double dijkstra_time = 0.0;
            //set of vertices visited by Disjkstra-sp
            int visited_by_dijkstra = 0;
            //total runtime for alg. A*
            double a_star_time = 0.0;
            //set of vertices visited by A*
            int visited_by_astar = 0;

            double temp_time;
            int temp_vertices;

            for (int j = 0; j < times_to_execute; ++j)
            {
                boost::tie(temp_time, temp_vertices) = Dijkstra_SP(graph, edge_weight, s, t);
                dijkstra_time += temp_time;
                visited_by_dijkstra += temp_vertices;

                boost::tie(temp_time, temp_vertices) = A_Star(graph, edge_weight, s, t, cols[i]);
                a_star_time += temp_time;
                visited_by_astar += temp_vertices;
            }

            //print results
            cout << "Cols: " << cols[i] << " Rows: " << rows << endl;
            cout << "Total time for dijkstra: " << dijkstra_time / times_to_execute << "s.\nTotal number of vertices visited: " << visited_by_dijkstra / times_to_execute << endl;
            cout << "Total time for a star : " << a_star_time / times_to_execute << "s.\nTotal number of vertices visited: " << visited_by_astar / times_to_execute << endl;
            cout << "=============================================" << endl;
            //print_graph_with_graphviz(graph);
        }
        range_option++;
    }

    return 0;
}