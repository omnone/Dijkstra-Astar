
/*
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
#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/graph/graphviz.hpp>
#include <set>

typedef boost::grid_graph<2> Graph;
typedef boost::graph_traits<Graph> Traits;

#ifndef A_STAR
#define A_STAR

std::pair<double,int> A_Star(Graph graph, std::map<Traits::edges_size_type, int> edge_weight, Traits::vertex_descriptor s, Traits::vertex_descriptor t,int cols);

void   printShortestPath_forAstar(std::map<Traits::vertices_size_type, int> parent, Traits::vertex_descriptor finish_vertex, Graph graph);

#endif