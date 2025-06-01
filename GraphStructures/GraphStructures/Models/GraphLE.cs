using System;
using System.Collections.Generic;
using System.Linq;
using GraphStructures.Interfaces;

namespace GraphStructures.Models
{
    public class GraphLE : IGraph
    {
        private readonly List<(int from, int to)> _edges;
        private readonly int _vertexCount;

        public GraphLE(int vertexCount)
        {
            _vertexCount = vertexCount;
            _edges = new List<(int, int)>();
        }

        public GraphLE(int vertexCount, List<(int from, int to)> edges)
        {
            _vertexCount = vertexCount;
            _edges = new List<(int, int)>(edges);
        }

        public int VertexCount => _vertexCount;

        public void AddEdge(int from, int to)
        {
            if (!HasEdge(from, to))
            {
                _edges.Add((from, to));
                _edges.Add((to, from));
            }
        }

        public void RemoveEdge(int from, int to)
        {
            _edges.RemoveAll(e => (e.from == from && e.to == to) || (e.from == to && e.to == from));
        }

        public bool HasEdge(int from, int to)
        {
            return _edges.Any(e => e.from == from && e.to == to);
        }

        public IEnumerable<int> GetNeighbors(int vertex)
        {
            return _edges.Where(e => e.from == vertex).Select(e => e.to);
        }

        public bool IsConnected()
        {
            if (VertexCount == 0) return true;

            bool[] visited = new bool[VertexCount];
            Queue<int> queue = new Queue<int>();
            queue.Enqueue(0);
            visited[0] = true;
            int visitedCount = 1;

            while (queue.Count > 0)
            {
                int current = queue.Dequeue();
                foreach (int neighbor in GetNeighbors(current))
                {
                    if (!visited[neighbor])
                    {
                        visited[neighbor] = true;
                        visitedCount++;
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return visitedCount == VertexCount;
        }

        public IGraph GetSpanningTreeBFS(int startVertex)
        {
            GraphLE spanningTree = new GraphLE(VertexCount);
            bool[] visited = new bool[VertexCount];
            Queue<int> queue = new Queue<int>();
            queue.Enqueue(startVertex);
            visited[startVertex] = true;

            while (queue.Count > 0)
            {
                int current = queue.Dequeue();
                foreach (int neighbor in GetNeighbors(current))
                {
                    if (!visited[neighbor])
                    {
                        visited[neighbor] = true;
                        spanningTree.AddEdge(current, neighbor);
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return spanningTree;
        }

        public IGraph GetSpanningTreeDFS(int startVertex)
        {
            GraphLE spanningTree = new GraphLE(VertexCount);
            bool[] visited = new bool[VertexCount];
            DFS(startVertex, visited, spanningTree);
            return spanningTree;
        }

        private void DFS(int vertex, bool[] visited, IGraph spanningTree)
        {
            visited[vertex] = true;
            foreach (int neighbor in GetNeighbors(vertex))
            {
                if (!visited[neighbor])
                {
                    spanningTree.AddEdge(vertex, neighbor);
                    DFS(neighbor, visited, spanningTree);
                }
            }
        }

        public List<(int from, int to)> ToEdgeList()
        {
            return new List<(int, int)>(_edges);
        }

        public GraphLN ToAdjacencyList()
        {
            GraphLN graphLN = new GraphLN(VertexCount);
            foreach ((int from, int to) edge in _edges)
            {
                graphLN.AddEdge(edge.from, edge.to);
            }
            return graphLN;
        }

        public GraphAM ToAdjacencyMatrix()
        {
            GraphAM graphAM = new GraphAM(VertexCount);
            foreach ((int from, int to) edge in _edges)
            {
                graphAM.AddEdge(edge.from, edge.to);
            }
            return graphAM;
        }

        public bool HasCycle()
        {
            GraphLN graphLN = ToAdjacencyList();
            return graphLN.HasCycle();
        }

        public bool IsBipartite()
        {
            GraphLN graphLN = ToAdjacencyList();
            return graphLN.IsBipartite();
        }

        public List<int> GetShortestPath(int start, int end)
        {
            GraphLN graphLN = ToAdjacencyList();
            return graphLN.GetShortestPath(start, end);
        }
    }
} 