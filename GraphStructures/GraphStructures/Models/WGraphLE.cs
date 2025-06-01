using System;
using System.Collections.Generic;
using System.Linq;
using GraphStructures.Interfaces;

namespace GraphStructures.Models
{
    public class WGraphLE : IWeightedGraph
    {
        private readonly List<(int from, int to, double weight)> _edges;
        private readonly int _vertexCount;

        public WGraphLE(int vertexCount)
        {
            _vertexCount = vertexCount;
            _edges = new List<(int, int, double)>();
        }

        public WGraphLE(int vertexCount, List<(int from, int to, double weight)> edges)
        {
            _vertexCount = vertexCount;
            _edges = new List<(int, int, double)>(edges);
        }

        public int VertexCount => _vertexCount;

        public void AddEdge(int from, int to)
        {
            AddEdge(from, to, 1.0);
        }

        public void AddEdge(int from, int to, double weight)
        {
            if (!HasEdge(from, to))
            {
                _edges.Add((from, to, weight));
                _edges.Add((to, from, weight));
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

        public double GetEdgeWeight(int from, int to)
        {
            (int from, int to, double weight) edge = _edges.FirstOrDefault(e => e.from == from && e.to == to);
            return edge.from == from && edge.to == to ? edge.weight : double.PositiveInfinity;
        }

        public IEnumerable<int> GetNeighbors(int vertex)
        {
            return _edges.Where(e => e.from == vertex).Select(e => e.to);
        }

        public IEnumerable<(int to, double weight)> GetWeightedNeighbors(int vertex)
        {
            return _edges.Where(e => e.from == vertex).Select(e => (e.to, e.weight));
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
            WGraphLE spanningTree = new WGraphLE(VertexCount);
            bool[] visited = new bool[VertexCount];
            Queue<int> queue = new Queue<int>();
            queue.Enqueue(startVertex);
            visited[startVertex] = true;

            while (queue.Count > 0)
            {
                int current = queue.Dequeue();
                foreach ((int neighbor, double weight) in GetWeightedNeighbors(current))
                {
                    if (!visited[neighbor])
                    {
                        visited[neighbor] = true;
                        spanningTree.AddEdge(current, neighbor, weight);
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return spanningTree;
        }

        public IGraph GetSpanningTreeDFS(int startVertex)
        {
            WGraphLE spanningTree = new WGraphLE(VertexCount);
            bool[] visited = new bool[VertexCount];
            DFS(startVertex, visited, spanningTree);
            return spanningTree;
        }

        private void DFS(int vertex, bool[] visited, IGraph spanningTree)
        {
            visited[vertex] = true;
            foreach ((int neighbor, double weight) in GetWeightedNeighbors(vertex))
            {
                if (!visited[neighbor])
                {
                    ((WGraphLE)spanningTree).AddEdge(vertex, neighbor, weight);
                    DFS(neighbor, visited, spanningTree);
                }
            }
        }

        public List<(int from, int to, double weight)> ToWeightedEdgeList()
        {
            return new List<(int, int, double)>(_edges);
        }

        public WGraphLN ToWeightedAdjacencyList()
        {
            WGraphLN wGraphLN = new WGraphLN(VertexCount);
            foreach ((int from, int to, double weight) edge in _edges)
            {
                wGraphLN.AddEdge(edge.from, edge.to, edge.weight);
            }
            return wGraphLN;
        }

        public WGraphAM ToWeightedAdjacencyMatrix()
        {
            WGraphAM wGraphAM = new WGraphAM(VertexCount);
            foreach ((int from, int to, double weight) edge in _edges)
            {
                wGraphAM.AddEdge(edge.from, edge.to, edge.weight);
            }
            return wGraphAM;
        }

        public (double[] distances, int[] previous) DijkstraShortestPath(int start)
        {
            WGraphLN wGraphLN = ToWeightedAdjacencyList();
            return wGraphLN.DijkstraShortestPath(start);
        }

        public List<int> GetShortestPath(int start, int end)
        {
            WGraphLN wGraphLN = ToWeightedAdjacencyList();
            return wGraphLN.GetShortestPath(start, end);
        }

        public WGraphLE GetMinimumSpanningTreePrim()
        {
            WGraphLN wGraphLN = ToWeightedAdjacencyList();
            return wGraphLN.GetMinimumSpanningTreePrim().ToWeightedEdgeList();
        }

        public WGraphLE GetMinimumSpanningTreeKruskal()
        {
            WGraphLN wGraphLN = ToWeightedAdjacencyList();
            return wGraphLN.GetMinimumSpanningTreeKruskal().ToWeightedEdgeList();
        }
    }
} 