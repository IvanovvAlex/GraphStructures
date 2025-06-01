using System;
using System.Collections.Generic;
using System.Linq;
using GraphStructures.Interfaces;

namespace GraphStructures.Models
{
    public class WGraphLN : IWeightedGraph
    {
        private readonly List<List<(int vertex, double weight)>> _adjacencyList;
        public int VertexCount => _adjacencyList.Count;

        public WGraphLN(int vertexCount)
        {
            _adjacencyList = new List<List<(int, double)>>(vertexCount);
            for (int i = 0; i < vertexCount; i++)
            {
                _adjacencyList.Add(new List<(int, double)>());
            }
        }

        public WGraphLN(Dictionary<int, List<(int vertex, double weight)>> adjacencyList)
        {
            int maxVertex = adjacencyList.Keys.Max();
            _adjacencyList = new List<List<(int, double)>>(maxVertex + 1);
            for (int i = 0; i <= maxVertex; i++)
            {
                _adjacencyList.Add(new List<(int, double)>());
            }

            foreach (KeyValuePair<int, List<(int vertex, double weight)>> kvp in adjacencyList)
            {
                foreach ((int vertex, double weight) in kvp.Value)
                {
                    AddEdge(kvp.Key, vertex, weight);
                }
            }
        }

        public void AddEdge(int from, int to)
        {
            AddEdge(from, to, 1.0);
        }

        public void AddEdge(int from, int to, double weight)
        {
            if (!_adjacencyList[from].Any(x => x.vertex == to))
            {
                _adjacencyList[from].Add((to, weight));
                _adjacencyList[to].Add((from, weight));
            }
        }

        public void RemoveEdge(int from, int to)
        {
            _adjacencyList[from].RemoveAll(x => x.vertex == to);
            _adjacencyList[to].RemoveAll(x => x.vertex == from);
        }

        public bool HasEdge(int from, int to)
        {
            return _adjacencyList[from].Any(x => x.vertex == to);
        }

        public double GetEdgeWeight(int from, int to)
        {
            (int vertex, double weight) edge = _adjacencyList[from].FirstOrDefault(x => x.vertex == to);
            return edge.vertex == to ? edge.weight : double.PositiveInfinity;
        }

        public IEnumerable<int> GetNeighbors(int vertex)
        {
            return _adjacencyList[vertex].Select(x => x.vertex);
        }

        public IEnumerable<(int to, double weight)> GetWeightedNeighbors(int vertex)
        {
            return _adjacencyList[vertex];
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
            WGraphLN spanningTree = new WGraphLN(VertexCount);
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
            WGraphLN spanningTree = new WGraphLN(VertexCount);
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
                    ((WGraphLN)spanningTree).AddEdge(vertex, neighbor, weight);
                    DFS(neighbor, visited, spanningTree);
                }
            }
        }

        public Dictionary<int, List<(int vertex, double weight)>> ToWeightedAdjacencyList()
        {
            Dictionary<int, List<(int, double)>> result = new Dictionary<int, List<(int, double)>>();
            for (int i = 0; i < VertexCount; i++)
            {
                result[i] = new List<(int, double)>(_adjacencyList[i]);
            }
            return result;
        }

        public WGraphAM ToWeightedAdjacencyMatrix()
        {
            WGraphAM wGraphAM = new WGraphAM(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                foreach ((int neighbor, double weight) in GetWeightedNeighbors(i))
                {
                    wGraphAM.AddEdge(i, neighbor, weight);
                }
            }
            return wGraphAM;
        }

        public WGraphLE ToWeightedEdgeList()
        {
            WGraphLE wGraphLE = new WGraphLE(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                foreach ((int neighbor, double weight) in GetWeightedNeighbors(i))
                {
                    if (i < neighbor) // Add each edge only once
                    {
                        wGraphLE.AddEdge(i, neighbor, weight);
                    }
                }
            }
            return wGraphLE;
        }

        public (double[] distances, int[] previous) DijkstraShortestPath(int start)
        {
            double[] distances = new double[VertexCount];
            int[] previous = new int[VertexCount];
            bool[] visited = new bool[VertexCount];

            Array.Fill(distances, double.PositiveInfinity);
            Array.Fill(previous, -1);
            distances[start] = 0;

            PriorityQueue<int, double> pq = new PriorityQueue<int, double>();
            pq.Enqueue(start, 0);

            while (pq.Count > 0)
            {
                int current = pq.Dequeue();
                if (visited[current]) continue;
                visited[current] = true;

                foreach ((int neighbor, double weight) in GetWeightedNeighbors(current))
                {
                    double newDist = distances[current] + weight;
                    if (newDist < distances[neighbor])
                    {
                        distances[neighbor] = newDist;
                        previous[neighbor] = current;
                        pq.Enqueue(neighbor, newDist);
                    }
                }
            }

            return (distances, previous);
        }

        public List<int> GetShortestPath(int start, int end)
        {
            var (distances, previous) = DijkstraShortestPath(start);
            
            if (distances[end] == double.PositiveInfinity)
                return null;

            List<int> path = new List<int>();
            for (int v = end; v != -1; v = previous[v])
                path.Add(v);
            path.Reverse();
            return path;
        }

        public WGraphLN GetMinimumSpanningTreePrim()
        {
            WGraphLN mst = new WGraphLN(VertexCount);
            bool[] visited = new bool[VertexCount];
            double[] key = new double[VertexCount];
            int[] parent = new int[VertexCount];

            Array.Fill(key, double.PositiveInfinity);
            Array.Fill(parent, -1);
            key[0] = 0;

            PriorityQueue<int, double> pq = new PriorityQueue<int, double>();
            pq.Enqueue(0, 0);

            while (pq.Count > 0)
            {
                int u = pq.Dequeue();
                if (visited[u]) continue;
                visited[u] = true;

                if (parent[u] != -1)
                {
                    mst.AddEdge(parent[u], u, key[u]);
                }

                foreach ((int v, double weight) in GetWeightedNeighbors(u))
                {
                    if (!visited[v] && weight < key[v])
                    {
                        parent[v] = u;
                        key[v] = weight;
                        pq.Enqueue(v, weight);
                    }
                }
            }

            return mst;
        }

        public WGraphLN GetMinimumSpanningTreeKruskal()
        {
            WGraphLN mst = new WGraphLN(VertexCount);
            var edges = new List<(int from, int to, double weight)>();
            
            // Collect all edges
            for (int i = 0; i < VertexCount; i++)
            {
                foreach ((int to, double weight) in GetWeightedNeighbors(i))
                {
                    if (i < to) // Add each edge only once
                    {
                        edges.Add((i, to, weight));
                    }
                }
            }

            // Sort edges by weight
            edges.Sort((a, b) => a.weight.CompareTo(b.weight));

            // Union-Find data structure
            int[] parent = new int[VertexCount];
            for (int i = 0; i < VertexCount; i++)
                parent[i] = i;

            int Find(int x)
            {
                if (parent[x] != x)
                    parent[x] = Find(parent[x]);
                return parent[x];
            }

            void Union(int x, int y)
            {
                parent[Find(x)] = Find(y);
            }

            // Process edges in order of increasing weight
            foreach (var (from, to, weight) in edges)
            {
                if (Find(from) != Find(to))
                {
                    mst.AddEdge(from, to, weight);
                    Union(from, to);
                }
            }

            return mst;
        }
    }
} 