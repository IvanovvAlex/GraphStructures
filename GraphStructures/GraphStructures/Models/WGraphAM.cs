using System;
using System.Collections.Generic;
using System.Linq;
using GraphStructures.Interfaces;

namespace GraphStructures.Models
{
    public class WGraphAM : IWeightedGraph
    {
        private readonly double[,] _weightMatrix;
        public int VertexCount => _weightMatrix.GetLength(0);

        public WGraphAM(int vertexCount)
        {
            _weightMatrix = new double[vertexCount, vertexCount];
            for (int i = 0; i < vertexCount; i++)
            {
                for (int j = 0; j < vertexCount; j++)
                {
                    _weightMatrix[i, j] = double.PositiveInfinity;
                }
            }
        }

        public WGraphAM(double[,] weightMatrix)
        {
            if (weightMatrix.GetLength(0) != weightMatrix.GetLength(1))
                throw new ArgumentException("Weight matrix must be square");

            _weightMatrix = new double[weightMatrix.GetLength(0), weightMatrix.GetLength(0)];
            Array.Copy(weightMatrix, _weightMatrix, weightMatrix.Length);
        }

        public void AddEdge(int from, int to)
        {
            AddEdge(from, to, 1.0);
        }

        public void AddEdge(int from, int to, double weight)
        {
            _weightMatrix[from, to] = weight;
            _weightMatrix[to, from] = weight;
        }

        public void RemoveEdge(int from, int to)
        {
            _weightMatrix[from, to] = double.PositiveInfinity;
            _weightMatrix[to, from] = double.PositiveInfinity;
        }

        public bool HasEdge(int from, int to)
        {
            return !double.IsPositiveInfinity(_weightMatrix[from, to]);
        }

        public double GetEdgeWeight(int from, int to)
        {
            return _weightMatrix[from, to];
        }

        public IEnumerable<int> GetNeighbors(int vertex)
        {
            for (int i = 0; i < VertexCount; i++)
            {
                if (HasEdge(vertex, i))
                    yield return i;
            }
        }

        public IEnumerable<(int to, double weight)> GetWeightedNeighbors(int vertex)
        {
            for (int i = 0; i < VertexCount; i++)
            {
                if (HasEdge(vertex, i))
                    yield return (i, _weightMatrix[vertex, i]);
            }
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
            WGraphAM spanningTree = new WGraphAM(VertexCount);
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
            WGraphAM spanningTree = new WGraphAM(VertexCount);
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
                    ((WGraphAM)spanningTree).AddEdge(vertex, neighbor, weight);
                    DFS(neighbor, visited, spanningTree);
                }
            }
        }

        public double[,] ToWeightMatrix()
        {
            double[,] result = new double[VertexCount, VertexCount];
            Array.Copy(_weightMatrix, result, _weightMatrix.Length);
            return result;
        }

        public WGraphLN ToWeightedAdjacencyList()
        {
            WGraphLN wGraphLN = new WGraphLN(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                for (int j = 0; j < VertexCount; j++)
                {
                    double weight = GetEdgeWeight(i, j);
                    if (!double.IsPositiveInfinity(weight))
                    {
                        wGraphLN.AddEdge(i, j, weight);
                    }
                }
            }
            return wGraphLN;
        }

        public WGraphLE ToWeightedEdgeList()
        {
            WGraphLE wGraphLE = new WGraphLE(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                for (int j = i + 1; j < VertexCount; j++)
                {
                    double weight = GetEdgeWeight(i, j);
                    if (!double.IsPositiveInfinity(weight))
                    {
                        wGraphLE.AddEdge(i, j, weight);
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

                for (int i = 0; i < VertexCount; i++)
                {
                    double weight = GetEdgeWeight(current, i);
                    if (!double.IsPositiveInfinity(weight))
                    {
                        double newDist = distances[current] + weight;
                        if (newDist < distances[i])
                        {
                            distances[i] = newDist;
                            previous[i] = current;
                            pq.Enqueue(i, newDist);
                        }
                    }
                }
            }

            return (distances, previous);
        }

        public List<int> GetShortestPath(int start, int end)
        {
            (double[] distances, int[] previous) = DijkstraShortestPath(start);
            
            if (distances[end] == double.PositiveInfinity)
                return null;

            List<int> path = new List<int>();
            for (int v = end; v != -1; v = previous[v])
                path.Add(v);
            path.Reverse();
            return path;
        }

        public WGraphAM GetMinimumSpanningTreePrim()
        {
            WGraphAM mst = new WGraphAM(VertexCount);
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

                for (int v = 0; v < VertexCount; v++)
                {
                    double weight = GetEdgeWeight(u, v);
                    if (!double.IsPositiveInfinity(weight) && !visited[v] && weight < key[v])
                    {
                        parent[v] = u;
                        key[v] = weight;
                        pq.Enqueue(v, weight);
                    }
                }
            }

            return mst;
        }

        public WGraphAM GetMinimumSpanningTreeKruskal()
        {
            WGraphAM mst = new WGraphAM(VertexCount);
            List<(int from, int to, double weight)> edges = new List<(int from, int to, double weight)>();
            
            for (int i = 0; i < VertexCount; i++)
            {
                for (int j = i + 1; j < VertexCount; j++)
                {
                    double weight = GetEdgeWeight(i, j);
                    if (!double.IsPositiveInfinity(weight))
                    {
                        edges.Add((i, j, weight));
                    }
                }
            }

            edges.Sort((a, b) => a.weight.CompareTo(b.weight));

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

            foreach ((int from, int to, double weight) in edges)
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