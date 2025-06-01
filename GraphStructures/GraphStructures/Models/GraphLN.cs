using System;
using System.Collections.Generic;
using System.Linq;
using GraphStructures.Interfaces;

namespace GraphStructures.Models
{
    public class GraphLN : IGraph
    {
        private readonly List<List<int>> _adjacencyList;
        public int VertexCount => _adjacencyList.Count;

        public GraphLN(int vertexCount)
        {
            _adjacencyList = new List<List<int>>(vertexCount);
            for (int i = 0; i < vertexCount; i++)
            {
                _adjacencyList.Add(new List<int>());
            }
        }

        public GraphLN(Dictionary<int, List<int>> adjacencyList)
        {
            int maxVertex = adjacencyList.Keys.Max();
            _adjacencyList = new List<List<int>>(maxVertex + 1);
            for (int i = 0; i <= maxVertex; i++)
            {
                _adjacencyList.Add(new List<int>());
            }

            foreach (KeyValuePair<int, List<int>> kvp in adjacencyList)
            {
                foreach (int neighbor in kvp.Value)
                {
                    AddEdge(kvp.Key, neighbor);
                }
            }
        }

        public void AddEdge(int from, int to)
        {
            if (!_adjacencyList[from].Contains(to))
            {
                _adjacencyList[from].Add(to);
                _adjacencyList[to].Add(from);
            }
        }

        public void RemoveEdge(int from, int to)
        {
            _adjacencyList[from].Remove(to);
            _adjacencyList[to].Remove(from);
        }

        public bool HasEdge(int from, int to)
        {
            return _adjacencyList[from].Contains(to);
        }

        public IEnumerable<int> GetNeighbors(int vertex)
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
            GraphLN spanningTree = new GraphLN(VertexCount);
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
            GraphLN spanningTree = new GraphLN(VertexCount);
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

        public Dictionary<int, List<int>> ToAdjacencyList()
        {
            Dictionary<int, List<int>> result = new Dictionary<int, List<int>>();
            for (int i = 0; i < VertexCount; i++)
            {
                result[i] = new List<int>(_adjacencyList[i]);
            }
            return result;
        }

        public GraphAM ToAdjacencyMatrix()
        {
            GraphAM graphAM = new GraphAM(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                foreach (int neighbor in GetNeighbors(i))
                {
                    graphAM.AddEdge(i, neighbor);
                }
            }
            return graphAM;
        }

        public GraphLE ToEdgeList()
        {
            GraphLE graphLE = new GraphLE(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                foreach (int neighbor in GetNeighbors(i))
                {
                    if (i < neighbor) // Add each edge only once
                    {
                        graphLE.AddEdge(i, neighbor);
                    }
                }
            }
            return graphLE;
        }

        public bool HasCycle()
        {
            bool[] visited = new bool[VertexCount];
            bool[] recursionStack = new bool[VertexCount];

            for (int i = 0; i < VertexCount; i++)
            {
                if (!visited[i] && HasCycleDFS(i, visited, recursionStack))
                    return true;
            }
            return false;
        }

        private bool HasCycleDFS(int vertex, bool[] visited, bool[] recursionStack)
        {
            visited[vertex] = true;
            recursionStack[vertex] = true;

            foreach (int neighbor in GetNeighbors(vertex))
            {
                if (!visited[neighbor])
                {
                    if (HasCycleDFS(neighbor, visited, recursionStack))
                        return true;
                }
                else if (recursionStack[neighbor])
                    return true;
            }

            recursionStack[vertex] = false;
            return false;
        }

        public bool IsBipartite()
        {
            int[] colors = new int[VertexCount];
            Array.Fill(colors, -1);

            for (int i = 0; i < VertexCount; i++)
            {
                if (colors[i] == -1)
                {
                    if (!IsBipartiteDFS(i, colors, 0))
                        return false;
                }
            }
            return true;
        }

        private bool IsBipartiteDFS(int vertex, int[] colors, int color)
        {
            colors[vertex] = color;

            foreach (int neighbor in GetNeighbors(vertex))
            {
                if (colors[neighbor] == -1)
                {
                    if (!IsBipartiteDFS(neighbor, colors, 1 - color))
                        return false;
                }
                else if (colors[neighbor] == color)
                    return false;
            }
            return true;
        }

        public List<int> GetShortestPath(int start, int end)
        {
            int[] previous = new int[VertexCount];
            Array.Fill(previous, -1);
            bool[] visited = new bool[VertexCount];
            Queue<int> queue = new Queue<int>();

            queue.Enqueue(start);
            visited[start] = true;

            while (queue.Count > 0)
            {
                int current = queue.Dequeue();
                if (current == end)
                    break;

                foreach (int neighbor in GetNeighbors(current))
                {
                    if (!visited[neighbor])
                    {
                        visited[neighbor] = true;
                        previous[neighbor] = current;
                        queue.Enqueue(neighbor);
                    }
                }
            }

            if (previous[end] == -1)
                return null;

            List<int> path = new List<int>();
            for (int v = end; v != -1; v = previous[v])
                path.Add(v);
            path.Reverse();
            return path;
        }
    }
} 