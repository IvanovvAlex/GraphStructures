using System;
using System.Collections.Generic;
using System.Linq;
using GraphStructures.Interfaces;

namespace GraphStructures.Models
{
    public class GraphAM : IGraph
    {
        private readonly bool[,] _adjacencyMatrix;
        public int VertexCount => _adjacencyMatrix.GetLength(0);

        public GraphAM(int vertexCount)
        {
            _adjacencyMatrix = new bool[vertexCount, vertexCount];
        }

        public GraphAM(bool[,] adjacencyMatrix)
        {
            if (adjacencyMatrix.GetLength(0) != adjacencyMatrix.GetLength(1))
                throw new ArgumentException("Adjacency matrix must be square");

            _adjacencyMatrix = new bool[adjacencyMatrix.GetLength(0), adjacencyMatrix.GetLength(0)];
            Array.Copy(adjacencyMatrix, _adjacencyMatrix, adjacencyMatrix.Length);
        }

        public void AddEdge(int from, int to)
        {
            _adjacencyMatrix[from, to] = true;
            _adjacencyMatrix[to, from] = true;
        }

        public void RemoveEdge(int from, int to)
        {
            _adjacencyMatrix[from, to] = false;
            _adjacencyMatrix[to, from] = false;
        }

        public bool HasEdge(int from, int to)
        {
            return _adjacencyMatrix[from, to];
        }

        public IEnumerable<int> GetNeighbors(int vertex)
        {
            for (int i = 0; i < VertexCount; i++)
            {
                if (_adjacencyMatrix[vertex, i])
                    yield return i;
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
            GraphAM spanningTree = new GraphAM(VertexCount);
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
            GraphAM spanningTree = new GraphAM(VertexCount);
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

        public bool[,] ToAdjacencyMatrix()
        {
            bool[,] result = new bool[VertexCount, VertexCount];
            Array.Copy(_adjacencyMatrix, result, _adjacencyMatrix.Length);
            return result;
        }

        public GraphLN ToAdjacencyList()
        {
            GraphLN graphLN = new GraphLN(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                for (int j = 0; j < VertexCount; j++)
                {
                    if (HasEdge(i, j))
                    {
                        graphLN.AddEdge(i, j);
                    }
                }
            }
            return graphLN;
        }

        public GraphLE ToEdgeList()
        {
            GraphLE graphLE = new GraphLE(VertexCount);
            for (int i = 0; i < VertexCount; i++)
            {
                for (int j = i + 1; j < VertexCount; j++)
                {
                    if (HasEdge(i, j))
                    {
                        graphLE.AddEdge(i, j);
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

            for (int i = 0; i < VertexCount; i++)
            {
                if (HasEdge(vertex, i))
                {
                    if (!visited[i])
                    {
                        if (HasCycleDFS(i, visited, recursionStack))
                            return true;
                    }
                    else if (recursionStack[i])
                        return true;
                }
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

            for (int i = 0; i < VertexCount; i++)
            {
                if (HasEdge(vertex, i))
                {
                    if (colors[i] == -1)
                    {
                        if (!IsBipartiteDFS(i, colors, 1 - color))
                            return false;
                    }
                    else if (colors[i] == color)
                        return false;
                }
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

                for (int i = 0; i < VertexCount; i++)
                {
                    if (HasEdge(current, i) && !visited[i])
                    {
                        visited[i] = true;
                        previous[i] = current;
                        queue.Enqueue(i);
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