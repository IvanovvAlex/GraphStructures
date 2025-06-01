using System.Collections.Generic;

namespace GraphStructures.Interfaces
{
    public interface IGraph
    {
        int VertexCount { get; }
        void AddEdge(int from, int to);
        void RemoveEdge(int from, int to);
        bool HasEdge(int from, int to);
        IEnumerable<int> GetNeighbors(int vertex);
        bool IsConnected();
        IGraph GetSpanningTreeBFS(int startVertex);
        IGraph GetSpanningTreeDFS(int startVertex);
    }
} 