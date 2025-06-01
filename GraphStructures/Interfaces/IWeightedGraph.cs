using System.Collections.Generic;

namespace GraphStructures.Interfaces
{
    public interface IWeightedGraph : IGraph
    {
        void AddEdge(int from, int to, double weight);
        double GetEdgeWeight(int from, int to);
        IEnumerable<(int to, double weight)> GetWeightedNeighbors(int vertex);
    }
} 