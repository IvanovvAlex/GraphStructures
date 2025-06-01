using System;
using System.Collections.Generic;
using System.Linq;
using GraphStructures.Models;

class Program
{
    static void Main(string[] args)
    {
        Console.WriteLine("Graph and Tree Structure Demonstrations\n");

        Console.WriteLine("1. GraphLN (Adjacency List) Demonstration:");
        GraphLN graphLN = new GraphLN(5);
        graphLN.AddEdge(0, 1);
        graphLN.AddEdge(0, 2);
        graphLN.AddEdge(1, 3);
        graphLN.AddEdge(2, 4);
        graphLN.AddEdge(3, 4);
        
        Console.WriteLine("Adjacency List Representation:");
        for (int i = 0; i < graphLN.VertexCount; i++)
        {
            Console.WriteLine($"Vertex {i}: {string.Join(", ", graphLN.GetNeighbors(i))}");
        }
        
        Console.WriteLine($"Is Connected: {graphLN.IsConnected()}");
        Console.WriteLine($"Has Cycle: {graphLN.HasCycle()}");
        Console.WriteLine($"Is Bipartite: {graphLN.IsBipartite()}");
        
        List<int> shortestPath = graphLN.GetShortestPath(0, 4);
        Console.WriteLine($"Shortest Path from 0 to 4: {(shortestPath != null ? string.Join(" -> ", shortestPath) : "No path exists")}");
        
        Console.WriteLine("\nTransforming to other representations:");
        GraphAM graphAM = graphLN.ToAdjacencyMatrix();
        GraphLE graphLE = graphLN.ToEdgeList();
        Console.WriteLine("Successfully transformed to Adjacency Matrix and Edge List\n");

        Console.WriteLine("2. WGraphLN (Weighted Adjacency List) Demonstration:");
        WGraphLN wGraphLN = new WGraphLN(5);
        wGraphLN.AddEdge(0, 1, 2.5);
        wGraphLN.AddEdge(0, 2, 1.5);
        wGraphLN.AddEdge(1, 3, 3.0);
        wGraphLN.AddEdge(2, 4, 2.0);
        wGraphLN.AddEdge(3, 4, 1.0);
        
        Console.WriteLine("Weighted Adjacency List Representation:");
        for (int i = 0; i < wGraphLN.VertexCount; i++)
        {
            Console.WriteLine($"Vertex {i}: {string.Join(", ", wGraphLN.GetWeightedNeighbors(i).Select(n => $"{n.to}({n.weight})"))}");
        }
        
        Console.WriteLine($"Is Connected: {wGraphLN.IsConnected()}");
        
        (double[] distances, int[] _) = wGraphLN.DijkstraShortestPath(0);
        Console.WriteLine("Shortest distances from vertex 0:");
        for (int i = 0; i < distances.Length; i++)
        {
            Console.WriteLine($"To vertex {i}: {distances[i]}");
        }
        
        WGraphLN mstPrim = wGraphLN.GetMinimumSpanningTreePrim();
        WGraphLN mstKruskal = wGraphLN.GetMinimumSpanningTreeKruskal();
        Console.WriteLine("\nMinimum Spanning Trees (Prim and Kruskal) created successfully");
        
        Console.WriteLine("\nTransforming to other representations:");
        WGraphAM wGraphAM = wGraphLN.ToWeightedAdjacencyMatrix();
        WGraphLE wGraphLE = wGraphLN.ToWeightedEdgeList();
        Console.WriteLine("Successfully transformed to Weighted Adjacency Matrix and Weighted Edge List\n");

        Console.WriteLine("3. GraphAM (Adjacency Matrix) Demonstration:");
        GraphAM graphAM2 = new GraphAM(5);
        graphAM2.AddEdge(0, 1);
        graphAM2.AddEdge(0, 2);
        graphAM2.AddEdge(1, 3);
        graphAM2.AddEdge(2, 4);
        graphAM2.AddEdge(3, 4);
        
        Console.WriteLine("Adjacency Matrix Representation:");
        for (int i = 0; i < graphAM2.VertexCount; i++)
        {
            for (int j = 0; j < graphAM2.VertexCount; j++)
            {
                Console.Write($"{(graphAM2.HasEdge(i, j) ? "1" : "0")} ");
            }
            Console.WriteLine();
        }
        Console.WriteLine($"Is Connected: {graphAM2.IsConnected()}\n");

        Console.WriteLine("4. WGraphAM (Weighted Adjacency Matrix) Demonstration:");
        WGraphAM wGraphAM2 = new WGraphAM(5);
        wGraphAM2.AddEdge(0, 1, 2.5);
        wGraphAM2.AddEdge(0, 2, 1.5);
        wGraphAM2.AddEdge(1, 3, 3.0);
        wGraphAM2.AddEdge(2, 4, 2.0);
        wGraphAM2.AddEdge(3, 4, 1.0);
        
        Console.WriteLine("Weight Matrix Representation:");
        for (int i = 0; i < wGraphAM2.VertexCount; i++)
        {
            for (int j = 0; j < wGraphAM2.VertexCount; j++)
            {
                double weight = wGraphAM2.GetEdgeWeight(i, j);
                Console.Write($"{(double.IsPositiveInfinity(weight) ? "∞" : weight.ToString("F1"))} ");
            }
            Console.WriteLine();
        }
        Console.WriteLine($"Is Connected: {wGraphAM2.IsConnected()}\n");

        Console.WriteLine("5. GraphLE (Edge List) Demonstration:");
        GraphLE graphLE2 = new GraphLE(5);
        graphLE2.AddEdge(0, 1);
        graphLE2.AddEdge(0, 2);
        graphLE2.AddEdge(1, 3);
        graphLE2.AddEdge(2, 4);
        graphLE2.AddEdge(3, 4);
        
        Console.WriteLine("Edge List Representation:");
        foreach ((int from, int to) edge in graphLE2.ToEdgeList())
        {
            Console.WriteLine($"{edge.from} -> {edge.to}");
        }
        Console.WriteLine($"Is Connected: {graphLE2.IsConnected()}\n");

        Console.WriteLine("6. WGraphLE (Weighted Edge List) Demonstration:");
        WGraphLE wGraphLE2 = new WGraphLE(5);
        wGraphLE2.AddEdge(0, 1, 2.5);
        wGraphLE2.AddEdge(0, 2, 1.5);
        wGraphLE2.AddEdge(1, 3, 3.0);
        wGraphLE2.AddEdge(2, 4, 2.0);
        wGraphLE2.AddEdge(3, 4, 1.0);
        
        Console.WriteLine("Weighted Edge List Representation:");
        foreach ((int from, int to, double weight) edge in wGraphLE2.ToWeightedEdgeList())
        {
            Console.WriteLine($"{edge.from} -> {edge.to} (weight: {edge.weight})");
        }
        Console.WriteLine($"Is Connected: {wGraphLE2.IsConnected()}\n");

        Console.WriteLine("7. TreeLP (Parent List) Demonstration:");
        TreeLP treeLP = new TreeLP(7, 0);
        treeLP.SetParent(1, 0);
        treeLP.SetParent(2, 0);
        treeLP.SetParent(3, 1);
        treeLP.SetParent(4, 1);
        treeLP.SetParent(5, 2);
        treeLP.SetParent(6, 2);
        
        Console.WriteLine("Tree Structure:");
        for (int i = 0; i < treeLP.VertexCount; i++)
        {
            Console.WriteLine($"Vertex {i}: Parent = {treeLP.GetParent(i)}, " +
                            $"Depth = {treeLP.GetDepth(i)}, " +
                            $"Height = {treeLP.GetHeight(i)}, " +
                            $"Is Leaf = {treeLP.IsLeaf(i)}");
        }
        
        Console.WriteLine($"Total Leaf Count: {treeLP.GetLeafCount()}");
        Console.WriteLine($"LCA of 3 and 5: {treeLP.GetLowestCommonAncestor(3, 5)}");
        Console.WriteLine($"Is 1 ancestor of 4: {treeLP.IsAncestor(1, 4)}");
        
        Console.WriteLine("\nAncestors of vertex 4:");
        Console.WriteLine(string.Join(" -> ", treeLP.GetAncestors(4)));
    }
}