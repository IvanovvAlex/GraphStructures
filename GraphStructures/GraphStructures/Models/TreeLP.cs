using System;
using System.Collections.Generic;
using System.Linq;

namespace GraphStructures.Models
{
    public class TreeLP
    {
        private readonly int[] _parents;
        private readonly int _root;

        public TreeLP(int vertexCount, int root = 0)
        {
            if (root < 0 || root >= vertexCount)
                throw new ArgumentException("Root must be a valid vertex index");

            _parents = new int[vertexCount];
            _root = root;
            _parents[root] = -1;
        }

        public TreeLP(int[] parents, int root = 0)
        {
            if (root < 0 || root >= parents.Length)
                throw new ArgumentException("Root must be a valid vertex index");

            _parents = new int[parents.Length];
            Array.Copy(parents, _parents, parents.Length);
            _root = root;
        }

        public int VertexCount => _parents.Length;
        public int Root => _root;

        public void SetParent(int vertex, int parent)
        {
            if (vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            if (parent < -1 || parent >= VertexCount)
                throw new ArgumentException("Invalid parent index");
            if (vertex == _root && parent != -1)
                throw new ArgumentException("Cannot set parent for root vertex");
            _parents[vertex] = parent;
        }

        public int GetParent(int vertex)
        {
            if (vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            return _parents[vertex];
        }

        public IEnumerable<int> GetChildren(int vertex)
        {
            if (vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            for (int i = 0; i < VertexCount; i++)
            {
                if (_parents[i] == vertex)
                    yield return i;
            }
        }

        public int GetDepth(int vertex)
        {
            if (vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            int depth = 0;
            int current = vertex;
            while (_parents[current] != -1)
            {
                depth++;
                current = _parents[current];
            }
            return depth;
        }

        public int GetHeight(int vertex)
        {
            if (vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            int maxHeight = 0;
            foreach (int child in GetChildren(vertex))
            {
                maxHeight = Math.Max(maxHeight, GetHeight(child) + 1);
            }
            return maxHeight;
        }

        public bool IsLeaf(int vertex)
        {
            if (vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            return !GetChildren(vertex).Any();
        }

        public int GetLeafCount()
        {
            return Enumerable.Range(0, VertexCount).Count(IsLeaf);
        }

        public int[] GetAncestors(int vertex)
        {
            if (vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            List<int> ancestors = new List<int>();
            int current = vertex;
            while (_parents[current] != -1)
            {
                current = _parents[current];
                ancestors.Add(current);
            }
            return ancestors.ToArray();
        }

        public bool IsAncestor(int potentialAncestor, int vertex)
        {
            if (potentialAncestor < 0 || potentialAncestor >= VertexCount ||
                vertex < 0 || vertex >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            return GetAncestors(vertex).Contains(potentialAncestor);
        }

        public int GetLowestCommonAncestor(int vertex1, int vertex2)
        {
            if (vertex1 < 0 || vertex1 >= VertexCount ||
                vertex2 < 0 || vertex2 >= VertexCount)
                throw new ArgumentException("Invalid vertex index");
            HashSet<int> ancestors1 = new HashSet<int>(GetAncestors(vertex1));
            int[] ancestors2 = GetAncestors(vertex2);
            foreach (int ancestor in ancestors2)
            {
                if (ancestors1.Contains(ancestor))
                    return ancestor;
            }
            return _root;
        }

        public int[] ToParentList()
        {
            int[] result = new int[VertexCount];
            Array.Copy(_parents, result, VertexCount);
            return result;
        }
    }
} 