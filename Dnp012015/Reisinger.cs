namespace Dnp012015.Reisinger
{
    using System;
    using contest.submission.contract;
    using System.Collections.Generic;
    using System.Collections;

    [Serializable]
    public class Solution : IDnp1501Solution
    {

        private static Path<Node> result;

        public void Start(BoolArray ground, Point startpoint, Point endpoint)
        {
            Node[,] nodeArray = new Node[1024, 1024];

            for (var i = 0; i < 1024; i++)
            {
                for (var j = 0; j < 1024; j++)
                {
                    if (!ground.Data[i, j])
                    {
                        nodeArray[i, j] = new Node(i, j, new List<Node>());
                    }
                }
            }

            #region Nachbarn
            var topLeft = nodeArray[0, 0];
            if (topLeft != null)
            {
                if (nodeArray[0, 1] != null) topLeft.AddNeighbour(nodeArray[0, 1]);
                if (nodeArray[1, 1] != null) topLeft.AddNeighbour(nodeArray[1, 1]);
                if (nodeArray[1, 0] != null) topLeft.AddNeighbour(nodeArray[1, 0]);
            }

            var topRight = nodeArray[1023, 0];
            if (topRight != null)
            {
                if (nodeArray[1022, 0] != null) topRight.AddNeighbour(nodeArray[1022, 0]);
                if (nodeArray[1022, 1] != null) topRight.AddNeighbour(nodeArray[1022, 1]);
                if (nodeArray[1023, 1] != null) topRight.AddNeighbour(nodeArray[1023, 1]);
            }

            var bottomLeft = nodeArray[0, 1023];
            if (bottomLeft != null)
            {
                if (nodeArray[0, 1022] != null) bottomLeft.AddNeighbour(nodeArray[0, 1022]);
                if (nodeArray[1, 1022] != null) bottomLeft.AddNeighbour(nodeArray[1, 1022]);
                if (nodeArray[1, 1023] != null) bottomLeft.AddNeighbour(nodeArray[1, 1023]);
            }

            var bottomRight = nodeArray[1023, 1023];
            if (bottomRight != null)
            {
                if (nodeArray[1023, 1022] != null) bottomRight.AddNeighbour(nodeArray[1023, 1022]);
                if (nodeArray[1022, 1022] != null) bottomRight.AddNeighbour(nodeArray[1022, 1022]);
                if (nodeArray[1022, 1023] != null) bottomRight.AddNeighbour(nodeArray[1022, 1023]);
            }

            for (var k = 1; k < 1023; k++)
            {
                if (nodeArray[0, k] != null)
                {
                    if (nodeArray[0, k - 1] != null) nodeArray[0, k].AddNeighbour(nodeArray[0, k - 1]);
                    if (nodeArray[0, k + 1] != null) nodeArray[0, k].AddNeighbour(nodeArray[0, k + 1]);
                    if (nodeArray[1, k - 1] != null) nodeArray[0, k].AddNeighbour(nodeArray[1, k - 1]);
                    if (nodeArray[1, k] != null) nodeArray[0, k].AddNeighbour(nodeArray[1, k]);
                    if (nodeArray[1, k + 1] != null) nodeArray[0, k].AddNeighbour(nodeArray[1, k + 1]);
                }
                if (nodeArray[1023, k] != null)
                {
                    if (nodeArray[1023, k - 1] != null) nodeArray[1023, k].AddNeighbour(nodeArray[1023, k - 1]);
                    if (nodeArray[1023, k + 1] != null) nodeArray[1023, k].AddNeighbour(nodeArray[1023, k + 1]);
                    if (nodeArray[1022, k - 1] != null) nodeArray[1023, k].AddNeighbour(nodeArray[1022, k - 1]);
                    if (nodeArray[1022, k] != null) nodeArray[1023, k].AddNeighbour(nodeArray[1022, k]);
                    if (nodeArray[1022, k + 1] != null) nodeArray[1023, k].AddNeighbour(nodeArray[1022, k + 1]);
                }
                if (nodeArray[k, 0] != null)
                {
                    if (nodeArray[k - 1, 0] != null) nodeArray[k, 0].AddNeighbour(nodeArray[k - 1, 0]);
                    if (nodeArray[k + 1, 0] != null) nodeArray[k, 0].AddNeighbour(nodeArray[k + 1, 0]);
                    if (nodeArray[k - 1, 1] != null) nodeArray[k, 0].AddNeighbour(nodeArray[k - 1, 1]);
                    if (nodeArray[k, 1] != null) nodeArray[k, 0].AddNeighbour(nodeArray[k, 1]);
                    if (nodeArray[k + 1, 1] != null) nodeArray[k, 0].AddNeighbour(nodeArray[k + 1, 1]);
                }
                if (nodeArray[k, 1023] != null)
                {
                    if (nodeArray[k - 1, 1023] != null) nodeArray[k, 1023].AddNeighbour(nodeArray[k - 1, 1023]);
                    if (nodeArray[k + 1, 1023] != null) nodeArray[k, 1023].AddNeighbour(nodeArray[k + 1, 1023]);
                    if (nodeArray[k - 1, 1022] != null) nodeArray[k, 1023].AddNeighbour(nodeArray[k - 1, 1022]);
                    if (nodeArray[k, 1022] != null) nodeArray[k, 1023].AddNeighbour(nodeArray[k, 1022]);
                    if (nodeArray[k + 1, 1022] != null) nodeArray[k, 1023].AddNeighbour(nodeArray[k + 1, 1022]);
                }
            }

            for (var i = 1; i < 1023; i++)
            {
                for (var j = 1; j < 1023; j++)
                {
                    var node = nodeArray[i, j];
                    if (node != null)
                    {
                        if (nodeArray[i - 1, j - 1] != null) node.AddNeighbour(nodeArray[i - 1, j - 1]);
                        if (nodeArray[i, j - 1] != null) node.AddNeighbour(nodeArray[i, j - 1]);
                        if (nodeArray[i + 1, j - 1] != null) node.AddNeighbour(nodeArray[i + 1, j - 1]);
                        if (nodeArray[i - 1, j] != null) node.AddNeighbour(nodeArray[i - 1, j]);
                        if (nodeArray[i + 1, j] != null) node.AddNeighbour(nodeArray[i + 1, j]);
                        if (nodeArray[i - 1, j + 1] != null) node.AddNeighbour(nodeArray[i - 1, j + 1]);
                        if (nodeArray[i, j + 1] != null) node.AddNeighbour(nodeArray[i, j + 1]);
                        if (nodeArray[i + 1, j + 1] != null) node.AddNeighbour(nodeArray[i + 1, j + 1]);
                    }
                }
            }

            # endregion

            Func<Node, Node, double> distance = (start, stop) =>
            {

                var deltaX = start.X - stop.X;
                var deltaY = start.Y - stop.Y;

                if (deltaX == 0)
                {
                    return Math.Abs(deltaY);
                }

                if (deltaY == 0)
                {
                    return Math.Abs(deltaX);
                }

                var cost = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);

                return cost;
            };

            Func<Node, double> estimate = (current) =>
            {

                var deltaX = current.X - startpoint.x;
                var deltaY = current.Y - startpoint.y;

                if (deltaX == 0)
                {
                    return Math.Abs(deltaY);
                }

                if (deltaY == 0)
                {
                    return Math.Abs(deltaX);
                }

                // var cost = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
                var cost = Math.Max(deltaX, deltaY);

                return cost;
            };

            result = FindPath(nodeArray[endpoint.x, endpoint.y], nodeArray[startpoint.x, startpoint.y], distance, estimate);

            NextStep();
        }

        public void NextStep()
        {
            Point pos = new Point();

            pos.x = result.LastStep.X;
            pos.y = result.LastStep.Y;

            result = result.PreviousSteps;

            MakeMove(pos);

        }

        public event Action<Point> MakeMove;

        static public Path<Node> FindPath<Node>(Node start, Node destination, Func<Node, Node, double> distance, Func<Node, double> estimate) where Node : IHasNeighbours<Node>
        {
            var queue = new PriorityQueue<double, Path<Node>>();
            queue.Enqueue(0, new Path<Node>(start));

            while (!queue.IsEmpty)
            {
                var path = queue.Dequeue();
                var currentNode = path.LastStep;

                if (currentNode.IsClosed == true)
                {
                    continue;
                }

                if (currentNode.Equals(destination))
                {
                    return path;
                }
                currentNode.IsClosed = true;

                foreach (Node neighbour in currentNode.Neighbours)
                {
                    if (!neighbour.IsClosed)
                    {
                        double d = distance(currentNode, neighbour);
                        var newPath = path.AddStepp(neighbour, d);
                        var heuristic = estimate(neighbour);
                        queue.Enqueue(newPath.TotalCost + heuristic, newPath);
                    }
                }
            }
            return null;
        }
    }

    public class Path<Node> : IEnumerable<Node>
    {
        public Node LastStep { get; private set; }
        public Path<Node> PreviousSteps { get; private set; }
        public double TotalCost { get; private set; }
        private Path(Node lastStep, Path<Node> previousSteps, double totalCost)
        {
            LastStep = lastStep;
            PreviousSteps = previousSteps;
            TotalCost = totalCost;
        }
        public Path(Node start) : this(start, null, 0) { }
        public Path<Node> AddStepp(Node step, double stepCost)
        {
            return new Path<Node>(step, this, TotalCost + stepCost);
        }
        public IEnumerator<Node> GetEnumerator()
        {
            for (Path<Node> p = this; p != null; p = p.PreviousSteps)
            {
                yield return p.LastStep;
            }
        }
        IEnumerator IEnumerable.GetEnumerator()
        {
            return this.GetEnumerator();
        }

    }

    public class PriorityQueue<P, V> where P : IComparable
    {
        // private SortedDictionary<P, Queue<V>> list = new SortedDictionary<P, Queue<V>>();
        private SortedList<P, Queue<V>> list = new SortedList<P, Queue<V>>(8192 * 2);
        public void Enqueue(P priority, V value)
        {
            Queue<V> q;
            var index = list.IndexOfKey(priority);
            if (index == -1)
            {
                //if (!list.TryGetValue(priority, out q)) {
                q = new Queue<V>();
                list.Add(priority, q);
            }
            else
            {
                q = list.Values[index];
            }
            q.Enqueue(value);
        }

        public V Dequeue()
        {
            var key = list.Keys[0];
            var queue = list.Values[0];
            // var pair = list.First();
            // var v = pair.Value.Dequeue();
            var v = queue.Dequeue();
            // if (pair.Value.Count == 0) {
            if (queue.Count == 0)
            {
                // nothing left of the top priority
                // list.Remove(pair.Key);
                list.RemoveAt(0);
            }
            return v;
        }

        public bool IsEmpty
        {
            get { return list.Count == 0; }
        }

    }

    public interface IHasNeighbours<N>
    {
        IEnumerable<N> Neighbours { get; }
        bool IsClosed { get; set; }
    }

    public class Node : IHasNeighbours<Node>
    {

        public int X { get; private set; }
        public int Y { get; private set; }

        public bool IsClosed { get; set; }

        private List<Node> _neighbours;

        public Node(int x, int y, List<Node> neighbours)
        {
            this.X = x;
            this.Y = y;
            this._neighbours = neighbours;
            this.IsClosed = false;
        }

        public void AddNeighbour(Node neighbour)
        {
            this._neighbours.Add(neighbour);
        }

        public IEnumerable<Node> Neighbours
        {
            get { return this._neighbours; }
        }

        public override bool Equals(object obj)
        {
            return this.Equals(obj as Node);
        }

        public bool Equals(Node obj)
        {
            if (ReferenceEquals(obj, null))
            {
                return false;
            }

            if (ReferenceEquals(obj, this))
            {
                return true;
            }

            return this.X == obj.X && this.Y == obj.Y;
        }

        public override int GetHashCode()
        {
            int hash = 17;
            hash = hash * 17 + X;
            hash = hash * 17 + Y;
            return hash;
        }

    }

}