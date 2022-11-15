import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class PathFinder {

    private int numberOfNodes;
    private int numberOfEdges;
    Map<Integer, Pair<Integer, Integer>> nodeCoordinates;
    Map<Integer, List<Integer>> adjGraph;

    public PathFinder() {

    }

    public void readInput(String fileName) throws FileNotFoundException {
        File file = new File(fileName);
        Scanner sc = new Scanner(file);
        numberOfNodes = sc.nextInt();
        numberOfEdges = sc.nextInt();
        nodeCoordinates = new HashMap<>();
        adjGraph = new HashMap<>();

//        System.out.println("Number of nodes : " + numberOfNodes);
//        System.out.println("Number of edges  : " + numberOfEdges);
        for (int i = 0; i < numberOfNodes; i++) {
            int node = sc.nextInt();
            nodeCoordinates.put(node, new Pair<>(sc.nextInt(), sc.nextInt()));
//             System.out.println("Node is : " + node);
            adjGraph.put(node, new ArrayList<>());
        }

        for (int i = 0; i < numberOfEdges; i++) {
            int start = sc.nextInt();
            int dest = sc.nextInt();
            adjGraph.get(start).add(dest);
            adjGraph.get(dest).add(start);
        }
    }


    public double distToDest(int source, int destination, int k) {
        Dijkstra dijkstra = new Dijkstra(numberOfNodes, nodeCoordinates, adjGraph, source, destination, k);
        dijkstra.run();
        return dijkstra.distToDest();
    }

    public int noOfShortestPaths(int source, int destination, int k) {
        Dijkstra dijkstra = new Dijkstra(numberOfNodes, nodeCoordinates, adjGraph, source, destination, k);
        dijkstra.run();
        return dijkstra.noOfShortestPaths();
    }

    public ArrayList<Integer> fromSrcToDest(int source, int destination, int k) {
        Dijkstra dijkstra = new Dijkstra(numberOfNodes, nodeCoordinates, adjGraph, source, destination, k);
        dijkstra.run();
        return dijkstra.fromSrcToDest();
    }

    public static class Pair<A, B> {
        A x;
        B y;

        public Pair(A x, B y) {
            this.x = x;
            this.y = y;
        }
    }

    public static class MinHeap {

        int[] heapKeys = new int[100001];
        double[] heapValues = new double[100001];
        private final int k;
        private int heapSize = 0;

        public MinHeap() {
            this.k = 2;
        }

        public MinHeap(int k) {
            this.k = k;
        }

        public int size() {
            return heapSize;
        }

        public void add(int key, double value) {
            heapKeys[heapSize] = key;
            heapValues[heapSize] = value;

            int currentNode = heapSize;
            while (currentNode > 0) {
                int parent = (currentNode - 1) / k;
                if ((heapValues[parent] > heapValues[currentNode])
                        || (heapValues[parent] == heapValues[currentNode] &&
                        heapKeys[currentNode] < heapKeys[parent])) {
                    swapValues(currentNode, parent);
                    swapKeys(currentNode, parent);
                } else {
                    break;
                }
                currentNode = parent;
            }

            heapSize++;
        }

        public void removeMinimum() {
            swapKeys(0, heapSize - 1);
            swapValues(0, heapSize - 1);
            heapSize--;

            int currentNode = 0;
            while (currentNode < heapSize) {
                int minNode = currentNode;
                for (int i = 1; i <= k; i++) {
                    int childNode = currentNode * k + i;
                    if (childNode > heapSize)
                        break;
                    if ((heapValues[childNode] < heapValues[minNode]) ||
                            (heapValues[childNode] == heapValues[minNode] &&
                                    heapKeys[childNode] < heapKeys[minNode])) {
                        minNode = childNode;
                    }
                }
                if (minNode == currentNode)
                    break;
                swapKeys(currentNode, minNode);
                swapKeys(currentNode, minNode);
                currentNode = minNode;
            }
        }

        private void swapKeys(int currentNode, int parent) {
            int temp = heapKeys[parent];
            heapKeys[parent] = heapKeys[currentNode];
            heapKeys[currentNode] = temp;
        }

        private void swapValues(int currentNode, int parent) {
            double temp = heapValues[parent];
            heapValues[parent] = heapValues[currentNode];
            heapValues[currentNode] = temp;
        }

        public ArrayList<Integer> getHeap() {
            ArrayList<Integer> heapElements = new ArrayList<>();
            for (int i = 0; i < heapSize; i++) {
                heapElements.add(heapKeys[i]);
            }
            return heapElements;
        }

        public Pair<Integer, Double> getMin() {
            return new Pair<>(heapKeys[0], heapValues[0]);
        }
    }

    public static class Dijkstra {

        private final MinHeap minHeap;
        private final Map<Integer, List<Integer>> adjGraph;
        private final Map<Integer, Pair<Integer, Integer>> nodes;
        private final int numberOfNodes;
        private final double[] distance;
        private final int[] minPaths;
        private final int[] parent;
        private final int destination;
        private final int source;

        public Dijkstra(int numberOfNodes,
                        Map<Integer, Pair<Integer, Integer>> nodes,
                        Map<Integer, List<Integer>> adjGraph,
                        int source, int destination, int k) {
            this.minHeap = new MinHeap(k);
            this.nodes = nodes;
            this.adjGraph = adjGraph;
            this.numberOfNodes = numberOfNodes;
            this.distance = new double[numberOfNodes + 1];
            this.minPaths = new int[numberOfNodes + 1];
            this.parent = new int[numberOfNodes + 1];
            this.source = source;
            this.destination = destination;
        }


        public int noOfShortestPaths() {
            return minPaths[destination];
        }

        public double distToDest() {
            if (parent[destination] == destination && source != destination) {
                return -1;
            }
            return distance[destination] + find_distance(source, destination);
        }

        public ArrayList<Integer> fromSrcToDest() {
            if (parent[destination] == destination && source != destination) {
                return null;
            }
            ArrayList<Integer> reversePath = new ArrayList<>();
            ArrayList<Integer> path = new ArrayList<>();
            int currentNode = destination;
            while (currentNode != source) {
                reversePath.add(currentNode);
                currentNode = parent[currentNode];
            }
            path.add(source);
            for (int i = 0; i < reversePath.size(); i++) {
                path.add(reversePath.get(reversePath.size() - 1 - i));
            }
            return path;
        }

        public void run() {
            for (int i = 0; i < numberOfNodes; i++) {
                distance[i] = 100000000000000.0;
                minPaths[i] = 0;
                parent[i] = i;
            }

            minHeap.add(source, 0);
            distance[source] = 0.0;
            minPaths[source] = 1;
            parent[source] = source;

            while (minHeap.size() > 0) {
                Pair<Integer, Double> min = minHeap.getMin();
                minHeap.removeMinimum();
                int u = min.x;
                double disu = min.y;

                if (disu > distance[u])
                    continue;

                for (Integer v : adjGraph.get(u)) {
                    double newDistance = disu + find_distance(u, v) + find_distance(v, destination) -
                            find_distance(u, destination);
                    if (newDistance < distance[v]) {
                        distance[v] = newDistance;
                        minHeap.add(v, newDistance);
                        minPaths[v] = 1;
                        parent[v] = u;
                    } else if (newDistance == distance[v]) {
                        minPaths[v]++;
                    }
                }
            }
        }

        private double find_distance(int x, int y) {
            Pair<Integer, Integer> nodeX = nodes.get(x);
            Pair<Integer, Integer> nodeY = nodes.get(y);
            return Math.sqrt((nodeX.x - nodeY.x) * (nodeX.x - nodeY.x) +
                    (nodeX.y - nodeY.y) * (nodeX.y - nodeY.y));
        }
    }
}
