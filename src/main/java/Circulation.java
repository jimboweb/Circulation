import com.sun.javafx.geom.Edge;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;

public class Circulation {
    FastScanner scanner;

    public Circulation(){
        scanner = new FastScanner();
    }

    public void findCirculation(Inputter input, Outputter output){
        FlowGraph graph = buildGraph(input);
        FlowPath fp = findPathBfs(graph,0, 0);
        graph = simplifyFlowGraph(graph);

    }

    // FIXME: 3/25/18 need to just do this for even edges
    // TODO: 3/25/18 I can do this all in the createGraph method faster probably
    /**
     * remove minimum demand and replace with demands on nodes
     * @param originalGraph
     * @return
     */
    public FlowGraph simplifyFlowGraph(FlowGraph originalGraph){
        FlowGraph newGraph = removeMinDemand(originalGraph);
        newGraph = createSourceAndSink(newGraph);

        return newGraph;
    }

    private FlowGraph createSourceAndSink(FlowGraph newGraph) {
        int source = -1;
        int sink = -1;
        List<Integer> nodeDemand = new ArrayList<>();
        Collections.copy(newGraph.getDemand(), nodeDemand);
        for (int i = 0; i < nodeDemand.size(); i++) {
            int demand = nodeDemand.get(i);
            if(demand>0){
                if (source < 0) {
                    source = newGraph.addNode(demand);
                } else {
                    newGraph.changeNodeDemand(source, -demand);
                }
                newGraph.linkNodeToNode(source,i);
                newGraph.setNodeDemand(i,0);
            } else if (demand>0){
                if(sink<0){
                    sink = newGraph.addNode(demand);
                } else {
                    newGraph.changeNodeDemand(source,demand);
                }
                newGraph.linkNodeToNode(i,sink);
                newGraph.setNodeDemand(i,0);
            }
        }
        return newGraph;
    }

    private FlowGraph removeMinDemand(FlowGraph originalGraph) {
        FlowGraph newGraph = originalGraph.copy();
        Iterator<Edge> edgeIterator = originalGraph.getEdgeIteratorForGraph();
        List<Edge> edges = originalGraph.getEdges();
        //only loop over the even numbers
       for(int i=0;i<edges.size();i+=2){
            Edge originalGraphEdge = originalGraph.getEdge(i);
            Edge newGraphForwardEdge = newGraph.getEdge(i);
            Edge newGraphBackwardEdge = newGraph.getEdge(i+1);
            int edgeMin = originalGraphEdge.getMinCapacity();
            if(edgeMin>0){
                newGraphForwardEdge.changeCapacity(-edgeMin);
                newGraphBackwardEdge.changeCapacity(-edgeMin);
                newGraph.changeNodeDemand(originalGraphEdge.getFrom(), edgeMin);
                newGraph.changeNodeDemand(originalGraphEdge.getTo(), -edgeMin);
            }
        }
        return newGraph;
    }

    public FlowPath findPathBfs(FlowGraph graph, int startNode, int endNode){
        FlowPath fp = new FlowPath();
        Deque<Integer> nodeQueue = new ArrayDeque<>();
        int[] incomingEdgeOnPath = new int[graph.size()];
        Arrays.fill(incomingEdgeOnPath,-1);
        incomingEdgeOnPath[startNode] = -2;
        nodeQueue.push(startNode);
        int lastNode = -1;
        while(!nodeQueue.isEmpty() && lastNode == -1){
            int nodeNumber = nodeQueue.pop();
            Iterator<Integer> edgeIterator = graph.getEdgeIteratorForNode(nodeNumber);
            while(edgeIterator.hasNext()){
                Edge nextEdge = graph.getEdge(edgeIterator.next());
                if(nextEdge.getIndex()%2!=0){
                    continue;
                }
                int nextNode = nextEdge.getTo();
                if(nextNode==endNode){
                    incomingEdgeOnPath[nextNode] = nextEdge.index;
                    lastNode = nextNode;
                    break;
                }

                if(incomingEdgeOnPath[nextNode]<0){
                    incomingEdgeOnPath[nextNode] = nextEdge.index;
                    nodeQueue.push(nextNode);
                }
            }
        }
        fp = drawPathFromStepBack(incomingEdgeOnPath, lastNode, startNode, graph);
        //if there is no path it will return an empty path; shouldn't happen
        return fp;
    }

    public FlowPath drawPathFromStepBack(int[] incomingEdgeOnPath, int lastNode, int startNode, FlowGraph graph){
        FlowPath fp = new FlowPath();
        int previousEdgeNum = incomingEdgeOnPath[lastNode];
        int maxFlow = Integer.MAX_VALUE;
        while(previousEdgeNum>0) {
            fp.push(previousEdgeNum);
            Edge prevEdge = graph.getEdge(previousEdgeNum);
            int prevFlow = prevEdge.getFlow();
            if(prevFlow<maxFlow)
            {
                maxFlow = prevFlow;
            }
            lastNode = prevEdge.from;
            if(lastNode==startNode){
                break;
            }
            previousEdgeNum = incomingEdgeOnPath[lastNode];
        }
        fp.setFlow(maxFlow);
        return fp;
    }

    public FlowGraph buildGraph(Inputter input){
        FlowGraph graph = new FlowGraph(input.getN());
        int[][] data = input.getData();
        for(int[] dataRow:data){
            graph.addEdge(dataRow[0],dataRow[1],dataRow[2], dataRow[3]);
        }
        return graph;
    }

    static class Edge {
        private int from, to, capacity, minCapacity, flow, index;

        public Edge(int from, int to, int capacity, int minCapacity, int index) {
            this.from = from;
            this.to = to;
            this.capacity = capacity;
            this.minCapacity = minCapacity;
            this.flow = 0;
            this.index = index;
        }

        public int getIndex() {
            return index;
        }

        public int getFrom() {
            return from;
        }

        public int getTo() {
            return to;
        }

        public int getCapacity() {
            return capacity;
        }

        public int getMinCapacity() {
            return minCapacity;
        }

        public int getFlow() {
            return flow;
        }

        public void setMinCapacity(int minCapacity) {
            this.minCapacity = minCapacity;
        }

        public void setFrom(int from) {
            this.from = from;
        }

        public void setTo(int to) {
            this.to = to;
        }

        public void setCapacity(int capacity) {
            this.capacity = capacity;
        }

        public void setFlow(int flow) {
            this.flow = flow;
        }

        public void setIndex(int index) {
            this.index = index;
        }

        public void changeCapacity(int amount){
            capacity+=amount;
        }

        public void changeMinCapacity(int amount){
            minCapacity+=amount;
        }
    }

    /**
     * A graph with an array of edges and a list of integer node links to edges
     */
    static class FlowGraph {
        /* List of all - forward and backward - edges */
        private List<Edge> edges;

        /* These adjacency lists store only indices of edges from the edges list */
        private List<List<Integer>> graph;

        private List<Integer> demand;

        /**
         * create a graph with n nodes
         * @param n
         */
        public FlowGraph(int n) {
            this.graph = new ArrayList<>(n);
            demand = new ArrayList<>();
            for(int i=0;i<n;i++){
                demand.add(0);
            }
            for (int i = 0; i < n; ++i)
                this.graph.add(new ArrayList());
            this.edges = new ArrayList<>();
        }

        /**
         * Adds two edges, a forward and backward edge.
         * Forward edges will always be even numbers and backward edges will be odd.
         * @param from edge from
         * @param to edge to
         * @param capacity
         */
        public void addEdge(int from, int to, int capacity, int minCapacity) {
            /* Note that we first append a forward edge and then a backward edge,
             * so all forward edges are stored at even indices (starting from 0),
             * whereas backward edges are stored at odd indices. */
            int forwardEdgeIndex = edges.size();
            Edge forwardEdge = new Edge(from, to, capacity, minCapacity, forwardEdgeIndex);
            Edge backwardEdge = new Edge(to, from, 0, minCapacity, forwardEdgeIndex + 1);
            graph.get(from).add(edges.size());
            edges.add(forwardEdge);
            graph.get(to).add(edges.size());
            edges.add(backwardEdge);
        }

        /**
         * @return size of node list
         */
        public int size() {
            return graph.size();
        }

        public Edge getEdge(int id) {
            return edges.get(id);
        }

        public void addFlow(int id, int flow) {
            /* To get a backward edge for a true forward edge (i.e id is even), we should get id + 1
             * due to the described above scheme. On the other hand, when we have to get a "backward"
             * edge for a backward edge (i.e. get a forward edge for backward - id is odd), id - 1
             * should be taken.
             *
             * It turns out that id ^ 1 works for both cases. Think this through! */
            edges.get(id).flow += flow;
            edges.get(id ^ 1).flow -= flow;
        }

        public Iterator<Integer> getEdgeIteratorForNode(int node){
            return graph.get(node).iterator();
        }

        public Iterator<Edge> getEdgeIteratorForGraph(){
            return edges.iterator();
        }

        /**
         * create graph with same number of nodes and same edges
         * @return copy of graph
         */
        public FlowGraph copy(){
            FlowGraph copy = new FlowGraph(size());
            copy.edges = edges;
            copy.graph = graph;
            return copy;
        }

        public void setNodeDemand(int node, int amount){
            demand.set(node, amount);
        }

        public void changeNodeDemand(int node, int amount){
            demand.set(node, demand.get(node) + amount);
        }

        public int addNode(int d){
            int nodeNumber = size();
            List<Integer> newNode = new ArrayList<>();
            graph.add(newNode);
            demand.add(d);
            return nodeNumber;
        }

        public List<Integer> getDemand(){
            return demand;
        }

        public void linkNodeToNode(int from, int to){
            graph.get(from).add(to);
        }

        public List<Edge> getEdges() {
            return edges;
        }
    }

    /**
     * A path through the edges of the graph
     * @param flow - the flow of the path
     */
    static class FlowPath{
        private Deque<Integer> edges;
        private int flow = 0;
        public FlowPath(){
            edges = new ArrayDeque<>();
        }
        public FlowPath(Deque<Integer> edges, int flow){
            this.edges=edges;
            this.flow=flow;
        }

        /**
         * push to edges queue
         * @param edge
         */
        public void push(int edge){
            edges.push(edge);
        }

        /**
         * pop from edges queue
         * @return the last edge (and delete it)
         */
        public int pop(){
            return edges.pop();
        }

        /**
         * set the flow
         * @param flow
         */
        public void setFlow(int flow) {
            this.flow = flow;
        }

        public boolean isEmpty(){
            return edges.isEmpty();
        }
    }











    public static void main(String[] args){
        new Circulation().run();
    }

    public void run(){
        Input input = input();
        Output output = new Output();
        findCirculation(input,output);
    }

    public Input input(){
        try {
            int n = scanner.nextInt();
            int m = scanner.nextInt();
            Input rtrn = new Input(n,m);
            for(int i=0;i<m;i++){
                for(int j=0;j<4;j++){
                    int num = (scanner.nextInt()-1);
                    rtrn.setData(num,i,j);
                }
            }
            return rtrn;
        } catch (IOException e){
            System.out.println(e);
            System.exit(1);
        }
        return null;
    }

    static class FastScanner {
        private BufferedReader reader;
        private StringTokenizer tokenizer;

        public FastScanner() {
            reader = new BufferedReader(new InputStreamReader(System.in));
            tokenizer = null;
        }

        public String next() throws IOException {
            while (tokenizer == null || !tokenizer.hasMoreTokens()) {
                tokenizer = new StringTokenizer(reader.readLine());
            }
            return tokenizer.nextToken();
        }

        public int nextInt() throws IOException {
            return Integer.parseInt(next());
        }
    }

    interface Inputter{
        int getN();
        int[][] getData();
    }
    static class Input implements Inputter{
        final int n;
        int[][] data;
        public Input(int n, int m){
            this.n=n;
            data = new int[m][4];
        }
        public void setData(int num, int i, int j){
            data[i][j] = num;
        }

        public int getN() {
            return n;
        }

        public int[][] getData() {
            return data;
        }
    }

    interface Outputter{
        public void print(String str);
        public void println(String str);
    }

    class Output implements Outputter {
        public void print(String str){
            System.out.print(str);
        }
        public void println(String str){
            System.out.println(str);
        }
    }
}
