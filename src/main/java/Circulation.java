import com.sun.javafx.geom.Edge;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;

// TODO: 4/6/18 rewrite input function so it creates array graph from beginning


public class Circulation {
    FastScanner scanner;

    public Circulation(){
        scanner = new FastScanner();
    }

    public void findCirculation(Inputter input, Outputter output){
        FlowGraph graph = buildGraph(input);
        graph = simplifyFlowGraph(graph);
//        FlowPath fp = findPathBfs(graph,graph.getSource(), graph.getSink());
        int[][] arrayGraph = flowGraphToArray(graph);
        int maxFlow = findMaxFlow(arrayGraph,graph.source,graph.sink);
        System.out.println("Maxflow = " + maxFlow);
    }

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
        List<Integer> nodeDemand = new ArrayList<>(newGraph.getDemand());
        for (int i = 0; i < nodeDemand.size(); i++) {
            int demand = nodeDemand.get(i);
            if(demand>0){
                newGraph.setSource(getSourceOrSink(newGraph, true, i, demand));
            } else if (demand<0){
                newGraph.setSink(getSourceOrSink(newGraph, false, i, demand));
            }
        }
        return newGraph;
    }

    /**
     * sets a source or sink, links to demand node and removes deman
     * @param newGraph
     * @param sourceOrSink
     * @param i
     * @param demand
     * @return the number of the source or sink
     * 
     */
    private int getSourceOrSink(FlowGraph newGraph, boolean source, int i, int demand) {
        int sourceOrSink = source?newGraph.getSource():newGraph.getSink();
        if (sourceOrSink < 0) {
            sourceOrSink = newGraph.addNode(demand);
        } else {
            newGraph.changeNodeDemand(sourceOrSink, demand);
        }
        if(source) {
            newGraph.linkNodeToNode(i, sourceOrSink, 0, demand);
        } else {
            newGraph.linkNodeToNode(sourceOrSink,  i, 0, -demand);
        }
        newGraph.setNodeDemand(i,0);
        return sourceOrSink;
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

    private FlowPath findPathBfs(FlowGraph graph, int startNode, int endNode){
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

    private FlowPath drawPathFromStepBack(int[] incomingEdgeOnPath, int lastNode, int startNode, FlowGraph graph){
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

    private FlowGraph buildGraph(Inputter input){
        FlowGraph graph = new FlowGraph(input.getN());
        int[][] data = input.getData();
        for(int[] dataRow:data){
            graph.addEdge(dataRow[0],dataRow[1],dataRow[2], dataRow[3]);
        }
        return graph;
    }

    static class Edge {
        private int from, to, capacity, minCapacity, flow, index;

        public Edge(int from, int to, int minCapacity, int capacity, int index) {
            this.from = from;
            this.to = to;
            this.minCapacity = minCapacity;
            this.capacity = capacity;
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
        private int source = -1;
        private int sink = -1;
        /* These adjacency lists store only indices of edges from the edges list */
        private List<List<Integer>> graph;

        private List<Integer> demand;

        public void setSource(int source) {
            this.source = source;
        }

        public void setSink(int sink) {
            this.sink = sink;
        }

        public int getSource() {
            return source;
        }

        public int getSink() {
            return sink;
        }

        public List<Integer> getNode(int index){
            return graph.get(index);
        }

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
        public void addEdge(int from, int to, int minCapacity, int capacity) {
            /* Note that we first append a forward edge and then a backward edge,
             * so all forward edges are stored at even indices (starting from 0),
             * whereas backward edges are stored at odd indices. */
            int forwardEdgeIndex = edges.size();
            Edge forwardEdge = new Edge(from, to, minCapacity, capacity, forwardEdgeIndex);
            Edge backwardEdge = new Edge(to, from, minCapacity, capacity, forwardEdgeIndex + 1);
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
            copy.edges = new ArrayList<>(edges);
            copy.graph = new ArrayList<>();
            copy.demand = new ArrayList<>(demand);
            Collections.copy(edges, copy.edges);
            for(List<Integer> node:graph){
                List<Integer> copyNode = new ArrayList<>(node);
                copy.graph.add(copyNode);
            }
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

        public void linkNodeToNode(int from, int to, int minCapacity, int capacity){
            addEdge(from,to,minCapacity,capacity);
            graph.get(from).add(edges.size()-1);
            graph.get(to).add(edges.size()-1);
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


    private int[][] flowGraphToArray(FlowGraph flowGraph){
        int numVertices = flowGraph.graph.size();
        int[][] rtrn = new int[numVertices][numVertices];
        for(int i=0;i<flowGraph.size();i++){
            List<Integer> node = flowGraph.getNode(i);
            for(int edgeNum:node){
                if(edgeNum%2!=0){
                    continue;
                }
                Edge edge = flowGraph.getEdge(edgeNum);
                rtrn[i][edge.getTo()]=edge.getCapacity();
            }
        }
    return rtrn;
    }

    private int findMaxFlow(int[][] mainGraph, int s, int t){
        int u;
        int v;
        int numVertices = mainGraph.length;

        int[][] reverseGraph = new int[numVertices][numVertices];
        for(u=0;u<numVertices;u++){
            for(v = 0; v< numVertices;v++){
                reverseGraph[v][u] = mainGraph[u][v];
            }
        }

        int parent[] = new int[numVertices];

        int maxFlow = 0;

        while (breadthFirstSearch(reverseGraph,s,t,parent)){
            int pathFlow = Integer.MAX_VALUE;

            v = t;
            while(v!=s){
                u=parent[v];
                pathFlow = Math.min(pathFlow, reverseGraph[u][v]);
                v=u;
            }

            v = t;
            while(v!=s){
                u = parent[v];
                reverseGraph[u][v] -= pathFlow;
                reverseGraph[v][u] += pathFlow;
                v = u;
            }
// TODO: 4/6/18 figure out the flow for each edge
            maxFlow += pathFlow;
        }


        return maxFlow;

    }

    private boolean breadthFirstSearch(int[][] reverseGraph, int s, int t, int[] parent){
        int numVertices = reverseGraph.length;
        boolean visited[] = new boolean[numVertices];
        Arrays.fill(visited,false);

        Deque<Integer> queue = new ArrayDeque<>();

        queue.add(s);
        visited[s] = true;
        parent[s] = -1;


        while(!queue.isEmpty()) {
            int u = queue.poll();

            for(int v=0;v<numVertices;v++){
                if(!visited[v] && reverseGraph[u][v]>0){
                    queue.add(v);
                    parent[v]=u;
                    visited[v] = true;
                }
            }

        }

        return visited[t];
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
                    int num = (scanner.nextInt());
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
            if(j<2){
                data[i][j] = num - 1;
            } else {
                data[i][j] = num;
            }
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
