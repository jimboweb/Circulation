import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;


// TODO: 4/21/18 make it work for graph with minflow of 0 
// TODO: 4/21/18 TASK 0:make it output NO for graph with no circulation
// TODO: 4/6/18 rewrite input function so it creates array graph from beginning


public class Circulation {
    FastScanner scanner;

    public Circulation(){
        scanner = new FastScanner();
    }

    public void findCirculation(Inputter input, Outputter output){
        DeprecatedFlowGraph deprecatedFlowGraph = buildDeprecatedGraph(input);
        deprecatedFlowGraph = simplifyFlowGraph(deprecatedFlowGraph);
        FlowGraph graph = new FlowGraph(deprecatedFlowGraph);
        graph = findMaxFlow(graph);
        int[] edgeFlows = getEdgeFlow(graph);
        // TODO: 4/21/18 TASK 0 STEP 2: call graph.isFullCircuit, print YES or NO, if statement to continue
        for(int edgeFlow: edgeFlows){
            System.out.println(edgeFlow);
        }
    }

    private int[] getEdgeFlow(FlowGraph graph){
        int[] edgeFlow = new int[graph.numEdges];
        int[][] edges = graph.getEdges();
        for(int v=0;v<edges.length;v++){
            int from = edges[v][0];
            int to = edges[v][1];
            edgeFlow[v] = graph.getFlow(from,to) + graph.getMinCapacity(from, to);
        }
        return edgeFlow;
    }

    /**
     * remove minimum demand and replace with demands on nodes
     * @param originalGraph
     * @return
     */
    public DeprecatedFlowGraph simplifyFlowGraph(DeprecatedFlowGraph originalGraph){
        DeprecatedFlowGraph newGraph = removeMinDemand(originalGraph);
        newGraph = createSourceAndSink(newGraph);
        return newGraph;
    }

    private DeprecatedFlowGraph createSourceAndSink(DeprecatedFlowGraph newGraph) {
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
    private int getSourceOrSink(DeprecatedFlowGraph newGraph, boolean source, int i, int demand) {
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

    private DeprecatedFlowGraph removeMinDemand(DeprecatedFlowGraph originalGraph) {
        DeprecatedFlowGraph newGraph = originalGraph.copy();
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

    private DeprecatedFlowGraph buildDeprecatedGraph(Inputter input){
        DeprecatedFlowGraph graph = new DeprecatedFlowGraph(input.getN());
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
    static class DeprecatedFlowGraph {
        private int originalNumEdges = 0;
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
        public DeprecatedFlowGraph(int n) {
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
            originalNumEdges++;
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
        public DeprecatedFlowGraph copy(){
            DeprecatedFlowGraph copy = new DeprecatedFlowGraph(size());
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

    static class FlowGraph{
        // TODO: 4/21/18 TASK 0 STEP 0: add demand array for edges
        private final int numVertices;
        private final int numEdges;
        private int originalNumEdges;
        private final int[][] mainGraph;
        private int[][] reverseGraph;
        private final int[][] edges;
        private int[][] flow;
        private final int[][] minCapacity;
        int source;
        int sink;



        /**
         * until I get rid of deprecated flow graph
         * @param deprecatedFlowGraph
         */
        public FlowGraph(DeprecatedFlowGraph deprecatedFlowGraph){
            numVertices = deprecatedFlowGraph.size();
            numEdges = deprecatedFlowGraph.edges.size()/2;
            mainGraph = flowGraphToArray(deprecatedFlowGraph);
            edges = createEdges(deprecatedFlowGraph, numEdges);
            reverseGraph = createReverseGraph(mainGraph);
            flow = new int[numVertices][numVertices];
            minCapacity = createMinCapacity(deprecatedFlowGraph);
            // TODO: 4/21/18 TASK 0 STEP 0: make a source and sink even if there isn't one 
            source = deprecatedFlowGraph.source;
            sink = deprecatedFlowGraph.sink;
            originalNumEdges = deprecatedFlowGraph.originalNumEdges;
        }
        private int[][] flowGraphToArray(DeprecatedFlowGraph deprecatedFlowGraph){
            int numVertices = deprecatedFlowGraph.graph.size();
            int[][] rtrn = new int[numVertices][numVertices];
            for(int i = 0; i< deprecatedFlowGraph.size(); i++){
                List<Integer> node = deprecatedFlowGraph.getNode(i);
                for(int edgeNum:node){
                    if(edgeNum%2!=0){
                        continue;
                    }
                    Edge edge = deprecatedFlowGraph.getEdge(edgeNum);
                    rtrn[i][edge.getTo()]=edge.getCapacity();
                }
            }
            return rtrn;
        }

        private int[][] createMinCapacity(DeprecatedFlowGraph deprecatedFlowGraph){
            int[][] minCapacity = new int[numVertices][numVertices];
            Iterator<Edge> edgeIterator = deprecatedFlowGraph.getEdgeIteratorForGraph();
            while (edgeIterator.hasNext()){
                Edge e = edgeIterator.next();
                minCapacity[e.getFrom()][e.getTo()] = e.getMinCapacity();
            }
            return minCapacity;
        }
        private int[][] createReverseGraph(int[][] mainGraph){

            int[][] rtrn = new int[numVertices][numVertices];
            for(int u=0;u<numVertices;u++){
                for(int v = 0; v< numVertices;v++){
                    rtrn[v][u] = mainGraph[u][v];
                }
            }
            return rtrn;
        }
        private int[][] createEdges(DeprecatedFlowGraph deprecatedFlowGraph, int numEdges){
            int[][] rtrn = new int[numEdges][2];
            Iterator<Edge> edgeIterator = deprecatedFlowGraph.getEdgeIteratorForGraph();
            int edgeNumber = 0;
            while (edgeIterator.hasNext()){
                Edge e = edgeIterator.next();
                if(e.getIndex()%2!=0){
                    continue;
                }
                rtrn[edgeNumber][0] = e.getFrom();
                rtrn[edgeNumber][1] = e.getTo();
                edgeNumber++;
            }
            return rtrn;
        }

        public int getNumVertices() {
            return numVertices;
        }

        public int getCapacity(int from, int to){
            return mainGraph[from][to];
        }

        public int getMinCapacity(int from, int to){
            return minCapacity[from][to];
        }

        public int getReverseGraphCapacity(int from, int to){
            return reverseGraph[from][to];
        }

        public void setReverseGraphCapacity(int from, int to, int val){
            reverseGraph[from][to] = val;
        }

        public void changeReverseCapacity(int from, int to, int val){
            reverseGraph[from][to] += val;
        }

        public int[][] getEdges() {
            return edges;
        }

        public int[][] getFlow() {
            return flow;
        }

        public int getFlow(int from, int to) {
            return flow[from][to];
        }

        public void setFlow(int from, int to, int amount){
            flow[from][to] = amount;
        }

        public void changeFlow(int from, int to, int amount){
            flow[from][to] += amount;
        }

        public int getNumEdges() {
            return numEdges;
        }

        public int getOriginalNumEdges() {
            return originalNumEdges;
        }

        // TODO: 4/21/18 TASK 0 STEP 1: see if input and output of edges equals demand
        /**
         * determine if graph has a circuit.
         * will determine the YES/NO output
         * @return true if graph has a circuit, false if not
         */
        public boolean isFullCircuit(){
            return false;
        }
    }

    private FlowGraph findMaxFlow(FlowGraph graph){
        int u;
        int v;
        int s = graph.source;
        int t = graph.sink;
        int numVertices = graph.getNumVertices();


        int parent[] = new int[numVertices];

        int maxFlow = 0;

        while (breadthFirstSearch(graph,s,t,parent)){
            int pathFlow = Integer.MAX_VALUE;

            v = t;
            while(v!=s){
                u=parent[v];
                pathFlow = Math.min(pathFlow, graph.getReverseGraphCapacity(u,v));
                v=u;
            }

            v = t;
            while(v!=s){
                u = parent[v];

                graph.changeReverseCapacity(u,v,-pathFlow);
                graph.changeReverseCapacity(v,u,pathFlow);
                graph.changeFlow(v,u,pathFlow);
                v = u;
            }

            maxFlow += pathFlow;
        }


        return graph;

    }

    private boolean breadthFirstSearch(FlowGraph graph, int s, int t, int[] parent){
        int numVertices = graph.getNumVertices();
        boolean visited[] = new boolean[numVertices];
        Arrays.fill(visited,false);

        Deque<Integer> queue = new ArrayDeque<>();

        queue.add(s);
        // FIXME: 4/21/18 TASK 1 STEP 0: bug is here; when source and sink aren't declared I'm calling visited[-1]
        visited[s] = true;
        parent[s] = -1;


        while(!queue.isEmpty()) {
            int u = queue.poll();

            for(int v=0;v<numVertices;v++){
                if(!visited[v] && graph.getReverseGraphCapacity(u,v)>0){
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
