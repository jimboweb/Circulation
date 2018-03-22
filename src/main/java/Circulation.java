import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;

public class Circulation {
    // TODO: 3/21/18 build the modified graph with no minimums 
    
    FastScanner scanner;

    public Circulation(){
        scanner = new FastScanner();
    }

    public void findCirculation(Inputter input, Outputter output){
        FlowGraph graph = buildGraph(input);
        FlowPath fp = findPathBfs(graph,0, 0);
        while(!fp.isEmpty()){
            System.out.print(fp.pop() + " ");
        }
        System.out.println();
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
    }

    static class FlowGraph {
        /* List of all - forward and backward - edges */
        private List<Edge> edges;

        /* These adjacency lists store only indices of edges from the edges list */
        private List<Integer>[] graph;

        public FlowGraph(int n) {
            this.graph = (ArrayList<Integer>[])new ArrayList[n];
            for (int i = 0; i < n; ++i)
                this.graph[i] = new ArrayList<>();
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
            graph[from].add(edges.size());
            edges.add(forwardEdge);
            graph[to].add(edges.size());
            edges.add(backwardEdge);
        }

        public int size() {
            return graph.length;
        }

        public List<Integer> getIds(int from) {
            return graph[from];
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
            return graph[node].iterator();
        }
    }

    static class FlowPath{
        Deque<Integer> edges;
        int flow = 0;
        public FlowPath(){
            edges = new ArrayDeque<>();
        }
        public FlowPath(Deque<Integer> edges, int flow){
            this.edges=edges;
            this.flow=flow;
        }
        public void push(int edge){
            edges.push(edge);
        }
        public int pop(){
            return edges.pop();
        }

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
