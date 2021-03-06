import org.junit.Test;
import java.util.Random;


import static org.junit.Assert.*;


/*
    Algorithm to generate random min/max flow graphs

    make a random number of nodes
    4 out of 5 times
        make a single circuit with random min and max flows
    make sure each node has at least one edge coming out
    add a random number of other max and min flow edges
    return as input
 */

public class CirculationTest {
    Circulation instance = new Circulation();
    NaiveCirculation naiveCirculation = new NaiveCirculation();
    Random rnd;
    int maxEdges = 5;
    int maxMinFlow = 3;
    int maxMaxFlowOverMin = 3;

    public CirculationTest() {
        rnd = new Random();
    }


    @Test
    public void testNaiveCirculation(){
        int n = 3;
        int m = 3;
        int[][] minCapacity = {{0,1,0},{0,0,2},{1,0,0}};
        int[][] mainGraph = {{0,2,0},{0,0,4},{2,0,0}};
        int[][] edges = {{0,1},{1,2},{2,1}};
        Circulation.FlowGraph graph = new Circulation.FlowGraph(n,m,minCapacity,mainGraph,edges);
        NaiveCirculation naiveCirculation = new NaiveCirculation();
        int[] correctCirculation = {2,2,2};
        int[] circulation = naiveCirculation.findCirculation(graph);
        assertArrayEquals(circulation,correctCirculation);
    }

    @Test
    public void testFindCirculation() {

        int n = rnd.nextInt(maxEdges)+3;
        int m = 0;
        Circulation.FlowGraph testGraph = makeFlowGraph(n, m);
        int[] correctFlows = naiveCirculation.findCirculation(testGraph);
        Circulation.FlowGraph returnedGraph = instance.findMaxFlow(testGraph);
        if(correctFlows.length==0){
            assertFalse(returnedGraph.isFullCircuit());
        } else {
            int[][] returnedFlowGraph = returnedGraph.getFlow();
            int[][] edgeNumbers = returnedGraph.getEdges();
            int[] returnedFlows = new int[n];
            for(int i=0;i<edgeNumbers.length;i++){
                int[] edge = edgeNumbers[i];
                int from = edge[0];
                int to = edge[1];
                returnedFlows[i] = returnedFlowGraph[from][to];
            }
            assertEquals(correctFlows,returnedFlows);
        }
    }



    private Circulation.FlowGraph makeFlowGraph(int n, int m){
        Circulation.FlowGraph rtrn;

        int maxEdges = triangular(n);
        int diff = maxEdges-n;
        m=rnd.nextInt(diff)+n;

        int[][] edgeMinFlow = new int[n][n];
        int[][] edgeMaxFlow = new int[n][n];
        int[][] edgeNums = new int[m][2];
        int start = -1;
        int end = -1;
        int edgeNum = 0;
        for(int i=0;i<m;i++){
            int newEdgeValue = Integer.MAX_VALUE;
            while (newEdgeValue > 0){
                start = rnd.nextInt(n);
                end = start;
                while(end==start){
                    end = rnd.nextInt(n);
                }
                newEdgeValue = edgeMinFlow[start][end];
            }
            int minFlow = rnd.nextInt(maxMinFlow)+1;
            int maxFlow = minFlow + rnd.nextInt(maxMaxFlowOverMin);
            edgeMinFlow[start][end] = minFlow;
            edgeMaxFlow[start][end] = maxFlow;
            edgeNums[edgeNum][0] = start;
            edgeNums[edgeNum][1] = end;
            edgeNum++;
        }
        rtrn = new Circulation.FlowGraph(n, m, edgeMinFlow, edgeMaxFlow, edgeNums);
        return rtrn;
    }

    private static int triangular(int n){
        int tri = 0;
        for(int i=1; i<n; i++){
            tri = tri + i;
        }
        return tri;
    }

    }

