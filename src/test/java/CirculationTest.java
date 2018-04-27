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
    int maxEdges = 20;
    int maxMinFlow = 10;
    int maxMaxFlowOverMin = 10;


    @Test
    public void testFindCirculation() {

        private int n;
        int n = rnd.nextInt(maxEdges);
        int m = 0;
        Circulation.FlowGraph testGraph = makeFlowGraph(n, m);
        int[] correctFlow = naiveCirculation.findCirculation(testGraph);
        Circulation.FlowGraph returnedGraph = instance.findMaxFlow(testGraph);
        if(correctFlow.length==0){
            assertFalse(returnedGraph.isFullCircuit());
        } else {
            int[][] returnedFlowGraph = returnedGraph.getFlow();
            int[][] edgeNumbers = returnedGraph.getEdges();
            

        }
    }



    private Circulation.FlowGraph makeFlowGraph(int n, int m){
        Circulation.FlowGraph rtrn;
        while(m<n){
            m=rnd.nextInt(triangular(n));
        }
        int[][] edgeMinFlow = new int[n][n];
        int[][] edgeMaxFlow = new int[n][n];
        int[][] edgeNums = new int[n][2];
        int newEdgeValue = Integer.MAX_VALUE;
        int start = -1;
        int end = -1;
        int edgeNum = 0;
        for(int i=0;i<m;i++){
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

