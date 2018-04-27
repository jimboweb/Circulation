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

    @Test
    public void testFindCirculation() {
        Circulation instance = new Circulation();

    }

    class TestInput implements Circulation.Inputter{
        Random rnd;
        int maxEdges = 20;
        int maxMinFlow = 10;
        int maxMaxFlowOverMin = 10;

        private int n;

        public TestInput(){
            n = rnd.nextInt(maxEdges);
            int m = 0;

        }

        private Circulation.FlowGraph makeFlowGraph(int n, int m){
            Circulation.FlowGraph rtrn;
            while(m<n){
                m=rnd.nextInt(triangular(n));
            }
            int[][] edgeMinFlow = new int[n][n];
            int[][] edgeMaxFlow = new int[n][n];
            int[][] edgeNums = new int[n][n];
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
                edgeNums[start][end] = edgeNum;
                edgeNum++;
            }
            rtrn = new Circulation.FlowGraph(n, m, edgeMinFlow, edgeMaxFlow, edgeNums);
            return rtrn;
        }

        @Override
        public int getN() {
            return n;
        }

        @Override
        public int[][] getData() {
            return new int[0][];
        }

        private static int triangular(int n){
            int tri = 0;
            for(int i=1; i<n; i++){
                tri = tri + i;
            }
            return tri;
        }

    }
}