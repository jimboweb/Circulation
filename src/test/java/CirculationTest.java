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

        private int n;

        public TestInput(){
            n = rnd.nextInt(20);
            int[][] edges = new int[n][n];
            int m = Math.random(n) + n;
            for(int i=0;i<m;i++){
                
            }

        }

        @Override
        public int getN() {
            return n;
        }

        @Override
        public int[][] getData() {
            return new int[0][];
        }
    }
}