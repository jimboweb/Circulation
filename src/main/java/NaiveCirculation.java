import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;


public class NaiveCirculation {
    public int[] findCirculation(Circulation.FlowGraph graph){
        int numEdges = graph.getNumEdges();
        int[] flows = new int[numEdges];
        int[][] edges = graph.getEdges();
        int[][] reverseEdgeLookup = getReverseEdgeLookup(graph, edges);
        ArrayList<ArrayList<Integer>> prevEdges = getPrevEdges(numEdges);
        int[][] minCapacities = graph.getMinCapacity();
        BitSet edgesSatisfied = new BitSet(numEdges);
        int currentEdgeNum = 0;
        while(edgesSatisfied.cardinality()<numEdges) {
            int[] currentEdge = edges[currentEdgeNum];
            int from = currentEdge[0];
            int to = currentEdge[1];
            int flow = graph.getMinCapacity(from, to);
            edgesSatisfied.set(currentEdgeNum);
            flows[currentEdgeNum] = flow;
            if(!setNextEdges(flows, reverseEdgeLookup[to], prevEdges.get(currentEdgeNum), minCapacities[to],  flow, prevEdges, minCapacities, edges)){
                return new int[0];
            }
            currentEdgeNum=(currentEdgeNum+1)%numEdges;
        }
        return flows;
    }

    private boolean setNextEdges(int[] flows, int[] reversEdgeLookup, List<Integer> integers, int[] minCapacity, int flow, ArrayList<ArrayList<Integer>> prevEdges, int[][] minCapacities, int[][] edges) {
        for(int i = 0; i< minCapacity.length; i++){
            int demand = minCapacity[i];
            int nextEdge = reversEdgeLookup[i];
            flow = checkAndUpdateFlow(flow, demand, nextEdge,flows, prevEdges, minCapacities, edges);
            if(flow<0){
                    return false;
            }
            flows[nextEdge] = demand;
            integers.add(nextEdge);

        }
        return true;
    }

    private int checkAndUpdateFlow(int flow, int demand, int nextEdge, int[] flows, ArrayList<ArrayList<Integer>> prevEdges, int[][] minCapacities, int[][] edges) {
        if(flow<demand){
            int newFlow = flow+demand;
            int flowFromPrevEdges = getFlowFromPrevEdges(nextEdge, nextEdge,newFlow,flows, prevEdges, minCapacities, edges);
            // FIXME: 5/5/18 this is always coming out false. why?
            if(flowFromPrevEdges==newFlow){
                flow=newFlow;
            } else {
                return -1; //fail if previous min cpacities don't allow flow
            }
        }
        return flow;
    }

    private ArrayList<ArrayList<Integer>> getPrevEdges(int numEdges) {
        ArrayList<ArrayList<Integer>> prevEdges = new ArrayList<>();
        for(int i=0;i<numEdges;i++){
            prevEdges.add(new ArrayList<>());
        }
        return prevEdges;
    }

    private int[][] getReverseEdgeLookup(Circulation.FlowGraph graph, int[][] edges) {
        int[][] reverseEdgeLookup = new int[graph.getNumVertices()][graph.getNumVertices()];
        for(int i=0;i<edges.length;i++){
            int from = edges[i][0];
            int to = edges[i][1];
            reverseEdgeLookup[from][to] = i;
        }
        return reverseEdgeLookup;
    }

    private int getFlowFromPrevEdges(int firstEdge, int edgeNum, int newFlow, int[] flows, ArrayList<ArrayList<Integer>> prevEdges, int[][] minCapacities, int[][] edges){
        //omg what a mess I'll have to comment the hell out of this the ugly way to know what I was doing
        int thisEdgeFlow = flows[edgeNum];
        int from = edges[edgeNum][0];
        int to = edges[edgeNum][1];
        int thisEdgeMinCapacity = minCapacities[from][to];
        int availableFlow = thisEdgeMinCapacity-thisEdgeFlow;
        if(availableFlow==0){ //stop here if we're out of flow
            return 0;
        }
        int availFromPrevEdges = 0; //this will be the sum of all the flow we can get from the previous edges
        List<Integer> prevEdge = prevEdges.get(edgeNum);
        //for each previous edge
        // if edge has remaining flow
        //    add edge to queue
        //    add remaining flow to


//        for (int e : prevEdge) {
//            availFromPrevEdges += getFlowFromPrevEdges(firstEdge,edgeNum, newFlow, flows, prevEdges, minCapacities, edges);
//            if (availFromPrevEdges >= newFlow) {
//                availFromPrevEdges = newFlow;
//                break;
//            }
//        }
         newFlow=newFlow>availFromPrevEdges?newFlow:availFromPrevEdges;
        if(availableFlow<newFlow){
            flows[edgeNum]=thisEdgeMinCapacity;
            return availableFlow;
        } else {
            flows[edgeNum]+=newFlow;
            return newFlow;
        }
    }
}
