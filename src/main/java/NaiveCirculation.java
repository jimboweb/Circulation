/*
    Algorithm for naive circulation:

    choose a random node to begin
    for each edge from random node
        edge.flow = edge.minflow
    for each edge with flow
        edge.end demand -= edge.flow
    for each node with -demand
        if outgoingEdges empty
            fail
        for each outgoingEdge from node
            if node.-demand < minFlow
                call method to request flow back to beginning and fail if it goes over max flow or has no previous edges
            outgoingEdge.flow = outgoingEdge.minFlow
            node += outgoingEdge.minFlow
        if node.demand != 0 fail
    for each edge
        if minFlow not met
            call method to request demand from earlier nodes and fail if goes over max or has no previous edges
    return success and flow for each edge

 */


import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;


public class NaiveCirculation {
    public int[] findCirculation(Circulation.FlowGraph graph){
        int numEdges = graph.getNumEdges();
        int[] flows = new int[numEdges];
        int[][] edges = graph.getEdges();
        int[][] reverseEdgeLookup = getReverseEdgeLookup(graph, edges);
        List<List<Integer>> prevEdges = getPrevEdges(numEdges);
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
            if(!setNextEdges(flows, reverseEdgeLookup[to], prevEdges.get(currentEdgeNum), minCapacities[to],  flow)){
                return new int[0];
            }
            currentEdgeNum=(currentEdgeNum+1)%numEdges;
        }
        return flows;
    }

    private boolean setNextEdges(int[] flows, int[] ints, List<Integer> integers, int[] minCapacity, int flow) {
        for(int i = 0; i< minCapacity.length; i++){
            int demand = minCapacity[i];
            int nextEdge = ints[i];
            flow = checkAndUpdateFlow(flow, demand, nextEdge);
            if(flow<0){
                    return false;
            }
            flows[nextEdge] = demand;
            integers.add(nextEdge);

        }
        return true;
    }

    private int checkAndUpdateFlow(int flow, int demand, int nextEdge) {
        if(flow<demand){
            int newFlow = flow+demand;
            if(increaseFlowBackwards(nextEdge,newFlow)){
                flow=newFlow;
            } else {
                return new -1; //fail if previous min cpacities don't allow flow
            }
        }
        return flow;
    }

    private List<List<Integer>> getPrevEdges(int numEdges) {
        List<List<Integer>> prevEdges = new ArrayList<>();
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

    private boolean increaseFlowBackwards(int edgeNum, int newFlow){
        // TODO: 5/1/18 cycle back through previous edges to get enough flow. will probably have to be recursive
        // return true at 0, return false if you can't get the flow
        return false;
    }
}
