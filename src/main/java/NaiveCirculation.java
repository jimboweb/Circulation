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


import java.util.BitSet;

public class NaiveCirculation {
    public int[] findCirculation(Circulation.FlowGraph graph){
        int numEdges = graph.getNumEdges();
        int[] flows = new int[numEdges];
        int[][] edges = graph.getEdges();
        int[][] reverseEdgeLookup = new int[graph.getNumVertices()][graph.getNumVertices()];
        for(int i=0;i<edges.length;i++){
            int from = edges[i][0];
            int to = edges[i][1];
            reverseEdgeLookup[from][to] = i;
        }
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
            for(int i=0;i<minCapacities[to].length;i++){
                int demand = minCapacities[to][i];
                int nextEdge = reverseEdgeLookup[to][i];
                // TODO: 4/28/18 set the flow of the next edge to demand
                // subtract demand from flow
                // if flow runs out increase flow of current edge
                // if flow of current edge is over maxCapacity fail
                // call increaseFlowBackwards to current flow
            }
        }
        return new int[0];
    }

    private boolean increaseFlowBackwards(int edgeNum, int newFlow){

    }
}
