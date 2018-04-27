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


public class NaiveCirculation {
    public int[] findCirculation(Circulation.FlowGraph graph){

        return new int[0];
    }
}
