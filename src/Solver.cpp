#include "Solver.hpp"

using namespace slam;

void Solver::optimize(graph_analysis::BaseGraph::Ptr baseGraph)
{
	using namespace graph_analysis;

        setBaseGraph(baseGraph);

        VertexIterator::Ptr vertexIterator = baseGraph->getVertexIterator();
        VertexObject::Ptr first;
        while(vertexIterator->next())
        {
                VertexObject::Ptr vertex = boost::dynamic_pointer_cast<VertexObject>(vertexIterator->current());
		if(!first)
                {
                    first = vertex;
                }
		addNode(vertex);
        }

	// Fix first node in the graph
	setFixed(first);

        EdgeIterator::Ptr edgeIterator = baseGraph->getEdgeIterator();
        while(edgeIterator->next())
        {
                EdgeObject::Ptr edge = boost::dynamic_pointer_cast<EdgeObject>(edgeIterator->current());
		addConstraint(edge);
        }
	
	// Optimize
	compute();

	// Retrieve results
	IdPoseVector res = getCorrections();
	for(IdPoseVector::iterator it = res.begin(); it < res.end(); it++)
	{
		unsigned int id = it->first;
		Transform tf = it->second;

                VertexObject::Ptr vertex = boost::dynamic_pointer_cast<VertexObject>( mBaseGraph->getVertex(id) );
                vertex->corrected_pose = tf;
	}
}
