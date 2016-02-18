#ifndef SLAM3D_GRAPHANALYSISMAPPER_HPP
#define SLAM3D_GRAPHANALYSISMAPPER_HPP

#include "GraphMapper.hpp"

#include <graph_analysis/BaseGraph.hpp>
#include <flann/flann.hpp>
#include <map>

namespace graph_analysis
{
	class VertexObject : public slam3d::VertexObject, public Vertex
	{
	public:
		typedef boost::shared_ptr<VertexObject> Ptr;
		typedef boost::shared_ptr<const VertexObject> ConstPtr;

		VertexObject(const std::string& l = ""):graph_analysis::Vertex(l)
		{
			label = l;
		}

		std::string getClassName() const { return "graph_analysis::VertexObject"; }
	};

	class EdgeObject : public slam3d::EdgeObject, public Edge
	{
	public:
		typedef boost::shared_ptr<EdgeObject> Ptr;
		typedef boost::shared_ptr<const EdgeObject> ConstPtr;

		EdgeObject(const std::string& s, const std::string& l = ""):graph_analysis::Edge(l)
		{
			label = l;
			sensor = s;
		}

		std::string getClassName() const { return "graph_analysis::EdgeObject"; }
	};
	
	typedef std::vector<VertexObject::Ptr> VertexList;
	typedef std::vector<EdgeObject::Ptr> EdgeList;
	typedef std::map<boost::uuids::uuid, VertexObject::Ptr> VertexIndex;
	
	typedef flann::Index< flann::L2<float> > NeighborIndex;
	
	/**
	 * @class BadElementType
	 * @brief Exception thrown when element types do not match.
	 * @details This happens when an element returned from 
	 * graph_analysis::BaseGraph is not of the expected slam type,
	 * e.g. VertexObject or EdgeObject.
	 */
	class BadElementType: public std::exception
	{
	public:
		BadElementType(){}
		virtual const char* what() const throw()
		{
			return "Could not convert from base-type to slam-type!";
		}
	};
	
	/**
	 * @class Mapper
	 * @brief Implementation of GraphMapper using graph_analysis library.
	 */
	class Mapper : public slam3d::GraphMapper
	{
	public:
		Mapper(slam3d::Logger* logger);
		~Mapper();
	
		/**
		 * @brief Add a new measurement to the graph.
		 * @details The sensor specified in the measurement has to be registered
		 * with the mapper before. If the change in robot pose since the last
		 * added scan is smaller then min-translation or min-rotation, the
		 * measurement will not be added. Use GraphMapper::setMinPoseDistance to
		 * adjust this distance.
		 * @param m pointer to a new measurement
		 * @return true if the measurement was added
		 */
		virtual bool addReading(slam3d::Measurement* m);
		
		/**
		 * @brief Add a new measurement from another robot.
		 * @param m pointer to a new measurement
		 * @param t pose in map coordinates
		 */
		virtual void addExternalReading(slam3d::Measurement* m, const slam3d::Transform& t);
		
		/**
		 * @brief Start the backend optimization process.
		 * @details Requires that a Solver has been set with setSolver.
		 * @return true if optimization was successful
		 */
		virtual bool optimize();

		/**
		 * @brief Get the last vertex, that was locally added to the graph.
		 * @details This will not return external vertices from other robots.
		 * @return last added vertex
		 */
		virtual const slam3d::VertexObject& getLastVertex();

		/**
		 * @brief Write the current graph to a file (currently dot).
		 * @details For larger graphs, this can take a very long time.
		 * @param name filename without type ending
		 */
		virtual void writeGraphToFile(const std::string &name);

		/**
		 * @brief Gets a vertex object by its given id. The id is given to each
		 * vertex upon creation and then never changed. These id's are local and
		 * cannot be compared between different agents in a distributed setup.
		 * @param id identifier for a vertex
		 */
		virtual const slam3d::VertexObject& getVertex(slam3d::IdType id);

		/**
		 * @brief Gets a list of all vertices from given sensor.
		 * @param sensor
		 */
		virtual slam3d::VertexObjectList getVertexObjectsFromSensor(const std::string& sensor);

		/**
		 * @brief Gets a list of all edges from given sensor.
		 * @param sensor
		 */
		virtual slam3d::EdgeObjectList getEdgeObjectsFromSensor(const std::string& sensor);
		
	private:
		VertexObject::Ptr addVertex(slam3d::Measurement* m,
		                            const slam3d::Transform &corrected);

		EdgeObject::Ptr addEdge(VertexObject::Ptr source,
		                        VertexObject::Ptr target,
		                        const slam3d::Transform &t,
		                        const slam3d::Covariance &c,
		                        const std::string &sensor,
		                        const std::string &label);

		/**
		 * @brief Gets a list with all vertices from a given sensor.
		 * @param sensor name of the sensor which vertices are requested
		 * @return list of all vertices from given sensor
		 */
		VertexList getVerticesFromSensor(const std::string& sensor);

		/**
		 * @brief Get a list with all edges from a given sensor.
		 * @param sensor name of the sensor which edges are requested
		 * @return list of all edges from given sensor
		 */
		EdgeList getEdgesFromSensor(const std::string& sensor);

		/**
		 * @brief Create the index for nearest neighbor search of nodes.
		 * @param sensor index nodes of this sensor
		 */
		void buildNeighborIndex(const std::string& sensor);

		/**
		 * @brief Search for nodes in the graph near the given pose.
		 * @details This does not refer to a NN-Search in the graph, but to search for
		 * spatially near poses according to their current corrected pose.
		 * If new nodes have been added, the index has to be created with
		 * a call to buildNeighborIndex.
		 * @param tf The pose where to search for nodes
		 * @param radius The radius within nodes should be returned
		 * @return list of spatially near vertices
		 */
		VertexList getNearbyVertices(const slam3d::Transform &tf, float radius);

		void linkToNeighbors(VertexObject::Ptr vertex, slam3d::Sensor* sensor, int max_links);

		static VertexObject::Ptr fromBaseGraph(Vertex::Ptr base);
		static EdgeObject::Ptr fromBaseGraph(Edge::Ptr base);

	private:
		BaseGraph::Ptr mPoseGraph;
		VertexObject::Ptr mLastVertex;
		VertexObject::Ptr mFirstVertex;

		// Index to use nearest neighbor search
		flann::SearchParams mSearchParams;
		NeighborIndex mIndex;
		std::map<int, VertexObject::Ptr> mIndexMap;

		// Index to find Vertices by their unique id
		VertexIndex mVertexIndex;
	};
}

#endif