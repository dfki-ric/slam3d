#ifndef SLAM_GRAPHMAPPER_HPP
#define SLAM_GRAPHMAPPER_HPP

#include "Odometry.hpp"
#include "Sensor.hpp"

#include <graph_analysis/BaseGraph.hpp>
#include <flann/flann.hpp>
#include <map>

namespace slam
{
	/**
	 * @class VertexObject
	 * @author Sebastian Kasperski
	 * @date 03/16/15
	 * @file GraphMapper.hpp
	 * @brief Object attached to a vertex in the pose graph.
	 * Contains a pointer to an abstract measurement, which could
	 * be anything, e.g. a range scan, point cloud or image.
	 */
	class VertexObject : public graph_analysis::Vertex
	{
	public:
		typedef boost::shared_ptr<VertexObject> Ptr;

		VertexObject(const std::string& label = ""):graph_analysis::Vertex(label){}

		Transform odometric_pose;
		Transform corrected_pose;
		Measurement* measurement;

		std::string getClassName() const { return "slam::VertexObject"; }
	};

	/**
	 * @class EdgeObject
	 * @author Sebastian Kasperski
	 * @date 03/16/15
	 * @file GraphMapper.hpp
	 * @brief Object attached to an edge in the pose graph.
	 * Contains the relative transform from source to target.
	 */
	class EdgeObject : public graph_analysis::Edge
	{
	public:
		typedef boost::shared_ptr<EdgeObject> Ptr;

		EdgeObject(const std::string& label = ""):graph_analysis::Edge(label){}

		Transform transform;
		Covariance covariance;
		unsigned int edge_type;

		std::string getClassName() const { return "slam::EdgeObject"; }
	};

	typedef std::vector<VertexObject::Ptr> VertexList;
	typedef std::vector<EdgeObject::Ptr> EdgeList;
	typedef std::map<std::string, Sensor*> SensorList;
	
	typedef flann::Index< flann::L2<float> > NeighborIndex;
	
	/**
	 * @class BadElementType
	 * @author Sebastian Kasperski
	 * @date 02/07/15
	 * @file GraphMapper.hpp
	 * @brief Exception thrown when an element returned from 
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

	class Solver;

	/**
	 * @class GraphMapper
	 * @author Sebastian Kasperski
	 * @date 04/27/15
	 * @file GraphMapper.hpp
	 * @brief 
	 */
	class GraphMapper
	{
	public:
		GraphMapper(Logger* log);
		~GraphMapper();

		/**
		 * @brief Set a specific solver (one that is derived from slam::Solver)
		 * to the mapper, so it can be used as SLAM backend. The mapper can be
		 * used without a backend, but mapping results might be inconsistant.
		 * @param solver The SLAM backend to be used for optimization.
		 */
		void setSolver(Solver* solver);

		/**
		 * @brief Set an odometry module to provide initial poses when adding new
		 * measurements to the graph. Depending on the matching abilities of the
		 * used sensors (e.g. a 360° laser-scanner), the mapping might work
		 * correctly without an odometry module.
		 * @param odom The robot's odometry module.
		 */
		void setOdometry(Odometry* odom);

		/**
		 * @brief Register a sensor, so its data can be added to the graph.
		 * Multiple sensors can be used, but in this case an odometry module
		 * is required for the mapping to work correctly. Matching is currently
		 * done only between measurements of the same sensor.
		 * @param s The sensor to be registered for mapping.
		 */
		void registerSensor(Sensor* s);

		/**
		 * @brief Add a new measurement to the graph. The sensor specified in
		 * the measurement has to be registered with the mapper before.
		 * If the change in robot pose since the last added scan is smaller
		 * then min-translation or min-rotation, the measurement will not be
		 * added. Use GraphMapper::setMinPoseDistance to adjust this distance.
		 * @param m Pointer to a new measurement.
		 * @return True if the Measurement was added, otherwise False.
		 */
		bool addReading(Measurement* m);

		/**
		 * @brief Get the current pose of the robot within the generated map.
		 * The pose is updated at least whenever a new node is added.
		 * Depending on the available information, it might be updated
		 * more often. (e.g. when odometry is available)
		 */
		Transform getCurrentPose();
		
		/**
		 * @brief Start the backend optimization process. Requires that a solver
		 * has been set with GraphMapper::setSolver.
		 */
		bool optimize();
		
		/**
		 * @brief Get a list with all vertices from a given sensor.
		 * @param sensor Name of the sensor which vertices are requested.
		 */
		VertexList getVerticesFromSensor(const std::string& sensor);

		/**
		 * @brief Get a list with all edges, that have been created by the
		 * given sensor module.
		 * @param sensor Name of the sensor which edges are requested.
		 */
		EdgeList getEdgesFromSensor(const std::string& sensor);

		/**
		 * @brief Create the index for nearest neighbor search of nodes.
		 */
		void buildNeighborIndex();

		/**
		 * @brief Search for nodes in the graph near the given pose.
		 * This does not refer to a NN-Search in the graph, but to search for
		 * spatially near poses according to their current corrected pose.
		 * If new nodes have been added, the index has to be created with
		 * a call to GraphMapper::buildNeighborIndex.
		 * @param tf The pose where to search for nodes.
		 * @param radius The radius within nodes should be returned.
		 */
		VertexList getNearbyVertices(const Transform &tf, float radius);

		/**
		 * @brief Write the current graph to a file (currently dot).
		 * For larger graphs, this can take a very long time.
		 * @param name Filename without type ending.
		 */
		void writeGraphToFile(const std::string &name);

		/**
		 * @brief New nodes are matched against all nodes of the same sensor
		 * within the given radius.
		 * @param r Radius to create additional edges.
		 */
		void setNeighborRadius(float r){ mNeighborRadius = r; }

		/**
		 * @brief Set minimal change in pose between adjacent nodes.
		 * @param t Minimum translation between nodes (in meter).
		 * @param r Minimum rotation between nodes (in rad).
		 */
		void setMinPoseDistance(float t, float r){ mMinTranslation = t; mMinRotation = r; }

	private:
		VertexObject::Ptr addVertex(Measurement* m,
		                            const Transform &odometric,
		                            const Transform &corrected);

		EdgeObject::Ptr addEdge(VertexObject::Ptr source,
		                        VertexObject::Ptr target,
		                        const Transform &t,
		                        const Covariance &c,
		                        const std::string &name);

		bool checkMinDistance(const Transform &t);

		static VertexObject::Ptr fromBaseGraph(graph_analysis::Vertex::Ptr base);
		static EdgeObject::Ptr fromBaseGraph(graph_analysis::Edge::Ptr base);

	private:
		graph_analysis::BaseGraph::Ptr mPoseGraph;
		VertexObject::Ptr mLastVertex;
		VertexObject::Ptr mFixedVertex;

		Solver* mSolver;
		Logger* mLogger;
		Odometry* mOdometry;
		SensorList mSensors;

		Transform mCurrentPose;

		// Index to use nearest neighbor search
		flann::SearchParams mSearchParams;
		NeighborIndex mIndex;
		std::map<int, VertexObject::Ptr> mIndexMap;

		// Parameters
		float mNeighborRadius;
		float mMinTranslation;
		float mMinRotation;
	};
}

#endif
