#include "PostgreStorage.hpp"

#include <boost/format.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>

#include <pqxx/pqxx>

using namespace slam3d;

struct slam3d::PQXXInternal
{
	PQXXInternal(Logger* l, const std::string& sql_connect) : mLogger(l), mConnection(sql_connect){}
	
	void initialize(const std::string& schema)
	{
		pqxx::work w1(mConnection);

		// Check if a schema with the given name exists
		pqxx::result res = w1.exec("SELECT schema_owner FROM information_schema.schemata WHERE schema_name = '" + schema + "';");
		if(res.size() > 0)
		{
			if(strcmp(res[0][0].c_str(), "slam3d") == 0)
				return;
			else
				throw std::runtime_error("Schema '" + schema + "' already exists, but is owned by user '" + res[0][0].c_str() + "'!");
		}

		w1.exec0("CREATE SCHEMA IF NOT EXISTS " + schema);
		w1.exec0("SET search_path TO " + schema + ",public");

		std::string create_quat("CREATE TYPE quat AS (");
		create_quat += "x double precision, ";
		create_quat += "y double precision, ";
		create_quat += "z double precision, ";
		create_quat += "w double precision);";
		w1.exec0(create_quat);

		std::string create_pos("CREATE TYPE pos AS (");
		create_pos += "x double precision, ";
		create_pos += "y double precision, ";
		create_pos += "z double precision);";
		w1.exec0(create_pos);
		w1.commit();

		pqxx::work w2(mConnection);
		std::string query("CREATE TABLE IF NOT EXISTS measurement(");
		query += "id            uuid PRIMARY KEY, ";
		query += "time          timestamp, ",
		query += "robot_name    text, ";
		query += "sensor_name   text, ";
		query += "sensor_pose_t pos, ";
		query += "sensor_pose_r quat, ";
		query += "data          bytea)";
		w2.exec0(query);
		w2.commit();
	}

	Logger* mLogger;
	pqxx::connection mConnection;
};

PostgreStorage::PostgreStorage(Logger* l, const std::string& host, const std::string& schema)
: mLogger(l), mHost(host), mSchema(schema)
{
	boost::format conn("postgresql://slam3d:dfki-slam@%1%");
	mInternal = new PQXXInternal(l, (conn % mHost).str());
	mInternal->initialize(schema);
}

PostgreStorage::~PostgreStorage()
{
	delete mInternal;
}

void PostgreStorage::add(Measurement::Ptr measurement)
{
	MeasurementStorage::add(measurement);
	writeMeasurement(measurement);
}

Measurement::Ptr PostgreStorage::get(const boost::uuids::uuid& uuid)
{
	try
	{
		return MeasurementStorage::get(uuid);
	}catch(std::out_of_range &e)
	{
		Measurement::Ptr m = readMeasurement(uuid);
		MeasurementStorage::add(m);
		return m;
	}
}

void PostgreStorage::writeMeasurement(const Measurement::Ptr m)
{
	pqxx::work w(mInternal->mConnection);
	w.exec0("SET search_path TO " + mSchema + ",public");

	Position sensor_pos(m->getSensorPose().translation());
	Quaternion sensor_quat(m->getSensorPose().linear());

	std::stringstream ss;
	boost::archive::text_oarchive oa(ss);
	oa << m;

	boost::format ins("INSERT INTO measurement VALUES ("
		"'%s', "
		"TIMESTAMP 'epoch' + %u.%06u * INTERVAL '1 second', "
		"'%s', "
		"'%s', "
		"('%d', '%d', '%d'), "
		"('%d', '%d', '%d', '%d'), "
		"'%s');");
	ins	% m->getUniqueId()
		% m->getTimestamp().tv_sec % m->getTimestamp().tv_usec
		% m->getRobotName()
		% m->getSensorName()
		% sensor_pos.x() % sensor_pos.y() % sensor_pos.z()
		% sensor_quat.x() % sensor_quat.y() % sensor_quat.z() % sensor_quat.w()
		% pqxx::to_string(ss);

	w.exec0(ins.str());
	w.commit();
}

Measurement::Ptr PostgreStorage::readMeasurement(const boost::uuids::uuid& uuid)
{
	pqxx::work w(mInternal->mConnection);
	boost::format query("SELECT data FROM measurement WHERE id = '%s';");
	query % uuid;
	std::string data = w.query_value<std::string>(query.str());

	Measurement::Ptr m;
	std::stringstream ss(data);
	boost::archive::text_iarchive ia(ss);
	ia >> m;
	return m;
}
