#include "DataStorage.hpp"
#include "Types.hpp"

#include <boost/format.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <pqxx/pqxx>
//#include <libpq-fe.h>

using namespace slam3d;

struct slam3d::DataStorageInternal
{
	DataStorageInternal(Logger* l, const std::string& sql_connect) : mLogger(l), mConnection(sql_connect){}
	
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
		query += "id              uuid PRIMARY KEY, ";
		query += "time            timestamp, ",
		query += "robot_name      text, ";
		query += "sensor_name     text, ";
		query += "sensor_position pos, ";
		query += "sensor_rotation quat, ";
		query += "data            bytea)";
		w2.exec0(query);
		w2.commit();
	}

	Logger* mLogger;
	pqxx::connection mConnection;
};

DataStorage::DataStorage(Logger* l, const std::string& host, const std::string& schema)
: mHost(host), mSchema(schema)
{
	boost::format conn("postgresql://slam3d:dfki-slam@%1%");
	mInternal = new DataStorageInternal(l, (conn % mHost).str());
	mInternal->initialize(schema);
}

DataStorage::~DataStorage()
{
	delete mInternal;
}

void DataStorage::writeMeasurement(Measurement::Ptr m)
{
	pqxx::work w(mInternal->mConnection);
	std::stringstream data;
	m->toStream(data);

	Position pos = m->getSensorPose().translation();
	Quaternion quat(m->getSensorPose().linear());

	boost::format ins("INSERT INTO measurement VALUES ('%s', TIMESTAMP 'epoch' + %u.%06u * INTERVAL '1 second', '%s', '%s', ('%d', '%d', '%d'), ('%d', '%d', '%d', '%d'), '%s');");
	ins	% m->getUniqueId()
		% m->getTimestamp().tv_sec % m->getTimestamp().tv_usec
		% m->getRobotName()
		% m->getSensorName()
		% pos.x() % pos.y() % pos.z()
		% quat.x() % quat.y() % quat.z() % quat.w()
		% pqxx::to_string(data);

	w.exec0(ins.str());
	w.commit();
}

void DataStorage::readMeasurement()
{
	pqxx::work w(mInternal->mConnection);
	pqxx::result res{w.exec("SELECT id, EXTRACT(EPOCH FROM time), robot_name, sensor_name, sensor_position, sensor_rotation, data FROM measurement;")};

	for (auto row : res)
	{
		boost::uuids::string_generator gen;
		boost::uuids::uuid id = gen(row[0].c_str());
		double tx, ty, tz, qx, qy, qz, qw;
		row[4].composite_to(tx,ty,tz);
		row[5].composite_to(qx, qy, qz, qw);
		Transform pose(Quaternion(qw, qx, qy, qz));
		pose.translation() = Position(tx, ty, tz);

		Measurement::Ptr m(new Measurement(row[2].c_str(), row[3].c_str(), pose, id));
	}
	
	w.commit();
}
