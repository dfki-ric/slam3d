#include "DataStorage.hpp"
#include "Types.hpp"

#include <boost/format.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <pqxx/pqxx>
//#include <libpq-fe.h>

using namespace slam3d;

struct slam3d::DataStorageInternal
{
	DataStorageInternal(Logger* l, const std::string& sql_connect) : mLogger(l), mConnection(sql_connect){}
	
	void initialize(const std::string& schema)
	{
		pqxx::work w(mConnection);
		w.exec0("CREATE SCHEMA IF NOT EXISTS " + schema);
		w.exec0("SET search_path TO " + schema + ",public");

		std::string query("CREATE TABLE IF NOT EXISTS measurement(");
		query += "id       uuid PRIMARY KEY, ";
		query += "time     timestamp, ",
		query += "robot    text, ";
		query += "sensor   text, ";
		query += "data     bytea)";
		w.exec0(query);
		w.commit();
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
	char data[3];
	data[0] = 'a';
	data[1] = 'b';
	data[2] = 'c';
	boost::format ins("INSERT INTO measurement VALUES ('%s', TIMESTAMP 'epoch' + %u.%06u * interval '1 second', '%s', '%s', '%s');");
	
	w.exec0((ins
		% m->getUniqueId()
		% m->getTimestamp().tv_sec % m->getTimestamp().tv_usec
		% m->getRobotName()
		% m->getSensorName()
		% pqxx::to_string(data)).str());
	w.commit();
}
