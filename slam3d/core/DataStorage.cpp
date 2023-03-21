#include "DataStorage.hpp"
#include "Types.hpp"

#include <boost/format.hpp>
#include <pqxx/pqxx>
//#include <libpq-fe.h>

using namespace slam3d;

class slam3d::DataStorageInternal
{
public:
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
		query += "position geometry, ";
		query += "qx       double precision, ";
		query += "qy       double precision, ";
		query += "qz       double precision, ";
		query += "qw       double precision, ";
		query += "data     bytea)";
		w.exec0(query);
		w.commit();
	}
	
	void writeMeasurement(Measurement* m)
	{
		pqxx::work w(mConnection);
		boost::format ins("INSERT INTO measurement VALUES (%1%, 'epoch' interval + %2%, %3%, %4%, %5%, %6%, %7%, %8%, %9%, %10%);");
		pqxx::exec0((ins
			% m->getUniqueId()
			% m->getTimestamp().tv_sec % m->getTimestamp().tv_usec
			% ))
		w.commit();
	}
	
protected:
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
