#include "DataStorage.hpp"

#include <boost/format.hpp>
#include <pqxx/pqxx>
//#include <libpq-fe.h>

using namespace slam3d;

class slam3d::DataStorageInternal
{
public:
	DataStorageInternal(Logger* l, const std::string& sql_connect) : mLogger(l), mConnection(sql_connect){}
	
	void createSchema(const std::string& schema)
	{
		pqxx::work w(mConnection);
		w.exec0("CREATE SCHEMA IF NOT EXISTS " + schema);
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
	mInternal->createSchema(schema);
}

DataStorage::~DataStorage()
{
	delete mInternal;
}
