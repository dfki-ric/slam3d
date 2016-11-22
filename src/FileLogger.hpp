#ifndef SLAM_FILELOGGER_HPP
#define SLAM_FILELOGGER_HPP

#include "Logger.hpp"
#include <fstream>

namespace slam3d
{
	/**
	 * @class FileLogger
	 * @brief A basic logger that prints messages to a log file.
	 */
	class FileLogger : public Logger
	{
	public:
		/**
		 * @brief Constructor for a FileLogger.
		 * @param c clock to get timestamps for messages
		 * @param f filename for the loggers log-file
		 */
		FileLogger(Clock c, std::string f) : Logger(c)
		{
			mLogFile.open(f.c_str());
		}
		
		~FileLogger()
		{
			mLogFile.close();
		}
		
		/**
		 * @brief Writes a message, showing the log-level and a timestamp.
		 * @param lvl the message's log-level
		 * @param message the message to be written
		 */
		virtual void message(LOG_LEVEL lvl, const std::string& message)
		{
			if(lvl < mLogLevel)
				return;
				
			timeval tp = mClock.now();

			switch(lvl)
			{
			case DEBUG:
				mLogFile << "[DEBUG][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case INFO:
				mLogFile << "[INFO ][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case WARNING:
				mLogFile << "[WARN ][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case ERROR:
				mLogFile << "[ERROR][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case FATAL:
				mLogFile << "[FATAL][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;		
			}
		}

	private:
		std::ofstream mLogFile;
	};
}

#endif