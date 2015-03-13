#ifndef SLAM_LOGGER_HPP
#define SLAM_LOGGER_HPP

#include <sys/time.h>

namespace slam
{
	enum LOG_LEVEL{DEBUG, INFO, WARNING, ERROR, FATAL};
	
	/**
	 * @class Logger
	 * @author Sebastian Kasperski
	 * @date 03/13/15
	 * @file Logger.hpp
	 * @brief A basic logger that uses standard output to print messages.
	 */
	class Logger
	{
	public:
		/**
		 * @brief Default constructor, which sets the log-level to INFO.
		 */
		Logger():mLogLevel(INFO){}
		~Logger(){}
		
		/**
		 * @brief Set the log level. All messages with lower log-level will be ignored.
		 * @param lvl The new log-level
		 */
		virtual void setLogLevel(LOG_LEVEL lvl){mLogLevel = lvl;}
		
		/**
		 * @brief Prints a message, showing the log-level and a timestamp.
		 * @param lvl The message's log-level
		 * @param message The message to be displayed
		 */
		virtual void message(LOG_LEVEL lvl, std::string message)
		{
			if(lvl < mLogLevel)
				return;
				
			struct timeval tp;
			gettimeofday(&tp, NULL);

			switch(lvl)
			{
			case DEBUG:
				std::cout << "[DEBUG][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case INFO:
				std::cout << "[INFO][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case WARNING:
				std::cout << "[WARN][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case ERROR:
				std::cerr << "[ERROR][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;
			case FATAL:
				std::cerr << "[FATAL][" << tp.tv_sec << "." << tp.tv_usec << "] " << message << std::endl;
				break;		
			}
		}
		
	protected:
		LOG_LEVEL mLogLevel;
	};
}

#endif