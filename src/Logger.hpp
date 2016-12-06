#ifndef SLAM_LOGGER_HPP
#define SLAM_LOGGER_HPP

#include "Clock.hpp"

#include <iostream>
#include <iomanip>

#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define USEC std::setw(6)<<std::left<<std::setfill('0')

namespace slam3d
{
	enum LOG_LEVEL{DEBUG, INFO, WARNING, ERROR, FATAL};
	
	/**
	 * @class Logger
	 * @brief A basic logger that uses standard output to print messages.
	 */
	class Logger
	{
	public:
		/**
		 * @brief Default constructor, sets log-level to INFO.
		 * @param c clock to get timestamps for messages
		 */
		Logger(Clock c) : mClock(c), mLogLevel(INFO){}
		virtual ~Logger(){}
		
		/**
		 * @brief Sets the log level.
		 * @details All messages with lower log-level will be ignored.
		 * @param lvl new log-level
		 */
		virtual void setLogLevel(LOG_LEVEL lvl){mLogLevel = lvl;}
		
		/**
		 * @brief Prints a message, showing log-level and timestamp.
		 * @param lvl the message's log-level
		 * @param message the message to be displayed
		 */
		virtual void message(LOG_LEVEL lvl, const std::string& message)
		{
			if(lvl < mLogLevel)
				return;
				
			timeval tp = mClock.now();

			switch(lvl)
			{
			case DEBUG:
				std::cout << KBLU << "[DEBUG][" << tp.tv_sec << "." << USEC << tp.tv_usec << "] " << message << RST << std::endl;
				break;
			case INFO:
				std::cout << KGRN << "[INFO ][" << tp.tv_sec << "." << USEC << tp.tv_usec << "] " << message << RST << std::endl;
				break;
			case WARNING:
				std::cout << KYEL << "[WARN ][" << tp.tv_sec << "." << USEC << tp.tv_usec << "] " << message << RST << std::endl;
				break;
			case ERROR:
				std::cerr << KRED << "[ERROR][" << tp.tv_sec << "." << USEC << tp.tv_usec << "] " << message << RST << std::endl;
				break;
			case FATAL:
				std::cerr << KRED << "[FATAL][" << tp.tv_sec << "." << USEC << tp.tv_usec << "] " << message << RST << std::endl;
				break;		
			}
		}
		
	protected:
		Clock mClock;
		LOG_LEVEL mLogLevel;
	};
}

#endif