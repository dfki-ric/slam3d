#ifndef SLAM_LOGGER_HPP
#define SLAM_LOGGER_HPP

#include <sys/time.h>

namespace slam
{
	enum LOG_LEVEL{DEBUG, INFO, WARNING, ERROR, FATAL};
	
	class Logger
	{
	public:
		Logger():mLogLevel(INFO){}
		~Logger(){}
		
		virtual void setLogLevel(LOG_LEVEL lvl){mLogLevel = lvl;}
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