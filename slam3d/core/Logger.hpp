// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <slam3d/core/Clock.hpp>

#include <iostream>
#include <iomanip>
#include <boost/thread/shared_mutex.hpp>

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

			boost::unique_lock<boost::mutex> guard(mLogMutex);
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
		boost::mutex mLogMutex;
	};
}
