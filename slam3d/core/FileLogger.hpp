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

#include <slam3d/core/Logger.hpp>

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
