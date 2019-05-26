#ifndef CVL_LOG__H_
#define CVL_LOG__H_

#include <iostream>
#include <fstream>
#include <cstring>
#include <ostream>
#include <sstream>
#include <ctime>
#include <sys/timeb.h>

#include "LogColors.h"

enum TLogLevel {logERROR = 0, logWARNING = 1, logINFO = 2, logDEBUG = 3, logDEBUG1 = 4, logDEBUG2 = 5, logDEBUG3 = 6, logDEBUG4 = 7};
static const std::string TLogLevelStrings[] = {"ERROR", "WARNING", "INFO",  "DEBUG", "DEBUG1", "DEBUG2", "DEBUG3", "DEBUG4"};
static const std::string TLogLevelColors[]  = {LOG_COL_RED, LOG_COL_BROWN, LOG_COL_GREEN, LOG_COL_CYAN, LOG_COL_CYAN, LOG_COL_CYAN, LOG_COL_CYAN, LOG_COL_CYAN};

#ifndef LogError
#define LogError   Log().Get(logERROR)
#endif
#ifndef LogWarning
#define LogWarning Log().Get(logWARNING)
#endif
#ifndef LogInfo
#define LogInfo    Log().Get(logINFO)
#endif
#ifndef LogDebug
#define LogDebug   Log().Get(logDEBUG)
#endif


/// \addtogroup IO_Module
/* @{ */

/// \defgroup LogTrace_Module LogTrace
/// \brief 
/// \ingroup LogTrace

///  \addtogroup LogTrace_Module
/* @{ */


// class vtostringstream:
// public std::ostringstream
// {
// 
// public:
// 	vtostringstream &operator<<(std::ostream& (*pf) (std::ostream&))
// 	{
// 		//std::cout << "got endl";
// 		// here we intercept std::endl and do nothing
// 		return *this;
// 	}
// };

///	\class Log
///	\author Luigi Freda
///	\brief A class implementing a standard log with different reporting levels
///	\note It is not capable of intercepting std::endl. So do not use it at the end of the row.
/// 	\todo 
///	\date
///	\warning
class Log
{
public:
	Log():_bWithTime(false),_messageLevel(logDEBUG){};

	inline virtual ~Log();

	inline std::ostringstream& Get(TLogLevel level = logERROR);
	inline std::ostringstream& Get(int level);
	/// report with time
	inline std::ostringstream& GetWT(int level);

public:
	static const int maxReportingLevel = 7;
	inline static TLogLevel& ReportingLevel() {return _reportingLevel;} 
	static TLogLevel _reportingLevel;

protected:

	std::ostringstream _os;
	//vtostringstream _os;

private:

	Log(const Log&);
	Log& operator =(const Log&);

private:

	bool	  _bWithTime;
	TLogLevel _messageLevel;
};

//TLogLevel Log::_reportingLevel = logERROR; /// defined in Log.cpp

std::ostringstream& Log::Get(TLogLevel level)
{
	return Get((int)level);
}

std::ostringstream& Log::Get(int level)
{ 
	level = std::min(level,(int)Log::maxReportingLevel);
	_os << TLogLevelColors[level] << "[" << TLogLevelStrings[level] << "]: ";
	if(level > logDEBUG)
		_os << std::string( level - logDEBUG, '\t');
	//_os << LOG_COL_NORMAL;
	_messageLevel = (TLogLevel)level;
	return _os;
}

std::ostringstream& Log::GetWT(int level)
{
	_bWithTime = true;
	return Get(level);
}

Log::~Log()
{
	if (_messageLevel >= Log::ReportingLevel())
	{
		if(_bWithTime)
		{	 
			/// reporting time with milliseconds precision
			struct tm* plocalTime;  
			time_t now = time(NULL);
			timeb timebuffer;
			ftime( &timebuffer );
			plocalTime = localtime(&now);
			_os << " (" << plocalTime->tm_hour << ":" << plocalTime->tm_min << ":" << plocalTime->tm_sec << ":" << timebuffer.millitm << ")";
		}
		_os << LOG_COL_NORMAL;
		_os << std::endl;
		//fprintf(stderr, "%s", _os.str().c_str());
		//fflush(stderr);
		std::cout << _os.str();
		//std::cout << std::endl;
	}
}


#endif


