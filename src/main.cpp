#include <stdio.h>
#include <signal.h>
#include <iostream>

#include "./helpers/colors.hpp"
#include "./helpers/getopt_cpp.hpp"
#include "version.hpp"
#include "tracker.hpp"
#include "configFile.hpp"


uavos::CConfigFile& cConfigFile = uavos::CConfigFile::getInstance();

bool m_exit = false;
static std::string configName = "config.module.json";




void uninit ()
{
    uavos::tracker::CTracker::getInstance().uninit();
	m_exit = true;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
{
	std::cout << _INFO_CONSOLE_TEXT << std::endl << "TERMINATING AT USER REQUEST" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
	
	try 
    {
        uninit();
	}
	catch (int error)
    {
        #ifdef DEBUG
            std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: quit_handler" << std::to_string(error)<< _NORMAL_CONSOLE_TEXT_ << std::endl;
        #endif

    }

    exit(0);

}


/**
 * @brief display version info
 * 
 */
void _version (void)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ "Drone-Engage Tracker Module version " << _INFO_CONSOLE_TEXT << version_string << _NORMAL_CONSOLE_TEXT_ << std::endl;
}


/**
 * @brief display help for -h command argument.
 * 
 */
void _usage(void)
{
    _version ();
    std::cout << std::endl << _INFO_CONSOLE_TEXT "Options" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t--config:          -c ./config.json   default [./config.module.json]" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT "\t--version:         -v" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

void initArguments (int argc, char *argv[])
{
    int opt;
    const struct GetOptLong::option options[] = {
        {"config",         true,   0, 'c'},
        {"version",        false,  0, 'v'},
        {"help",           false,  0, 'h'},
        {0, false, 0, 0}
    };
    // adding ':' means there is extra parameter needed
    GetOptLong gopt(argc, argv, "c:vh",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'c':
            configName = gopt.optarg;
            break;
        case 'v':
            _version();
            exit(0);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }
}

void init (int argc, char *argv[]) 
{
	signal(SIGINT,quit_handler);
    signal(SIGTERM,quit_handler);
    
    initArguments (argc, argv);

    // Reading Configuration
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "=================== " << "STARTING PLUGIN ===================" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    _version();

    cConfigFile.initConfigFile (configName.c_str());
    const Json& jsonConfig = cConfigFile.GetConfigJSON();
    

    uavos::tracker::CTracker::getInstance().init(( enum uavos::tracker::ENUM_TRACKER_TYPE) jsonConfig["tracker_algorithm_index"].get<int>()); //uavos::tracker::ENUM_TRACKER_TYPE::TRACKER_TLD);
    uavos::tracker::CTracker::getInstance().track(jsonConfig["video_device"].get<std::string>(), true);
}




int main (int argc, char *argv[]) 
{
	init(argc, argv);

    #ifdef DEBUG
        std::cout << "DEBUG" <<  std::endl;
    #elif defined(RELEASE)
        std::cout << "RELEASE" <<  std::endl;
    #endif
}




