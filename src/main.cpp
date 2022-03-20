#include <stdio.h>
#include <signal.h>
#include <iostream>

#include "tracker.hpp"

bool m_exit = false;


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
	std::cout <<  "TERMINATING AT USER REQUEST" <<  std::endl;
	
	try 
    {
        uninit();
	}
	catch (int error){}

}



void init (int argc, char** argv)
{
	signal(SIGINT,quit_handler);

    uavos::tracker::CTracker::getInstance().init(uavos::tracker::ENUM_TRACKER_TYPE::TRACKER_TLD);
    uavos::tracker::CTracker::getInstance().track("/dev/video0", true);
}




int main(int argc, char** argv)
{
	init(argc, argv);

    #ifdef DEBUG
        std::cout << "DEBUG" <<  std::endl;
    #elif defined(RELEASE)
        std::cout << "RELEASE" <<  std::endl;
    #endif
}




