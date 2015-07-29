#ifndef FPSCOUNTER
#define FPSCOUNTER

#include <time.h>
#include <boost/thread.hpp>

class FpsCounter
{
    
public:
    FpsCounter();
    
    void update();
    
    unsigned int get();

protected:
    time_t initial_time;
    unsigned int m_fpscount;

    boost::mutex mMutexFps;
    unsigned int m_fps;
        
};

#endif // FPSCOUNTER
