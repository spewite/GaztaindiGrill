#include <Utils.h>

uint32_t Utils::get_current_unix_time() {

    time_t now;

    // Get the current time of the system
    time(&now);
    
    // If the time is lower than the year 2025
    // is that it has not been synronized correctly
    if (now < 1735686000) { 
        return -1; 
    }
    
    return (uint32_t)now;
}