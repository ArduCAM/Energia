///
/// @file		Clock.h
/// @brief		Library header
/// @details	Clock Library for Energia MT
/// @n	
/// @n @b		Project EMT-ClockLibrary
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Rei Vilo
/// @author		http://embeddedcomputing.weebly.com
///
/// @date		Rei Vilo, Jun 17, 2015 09:59
/// @version	101
/// 
/// @copyright	(c) Rei Vilo, 2015
/// @copyright	CC = BY SA NC
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
// Include application, user and local libraries
#include <Energia.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>

#ifndef Clock_h
#define Clock_h

///
/// @brief      RTOS Clock as an object
/// @details    The RTOS Clock is encapsulated as a C++ object for easier use
/// @note       Multiple Clock objects possible.
///
class Clock
{
private:
    Clock_Handle ClockHandle;
    
public:
    ///
    /// @brief      Define the Clock
    ///
    Clock();
    
    ///
    /// @brief      Create the Clock
    /// @param      ClockFunction function to be called
    /// @param      ClockTimeOut_ms initial start delay
    /// @param      ClockPeriod_ms period in ms, default = 0 for one-shot
    /// @note       The function must be void ClockFunction()
    /// @code   void ClockFunction()
    ///         {
    ///             digitalWrite(RED_LED, HIGH);
    ///         }
    /// @endcode
    ///
    void begin(void (*ClockFunction)(void), uint32_t clockTimeOut_ms, uint32_t clockPeriod_ms = 0);

    ///
    /// @brief      Start the Clock
    ///
    void start();
    
    ///
    /// @brief      Stop the Clock
    ///
    void stop();
};

#endif
