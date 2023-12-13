# panther_gpiod

Package comprises a wrapper implementation for the GPIOD library, enabling real-time manipulation of GPIO pins on the Raspberry Pi. Offering convenient interfaces for setting pin values, altering their direction, monitoring events, and conducting other GPIO operations, this library facilitates effective GPIO pin management on the Panther robot. It simplifies integration within robotic applications.

## GPIODriver Usage Guide

Below is a minimal example demonstrating how to use the GPIODriver class:

```cpp
#include <iostream>
#include <vector>

#include "panther_gpiod/gpio_driver.hpp"

int main() {
    // Prepare vector with GPIO pin configurations
    std::vector<GPIOInfo> gpio_configurations = {
        GPIOInfo{GPIOPin::CHRG_SENSE, gpiod::line::direction::INPUT, true},
        GPIOInfo{GPIOPin::AUX_PW_EN, gpiod::line::direction::OUTPUT},
        // Add more GPIO pin configurations...
    };

    // Create GPIODriver object and pass GPIO configurations
    GPIODriver gpio_driver(gpio_configurations);

    // Enable GPIO monitoring with Real-Time (RT) configuration and a specific thread priorit value
    gpio_driver.GPIOMonitorEnable(/* use_rt = */ true, /* gpio_monit_thread_sched_priority = */ 50);

    // Set callback function for GPIO edge events (optional)
    gpio_driver.ConfigureEdgeEventCallback(your_callback_function);

    // Perform GPIO operations, such as reading pin values

    return 0;
}
```

> **NOTE**
>
> Not invoking the `GPIOMonitorEnable()` method will result in the absence of functionality to read pin values. It is not mandatory to call this method, but the `IsPinActive()` method throws a runtime error when the GPIO monitor thread is not running.
