#ifndef DEBUG_H
#define DEBUG_H

#ifdef _DEBUG
    #include <iostream>
    #define DEBUG_OUT(x) do { std::cout << __FILE__ << ": " << x << std::endl; } while(false)
#else
    #define DEBUG_OUT(x) do { } while(false)
#endif

#endif // DEBUG_H

