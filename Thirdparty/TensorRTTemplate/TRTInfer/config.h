#ifndef CONFIG_H
#define CONFIG_H

// Windows platform export/import macros
#ifdef _WIN32
    #ifdef BUILD_SHARED
        #define TRTInfer_API __declspec(dllexport)
    #else
        #define TRTInfer_API __declspec(dllimport)
    #endif
#else
    // Linux platform does not need special handling
    #define TRTInfer_API
#endif

#endif
