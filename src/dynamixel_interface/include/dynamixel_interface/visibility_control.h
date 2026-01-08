#ifndef DYNAMIXEL_INTERFACE__VISIBILITY_CONTROL_H_
#define DYNAMIXEL_INTERFACE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIXEL_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define DYNAMIXEL_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIXEL_INTERFACE_EXPORT __declspec(dllexport)
    #define DYNAMIXEL_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIXEL_INTERFACE_BUILDING_DLL
    #define DYNAMIXEL_INTERFACE_PUBLIC DYNAMIXEL_INTERFACE_EXPORT
  #else
    #define DYNAMIXEL_INTERFACE_PUBLIC DYNAMIXEL_INTERFACE_IMPORT
  #endif
  #define DYNAMIXEL_INTERFACE_PUBLIC_TYPE DYNAMIXEL_INTERFACE_PUBLIC
  #define DYNAMIXEL_INTERFACE_LOCAL
#else
  #define DYNAMIXEL_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIXEL_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIXEL_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIXEL_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIXEL_INTERFACE_PUBLIC
    #define DYNAMIXEL_INTERFACE_LOCAL
  #endif
  #define DYNAMIXEL_INTERFACE_PUBLIC_TYPE
#endif

#endif  // DYNAMIXEL_INTERFACE__VISIBILITY_CONTROL_H_