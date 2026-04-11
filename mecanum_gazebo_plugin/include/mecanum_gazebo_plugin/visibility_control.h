#ifndef MECANUM_GAZEBO_PLUGIN__VISIBILITY_CONTROL_H_
#define MECANUM_GAZEBO_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MECANUM_GAZEBO_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define MECANUM_GAZEBO_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define MECANUM_GAZEBO_PLUGIN_EXPORT __declspec(dllexport)
    #define MECANUM_GAZEBO_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef MECANUM_GAZEBO_PLUGIN_BUILDING_LIBRARY
    #define MECANUM_GAZEBO_PLUGIN_PUBLIC MECANUM_GAZEBO_PLUGIN_EXPORT
  #else
    #define MECANUM_GAZEBO_PLUGIN_PUBLIC MECANUM_GAZEBO_PLUGIN_IMPORT
  #endif
  #define MECANUM_GAZEBO_PLUGIN_PUBLIC_TYPE MECANUM_GAZEBO_PLUGIN_PUBLIC
  #define MECANUM_GAZEBO_PLUGIN_LOCAL
#else
  #define MECANUM_GAZEBO_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define MECANUM_GAZEBO_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define MECANUM_GAZEBO_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define MECANUM_GAZEBO_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MECANUM_GAZEBO_PLUGIN_PUBLIC
    #define MECANUM_GAZEBO_PLUGIN_LOCAL
  #endif
  #define MECANUM_GAZEBO_PLUGIN_PUBLIC_TYPE
#endif

#endif  // MECANUM_GAZEBO_PLUGIN__VISIBILITY_CONTROL_H_
