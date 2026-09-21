#pragma once

/* Symbol visibility of fsim.dll and fsim_vision.dll (design 5, rule 5).
 * Defined once here and reused by fsim_c.h and the C++ headers. */
#if defined(_WIN32)
#  if defined(FSIM_BUILDING) && !defined(FSIM_VISION_BUILDING)
#    define FSIM_API __declspec(dllexport)
#  elif defined(FSIM_STATIC)
#    define FSIM_API
#  else
#    define FSIM_API __declspec(dllimport)
#  endif
#  if defined(FSIM_VISION_BUILDING)
#    define FSIM_VISION_API __declspec(dllexport)
#  elif defined(FSIM_STATIC)
#    define FSIM_VISION_API
#  else
#    define FSIM_VISION_API __declspec(dllimport)
#  endif
#else
#  define FSIM_API __attribute__((visibility("default")))
#  define FSIM_VISION_API __attribute__((visibility("default")))
#endif
