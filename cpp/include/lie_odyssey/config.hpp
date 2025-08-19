#pragma once

#define LIE_BACKEND_LIEPP   1
#define LIE_BACKEND_MANIF   2
#define LIE_BACKEND_SOPHUS  3

#ifndef LIE_BACKEND
  #define LIE_BACKEND LIE_BACKEND_LIEPP  // default
#endif
