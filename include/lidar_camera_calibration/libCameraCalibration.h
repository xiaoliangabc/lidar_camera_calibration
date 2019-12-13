//
// MATLAB Compiler: 7.0.1 (R2019a)
// Date: Fri Dec 13 19:28:55 2019
// Arguments:
// "-B""macro_default""-W""cpplib:libCameraCalibration""-T""link:lib""camera_cal
// ibration.m"
//

#ifndef __libCameraCalibration_h
#define __libCameraCalibration_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libCameraCalibration_C_API 
#define LIB_libCameraCalibration_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_libCameraCalibration_C_API 
bool MW_CALL_CONV libCameraCalibrationInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libCameraCalibration_C_API 
bool MW_CALL_CONV libCameraCalibrationInitialize(void);

extern LIB_libCameraCalibration_C_API 
void MW_CALL_CONV libCameraCalibrationTerminate(void);

extern LIB_libCameraCalibration_C_API 
void MW_CALL_CONV libCameraCalibrationPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_libCameraCalibration_C_API 
bool MW_CALL_CONV mlxCamera_calibration(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                        *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_libCameraCalibration
#define PUBLIC_libCameraCalibration_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libCameraCalibration_CPP_API __declspec(dllimport)
#endif

#define LIB_libCameraCalibration_CPP_API PUBLIC_libCameraCalibration_CPP_API

#else

#if !defined(LIB_libCameraCalibration_CPP_API)
#if defined(LIB_libCameraCalibration_C_API)
#define LIB_libCameraCalibration_CPP_API LIB_libCameraCalibration_C_API
#else
#define LIB_libCameraCalibration_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libCameraCalibration_CPP_API void MW_CALL_CONV camera_calibration(const mwArray& imagePath, const mwArray& resultPath);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
