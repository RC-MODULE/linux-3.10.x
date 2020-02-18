//------------------------------------------------------------------------------
// File: config.h
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
// This file should be modified by some developers of C&M according to product version.
//------------------------------------------------------------------------------


#ifndef __RCM_VPU_CODA980_DEFS_H__
#define __RCM_VPU_CODA980_DEFS_H__


#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(WIN32) || defined(__MINGW32__)
#	define PLATFORM_WIN32
#elif defined(linux) || defined(__linux) || defined(ANDROID)
#	define PLATFORM_LINUX
#else
#	define PLATFORM_NON_OS
#endif

#ifdef PLATFORM_WIN32
#	include <windows.h>
#endif

#if defined(_MSC_VER)
#	define inline _inline
#elif defined(__GNUC__)
#elif defined(__ARMCC__)
#else
#  error "Unknown compiler."
#endif


#define API_VERSION 409



#if defined(PLATFORM_NON_OS) || defined (ANDROID) || defined(MFHMFT_EXPORTS) || defined(OMXIL_COMPONENT) || defined(DXVA_UMOD_DRIVER) || defined(__MINGW32__)
//#define SUPPORT_FFMPEG_DEMUX
#else
// #define SUPPORT_FFMPEG_DEMUX
#endif



//#define REPORT_PERFORMANCE


//#define SUPPORT_MEM_PROTECT






#define BIT_CODE_FILE_PATH <firmware/coda980.h>
#define PROJECT_ROOT "./"

// if BIT_CODE_FILE_PATH is not correct or is not present. vpurun will try to load bitcode from file which is specified in CORE_0_BIT_CODE_FILE_PATH. QC will try to load the file from 'firmware' field in qctool.conf.
// #define CORE_0_BIT_CODE_FILE_PATH "../../../../design/asm_s/out/coda980.out"




#define SUPPORT_NV21



//#define SLICE_END_INTERRUPT
//#define ENC_MB_POSITION

#define NEW_INTRA_REFRESH
#define RC_CHANGE_PARAMETER_DEF

// SUPPORT_ENC_REPORT has higher priority for testing new feature on CODA980_REV


#define  STRICT_CBR


#endif	// __RCM_VPU_CODA980_DEFS_H__
