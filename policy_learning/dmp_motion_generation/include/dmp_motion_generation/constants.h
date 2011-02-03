/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Peter Pastor */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace dmp
{

static const int MIN_NUM_DATA_POINTS = 90;

static const int NUM_DEBUG_CANONICAL_SYSTEM_VALUES = 3;
static const int NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES = 6;

static const int MAX_NUMBER_OF_LIBRARY_ITEMS = 100;

static const int FILENAME_LENGTH = 200;
static const int MAX_VARNAME_LENGTH = 20;

static const int MAX_NUM_TRANSFORMATION_SYSTEMS = 20;

static const double ADDITIONAL_TIME_IN_SEC_FOR_SMOOTHING = 0.02;
static const double DEFAULT_SAMPLING_FREQUENCY = 1000.0;

static const int POS_VEL_ACC = (1+1+1);
static const int _POS_ = 1;
static const int _VEL_ = 2;
static const int _ACC_ = 3;

static const int N_CART = 3;
static const int  _XX_ = 0;
static const int  _YY_ = 1;
static const int  _ZZ_ = 2;

static const int N_QUAT = 4;
static const int _QX_ = 0;
static const int _QY_ = 1;
static const int _QZ_ = 2;
static const int _QW_ = 3;

static const int N_JOINTS = 7;

// TODO: move this into a PR2 specific common place
const int CONTINOUS_FOREARM_ROLL_JOINT = 4;
const int CONTINOUS_WRIST_ROLL_JOINT = 6;

enum eVerbosityLevel
{
	eQUIET=0, eNORMAL, eVERBOSE
};

// #define HUGE_VALUE (10000000) // TODO: change this to use the math.h INF
// #define SEC2NANO 1000000000
// #ifndef DELETE_POINTER
// #define DELETE_POINTER(ptr) if ( NULL != (ptr) ) { delete (ptr); (ptr) = NULL; }
// #endif

}

#endif /* CONSTANTS_H_ */
