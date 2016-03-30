/*  Copyright (C) 2011 - 2020
 *  ALV research group.
 *
 *  This file is part of the ROBIX Library.
 *
 *  ROBIX Library is "CLOSE SOURCE" software; Member of ALV research group
 *  of ZJU can redistribute it and/or modify it under the terms of ALV Lab 
 *  Software License 0.3.1415.
 */

/*! 
 *  \file   protocol_geo.h
 *  \brief  Road Guide points related.
 *  \author Qian Hui ( qianhui@zju.edu.cn )
 *  \history:
 *    xsl    2014/10/07   Creation.
 */

#ifndef PROTOCOL_GEO_H
#define PROTOCOL_GEO_H

#include "protocol_head.h"

#pragma pack(push)
#pragma pack(1)

/*!< Road level. */
typedef enum {
    R_LEVEL0 = 0,	/*!< default road. */
    R_LEVEL1,
    R_LEVEL2,
    R_LEVEL3,
    R_LEVEL4,		/*!< country road. */
    R_LEVEL5,		/*!< struct road. */
} ROAD_LEVEL;

/*!< Curve level. */
typedef enum {
    C_LEVEL0 = 0,
    C_LEVEL1,
    C_LEVEL2,
    C_LEVEL3,
} CURVE_LEVEL;

/*!< Scene data = lane_num + roadlevel + curvelevel  */
typedef struct {
	UINT8	lane_num;		/*!< number of lane */
    ROAD_LEVEL road_level; /*!< road level. */
    CURVE_LEVEL curve_level; /*!< curve level. */
} SCENE;

/*
 * GEO --> PL, FU
 */
typedef struct {
    RBX_HEADER header;                    /*!< robix header. */
    GUIDE_LINES gls; 	  				  /*!< guide lines*/
	SCENE scene;						  /*!< scene information */
    STATE state; 
}GEO_INFO;

#pragma pack(pop)

#endif /* PROTOCOL_GEO_H */
