/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
 *     TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

//#include "msdk_nvram_camera_exp.h"
//#include "msdk_lens_exp.h"
#include "camera_custom_nvram.h"
#include "camera_custom_lens.h"


const NVRAM_LENS_PARA_STRUCT S5K3H2YXAF_LENS_PARA_DEFAULT_VALUE =
{
    //Version
    NVRAM_CAMERA_LENS_FILE_VERSION,

    // Focus Range NVRAM
    {0, 1023},

    // AF NVRAM
    {
        // -------- AF ------------
        {140, // i4Offset
          14, // i4NormalNum
          14, // i4MacroNum
           0, // i4InfIdxOffset
           0, //i4MacroIdxOffset          
    	{
                 0,  46,  93,  141, 190, 240, 291, 343, 395, 447,
               499, 551, 603, 655, 0,  0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0
            },
          20, // i4THRES_MAIN;
          15, // i4THRES_SUB;            
          4,  // i4INIT_WAIT;
          {500, 500, 500, 500, 500}, // i4FRAME_WAIT
          0,  // i4DONE_WAIT;
              
          -1,  // i4FAIL_POS;

          33,  // i4FRAME_TIME                        
          2,  // i4FIRST_FV_WAIT; 
            
        30,  // i4FV_CHANGE_THRES
          8000,  // i4FV_CHANGE_OFFSET;        
          10,  // i4FV_CHANGE_CNT;
          20,  // i4GS_CHANGE_THRES;    
          8000,  // i4GS_CHANGE_OFFSET;   
          8,  // i4GS_CHANGE_CNT;            
          12,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
          10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
          12,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
        12,  // i4FV_STABLE_CNT
          12,  // i4FV_1ST_STABLE_THRES;        
          10000,  // i4FV_1ST_STABLE_OFFSET;
          6,  // i4FV_1ST_STABLE_NUM;                        
          6  // i4FV_1ST_STABLE_CNT;      
         },
         
         // -------- ZSD AF ------------
        {140, // i4Offset
          14, // i4NormalNum
          14, // i4MacroNum
           0, // i4InfIdxOffset
           0, //i4MacroIdxOffset          
    	{
                 0,  46,  93,  141, 190, 240, 291, 343, 395, 447,
               499, 551, 603, 655, 0,  0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0
            },
          20, // i4THRES_MAIN;
          15, // i4THRES_SUB;            
          4,  // i4INIT_WAIT;
          {500, 500, 500, 500, 500}, // i4FRAME_WAIT
          0,  // i4DONE_WAIT;
              
          -1,  // i4FAIL_POS;

          33,  // i4FRAME_TIME                        
          2,  // i4FIRST_FV_WAIT; 
            
        30,  // i4FV_CHANGE_THRES
          8000,  // i4FV_CHANGE_OFFSET;        
          10,  // i4FV_CHANGE_CNT;
          20,  // i4GS_CHANGE_THRES;    
          8000,  // i4GS_CHANGE_OFFSET;   
          8,  // i4GS_CHANGE_CNT;            
          12,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
          10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
          12,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
        12,  // i4FV_STABLE_CNT
          12,  // i4FV_1ST_STABLE_THRES;        
          10000,  // i4FV_1ST_STABLE_OFFSET;
          6,  // i4FV_1ST_STABLE_NUM;                        
          6  // i4FV_1ST_STABLE_CNT;      


           }, 
           
           // -------- VAFC ------------
        {140, // i4Offset
          14, // i4NormalNum
          14, // i4MacroNum
           0, // i4InfIdxOffset
           0, //i4MacroIdxOffset          
    	{
                 0,  46,  93,  141, 190, 240, 291, 343, 395, 447,
               499, 551, 603, 655, 0,  0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0
            },
          20, // i4THRES_MAIN;
          15, // i4THRES_SUB;            
          4,  // i4INIT_WAIT;
          {500, 500, 500, 500, 500}, // i4FRAME_WAIT
          0,  // i4DONE_WAIT;
              
          -1,  // i4FAIL_POS;

          33,  // i4FRAME_TIME                        
          2,  // i4FIRST_FV_WAIT; 
            
        30,  // i4FV_CHANGE_THRES
          8000,  // i4FV_CHANGE_OFFSET;        
          10,  // i4FV_CHANGE_CNT;
          20,  // i4GS_CHANGE_THRES;    
          8000,  // i4GS_CHANGE_OFFSET;   
          8,  // i4GS_CHANGE_CNT;            
          12,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
          10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
          12,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
        12,  // i4FV_STABLE_CNT
          12,  // i4FV_1ST_STABLE_THRES;        
          10000,  // i4FV_1ST_STABLE_OFFSET;
          6,  // i4FV_1ST_STABLE_NUM;                        
          6  // i4FV_1ST_STABLE_CNT;      

          },

        // --- sAF_TH ---
         {
          8,   // i4ISONum;
          {100, 150, 200, 300, 400, 600, 800, 1600},       // i4ISO[ISO_MAX_NUM];
                  
          6,   // i4GMeanNum;
          {20, 55, 105, 150, 180, 205},        // i4GMean[GMEAN_MAX_NUM];

          { 89, 89, 89, 88, 88, 88, 87, 72,
     127, 127, 127, 126, 126, 126, 126, 112,
     180, 180, 180, 180, 180, 180, 180, 171},        // i4GMR[3][ISO_MAX_NUM];
          
// ------------------------------------------------------------------------                  
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},        // i4FV_DC[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
          {50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000},         // i4MIN_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];        

          { 4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9}, // i4HW_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},        // i4FV_DC2[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},         // i4MIN_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];
          
          {5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15}      // i4HW_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];       
          
         },
// ------------------------------------------------------------------------

         // --- sZSDAF_TH ---
          {
           8,   // i4ISONum;
           {100, 150, 200, 300, 400, 600, 800, 1600},       // i4ISO[ISO_MAX_NUM];
                   
           6,   // i4GMeanNum;
           {20, 55, 105, 150, 180, 205},        // i4GMean[GMEAN_MAX_NUM];

           { 88, 88, 87, 87, 86, 85, 85, 83,
     126, 126, 125, 125, 125, 124, 124, 122,
     180, 180, 180, 179, 179, 179, 178, 177},        // i4GMR[3][ISO_MAX_NUM];
           
// ------------------------------------------------------------------------                   
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},        // i4FV_DC[GMEAN_MAX_NUM][ISO_MAX_NUM];
            
           {50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000},         // i4MIN_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];        
         
           {6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17},       // i4HW_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},        // i4FV_DC2[GMEAN_MAX_NUM][ISO_MAX_NUM];
            
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},         // i4MIN_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
           {9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27}          // i4HW_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------           
          },

          1, // i4VAFC_FAIL_CNT;
          0, // i4CHANGE_CNT_DELTA;

          0, // i4LV_THRES;

          18, // i4WIN_PERCENT_W;
          24, // i4WIN_PERCENT_H;                
          200,  // i4InfPos;
          20, //i4AFC_STEP_SIZE;

          {
              {50, 100, 150, 200, 250}, // back to bestpos step
              { 0,   0,   0,   0,   0}  // hysteresis compensate step
          },

          {0, 50, 150, 250, 350}, // back jump
          400,  //i4BackJumpPos

          80, // i4FDWinPercent;
          100, // i4FDSizeDiff;

          3,   //i4StatGain          

          {0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0}// i4Coef[20];
    },

    {0}
};


//------------------------- VCM:1 --> new vcm ---------------------------
const NVRAM_LENS_PARA_STRUCT S5K3H2YXAF_LENS_PARA1_VALUE =
{
    //Version
    NVRAM_CAMERA_LENS_FILE_VERSION,

    // Focus Range NVRAM
    {0, 1023},

    // AF NVRAM
    {
        // -------- AF ------------
        {140, // i4Offset
          14, // i4NormalNum
          14, // i4MacroNum
           0, // i4InfIdxOffset
           0, //i4MacroIdxOffset          
    	{
                 0,  46,  93,  141, 190, 240, 291, 343, 395, 447,
               499, 551, 603, 655, 0,  0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0
            },
          20, // i4THRES_MAIN;
          15, // i4THRES_SUB;            
          4,  // i4INIT_WAIT;
          {500, 500, 500, 500, 500}, // i4FRAME_WAIT
          0,  // i4DONE_WAIT;
              
          -1,  // i4FAIL_POS;

          33,  // i4FRAME_TIME                        
          2,  // i4FIRST_FV_WAIT; 
            
        30,  // i4FV_CHANGE_THRES
          8000,  // i4FV_CHANGE_OFFSET;        
          10,  // i4FV_CHANGE_CNT;
          20,  // i4GS_CHANGE_THRES;    
          8000,  // i4GS_CHANGE_OFFSET;   
          8,  // i4GS_CHANGE_CNT;            
          12,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
          10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
          12,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
        12,  // i4FV_STABLE_CNT
          12,  // i4FV_1ST_STABLE_THRES;        
          10000,  // i4FV_1ST_STABLE_OFFSET;
          6,  // i4FV_1ST_STABLE_NUM;                        
          6  // i4FV_1ST_STABLE_CNT;      
         },
         
         // -------- ZSD AF ------------
        {140, // i4Offset
          14, // i4NormalNum
          14, // i4MacroNum
           0, // i4InfIdxOffset
           0, //i4MacroIdxOffset          
    	{
                 0,  46,  93,  141, 190, 240, 291, 343, 395, 447,
               499, 551, 603, 655, 0,  0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0
            },
          20, // i4THRES_MAIN;
          15, // i4THRES_SUB;            
          4,  // i4INIT_WAIT;
          {500, 500, 500, 500, 500}, // i4FRAME_WAIT
          0,  // i4DONE_WAIT;
              
          -1,  // i4FAIL_POS;

          33,  // i4FRAME_TIME                        
          2,  // i4FIRST_FV_WAIT; 
            
        30,  // i4FV_CHANGE_THRES
          8000,  // i4FV_CHANGE_OFFSET;        
          10,  // i4FV_CHANGE_CNT;
          20,  // i4GS_CHANGE_THRES;    
          8000,  // i4GS_CHANGE_OFFSET;   
          8,  // i4GS_CHANGE_CNT;            
          12,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
          10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
          12,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
        12,  // i4FV_STABLE_CNT
          12,  // i4FV_1ST_STABLE_THRES;        
          10000,  // i4FV_1ST_STABLE_OFFSET;
          6,  // i4FV_1ST_STABLE_NUM;                        
          6  // i4FV_1ST_STABLE_CNT;      


           }, 
           
           // -------- VAFC ------------
        {140, // i4Offset
          14, // i4NormalNum
          14, // i4MacroNum
           0, // i4InfIdxOffset
           0, //i4MacroIdxOffset          
    	{
                 0,  46,  93,  141, 190, 240, 291, 343, 395, 447,
               499, 551, 603, 655, 0,  0,   0,   0,   0,   0,
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0
            },
          20, // i4THRES_MAIN;
          15, // i4THRES_SUB;            
          4,  // i4INIT_WAIT;
          {500, 500, 500, 500, 500}, // i4FRAME_WAIT
          0,  // i4DONE_WAIT;
              
          -1,  // i4FAIL_POS;

          33,  // i4FRAME_TIME                        
          2,  // i4FIRST_FV_WAIT; 
            
        30,  // i4FV_CHANGE_THRES
          8000,  // i4FV_CHANGE_OFFSET;        
          10,  // i4FV_CHANGE_CNT;
          20,  // i4GS_CHANGE_THRES;    
          8000,  // i4GS_CHANGE_OFFSET;   
          8,  // i4GS_CHANGE_CNT;            
          12,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
          10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
          12,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
        12,  // i4FV_STABLE_CNT
          12,  // i4FV_1ST_STABLE_THRES;        
          10000,  // i4FV_1ST_STABLE_OFFSET;
          6,  // i4FV_1ST_STABLE_NUM;                        
          6  // i4FV_1ST_STABLE_CNT;      

          },

        // --- sAF_TH ---
         {
          8,   // i4ISONum;
          {100, 150, 200, 300, 400, 600, 800, 1600},       // i4ISO[ISO_MAX_NUM];
                  
          6,   // i4GMeanNum;
          {20, 55, 105, 150, 180, 205},        // i4GMean[GMEAN_MAX_NUM];

          { 89, 89, 89, 88, 88, 88, 87, 72,
     127, 127, 127, 126, 126, 126, 126, 112,
     180, 180, 180, 180, 180, 180, 180, 171},        // i4GMR[3][ISO_MAX_NUM];
          
// ------------------------------------------------------------------------                  
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},        // i4FV_DC[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
          {50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000,
           50000,50000,50000,50000,50000,50000,50000,50000},         // i4MIN_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];        

          { 4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9,
     4, 4, 4, 5, 6, 6, 7, 9}, // i4HW_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},        // i4FV_DC2[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},         // i4MIN_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];
          
          {5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15,
     5, 7, 7, 7, 8, 10, 9, 15}      // i4HW_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];       
          
         },
// ------------------------------------------------------------------------

         // --- sZSDAF_TH ---
          {
           8,   // i4ISONum;
           {100, 150, 200, 300, 400, 600, 800, 1600},       // i4ISO[ISO_MAX_NUM];
                   
           6,   // i4GMeanNum;
           {20, 55, 105, 150, 180, 205},        // i4GMean[GMEAN_MAX_NUM];

           { 88, 88, 87, 87, 86, 85, 85, 83,
     126, 126, 125, 125, 125, 124, 124, 122,
     180, 180, 180, 179, 179, 179, 178, 177},        // i4GMR[3][ISO_MAX_NUM];
           
// ------------------------------------------------------------------------                   
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},        // i4FV_DC[GMEAN_MAX_NUM][ISO_MAX_NUM];
            
           {50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000},         // i4MIN_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];        
         
           {6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17,
     6, 7, 7, 8, 10, 11, 12, 17},       // i4HW_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},        // i4FV_DC2[GMEAN_MAX_NUM][ISO_MAX_NUM];
            
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},         // i4MIN_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
           {9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27,
     9, 10, 11, 14, 15, 18, 20, 27}          // i4HW_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------           
          },

          1, // i4VAFC_FAIL_CNT;
          0, // i4CHANGE_CNT_DELTA;

          0, // i4LV_THRES;

          18, // i4WIN_PERCENT_W;
          24, // i4WIN_PERCENT_H;                
          200,  // i4InfPos;
          20, //i4AFC_STEP_SIZE;

          {
              {50, 100, 150, 200, 250}, // back to bestpos step
              { 0,   0,   0,   0,   0}  // hysteresis compensate step
          },

          {0, 50, 150, 250, 350}, // back jump
          400,  //i4BackJumpPos

          80, // i4FDWinPercent;
          100, // i4FDSizeDiff;

          3,   //i4StatGain          

          {0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0}// i4Coef[20];
    },

    {0}
};
//------------------------- VCM:1 --> new vcm ---------------------------

#if 1
static int get_path_value(const char * path)
{
	int  size = 32;
    int  fd;
	int  count, value;
    char buf[size];

	fd = open(path, O_RDONLY);
	
	if (fd == -1) {
		return 0;
	}
	
	count = read(fd, buf, size); 
	if (count > 0) {
		count = (count < size) ? count : size - 1;
		while (count > 0 && buf[count-1] == '\n') count--;
		buf[count] = '\0';
	} else {
		buf[0] = '\0';
	}
	close(fd);
	
	sscanf(buf, "%x", &value);
	
	return value;
}	
		
static int S5K3H2YXAF_get_sensor_id(void)
{
	const char *sensor_id_file = "/sys/devices/virtual/sensordrv/kd_camera_hw/MainSensor_ID";

	int sensor_id = get_path_value(sensor_id_file);
	return sensor_id;
}
static int S5K3H2YXAF_get_module_id(void)
{
	const char *module_id_file = "/sys/devices/virtual/sensordrv/kd_camera_hw/MainSensor_ModuleID";

	int module_id = get_path_value(module_id_file);
	return module_id;
}
static int S5K3H2YXAF_get_vcm_id(void)
{
	const char *vcm_id_file = "/sys/devices/virtual/sensordrv/kd_camera_hw/MainSensor_VcmID";

	int vcm_id = get_path_value(vcm_id_file);
	return vcm_id;
}
#endif

UINT32 S5K3H2YXAF_getDefaultData(VOID *pDataBuf, UINT32 size)
{
    UINT32 dataSize = sizeof(NVRAM_LENS_PARA_STRUCT);

    if ((pDataBuf == NULL) || (size < dataSize))
    {
        return 1;
    }

	if((S5K3H2YXAF_get_sensor_id() == 0x382b)&&(S5K3H2YXAF_get_module_id() == 0x07)&&(S5K3H2YXAF_get_vcm_id() == 0x01)){
	// new vcm id:1
		memcpy(pDataBuf, &S5K3H2YXAF_LENS_PARA1_VALUE, dataSize);  
	}else{
    // copy from Buff to global struct
    	memcpy(pDataBuf, &S5K3H2YXAF_LENS_PARA_DEFAULT_VALUE, dataSize);
	}
    return 0;
}

PFUNC_GETLENSDEFAULT pS5K3H2YXAF_getDefaultData = S5K3H2YXAF_getDefaultData;


