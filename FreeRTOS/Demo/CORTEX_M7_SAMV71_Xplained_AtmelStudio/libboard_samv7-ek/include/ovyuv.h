/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,

 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef _YUV_H_
#define _YUV_H_

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include <board.h>


/*---------------------------------------------------------------------------
 *         Exported variable
 *---------------------------------------------------------------------------*/
extern const struct ov_reg ov2640_yuv_vga[];
extern const struct ov_reg ov2640_yuv_qvga[];

extern const struct ov_reg ov2643_yuv_vga[];
extern const struct ov_reg ov2643_yuv_swvga[];
extern const struct ov_reg ov2643_yuv_uxga[];
extern const struct ov_reg ov2643_yuv_qvga[];

extern const struct ov_reg ov5640_yuv_vga[];
extern const struct ov_reg ov5640_yuv_sxga[];
extern const struct ov_reg ov5640_afc[];

extern const struct ov_reg OV7740_VGA_YUV422[];
extern const struct ov_reg OV7740_QVGA_YUV422[];
extern const struct ov_reg OV7740_QVGA_RGB888[];
extern const struct ov_reg OV7740_QQVGA_YUV422[];
extern const struct ov_reg OV7740_QQVGA_RGB888[];
extern const struct ov_reg OV7740_CIF_YUV422[];
extern const struct ov_reg OV7740_TEST_PATTERN[];


extern const struct ov_reg ov9740_yuv_sxga[];
extern const struct ov_reg ov9740_yuv_vga[];

#endif // #ifndef _YUV_H_

