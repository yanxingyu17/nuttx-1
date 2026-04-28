/****************************************************************************
 * drivers/video/ov2640_v4l2.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/arch.h>
#include <nuttx/video/video.h>

#include <nuttx/video/ov2640_v4l2.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV2640_I2C_ADDR   0x30
#define OV2640_I2C_FREQ   100000

#define WRITE_REGS(priv, arr) \
  ov2640_write_reglist((priv)->i2c, arr, \
                       sizeof(arr) / sizeof(arr[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ov2640_reg_s
{
  uint8_t reg;
  uint8_t val;
};

struct ov2640_dev_s
{
  struct imgsensor_s sensor;
  struct i2c_master_s *i2c;
  uint16_t width;
  uint16_t height;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool ov2640_is_available(struct imgsensor_s *sensor);
static int ov2640_init(struct imgsensor_s *sensor);
static int ov2640_uninit(struct imgsensor_s *sensor);
static const char *ov2640_get_driver_name(struct imgsensor_s *sensor);
static int ov2640_validate_frame_setting(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval);
static int ov2640_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval);
static int ov2640_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);

/****************************************************************************
 * Private Data - OV2640 Register Tables
 ****************************************************************************/

static const struct ov2640_reg_s g_ov2640_reset[] =
{
  { 0xff, 0x01 },
  { 0x12, 0x80 }, /* COM7 soft reset */
};

static const struct ov2640_reg_s g_ov2640_cif_base[] =
{
  { 0xff, 0x00 },
  { 0xff, 0x01 },
  { 0x3c, 0x32 },
  { 0x11, 0x01 }, /* CLKRC */
  { 0x09, 0x02 }, /* COM2 */
  { 0x04, 0x28 }, /* REG04: VFLIP */
  { 0x13, 0xe7 }, /* COM8: AGC+AEC+AWB */
  { 0x14, 0x68 }, /* COM9: AGC gain 32x */
  { 0x2c, 0x0c },
  { 0x33, 0x78 },
  { 0x3a, 0x33 },
  { 0x3e, 0x00 },
  { 0x39, 0x92 },
  { 0x37, 0xc3 },
  { 0x06, 0x88 },
  { 0x0e, 0x41 },
  { 0x21, 0x99 },
  { 0x24, 0x40 }, /* AEW */
  { 0x25, 0x38 }, /* AEB */
  { 0x26, 0x82 }, /* VV */
  { 0x5c, 0x00 },
  { 0x61, 0x70 },
  { 0x7c, 0x05 },
  { 0x6c, 0x00 },
  { 0x70, 0x02 },
  { 0x3d, 0x34 },
  { 0x5a, 0x57 }, /* BD50 */
  { 0x4f, 0xbb }, /* BD60 */
  { 0x50, 0x9c },
  { 0x12, 0x20 }, /* COM7: CIF */
  { 0x17, 0x11 }, /* HSTART */
  { 0x18, 0x43 }, /* HSTOP */
  { 0x19, 0x00 }, /* VSTART */
  { 0x1a, 0x25 }, /* VSTOP */
  { 0x32, 0x89 }, /* REG32 */
  { 0x37, 0xc0 },
  { 0x4f, 0xca }, /* BD50 */
  { 0x50, 0xa8 }, /* BD60 */
  { 0x6d, 0x00 },
  { 0xff, 0x00 },
  { 0xe5, 0x7f },
  { 0xf9, 0xc0 }, /* MC_BIST */
  { 0x41, 0x24 },
  { 0xe0, 0x14 }, /* RESET: JPEG+DVP */
  { 0x76, 0xff },
  { 0x43, 0x18 },
  { 0x87, 0xd0 }, /* CTRL3 */
  { 0x88, 0x3f },
  { 0xd7, 0x03 },
  { 0xd3, 0x82 }, /* R_DVP_SP: auto + 0x02 */
  { 0xc8, 0x08 },
  { 0x7c, 0x00 },
  { 0x7c, 0x03 },
  { 0x7c, 0x08 },
  { 0x7d, 0x20 },
  { 0x90, 0x00 },
  { 0x91, 0x0e },
  { 0x91, 0x5a },
  { 0x91, 0x7e },
  { 0x91, 0x96 },
  { 0x91, 0xc4 },
  { 0x91, 0x20 },
  { 0x92, 0x00 },
  { 0x93, 0x06 },
  { 0x93, 0x05 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x96, 0x00 },
  { 0x97, 0x08 },
  { 0x97, 0x0c },
  { 0x97, 0x28 },
  { 0x97, 0x98 },
  { 0x97, 0x00 },
  { 0xa4, 0x00 },
  { 0xc5, 0x11 },
  { 0xbf, 0x80 },
  { 0xb6, 0x66 },
  { 0xb9, 0x7c },
  { 0xb5, 0xff },
  { 0xb2, 0x0f },
  { 0xc4, 0x5c },
  { 0xc0, 0xfd }, /* CTRL1 */
  { 0x7f, 0x00 },
  { 0xe5, 0x1f },
  { 0xda, 0x00 }, /* IMAGE_MODE: YUV422 default */
  { 0xe0, 0x00 }, /* RESET: enable all */
  { 0x05, 0x00 }, /* R_BYPASS: DSP_EN */
};

static const struct ov2640_reg_s g_ov2640_to_cif[] =
{
  { 0xff, 0x01 },
  { 0x12, 0x20 }, /* COM7: CIF */
  { 0x03, 0x0a }, /* COM1 */
  { 0x32, 0x89 }, /* REG32: CIF */
  { 0x17, 0x11 }, /* HSTART */
  { 0x18, 0x43 }, /* HSTOP */
  { 0x19, 0x00 }, /* VSTART */
  { 0x1a, 0x25 }, /* VSTOP */
  { 0x4f, 0xca }, /* BD50 */
  { 0x50, 0xa8 }, /* BD60 */
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x92 },
  { 0x37, 0xc3 },
  { 0x06, 0x88 },
  { 0x0e, 0x41 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 }, /* RESET: DVP reset */
  { 0xc0, 0x32 }, /* HSIZE8 */
  { 0xc1, 0x25 }, /* VSIZE8 */
  { 0x8c, 0x00 }, /* SIZEL */
  { 0x51, 0x64 }, /* HSIZE */
  { 0x52, 0x4a }, /* VSIZE */
  { 0x53, 0x00 }, /* XOFFL */
  { 0x54, 0x00 }, /* YOFFL */
  { 0x55, 0x00 }, /* VHYX */
  { 0x57, 0x00 }, /* TEST */
  { 0x86, 0x3d }, /* CTRL2: DCW_EN */
  { 0x50, 0x80 }, /* CTRLI: LP_DP */
};

static const struct ov2640_reg_s g_ov2640_qvga_window[] =
{
  { 0xff, 0x00 },
  { 0x5a, 0x50 }, /* ZMOW: 80 = 320/4 */
  { 0x5b, 0x3c }, /* ZMOH: 60 = 240/4 */
  { 0x5c, 0x00 }, /* ZMHH */
};

static const struct ov2640_reg_s g_ov2640_clock[] =
{
  { 0xff, 0x01 },
  { 0x11, 0x83 }, /* CLKRC: clk_2x=1, div=3 */
  { 0xff, 0x00 },
  { 0xd3, 0x88 }, /* R_DVP_SP: pclk_auto=1, div=8 */
};

static const struct ov2640_reg_s g_ov2640_dsp_en[] =
{
  { 0xff, 0x00 },
  { 0x05, 0x00 }, /* R_BYPASS: DSP_EN */
};

static const struct ov2640_reg_s g_ov2640_rgb565[] =
{
  { 0xff, 0x00 },
  { 0xe0, 0x04 }, /* RESET: DVP */
  { 0xda, 0x09 }, /* IMAGE_MODE: RGB565 + byte swap */
  { 0xd7, 0x03 },
  { 0xe1, 0x77 },
  { 0xe0, 0x00 }, /* RESET: enable all */
};

static const struct imgsensor_ops_s g_ov2640_ops =
{
  .is_available           = ov2640_is_available,
  .init                   = ov2640_init,
  .uninit                 = ov2640_uninit,
  .get_driver_name        = ov2640_get_driver_name,
  .validate_frame_setting = ov2640_validate_frame_setting,
  .start_capture          = ov2640_start_capture,
  .stop_capture           = ov2640_stop_capture,
};

static const struct v4l2_fmtdesc g_ov2640_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_RGB565,
    .description = "RGB565",
  },
};

static const struct v4l2_frmsizeenum g_ov2640_frmsizes[] =
{
  {
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
      {
        .width = 320, .height = 240
      },
  },
};

static const struct v4l2_frmivalenum g_ov2640_frmintervals[] =
{
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete =
      {
        .numerator = 1, .denominator = 15
      },
  },
};

static struct ov2640_dev_s g_ov2640_dev =
{
  .sensor =
    {
      .ops = &g_ov2640_ops,
      .fmtdescs_num = 1,
      .fmtdescs = g_ov2640_fmtdescs,
      .frmsizes_num = 1,
      .frmsizes = g_ov2640_frmsizes,
      .frmintervals_num = 1,
      .frmintervals = g_ov2640_frmintervals,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_putreg
 ****************************************************************************/

static int ov2640_putreg(struct i2c_master_s *i2c,
                         uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];
  int ret;

  buf[0] = regaddr;
  buf[1] = regval;

  msg.frequency = OV2640_I2C_FREQ;
  msg.addr      = OV2640_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  ret = I2C_TRANSFER(i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C write 0x%02x=0x%02x failed: %d\n",
            regaddr, regval, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ov2640_write_reglist
 ****************************************************************************/

static int ov2640_write_reglist(struct i2c_master_s *i2c,
                                const struct ov2640_reg_s *regs,
                                size_t count)
{
  size_t i;
  int ret;

  for (i = 0; i < count; i++)
    {
      ret = ov2640_putreg(i2c, regs[i].reg, regs[i].val);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ov2640_is_available
 ****************************************************************************/

static bool ov2640_is_available(struct imgsensor_s *sensor)
{
  return true;
}

/****************************************************************************
 * Name: ov2640_init
 ****************************************************************************/

static int ov2640_init(struct imgsensor_s *sensor)
{
  struct ov2640_dev_s *priv = (struct ov2640_dev_s *)sensor;
  int ret;

  ret = WRITE_REGS(priv, g_ov2640_reset);
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(10);

  ret = WRITE_REGS(priv, g_ov2640_cif_base);
  if (ret < 0)
    {
      return ret;
    }

  ret = WRITE_REGS(priv, g_ov2640_to_cif);
  if (ret < 0)
    {
      return ret;
    }

  ret = WRITE_REGS(priv, g_ov2640_qvga_window);
  if (ret < 0)
    {
      return ret;
    }

  ret = WRITE_REGS(priv, g_ov2640_clock);
  if (ret < 0)
    {
      return ret;
    }

  ret = WRITE_REGS(priv, g_ov2640_dsp_en);
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(10);

  ret = WRITE_REGS(priv, g_ov2640_rgb565);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ov2640_uninit
 ****************************************************************************/

static int ov2640_uninit(struct imgsensor_s *sensor)
{
  return OK;
}

/****************************************************************************
 * Name: ov2640_get_driver_name
 ****************************************************************************/

static const char *ov2640_get_driver_name(struct imgsensor_s *sensor)
{
  return "OV2640";
}

/****************************************************************************
 * Name: ov2640_validate_frame_setting
 ****************************************************************************/

static int ov2640_validate_frame_setting(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval)
{
  UNUSED(type);
  if (nr_datafmts < 1 || !datafmts)
    {
      return -EINVAL;
    }

  if (datafmts[IMGSENSOR_FMT_MAIN].width == 0 ||
      datafmts[IMGSENSOR_FMT_MAIN].height == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: ov2640_start_capture
 ****************************************************************************/

static int ov2640_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval)
{
  return OK;
}

/****************************************************************************
 * Name: ov2640_stop_capture
 ****************************************************************************/

static int ov2640_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_initialize
 ****************************************************************************/

struct imgsensor_s *ov2640_initialize(struct i2c_master_s *i2c,
                                     uint16_t width, uint16_t height)
{
  g_ov2640_dev.i2c    = i2c;
  g_ov2640_dev.width  = width;
  g_ov2640_dev.height = height;
  return &g_ov2640_dev.sensor;
}
