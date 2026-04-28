/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_board_camera.c
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

#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/ov2640_v4l2.h>

#include "esp32s3-eye.h"
#include "esp32s3_cam.h"
#include "esp32s3_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV2640_I2C_BUS    0
#define OV2640_WIDTH      320
#define OV2640_HEIGHT     240

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_camera_initialize
 *
 * Description:
 *   Initialize the camera subsystem:
 *   1. Register ESP32-S3 CAM imgdata (this starts XCLK output)
 *   2. Wait for sensor power-up
 *   3. Initialize OV2640 sensor via I2C
 *   4. Register imgdata and imgsensor for V4L2
 *
 ****************************************************************************/

int esp32s3_camera_initialize(void)
{
  struct imgdata_s *imgdata;
  struct imgsensor_s *imgsensor;
  struct i2c_master_s *i2c;
  int ret;

  /* Step 1: Register ESP32-S3 CAM imgdata driver.
   * This also starts XCLK output — OV2640 needs clock before I2C.
   */

  imgdata = esp32s3_cam_initialize();
  if (imgdata == NULL)
    {
      snerr("ERROR: Failed to register CAM imgdata\n");
      return -ENODEV;
    }

  /* OV2640 requires XCLK to be stable before I2C communication */

  up_mdelay(200);

  /* Step 2: Initialize OV2640 sensor via I2C */

  i2c = esp32s3_i2cbus_initialize(OV2640_I2C_BUS);
  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C bus\n");
      return -ENODEV;
    }

  imgsensor = ov2640_initialize(i2c, OV2640_WIDTH, OV2640_HEIGHT);
  if (imgsensor == NULL)
    {
      snerr("ERROR: Failed to initialize OV2640\n");
      return -ENODEV;
    }

  /* Step 3: Register imgdata and imgsensor globally.
   * capture_initialize() in the camera app will create /dev/video.
   */

  imgdata_register(imgdata);

  ret = imgsensor_register(imgsensor);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register imgsensor: %d\n", ret);
      return ret;
    }

  sninfo("ESP32-S3-EYE camera initialized\n");
  return OK;
}
