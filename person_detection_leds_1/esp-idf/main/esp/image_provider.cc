
/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#include "../image_provider.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include "app_camera_esp.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <numeric>
#include <vector>

//BOF for motion detection
#define WIDTH 96
#define HEIGHT 96
#define WINDOW_SIZE 10
//hCount = (WIDTH-1) // WINDOW_SIZE + 1
//vCount = (HEIGHT-1) // WINDOW_SIZE + 1
#define HCOUNT 10
#define VCOUNT 10
#define MOVEMENT_THRESHOLD 5

uint8_t pixel_averages[HCOUNT*VCOUNT] = {0};
uint8_t arrays[HCOUNT*VCOUNT][WINDOW_SIZE*WINDOW_SIZE] = {0};
//EOF for motion detection

camera_fb_t* fb = NULL;
static const char* TAG = "app_camera";


//outputs a list of x,y pairs where the motion is happening
//x,y starting from the top left corner, I think
//compares to the previous image, not some static image taken in the beginning
std::vector<std::pair<int, int> > detect_motion(camera_fb_t *frame_buffer){
    std::vector<std::pair<int, int>> movements;

    //size_t hCount = (WIDTH-1) / WINDOW_SIZE + 1;
    //size_t vCount = (HEIGHT-1) / WINDOW_SIZE + 1;

    int sizes [HCOUNT*VCOUNT] = {0};


    for (uint32_t j = 0; j < (HEIGHT * WIDTH); j++){
        uint8_t b  = j / WIDTH / WINDOW_SIZE * HCOUNT + j % WIDTH / WINDOW_SIZE;

        arrays[b][sizes[b]] = frame_buffer->buf[j];

        sizes[b] = sizes[b] + 1;
    }


    for (uint32_t j = 0; j < HCOUNT*VCOUNT; j++){
        uint32_t sum = 0;
        for(uint32_t i = 0; i < sizes[j]; i++){
            sum += arrays[j][i];
        }
        int avg = sum / sizes[j];

        if (std::abs(pixel_averages[j] - avg) > MOVEMENT_THRESHOLD){
            //Serial.printf("change in %d %d \n", j / hCount, j % vCount);

            movements.push_back(std::make_pair(j / HCOUNT, j - int(j / HCOUNT) * HCOUNT));
        }

        pixel_averages[j] = avg;
    }

    return movements;
}


// Get the camera module ready
TfLiteStatus InitCamera(tflite::ErrorReporter* error_reporter) {
  int ret = app_camera_init();
  if (ret != 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "Camera init failed\n");
    return kTfLiteError;
  }
  TF_LITE_REPORT_ERROR(error_reporter, "Camera Initialized\n");
  return kTfLiteOk;
}

extern "C" int capture_image() {
  fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE(TAG, "Camera capture failed");
    return -1;
  }
  return 0;
}
// Begin the capture and wait for it to finish
TfLiteStatus PerformCapture(tflite::ErrorReporter* error_reporter,
                            uint8_t* image_data) {
  /* 2. Get one image with camera */
  int ret = capture_image();
  if (ret != 0) {
    return kTfLiteError;
  }
  TF_LITE_REPORT_ERROR(error_reporter, "Image Captured\n");
  memcpy(image_data, fb->buf, fb->len);
  esp_camera_fb_return(fb);
  /* here the esp camera can give you grayscale image directly */
  return kTfLiteOk;
}

// Get an image from the camera module
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width,
                      int image_height, int channels, uint8_t* image_data) {
  static bool g_is_camera_initialized = false;
  if (!g_is_camera_initialized) {
    TfLiteStatus init_status = InitCamera(error_reporter);
    if (init_status != kTfLiteOk) {
      TF_LITE_REPORT_ERROR(error_reporter, "InitCamera failed\n");
      return init_status;
    }
    g_is_camera_initialized = true;
  }
  /* Camera Captures Image of size 96 x 96  which is of the format grayscale
   * thus, no need to crop or process further , directly send it to tf */
  TfLiteStatus capture_status = PerformCapture(error_reporter, image_data);
  if (capture_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "PerformCapture failed\n");
    return capture_status;
  }
  return kTfLiteOk;
}
