// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chrono>

#include "yolov8.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"

int init_yolov8_model(const char *model_path, rknn_app_context_t *app_ctx)
{
    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    // Load RKNN Model
    model_len = read_data_from_file(model_path, &model);
    if (model == NULL)
    {
        printf("load_model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, RKNN_FLAG_ASYNC_MASK, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    // Get Model Input Output Number
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    // Get Model Input Info
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
    }

    // Get Model Output Info
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
    }

    // Set to context
    app_ctx->rknn_ctx = ctx;

    // TODO
    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type == RKNN_TENSOR_INT8)
    {
        app_ctx->is_quant = true;
    }
    else
    {
        app_ctx->is_quant = false;
    }

    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[3];
    }
    else
    {
        app_ctx->model_height = input_attrs[0].dims[1];
        app_ctx->model_width = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }

    return 0;
}

int release_yolov8_model(rknn_app_context_t *app_ctx)
{
    if (app_ctx->input_attrs != NULL)
    {
        free(app_ctx->input_attrs);
        app_ctx->input_attrs = NULL;
    }
    if (app_ctx->output_attrs != NULL)
    {
        free(app_ctx->output_attrs);
        app_ctx->output_attrs = NULL;
    }
    if (app_ctx->rknn_ctx != 0)
    {
        rknn_destroy(app_ctx->rknn_ctx);
        app_ctx->rknn_ctx = 0;
    }
    return 0;
}

int inference_yolov8_model(rknn_app_context_t *app_ctx, image_buffer_t *img, object_detect_result_list *od_results)
{
    int ret;
    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_input inputs[app_ctx->io_num.n_input];
    rknn_output outputs[app_ctx->io_num.n_output];
    const float nms_threshold = NMS_THRESH;      // 默认的NMS阈值
    const float box_conf_threshold = BOX_THRESH; // 默认的置信度阈值
    int bg_color = 114;
    double t_letterbox = 0;
    double t_inputs_set = 0;
    double t_run = 0;
    double t_outputs_get = 0;
    double t_post_process = 0;
    double t_release = 0;
    std::chrono::high_resolution_clock::time_point start_time;

    if ((!app_ctx) || !(img) || (!od_results))
    {
        return -1;
    }

    memset(od_results, 0x00, sizeof(*od_results));
    memset(&letter_box, 0, sizeof(letterbox_t));
    memset(&dst_img, 0, sizeof(image_buffer_t));
    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    // Pre Process
    dst_img.width = app_ctx->model_width;
    dst_img.height = app_ctx->model_height;
    dst_img.format = IMAGE_FORMAT_RGB888;
    dst_img.size = get_image_size(&dst_img);
    dst_img.virt_addr = (unsigned char *)malloc(dst_img.size);
    if (dst_img.virt_addr == NULL)
    {
        printf("malloc buffer size:%d fail!\n", dst_img.size);
        return -1;
    }

    // letterbox
    start_time = std::chrono::high_resolution_clock::now();
    ret = convert_image_with_letterbox(img, &dst_img, &letter_box, bg_color);
    t_letterbox = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();
    if (ret < 0)
    {
        printf("convert_image_with_letterbox fail! ret=%d\n", ret);
        return -1;
    }

    // Set Input Data
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].buf = dst_img.virt_addr;

    start_time = std::chrono::high_resolution_clock::now();
    ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
    t_inputs_set = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        return -1;
    }

    // Run
    start_time = std::chrono::high_resolution_clock::now();
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    t_run = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        return -1;
    }

    // Get Output
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < app_ctx->io_num.n_output; i++)
    {
        outputs[i].index = i;
        outputs[i].want_float = (!app_ctx->is_quant);
    }
    start_time = std::chrono::high_resolution_clock::now();
    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    t_outputs_get = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        goto out;
    }

    // Post Process
    start_time = std::chrono::high_resolution_clock::now();
    post_process(app_ctx, outputs, &letter_box, box_conf_threshold, nms_threshold, od_results);
    t_post_process = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();

    // Remeber to release rknn output
    start_time = std::chrono::high_resolution_clock::now();
    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);
    t_release = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time).count();

    // Print stats every 60 frames to avoid console flood
    static int frame_count = 0;
    // if (++frame_count % 60 == 0) {
    //     printf("[Profiling] letterbox: %.2f ms | inputs_set: %.2f ms | rknn_run: %.2f ms | outputs_get: %.2f ms | post_process: %.2f ms | release: %.2f ms\n",
    //            t_letterbox, t_inputs_set, t_run, t_outputs_get, t_post_process, t_release);
    // }

out:
    if (dst_img.virt_addr != NULL)
    {
        free(dst_img.virt_addr);
    }

    return ret;
}