/*
 * SPDX-FileCopyrightText: Copyright (c) 2018-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <gst/gst.h>
#include <glib.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <cuda_runtime_api.h>
//#include <sys/timeb.h>
#include <math.h>
//#include <iostream>
//#include <vector>
#include "gstnvdsmeta.h"
#include "nvdsmeta_schema.h"
#include "nvds_yml_parser.h"
//#include "gstnvdsmeta.h"
//#include "nvds_analytics_meta.h"
#ifndef PLATFORM_TEGRA
#include "gst-nvmessage.h"
#endif
#define MAX_DISPLAY_LEN 64
#define MAX_TIME_STAMP_LEN 32
#define TRACKER_CONFIG_FILE "ds_tracker_config.txt"
#define MAX_TRACKING_ID_LEN 16

#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

#define PGIE_CONFIG_FILE  "dstest4_pgie_config.txt"
#define MSCONV_CONFIG_FILE "dstest4_msgconv_config.txt"

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1080
static gboolean PERF_MODE = FALSE;
/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 4000000
#define IS_YAML(file) (g_str_has_suffix (file, ".yml") || g_str_has_suffix (file, ".yaml"))
#define TILED_OUTPUT_WIDTH 1920
#define TILED_OUTPUT_HEIGHT 1080
/* Check for parsing error. */
#define RETURN_ON_PARSER_ERROR(parse_expr) \
  if (NVDS_YAML_PARSER_SUCCESS !=  parse_expr) { \
    g_printerr("Error in parsing configuration file.\n"); \
    return -1; \
  }


static gchar *cfg_file = NULL;
static gchar **input_file = NULL;
static gchar *topic = NULL;
static gchar *conn_str = NULL;
static gchar *proto_lib = NULL;

static gint schema_type = 0;
static gint msg2p_meta = 0;
static gint frame_interval = 30;
static gboolean display_off = FALSE;


//2D array of input files
static gchar *input_files[16][100] = {NULL};

gint frame_number = 0;
gchar pgie_classes_str[4][32] = {"Person"
};


GOptionEntry entries[] = {
        {"cfg-file",       'c', 0,                     G_OPTION_ARG_FILENAME,     &cfg_file,
                                                                                                "Set the adaptor config file. Optional if connection string has relevant  details.",
                                                                                                                                                                     NULL},
        {"input-file",     'i', G_OPTION_FLAG_IN_MAIN, G_OPTION_ARG_STRING_ARRAY, &input_file,
                                                                                                "Set the input file. Optional if connection string has relevant details.",
                                                                                                                                                                     NULL},
        {"topic",          't', 0,                     G_OPTION_ARG_STRING,       &topic,
                                                                                                "Name of message topic. Optional if it is part of connection string or config file.",
                                                                                                                                                                     NULL},
        {"conn-str",       0,   0,                     G_OPTION_ARG_STRING,       &conn_str,
                                                                                                "Connection string of backend server. Optional if it is part of config file.",
                                                                                                                                                                     NULL},
        {"proto-lib",      'p', 0,                     G_OPTION_ARG_STRING,       &proto_lib,
                                                                                                "Absolute path of adaptor library",                                  NULL},
        {"schema",         's', 0,                     G_OPTION_ARG_INT,          &schema_type,
                                                                                                "Type of message schema (0=Full, 1=minimal, 2=protobuf), default=0", NULL},
        {"msg2p-meta",     0,   0,                     G_OPTION_ARG_INT,          &msg2p_meta,
                                                                                                "msg2payload generation metadata type (0=Event Msg meta, 1=nvds meta), default=0",
                                                                                                                                                                     NULL},
        {"frame-interval", 0,   0,                     G_OPTION_ARG_INT,          &frame_interval,
                                                                                                "Frame interval at which payload is generated , default=30",         NULL},
        {"no-display",     0,   0,                     G_OPTION_ARG_NONE,         &display_off, "Disable display",
                                                                                                                                                                     NULL},

        {NULL}
};
#define CONFIG_GROUP_TRACKER "tracker"
#define CONFIG_GROUP_TRACKER_WIDTH "tracker-width"
#define CONFIG_GROUP_TRACKER_HEIGHT "tracker-height"
#define CONFIG_GROUP_TRACKER_LL_CONFIG_FILE "ll-config-file"
#define CONFIG_GROUP_TRACKER_LL_LIB_FILE "ll-lib-file"
#define CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS "enable-batch-process"
#define CONFIG_GPU_ID "gpu-id"
#define CHECK_ERROR(error) \
    if (error) { \
        g_printerr ("Error while parsing config file: %s\n", error->message); \
        goto done; \
    }

static gchar *
get_absolute_file_path(gchar *cfg_file_path, gchar *file_path) {
    gchar abs_cfg_path[PATH_MAX + 1];
    gchar *abs_file_path;
    gchar *delim;

    if (file_path && file_path[0] == '/') {
        return file_path;
    }

    if (!realpath(cfg_file_path, abs_cfg_path)) {
        g_free(file_path);
        return NULL;
    }

    // Return absolute path of config file if file_path is NULL.
    if (!file_path) {
        abs_file_path = g_strdup(abs_cfg_path);
        return abs_file_path;
    }

    delim = g_strrstr(abs_cfg_path, "/");
    *(delim + 1) = '\0';

    abs_file_path = g_strconcat(abs_cfg_path, file_path, NULL);
    g_free(file_path);

    return abs_file_path;
}

static gboolean
set_tracker_properties(GstElement *nvtracker) {
    gboolean ret = FALSE;
    GError *error = NULL;
    gchar **keys = NULL;
    gchar **key = NULL;
    GKeyFile *key_file = g_key_file_new();

    if (!g_key_file_load_from_file(key_file, TRACKER_CONFIG_FILE, G_KEY_FILE_NONE,
                                   &error)) {
        g_printerr("Failed to load config file: %s\n", error->message);
        return FALSE;
    }

    keys = g_key_file_get_keys(key_file, CONFIG_GROUP_TRACKER, NULL, &error);
    CHECK_ERROR (error);

    for (key = keys; *key; key++) {
        if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_WIDTH)) {
            gint width =
                    g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                           CONFIG_GROUP_TRACKER_WIDTH, &error);
            CHECK_ERROR (error);
            g_object_set(G_OBJECT (nvtracker), "tracker-width", width, NULL);
        } else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_HEIGHT)) {
            gint height =
                    g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                           CONFIG_GROUP_TRACKER_HEIGHT, &error);
            CHECK_ERROR (error);
            g_object_set(G_OBJECT (nvtracker), "tracker-height", height, NULL);
        } else if (!g_strcmp0(*key, CONFIG_GPU_ID)) {
            guint gpu_id =
                    g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                           CONFIG_GPU_ID, &error);
            CHECK_ERROR (error);
            g_object_set(G_OBJECT (nvtracker), "gpu_id", gpu_id, NULL);
        } else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_LL_CONFIG_FILE)) {
            char *ll_config_file = get_absolute_file_path(TRACKER_CONFIG_FILE,
                                                          g_key_file_get_string(key_file,
                                                                                CONFIG_GROUP_TRACKER,
                                                                                CONFIG_GROUP_TRACKER_LL_CONFIG_FILE,
                                                                                &error));
            CHECK_ERROR (error);
            g_object_set(G_OBJECT (nvtracker), "ll-config-file", ll_config_file, NULL);
        } else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_LL_LIB_FILE)) {
            char *ll_lib_file = get_absolute_file_path(TRACKER_CONFIG_FILE,
                                                       g_key_file_get_string(key_file,
                                                                             CONFIG_GROUP_TRACKER,
                                                                             CONFIG_GROUP_TRACKER_LL_LIB_FILE, &error));
            CHECK_ERROR (error);
            g_object_set(G_OBJECT (nvtracker), "ll-lib-file", ll_lib_file, NULL);
        } else if (!g_strcmp0(*key, CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS)) {
            gboolean enable_batch_process =
                    g_key_file_get_integer(key_file, CONFIG_GROUP_TRACKER,
                                           CONFIG_GROUP_TRACKER_ENABLE_BATCH_PROCESS, &error);
            CHECK_ERROR (error);
            g_object_set(G_OBJECT (nvtracker), "enable_batch_process",
                         enable_batch_process, NULL);
        } else {
            g_printerr("Unknown key '%s' for group [%s]", *key,
                       CONFIG_GROUP_TRACKER);
        }
    }

    ret = TRUE;
    done:
    if (error) {
        g_error_free(error);
    }
    if (keys) {
        g_strfreev(keys);
    }
    if (!ret) {
        g_printerr("%s failed", __func__);
    }
    return ret;
}

static void
generate_ts_rfc3339(char *buf, int buf_size) {
    time_t tloc;
    struct tm tm_log;
    struct timespec ts;
    char strmsec[6];              //.nnnZ\0

    clock_gettime(CLOCK_REALTIME, &ts);
    memcpy(&tloc, (void *) (&ts.tv_sec), sizeof(time_t));
    gmtime_r(&tloc, &tm_log);
    strftime(buf, buf_size, "%Y-%m-%dT%H:%M:%S", &tm_log);
    int ms = ts.tv_nsec / 1000000;
    g_snprintf(strmsec, sizeof(strmsec), ".%.3dZ", ms);
    strncat(buf, strmsec, buf_size);
}

/* nvdsanalytics_src_pad_buffer_probe  will extract metadata received on tiler sink pad
 * and extract nvanalytics metadata etc. */
//static GstPadProbeReturn
//nvdsanalytics_src_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
//                                    gpointer u_data)
//{
//    GstBuffer *buf = (GstBuffer *) info->data;
//    guint num_rects = 0;
//    NvDsObjectMeta *obj_meta = NULL;
//    guint vehicle_count = 0;
//    guint person_count = 0;
//    NvDsMetaList * l_frame = NULL;
//    NvDsMetaList * l_obj = NULL;
//    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);
//    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
//         l_frame = l_frame->next) {
//        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
//        std::stringstream out_string;
//        vehicle_count = 0;
//        num_rects = 0;
//        person_count = 0;
//        for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
//             l_obj = l_obj->next) {
//            obj_meta = (NvDsObjectMeta *) (l_obj->data);
//            if (obj_meta->class_id == PGIE_CLASS_ID_VEHICLE) {
//                vehicle_count++;
//                num_rects++;
//            }
//            if (obj_meta->class_id == PGIE_CLASS_ID_PERSON) {
//                person_count++;
//                num_rects++;
//            }
//
//            // Access attached user meta for each object
//            for (NvDsMetaList *l_user_meta = obj_meta->obj_user_meta_list; l_user_meta != NULL;
//                 l_user_meta = l_user_meta->next) {
//                NvDsUserMeta *user_meta = (NvDsUserMeta *) (l_user_meta->data);
//                if(user_meta->base_meta.meta_type == NVDS_USER_OBJ_META_NVDSANALYTICS)
//                {
//                    NvDsAnalyticsObjInfo * user_meta_data = (NvDsAnalyticsObjInfo *)user_meta->user_meta_data;
//                    if (user_meta_data->dirStatus.length()){
//                        g_print ("object %lu moving in %s\n", obj_meta->object_id, user_meta_data->dirStatus.c_str());
//                    }
//                }
//            }
//        }
//
//        /* Iterate user metadata in frames to search analytics metadata */
//        for (NvDsMetaList * l_user = frame_meta->frame_user_meta_list;
//             l_user != NULL; l_user = l_user->next) {
//            NvDsUserMeta *user_meta = (NvDsUserMeta *) l_user->data;
//            if (user_meta->base_meta.meta_type != NVDS_USER_FRAME_META_NVDSANALYTICS)
//                continue;
//
//            /* convert to  metadata */
//            NvDsAnalyticsFrameMeta *meta =
//                    (NvDsAnalyticsFrameMeta *) user_meta->user_meta_data;
//            /* Get the labels from nvdsanalytics config file */
//            for (std::pair<std::string, uint32_t> status : meta->objInROIcnt){
//                out_string << "Objs in ROI ";
//                out_string << status.first;
//                out_string << " = ";
//                out_string << status.second;
//            }
//            for (std::pair<std::string, uint32_t> status : meta->objLCCumCnt){
//                out_string << " LineCrossing Cumulative ";
//                out_string << status.first;
//                out_string << " = ";
//                out_string << status.second;
//            }
//            for (std::pair<std::string, uint32_t> status : meta->objLCCurrCnt){
//                out_string << " LineCrossing Current Frame ";
//                out_string << status.first;
//                out_string << " = ";
//                out_string << status.second;
//            }
//            for (std::pair<std::string, bool> status : meta->ocStatus){
//                out_string << " Overcrowding status ";
//                out_string << status.first;
//                out_string << " = ";
//                out_string << status.second;
//            }
//        }
//        g_print ("Frame Number = %d of Stream = %d, Number of objects = %d "
//                 "Vehicle Count = %d Person Count = %d %s\n",
//                 frame_meta->frame_num, frame_meta->pad_index,
//                 num_rects, vehicle_count, person_count, out_string.str().c_str());
//    }
//    return GST_PAD_PROBE_OK;
//}

static gpointer
meta_copy_func(gpointer data, gpointer user_data) {
    NvDsUserMeta *user_meta = (NvDsUserMeta *) data;
    NvDsEventMsgMeta *srcMeta = (NvDsEventMsgMeta *) user_meta->user_meta_data;
    NvDsEventMsgMeta *dstMeta = NULL;

    dstMeta = g_memdup(srcMeta, sizeof(NvDsEventMsgMeta));

    if (srcMeta->ts)
        dstMeta->ts = g_strdup(srcMeta->ts);

    if (srcMeta->sensorStr)
        dstMeta->sensorStr = g_strdup(srcMeta->sensorStr);

    if (srcMeta->objSignature.size > 0) {
        dstMeta->objSignature.signature = g_memdup(srcMeta->objSignature.signature,
                                                   srcMeta->objSignature.size);
        dstMeta->objSignature.size = srcMeta->objSignature.size;
    }

    if (srcMeta->objectId) {
        dstMeta->objectId = g_strdup(srcMeta->objectId);
    }

    if (srcMeta->extMsgSize > 0) {
        if (srcMeta->objType == NVDS_OBJECT_TYPE_VEHICLE) {
            NvDsVehicleObject *srcObj = (NvDsVehicleObject *) srcMeta->extMsg;
            NvDsVehicleObject *obj =
                    (NvDsVehicleObject *) g_malloc0(sizeof(NvDsVehicleObject));
            if (srcObj->type)
                obj->type = g_strdup(srcObj->type);
            if (srcObj->make)
                obj->make = g_strdup(srcObj->make);
            if (srcObj->model)
                obj->model = g_strdup(srcObj->model);
            if (srcObj->color)
                obj->color = g_strdup(srcObj->color);
            if (srcObj->license)
                obj->license = g_strdup(srcObj->license);
            if (srcObj->region)
                obj->region = g_strdup(srcObj->region);

            dstMeta->extMsg = obj;
            dstMeta->extMsgSize = sizeof(NvDsVehicleObject);
        } else if (srcMeta->objType == NVDS_OBJECT_TYPE_PERSON) {
            NvDsPersonObject *srcObj = (NvDsPersonObject *) srcMeta->extMsg;
            NvDsPersonObject *obj =
                    (NvDsPersonObject *) g_malloc0(sizeof(NvDsPersonObject));

            obj->age = srcObj->age;

            if (srcObj->gender)
                obj->gender = g_strdup(srcObj->gender);
            if (srcObj->cap)
                obj->cap = g_strdup(srcObj->cap);
            if (srcObj->hair)
                obj->hair = g_strdup(srcObj->hair);
            if (srcObj->apparel)
                obj->apparel = g_strdup(srcObj->apparel);

            dstMeta->extMsg = obj;
            dstMeta->extMsgSize = sizeof(NvDsPersonObject);
        }
    }
    return dstMeta;
}

static void
meta_free_func(gpointer data, gpointer user_data) {
    NvDsUserMeta *user_meta = (NvDsUserMeta *) data;
    NvDsEventMsgMeta *srcMeta = (NvDsEventMsgMeta *) user_meta->user_meta_data;

    g_free(srcMeta->ts);
    g_free(srcMeta->sensorStr);

    if (srcMeta->objSignature.size > 0) {
        g_free(srcMeta->objSignature.signature);
        srcMeta->objSignature.size = 0;
    }

    if (srcMeta->objectId) {
        g_free(srcMeta->objectId);
    }

    if (srcMeta->extMsgSize > 0) {
        if (srcMeta->objType == NVDS_OBJECT_TYPE_VEHICLE) {
            NvDsVehicleObject *obj = (NvDsVehicleObject *) srcMeta->extMsg;
            if (obj->type)
                g_free(obj->type);
            if (obj->color)
                g_free(obj->color);
            if (obj->make)
                g_free(obj->make);
            if (obj->model)
                g_free(obj->model);
            if (obj->license)
                g_free(obj->license);
            if (obj->region)
                g_free(obj->region);
        } else if (srcMeta->objType == NVDS_OBJECT_TYPE_PERSON) {
            NvDsPersonObject *obj = (NvDsPersonObject *) srcMeta->extMsg;

            if (obj->gender)
                g_free(obj->gender);
            if (obj->cap)
                g_free(obj->cap);
            if (obj->hair)
                g_free(obj->hair);
            if (obj->apparel)
                g_free(obj->apparel);
        }
        g_free(srcMeta->extMsg);
        srcMeta->extMsgSize = 0;
    }
    g_free(user_meta->user_meta_data);
    user_meta->user_meta_data = NULL;
}

static void
generate_vehicle_meta(gpointer data) {
    NvDsVehicleObject *obj = (NvDsVehicleObject *) data;

    obj->type = g_strdup("sedan");
    obj->color = g_strdup("blue");
    obj->make = g_strdup("Bugatti");
    obj->model = g_strdup("M");
    obj->license = g_strdup("XX1234");
    obj->region = g_strdup("CA");
}

static void
generate_person_meta(gpointer data) {
    NvDsPersonObject *obj = (NvDsPersonObject *) data;
    obj->age = 45;
    obj->cap = g_strdup("none");
    obj->hair = g_strdup("black");
    obj->gender = g_strdup("male");
    obj->apparel = g_strdup("formal");
}

static void
generate_event_msg_meta(gpointer data, gint class_id,
                        NvDsObjectMeta *obj_params) {
    NvDsEventMsgMeta *meta = (NvDsEventMsgMeta *) data;
    meta->sensorId = 0;
    meta->placeId = 0;
    meta->moduleId = 0;
    meta->sensorStr = g_strdup("sensor-0");

    meta->ts = (gchar *) g_malloc0(MAX_TIME_STAMP_LEN + 1);
    meta->objectId = (gchar *) g_malloc0(MAX_LABEL_SIZE);

    strncpy(meta->objectId, obj_params->obj_label, MAX_LABEL_SIZE);

    generate_ts_rfc3339(meta->ts, MAX_TIME_STAMP_LEN);

    /*
     * This demonstrates how to attach custom objects.
     * Any custom object as per requirement can be generated and attached
     * like NvDsVehicleObject / NvDsPersonObject. Then that object should
     * be handled in payload generator library (nvmsgconv.cpp) accordingly.
     */
    if (class_id == PGIE_CLASS_ID_VEHICLE) {
        meta->type = NVDS_EVENT_MOVING;
        meta->objType = NVDS_OBJECT_TYPE_VEHICLE;
        meta->objClassId = PGIE_CLASS_ID_VEHICLE;

        NvDsVehicleObject *obj =
                (NvDsVehicleObject *) g_malloc0(sizeof(NvDsVehicleObject));
        generate_vehicle_meta(obj);

        meta->extMsg = obj;
        meta->extMsgSize = sizeof(NvDsVehicleObject);
    } else if (class_id == PGIE_CLASS_ID_PERSON) {
        meta->type = NVDS_EVENT_ENTRY;
        meta->objType = NVDS_OBJECT_TYPE_PERSON;
        meta->objClassId = PGIE_CLASS_ID_PERSON;

        NvDsPersonObject *obj =
                (NvDsPersonObject *) g_malloc0(sizeof(NvDsPersonObject));
        generate_person_meta(obj);

        meta->extMsg = obj;
        meta->extMsgSize = sizeof(NvDsPersonObject);
    }
}


/* NVIDIA Decoder source pad memory feature. This feature signifies that source
 * pads having this capability will push GstBuffers containing cuda buffers. */
#define GST_CAPS_FEATURES_NVMM "memory:NVMM"

/* osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */

static GstPadProbeReturn
osd_sink_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info,
                          gpointer u_data) {
    GstBuffer *buf = (GstBuffer *) info->data;
    NvDsFrameMeta *frame_meta = NULL;
    NvOSD_TextParams *txt_params = NULL;
    guint vehicle_count = 0;
    guint person_count = 0;
    gboolean is_first_object = TRUE;
    NvDsMetaList *l_frame, *l_obj;

    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    if (!batch_meta) {
        // No batch meta attached.
        return GST_PAD_PROBE_OK;
    }

    for (l_frame = batch_meta->frame_meta_list; l_frame; l_frame = l_frame->next) {
        frame_meta = (NvDsFrameMeta *) l_frame->data;

        if (frame_meta == NULL) {
            // Ignore Null frame meta.
            continue;
        }

        is_first_object = TRUE;

        for (l_obj = frame_meta->obj_meta_list; l_obj; l_obj = l_obj->next) {
            NvDsObjectMeta *obj_meta = (NvDsObjectMeta *) l_obj->data;

            if (obj_meta == NULL) {
                // Ignore Null object.
                continue;
            }

            txt_params = &(obj_meta->text_params);
            if (txt_params->display_text)
                g_free(txt_params->display_text);

            txt_params->display_text = g_malloc0(MAX_DISPLAY_LEN);

            g_snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "%s ",
                       pgie_classes_str[obj_meta->class_id]);

            if (obj_meta->class_id == PGIE_CLASS_ID_VEHICLE)
                vehicle_count++;
            if (obj_meta->class_id == PGIE_CLASS_ID_PERSON)
                person_count++;

            /* Now set the offsets where the string should appear */
            txt_params->x_offset = obj_meta->rect_params.left;
            txt_params->y_offset = obj_meta->rect_params.top - 25;

            /* Font , font-color and font-size */
            txt_params->font_params.font_name = "Serif";
            txt_params->font_params.font_size = 10;
            txt_params->font_params.font_color.red = 1.0;
            txt_params->font_params.font_color.green = 1.0;
            txt_params->font_params.font_color.blue = 1.0;
            txt_params->font_params.font_color.alpha = 1.0;

            /* Text background color */
            txt_params->set_bg_clr = 1;
            txt_params->text_bg_clr.red = 0.0;
            txt_params->text_bg_clr.green = 0.0;
            txt_params->text_bg_clr.blue = 0.0;
            txt_params->text_bg_clr.alpha = 1.0;

            /*
             * Ideally NVDS_EVENT_MSG_META should be attached to buffer by the
             * component implementing detection / recognition logic.
             * Here it demonstrates how to use / attach that meta data.
             */
            if (is_first_object && !(frame_number % frame_interval)) {
                /* Frequency of messages to be send will be based on use case.
                 * Here message is being sent for first object every frame_interval(default=30).
                 */

                NvDsEventMsgMeta *msg_meta =
                        (NvDsEventMsgMeta *) g_malloc0(sizeof(NvDsEventMsgMeta));
                msg_meta->bbox.top = obj_meta->rect_params.top;
                msg_meta->bbox.left = obj_meta->rect_params.left;
                msg_meta->bbox.width = obj_meta->rect_params.width;
                msg_meta->bbox.height = obj_meta->rect_params.height;
                msg_meta->frameId = frame_number;
                msg_meta->trackingId = obj_meta->object_id;
                msg_meta->confidence = obj_meta->confidence;
                generate_event_msg_meta(msg_meta, obj_meta->class_id, obj_meta);

                NvDsUserMeta *user_event_meta =
                        nvds_acquire_user_meta_from_pool(batch_meta);
                if (user_event_meta) {
                    user_event_meta->user_meta_data = (void *) msg_meta;
                    user_event_meta->base_meta.meta_type = NVDS_EVENT_MSG_META;
                    user_event_meta->base_meta.copy_func =
                            (NvDsMetaCopyFunc) meta_copy_func;
                    user_event_meta->base_meta.release_func =
                            (NvDsMetaReleaseFunc) meta_free_func;
                    nvds_add_user_meta_to_frame(frame_meta, user_event_meta);
                } else {
                    g_print("Error in attaching event meta to buffer\n");
                }
                is_first_object = FALSE;
            }
        }
    }
    g_print("Frame Number = %d "
            "Vehicle Count = %d Person Count = %d\n",
            frame_number, vehicle_count, person_count);
    frame_number++;

    return GST_PAD_PROBE_OK;
}

static void
cb_newpad(GstElement *decodebin, GstPad *decoder_src_pad, gpointer data) {
    GstCaps *caps = gst_pad_get_current_caps(decoder_src_pad);
    if (!caps) {
        caps = gst_pad_query_caps(decoder_src_pad, NULL);
    }
    const GstStructure *str = gst_caps_get_structure(caps, 0);
    const gchar *name = gst_structure_get_name(str);
    GstElement *source_bin = (GstElement *) data;
    GstCapsFeatures *features = gst_caps_get_features(caps, 0);

    /* Need to check if the pad created by the decodebin is for video and not
     * audio. */
    if (!strncmp(name, "video", 5)) {
        /* Link the decodebin pad only if decodebin has picked nvidia
         * decoder plugin nvdec_*. We do this by checking if the pad caps contain
         * NVMM memory features. */
        if (gst_caps_features_contains(features, GST_CAPS_FEATURES_NVMM)) {
            /* Get the source bin ghost pad */
            GstPad *bin_ghost_pad = gst_element_get_static_pad(source_bin, "src");
            if (!gst_ghost_pad_set_target(GST_GHOST_PAD (bin_ghost_pad),
                                          decoder_src_pad)) {
                g_printerr("Failed to link decoder src pad to source bin ghost pad\n");
            }
            gst_object_unref(bin_ghost_pad);
        } else {
            g_printerr("Error: Decodebin did not pick nvidia decoder plugin.\n");
        }
    }
}

static void
decodebin_child_added(GstChildProxy *child_proxy, GObject *object,
                      gchar *name, gpointer user_data) {
    g_print("Decodebin child added: %s\n", name);
    if (g_strrstr(name, "decodebin") == name) {
        g_signal_connect (G_OBJECT(object), "child-added",
                          G_CALLBACK(decodebin_child_added), user_data);
    }
    if (g_strrstr(name, "source") == name) {
        g_object_set(G_OBJECT(object), "drop-on-latency", true, NULL);
    }

}

static GstElement *
create_source_bin(guint index, gchar *uri) {
    GstElement *bin = NULL, *uri_decode_bin = NULL;
    gchar bin_name[16] = {};

    g_snprintf(bin_name, 15, "source-bin-%02d", index);
    /* Create a source GstBin to abstract this bin's content from the rest of the
     * pipeline */
    bin = gst_bin_new(bin_name);

    /* Source element for reading from the uri.
     * We will use decodebin and let it figure out the container format of the
     * stream and the codec and plug the appropriate demux and decode plugins. */
    if (PERF_MODE) {
        uri_decode_bin = gst_element_factory_make("nvurisrcbin", "uri-decode-bin");
        g_object_set(G_OBJECT (uri_decode_bin), "file-loop", TRUE, NULL);
        g_object_set(G_OBJECT (uri_decode_bin), "cudadec-memtype", 0, NULL);
    } else {
        uri_decode_bin = gst_element_factory_make("uridecodebin", "uri-decode-bin");
    }

    if (!bin || !uri_decode_bin) {
        g_printerr("One element in source bin could not be created.\n");
        return NULL;
    }

    /* We set the input uri to the source element */
    g_object_set(G_OBJECT (uri_decode_bin), "uri", uri, NULL);

    /* Connect to the "pad-added" signal of the decodebin which generates a
     * callback once a new pad for raw data has beed created by the decodebin */
    g_signal_connect (G_OBJECT(uri_decode_bin), "pad-added",
                      G_CALLBACK(cb_newpad), bin);
    g_signal_connect (G_OBJECT(uri_decode_bin), "child-added",
                      G_CALLBACK(decodebin_child_added), bin);

    gst_bin_add(GST_BIN (bin), uri_decode_bin);

    /* We need to create a ghost pad for the source bin which will act as a proxy
     * for the video decoder src pad. The ghost pad will not have a target right
     * now. Once the decode bin creates the video decoder and generates the
     * cb_newpad callback, we will set the ghost pad target to the video decoder
     * src pad. */
    if (!gst_element_add_pad(bin, gst_ghost_pad_new_no_target("src",
                                                              GST_PAD_SRC))) {
        g_printerr("Failed to add ghost pad in source bin\n");
        return NULL;
    }

    return bin;
}

static gboolean
bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
    GMainLoop *loop = (GMainLoop *) data;
    switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR: {
            gchar *debug;
            GError *error;
            gst_message_parse_error(msg, &error, &debug);
            g_printerr("ERROR from element %s: %s\n",
                       GST_OBJECT_NAME (msg->src), error->message);
            if (debug)
                g_printerr("Error details: %s\n", debug);
            g_free(debug);
            g_error_free(error);
            g_main_loop_quit(loop);
            break;
        }
        default:
            break;
    }
    return TRUE;
}

int
main(int argc, char *argv[]) {
    GMainLoop *loop = NULL;
    GstElement *pipeline = NULL, *source = NULL, *h264parser = NULL,
            *decoder = NULL, *sink = NULL, *tiler = NULL, *pgie = NULL, *tracker = NULL, *nvvidconv = NULL,
            *nvdsanalytics = NULL,
            *nvosd = NULL, *nvstreammux;
    GstElement *msgconv = NULL, *msgbroker = NULL, *tee = NULL;
    GstElement *queue1 = NULL, *queue2 = NULL;
    GstBus *bus = NULL;
    guint bus_watch_id;
    GstPad *osd_sink_pad = NULL;
    GstPad *tee_render_pad = NULL;
    GstPad *tee_msg_pad = NULL;
    GstPad *sink_pad = NULL;
    GstPad *src_pad = NULL;
    GOptionContext *ctx = NULL;
    GOptionGroup *group = NULL;
    GError *error = NULL;
    NvDsGieType pgie_type = NVDS_GIE_PLUGIN_INFER;
    guint i = 0, num_sources = 0;
    guint tiler_rows, tiler_columns;
    int current_device = -1;
    cudaGetDevice(&current_device);
    struct cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, current_device);

    ctx = g_option_context_new("Nvidia DeepStream Test4");
    group = g_option_group_new("test4", NULL, NULL, NULL, NULL);
    g_option_group_add_entries(group, entries);

    g_option_context_set_main_group(ctx, group);
    g_option_context_add_group(ctx, gst_init_get_option_group());


    if (!g_option_context_parse(ctx, &argc, &argv, &error)) {
        g_option_context_free(ctx);
        g_printerr("%s", error->message);
        return -1;
    }
    g_option_context_free(ctx);

    if (!proto_lib || !input_file) {
        if (argc > 1 && !IS_YAML (argv[1])) {
            g_printerr("missing arguments\n");
            g_printerr("Usage: %s <yml file>\n", argv[0]);
            g_printerr
                    ("Usage: %s -i <H264 filename> -p <Proto adaptor library> --conn-str=<Connection string>\n",
                     argv[0]);
            return -1;
        } else if (!argv[1]) {
            g_printerr("missing arguments\n");
            g_printerr("Usage: %s <yml file>\n", argv[0]);
            g_printerr
                    ("Usage: %s -i <H264 filename> -p <Proto adaptor library> --conn-str=<Connection string>\n",
                     argv[0]);
            return -1;
        }
    }

    loop = g_main_loop_new(NULL, FALSE);



    /* Create gstreamer elements */
    /* Create Pipeline element that will form a connection of other elements */
    pipeline = gst_pipeline_new("dstest4-pipeline");

    /* Create nvstreammux instance to form batches from one or more sources. */
    nvstreammux = gst_element_factory_make("nvstreammux", "stream-muxer");

    if (!pipeline || !nvstreammux) {
        g_printerr("One of 2 element could not be created. Exiting.\n");
        return -1;
    }
    gst_bin_add(GST_BIN (pipeline), nvstreammux);



//assing number of sources from input file list
    num_sources = g_strv_length(input_file);
    g_print("num_sources = %d", num_sources);
    for (i = 0; i < num_sources; i++) {
        GstPad *sinkpad, *srcpad;
        gchar pad_name[16] = {};
        GstElement *source_bin = NULL;
        source_bin = create_source_bin(i, input_file[i]);
        g_print("input_file = %s", input_file[i]);
        if (!source_bin) {
            g_printerr("Failed to create source bin. Exiting.\n");
            return -1;
        }

        gst_bin_add(GST_BIN (pipeline), source_bin);

        g_snprintf(pad_name, 15, "sink_%u", i);
        sinkpad = gst_element_get_request_pad(nvstreammux, pad_name);
        if (!sinkpad) {
            g_printerr("Streammux request sink pad failed. Exiting.\n");
            return -1;
        }

        srcpad = gst_element_get_static_pad(source_bin, "src");
        if (!srcpad) {
            g_printerr("Failed to get src pad of source bin. Exiting.\n");
            return -1;
        }

        if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK) {
            g_printerr("Failed to link source bin to stream muxer. Exiting.\n");
            return -1;
        }
        gst_object_unref(srcpad);
        gst_object_unref(sinkpad);
    }

    /* Use nvinfer or nvinferserver to run inferencing on decoder's output,
     * behaviour of inferencing is set through config file */
    if (pgie_type == NVDS_GIE_PLUGIN_INFER_SERVER) {
        pgie = gst_element_factory_make("nvinferserver", "primary-nvinference-engine");
    } else {
        pgie = gst_element_factory_make("nvinfer", "primary-nvinference-engine");
    };

    tracker = gst_element_factory_make("nvtracker", "tracker");

    /* Use nvdsanalytics to perform analytics on object */
    nvdsanalytics = gst_element_factory_make ("nvdsanalytics", "nvdsanalytics");

    /* Use convertor to convert from NV12 to RGBA as required by nvosd */
    nvvidconv = gst_element_factory_make("nvvideoconvert", "nvvideo-converter");

    /* Create OSD to draw on the converted RGBA buffer */
    nvosd = gst_element_factory_make("nvdsosd", "nv-onscreendisplay");


    /* Create msg converter to generate payload from buffer metadata */
    msgconv = gst_element_factory_make("nvmsgconv", "nvmsg-converter");

    /* Create msg broker to send payload to server */
    msgbroker = gst_element_factory_make("nvmsgbroker", "nvmsg-broker");

    /* Create tee to render buffer and send message simultaneously */
    tee = gst_element_factory_make("tee", "nvsink-tee");

    /* Create queues */
    queue1 = gst_element_factory_make("queue", "nvtee-que1");
    queue2 = gst_element_factory_make("queue", "nvtee-que2");

    tiler = gst_element_factory_make("nvmultistreamtiler", "nvtiler");

    /* Finally render the osd output */
    if (display_off) {
        sink = gst_element_factory_make("fakesink", "nvvideo-renderer");
    } else if (prop.integrated) {
        sink = gst_element_factory_make("nv3dsink", "nv3d-sink");
    } else {
        sink = gst_element_factory_make("nveglglessink", "nvvideo-renderer");
    }

    if (!pgie || !tracker|| !nvdsanalytics|| !tiler
        || !nvvidconv || !nvosd || !msgconv || !msgbroker || !tee
        || !queue1 || !queue2 || !sink) {
        g_printerr("One element could not be created. Exiting.\n");
        return -1;
    }


    if (argc > 1 && IS_YAML (argv[1])) {
        RETURN_ON_PARSER_ERROR(nvds_parse_file_source(source, argv[1], "source"));
        RETURN_ON_PARSER_ERROR(nvds_parse_streammux(nvstreammux, argv[1], "streammux"));

        RETURN_ON_PARSER_ERROR(nvds_parse_gie(pgie, argv[1], "primary-gie"));

        g_object_set(G_OBJECT (msgconv), "config", "dstest4_msgconv_config.yml",
                     NULL);
        RETURN_ON_PARSER_ERROR(nvds_parse_msgconv(msgconv, argv[1], "msgconv"));

        RETURN_ON_PARSER_ERROR(nvds_parse_msgbroker(msgbroker, argv[1], "msgbroker"));

        if (display_off) {
            RETURN_ON_PARSER_ERROR(nvds_parse_fake_sink(sink, argv[1], "sink"));
        } else if (prop.integrated) {
            RETURN_ON_PARSER_ERROR(nvds_parse_3d_sink(sink, argv[1], "sink"));
        } else {
            RETURN_ON_PARSER_ERROR(nvds_parse_egl_sink(sink, argv[1], "sink"));
        }

    } else {
        /* we set the input filename to the source element */
//    g_object_set (G_OBJECT (source), "location", input_file, NULL);

        g_object_set(G_OBJECT (nvstreammux), "batch-size", num_sources, NULL);

        g_object_set(G_OBJECT (nvstreammux), "width", MUXER_OUTPUT_WIDTH, "height",
                     MUXER_OUTPUT_HEIGHT,
                     "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, NULL);


        /* Set all the necessary properties of the nvinfer element,
         * the necessary ones are : */
        g_object_set(G_OBJECT (pgie), "config-file-path", PGIE_CONFIG_FILE, NULL);
        g_object_get(G_OBJECT (pgie), "batch-size", &num_sources, NULL);
        g_object_set (G_OBJECT (nvdsanalytics),
                      "config-file", "config_nvdsanalytics.txt",
                      NULL);

        tiler_rows = (guint) sqrt (num_sources);
        tiler_columns = (guint) ceil (1.0 * num_sources / tiler_rows);
        /* we set the tiler properties here */
        g_object_set (G_OBJECT (tiler), "rows", tiler_rows, "columns", tiler_columns,
                      "width", TILED_OUTPUT_WIDTH, "height", TILED_OUTPUT_HEIGHT, NULL);

        g_object_set(G_OBJECT (sink), "qos", 0, NULL);


        if (!set_tracker_properties(tracker)) {
            g_printerr("Failed to set tracker properties. Exiting.\n");
            return -1;
        }
        g_object_set(G_OBJECT (msgconv), "config", MSCONV_CONFIG_FILE, NULL);
        g_object_set(G_OBJECT (msgconv), "payload-type", schema_type, NULL);
        g_object_set(G_OBJECT (msgconv), "msg2p-newapi", msg2p_meta, NULL);
        g_object_set(G_OBJECT (msgconv), "frame-interval", frame_interval, NULL);

        g_object_set(G_OBJECT (msgbroker), "proto-lib", proto_lib,
                     "conn-str", conn_str, "sync", FALSE, NULL);

        if (topic) {
            g_object_set(G_OBJECT (msgbroker), "topic", topic, NULL);
        }

        if (cfg_file) {
            g_object_set(G_OBJECT (msgbroker), "config", cfg_file, NULL);
        }

        g_object_set(G_OBJECT (sink), "sync", TRUE, NULL);
    }
    /* we add a message handler */
    bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));
    bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    /* Set up the pipeline */
    /* we add all elements into the pipeline */
    gst_bin_add_many(GST_BIN (pipeline),
                     pgie, tracker, nvdsanalytics, tiler,
                     nvvidconv, nvosd, tee, queue1, queue2, msgconv, msgbroker, sink, NULL);

    /* we link the elements together */
    /* file-source -> h264-parser -> nvh264-decoder -> nvstreammux ->
     * pgie -> tracker -> nvdsanalytics ->tiler-> nvvidconv  -> nvosd -> tee -> video-renderer
     *                                                                      |
     *                                                                      |-> msgconv -> msgbroker  */

    if (!gst_element_link_many(nvstreammux, pgie, tracker, nvdsanalytics, tiler, nvvidconv, nvosd, tee, NULL)) {
        g_printerr("Elements could not be linked. Exiting.\n");
        return -1;
    }

    if (!gst_element_link_many(queue1, msgconv, msgbroker, NULL)) {
        g_printerr("Elements could not be linked. Exiting.\n");
        return -1;
    }

    if (!gst_element_link(queue2, sink)) {
        g_printerr("Elements could not be linked. Exiting.\n");
        return -1;
    }

    sink_pad = gst_element_get_static_pad(queue1, "sink");
    tee_msg_pad = gst_element_get_request_pad(tee, "src_%u");
    tee_render_pad = gst_element_get_request_pad(tee, "src_%u");
    if (!tee_msg_pad || !tee_render_pad) {
        g_printerr("Unable to get request pads\n");
        return -1;
    }

    if (gst_pad_link(tee_msg_pad, sink_pad) != GST_PAD_LINK_OK) {
        g_printerr("Unable to link tee and message converter\n");
        gst_object_unref(sink_pad);
        return -1;
    }

    gst_object_unref(sink_pad);

    sink_pad = gst_element_get_static_pad(queue2, "sink");
    if (gst_pad_link(tee_render_pad, sink_pad) != GST_PAD_LINK_OK) {
        g_printerr("Unable to link tee and render\n");
        gst_object_unref(sink_pad);
        return -1;
    }

    gst_object_unref(sink_pad);

    /* Lets add probe to get informed of the meta data generated, we add probe to
     * the sink pad of the osd element, since by that time, the buffer would have
     * had got all the metadata. */
    osd_sink_pad = gst_element_get_static_pad(nvosd, "sink");
    if (!osd_sink_pad)
        g_print("Unable to get sink pad\n");
    else {
        if (msg2p_meta == 0)        //generate payload using eventMsgMeta
            gst_pad_add_probe(osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                              osd_sink_pad_buffer_probe, NULL, NULL);
    }
    gst_object_unref(osd_sink_pad);

    /* Set the pipeline to "playing" state */
    if (argc > 1 && IS_YAML (argv[1])) {
        g_print("Using file: %s\n", argv[1]);
    } else {
        g_print ("Now playing:");
        for (i = 0; i < num_sources; i++) {
            g_print (" %s,", input_file[i]);
        }
        g_print ("\n");
    }
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    /* Wait till pipeline encounters an error or EOS */
    g_print("Running...\n");
    g_main_loop_run(loop);

    /* Out of the main loop, clean up nicely */
    g_print("Returned, stopping playback\n");

    g_free(cfg_file);
    g_free(input_file);
    g_free(topic);
    g_free(conn_str);
    g_free(proto_lib);

    /* Release the request pads from the tee, and unref them */
    gst_element_release_request_pad(tee, tee_msg_pad);
    gst_element_release_request_pad(tee, tee_render_pad);
    gst_object_unref(tee_msg_pad);
    gst_object_unref(tee_render_pad);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    g_print("Deleting pipeline\n");
    gst_object_unref(GST_OBJECT (pipeline));
    g_source_remove(bus_watch_id);
    g_main_loop_unref(loop);
    return 0;
}
