#define _GNU_SOURCE
#include <gst/gst.h>
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef GstElement *(*gst_parse_launch_fn)(const gchar *, GError **);

static gst_parse_launch_fn resolve_real(void) {
    void *handle = dlopen("libgstreamer-1.0.so.0", RTLD_NOW | RTLD_GLOBAL);
    if (handle == NULL) {
        fprintf(stderr, "[scam-gst-compat] dlopen gstreamer failed: %s\n", dlerror());
        return NULL;
    }
    dlerror();
    gst_parse_launch_fn fn = (gst_parse_launch_fn)dlsym(handle, "gst_parse_launch");
    const char *err = dlerror();
    if (err != NULL || fn == NULL) {
        fprintf(stderr, "[scam-gst-compat] dlsym gst_parse_launch failed: %s\n",
                err != NULL ? err : "null");
        return NULL;
    }
    return fn;
}

__attribute__((constructor))
static void compat_loaded(void) {
    fprintf(stderr, "[scam-gst-compat] loaded\n");
}

GstElement *gst_parse_launch(const gchar *description, GError **error) {
    static gst_parse_launch_fn real_fn = NULL;
    if (real_fn == NULL) {
        real_fn = resolve_real();
        if (real_fn == NULL) {
            if (error != NULL) {
                *error = g_error_new_literal(g_quark_from_static_string("scam-gst-compat"),
                                              1, "cannot resolve real gst_parse_launch");
            }
            return NULL;
        }
    }

    if (description != NULL && strstr(description, "nvv4l2decoder") != NULL) {
        const gchar *format = strstr(description, "format=RGBA") != NULL ? "RGBA" : "NV12";
        const gchar *decoder = g_getenv("SCAM_GST_DECODER");
        if (decoder == NULL || decoder[0] == 0) {
            decoder = "jpegdec";
        }
        gchar replacement[1024];
        if (g_strcmp0(decoder, "nvjpegdec") == 0) {
            /*
             * nvjpegdec produces NVMM frames on this Jetson. nvvidconv
             * performs the hardware color conversion and copies the
             * requested RGBA/NV12 result into host-readable memory for
             * the vendor SDK appsink.
            */
            g_snprintf(replacement, sizeof(replacement),
                       "appsrc name=mysrc is-live=true format=time do-timestamp=true "
                       "block=false max-buffers=1 max-bytes=0 max-time=0 "
                       "leaky-type=downstream ! "
                       "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
                       "leaky=downstream ! jpegparse ! "
                       "nvjpegdec ! nvvidconv ! video/x-raw,format=%s ! "
                       "appsink name=mysink sync=false max-buffers=1 drop=true "
                       "enable-last-sample=false wait-on-eos=false",
                       format);
        } else {
            g_snprintf(replacement, sizeof(replacement),
                       "appsrc name=mysrc is-live=true format=time do-timestamp=true "
                       "block=false max-buffers=1 max-bytes=0 max-time=0 "
                       "leaky-type=downstream ! "
                       "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
                       "leaky=downstream ! jpegparse ! %s ! "
                       "nvvidconv ! video/x-raw,format=%s ! "
                       "appsink name=mysink sync=false max-buffers=1 drop=true "
                       "enable-last-sample=false wait-on-eos=false",
                       decoder, format);
        }
        fprintf(stderr, "[scam-gst-compat] replace vendor pipeline with %s + nvvidconv (%s), bounded low-latency queues\n",
                decoder, format);
        return real_fn(replacement, error);
    }

    return real_fn(description, error);
}
