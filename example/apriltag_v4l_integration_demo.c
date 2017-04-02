/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

//// START - from uvccapture.c
// #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
// #include <unistd.h>
//#include <jpeglib.h>
//#include    "jpeglib.h"
#include <time.h>
#include <linux/videodev2.h>

#include "v4l2uvc.h" //// NOTE - v4l2uvc.h and v4l2uvc.c copied over from uvccapture
//// END - from uvccapture.c

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"

#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

//// from uvccapture.c
int run = 1;
void sigcatch (int sig)
{
    fprintf (stderr, "Exiting...\n");
    run = 0;
}

// Invoke:
//
// tagtest [options] input.pnm

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 'i', "iters", "1", "Repeat processing on input set this many times");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input; negative sharpens");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const zarray_t *inputs = getopt_get_extra_args(getopt);

    apriltag_family_t *tagFamily = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
        tagFamily = tag36h11_create();
    else if (!strcmp(famname, "tag36h10"))
        tagFamily = tag36h10_create();
    else if (!strcmp(famname, "tag36artoolkit"))
        tagFamily = tag36artoolkit_create();
    else if (!strcmp(famname, "tag25h9"))
        tagFamily = tag25h9_create();
    else if (!strcmp(famname, "tag25h7"))
        tagFamily = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    tagFamily->black_border = getopt_get_int(getopt, "border");

    apriltag_detector_t *tagDetector = apriltag_detector_create();
    apriltag_detector_add_family(tagDetector, tagFamily);
    tagDetector->quad_decimate = getopt_get_double(getopt, "decimate");
    tagDetector->quad_sigma = getopt_get_double(getopt, "blur");
    tagDetector->nthreads = getopt_get_int(getopt, "threads");
    tagDetector->debug = getopt_get_bool(getopt, "debug");
    tagDetector->refine_edges = getopt_get_bool(getopt, "refine-edges");
    tagDetector->refine_decode = getopt_get_bool(getopt, "refine-decode");
    tagDetector->refine_pose = getopt_get_bool(getopt, "refine-pose");

    int quiet = getopt_get_bool(getopt, "quiet");

    int maxiters = getopt_get_int(getopt, "iters");

    const int hamm_hist_max = 10;

    //// START - from uvccapture.c
    char *videodevice = "/dev/video2";
    char  thisfile[2000]; /* used as filename buffer in multi-file seq. */
    char *post_capture_command[3];
    int format = V4L2_PIX_FMT_YUYV; //V4L2_PIX_FMT_MJPEG;
    int grabmethod = 1;
    int width = 320;
    int height = 240;
    int brightness = 0, contrast = 0, saturation = 0, gain = 0;
    int verbose = 10;
    int delay = 0;
    int skip = 0;
    int quality = 95;
    int post_capture_command_wait = 0;
    int multifile = 0;   /* flag indicating that we save to a multi-file sequence */
    int captureCount = 0;
    int numToCapture = -1;

    struct vdIn *videoIn;

    (void) signal (SIGINT, sigcatch);
    (void) signal (SIGQUIT, sigcatch);
    (void) signal (SIGKILL, sigcatch);
    (void) signal (SIGTERM, sigcatch);
    (void) signal (SIGABRT, sigcatch);
    (void) signal (SIGTRAP, sigcatch);
    //// END - from uvccapture.c


    for (int iter = 0; iter < maxiters; iter++) {

        int total_quads = 0;
        int total_hamm_hist[hamm_hist_max];
        memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
        double total_time = 0;

        if (maxiters > 1)
            printf("iter %d / %d\n", iter + 1, maxiters);

        for (int input = 0; input < zarray_size(inputs); input++) {

            int hamm_hist[hamm_hist_max];
            memset(hamm_hist, 0, sizeof(hamm_hist));

            char *path;
            zarray_get(inputs, input, &path);
            if (!quiet)
                printf("loading %s\n", path);
            else
                printf("%20s ", path);

            image_u8_t *im = NULL;
            if (str_starts_with(path, "/dev/video")) {  //// NEW: capture from webcam with V4L
              printf("reading from video device '%s'\n", path);
              //// take V4L code from uvccapture.c

              //// allocate memory for frame
              videoIn = (struct vdIn *) calloc (1, sizeof (struct vdIn));
              //// acquire a handle for the camera
              int frame_capture_status = init_videoIn(videoIn, (char *) videodevice, width, height, format, grabmethod);
              //// quit on error
              if ( frame_capture_status < 0) { printf("\nERROR: Problem capturing frame\n\n"); exit (1); }
              else { printf("INFO: captured frame\n"); }
              //// Reset all camera controls
              if (verbose >= 1) { printf ("Resetting camera settings\n"); }
              v4l2ResetControl (videoIn, V4L2_CID_BRIGHTNESS);
              v4l2ResetControl (videoIn, V4L2_CID_CONTRAST);
              v4l2ResetControl (videoIn, V4L2_CID_SATURATION);
              v4l2ResetControl (videoIn, V4L2_CID_GAIN);

              //// Setup Camera Parameters
              if (brightness != 0) {
                  if (verbose >= 1) { fprintf (stderr, "Setting camera brightness to %d\n", brightness); }
                  v4l2SetControl (videoIn, V4L2_CID_BRIGHTNESS, brightness);
              } else {
                  if (verbose >= 1) { fprintf (stderr, "Camera brightness level is %d\n", v4l2GetControl (videoIn, V4L2_CID_BRIGHTNESS)); }
              }
              if (contrast != 0) {
                  if (verbose >= 1) { fprintf (stderr, "Setting camera contrast to %d\n", contrast); }
                  v4l2SetControl (videoIn, V4L2_CID_CONTRAST, contrast);
              } else {
                if (verbose >= 1) { fprintf (stderr, "Camera contrast level is %d\n", v4l2GetControl (videoIn, V4L2_CID_CONTRAST)); }
              }
              if (saturation != 0) {
                  if (verbose >= 1) { fprintf (stderr, "Setting camera saturation to %d\n", saturation); }
                  v4l2SetControl (videoIn, V4L2_CID_SATURATION, saturation);
              } else if (verbose >= 1) { fprintf (stderr, "Camera saturation level is %d\n", v4l2GetControl (videoIn, V4L2_CID_SATURATION)); }
              if (gain != 0) {
                  if (verbose >= 1) { fprintf (stderr, "Setting camera gain to %d\n", gain); }
                  v4l2SetControl (videoIn, V4L2_CID_GAIN, gain);
              } else if (verbose >= 1) { fprintf (stderr, "Camera gain level is %d\n", v4l2GetControl (videoIn, V4L2_CID_GAIN)); }
              printf ("INFO: finished setting up Camera Parameters \n");

              //// END preparing the camera for capture

              //// capture frame
              printf ("INFO: grabbing frame \n");
              int frame_grab_status = uvcGrab (videoIn);
              printf ("INFO: finished grabbing frame \n");
              if ( frame_grab_status < 0) {
                printf ("ERROR: Error grabbing frame: closing resources and exiting.\n");
                close_v4l2 (videoIn);
                free (videoIn);
                exit (1);
              }
              printf ("INFO: grabbed frame successfully \n");

              //// convert to jpeg, write into im
                    //////// write jpeg to file
                    //  ptr − This is the pointer to the array of elements to be written.
                    //  size − This is the size in bytes of each element to be written.
                    //  nmemb − This is the number of elements, each one with a size of size bytes.
                    //  stream − This is the pointer to a FILE object that specifies an output stream.
                    // fwrite (videoIn->tmpbuffer, videoIn->buf.bytesused + DHT_SIZE, 1, file);
              //// ??? memcpy, or loop,
                //  This is pointer to the destination array where the content is to be copied, type-casted to a pointer of type void*.
                //  This is pointer to the source of data to be copied, type-casted to a pointer of type void*.
                //  This is the number of bytes to be copied.
              // memcpy(varC, varA + 32, 32);

              ////image_u8_t *im = image_u8_create(pj->width, pj->height);  // make the space for the image


              printf ("INFO: copying from V4L into im \n");
              printf ("INFO: videoIn->tmpbuffer has %u bytes \n",sizeof(videoIn->tmpbuffer));
              printf ("INFO: videoIn->tmpbuffer elements have size %u bytes \n",sizeof(videoIn->tmpbuffer[0]));
              printf ("INFO: videoIn->tmpbuffer has %u elements \n",sizeof(videoIn->tmpbuffer)/sizeof(videoIn->tmpbuffer[0]));
              printf ("INFO: videoIn->buf.bytesused = %i \n",videoIn->buf.bytesused);
              printf ("INFO: DHT_SIZE = %i \n",DHT_SIZE);

              printf ("INFO: allocating space for im \n");
              im = image_u8_create(width, height);                              ///// Line 704 ; /mnt/nixbig/downloads/apriltags_umich/apriltag-2016-12-01__0_9_8/apriltag-2016-12-01/common/pjpeg.c
              printf("image_u8_create(width, height): im->width=%5d,  im->height=%5d,  im->stride=%5d \n",im->width,im->height,im->stride);
              printf ("INFO: allocated space for im \n");

              // take the YUYV, convert it to greyscale, and detect AprilTags in it
              // from uvccapture.c
              unsigned char *line_buffer, *yuyv;
              printf ("INFO: did unsigned char *line_buffer, *yuyv\n");
              line_buffer = calloc (videoIn->width * 3, 1);
              printf ("INFO: did line_buffer = calloc (videoIn->width * 3, 1);\n");
              yuyv = videoIn->framebuffer;
              int z;
              z = 0;
              for (int row_ = 0; row_ < videoIn->height; row_++) {
                printf ("INFO: START line %d of %d\n",row_,videoIn->height);
                unsigned char *ptr = line_buffer;
                // printf ("INFO: did *ptr = line_buffer\n");
                // printf ("INFO: videoIn->width=%d.\n",videoIn->width);
                for (int col_ = 0; col_ < videoIn->width; col_++) {
                  int r, g, b;
                  int y, u, v;
                  // printf ("INFO: pixel %d.\n",col_);
                  if (!z)
                      y = yuyv[0] << 8;     //// no alpha, so Y,U,?,V
                  else
                      y = yuyv[2] << 8;     //// has alpha, so ?,U,Y,V
                      // printf ("INFO: pixel %d; y=%d\n",col_,y);
                  u = yuyv[1] - 128;
                  v = yuyv[3] - 128;

                  r = (y + (359 * v)) >> 8;
                  g = (y - (88 * u) - (183 * v)) >> 8;
                  b = (y + (454 * u)) >> 8;
                  r = (r > 255) ? 255 : ((r < 0) ? 0 : r);
                  *(ptr++) = r;  //// increment pointer, set to red, limit between 0 and 255 inclusive
                  *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
                  *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

                  if (z++) {
                      z = 0;
                      yuyv += 4;
                  }
                  im->buf[row_*im->stride + col_ ] = r;  // yth row (from 0), xth pixel (from 0), greyscale so only one channel
                }
                // image_u8_write_pnm(im, "debug_V4L_from_YUYV.pnm");
                printf ("INFO: END line %d of %d\n",row_,videoIn->height);
                // here uvccapture.c writes the line to the JPEG via jpeg_write_scanlines
              }
              // here uvccapture.c finishes the JPEG compression via jpeg_finish_compress
              free (line_buffer);
              printf ("INFO: did :  free (line_buffer);\n\n");
              image_u8_write_pnm(im, "debug_V4L_from_YUYV.pnm");
              printf ("INFO: did :  image_u8_write_pnm\n\n");

              // char *outputfile = "raw_outputfile_framebuffer.jpg";
              // FILE *file;
              // file = fopen (outputfile, "wb");
              // fwrite (videoIn->framebuffer, videoIn->buf.bytesused + DHT_SIZE, 1, file);
              // fclose (file);
//
//               unsigned char *rgbFrameBuffer = videoIn->framebuffer;
//               for (int y = 0; y < im->height; y++) {
//                 for (int x = 0; x < im->width; x++) {
//                   int r_val = rgbFrameBuffer[ (y*width) + x ];
//                   im->buf[y*im->stride + x + 0 ] = r_val; // (r_val>255 ? 255 : (r_val<0 ? 0 : r_val));
//                   // videoIn is a buffer of unsigned char
//                         // to position        , , number of bytes
//                 //  memcpy(&im->buf[y*im->stride], &videoIn->data[videoIn->buf.bytesused], width);
// // TODO:  from  pjpeg.c :  image_u8_t *pjpeg_to_u8_baseline(pjpeg_t *pj)
// // NOTE:  pjpeg.c : pjpeg_to_u8x3_baseline iterates each array element because the rgb are interleaved so cannot block memcpy
//                 }
//               }
//               image_u8_write_pnm(im, "debug_V4L_framebuffer.pnm");
//
//               printf("videoIn->formatIn=",videoIn->formatIn);
//               outputfile = "raw_outputfile_tmpbuffer.jpg";
//               file = fopen (outputfile, "wb");
//               fwrite (videoIn->tmpbuffer, videoIn->buf.bytesused + DHT_SIZE, 1, file);
//               fclose (file);
//
//               rgbFrameBuffer = videoIn->tmpbuffer;
//               for (int y = 0; y < im->height; y++) {
//                 for (int x = 0; x < im->width; x++) {
//                   int r_val = rgbFrameBuffer[ (y*width) + x ];
//                   im->buf[y*im->stride + x + 0 ] = r_val; // (r_val>255 ? 255 : (r_val<0 ? 0 : r_val));
//                   // videoIn is a buffer of unsigned char
//                         // to position        , , number of bytes
//                 //  memcpy(&im->buf[y*im->stride], &videoIn->data[videoIn->buf.bytesused], width);
// // TODO:  from  pjpeg.c :  image_u8_t *pjpeg_to_u8_baseline(pjpeg_t *pj)
// // NOTE:  pjpeg.c : pjpeg_to_u8x3_baseline iterates each array element because the rgb are interleaved so cannot block memcpy
//                 }
//               }
//               image_u8_write_pnm(im, "debug_V4L_tmpbuffer.pnm");

              // memcpy(im, videoIn->tmpbuffer, videoIn->buf.bytesused + DHT_SIZE);

  ////  image_u8_t *im = NULL;
  ////   see /mnt/nixbig/downloads/apriltags_umich/apriltag-2016-12-01__0_9_8/apriltag-2016-12-01/common/pjpeg.c
  // image_u8_t *pjpeg_to_u8_baseline(pjpeg_t *pj)
  // {
  //     assert(pj->ncomponents > 0);
  //
  //     pjpeg_component_t *comp = &pj->components[0];
  //
  //     assert(comp->width >= pj->width && comp->height >= pj->height);
  //
  //     image_u8_t *im = image_u8_create(pj->width, pj->height);
  //     for (int y = 0; y < im->height; y++)
  //         memcpy(&im->buf[y*im->stride], &comp->data[y*comp->stride], pj->width);
  //
  //     return im;
  // }

              printf ("INFO: copied from V4L into im \n");

              //// START - DEALLOCATE the video device
              printf ("INFO: closing video resources \n");
              close_v4l2 (videoIn);
              free (videoIn);
              printf ("INFO: closed video resources \n");
              //// END - DEALLOCATE the video device
            }
            else if (str_ends_with(path, "pnm") || str_ends_with(path, "PNM") ||
                str_ends_with(path, "pgm") || str_ends_with(path, "PGM")) {
                im = image_u8_create_from_pnm(path);
            }
            else if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG")) {
//// entry point: should get JPEG from V4L
                int err = 0;
                pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);
                if (pjpeg == NULL) {
                    printf("pjpeg error %d\n", err);
                    continue;
                }
                if (1) {  //// ALWAYS TRUE?
                    printf("im = pjpeg_to_u8_baseline(pjpeg)\n");
                    im = pjpeg_to_u8_baseline(pjpeg);                           ////  just takes the first/red channel/component as the intensity

                    printf("pjpeg_to_u8_baseline(pjpeg):     im->width=%5d,  im->height=%5d,  im->stride=%5d : dim=1, dim*width=%5d \n",im->width,im->height,im->stride, (im->width * 1));
                    image_u8x3_t *imc =  pjpeg_to_u8x3_baseline(pjpeg);         ////  NOT USED! TODO - remove. Just getting the dimensions of this, to get a feeling for the stride.

                    printf("pjpeg_to_u8x3_baseline(pjpeg):  imc->width=%5d, imc->height=%5d, imc->stride=%5d : dim=3, dim*width=%5d \n",imc->width,imc->height,imc->stride, (im->width * 3));
                } else {  //// some sort of intensity calculation - looks expensive, and colour-balance / light-source assumption
                    printf("illumination invariant\n");  //// ??
                    image_u8x3_t *imc =  pjpeg_to_u8x3_baseline(pjpeg);
                    printf("pjpeg_to_u8x3_baseline(pjpeg):           imc->width=%5d, imc->height=%5d, imc->stride=%5d \n",imc->width,imc->height,imc->stride);
                    im = image_u8_create(imc->width, imc->height);
                    printf("image_u8_create(imc->width, imc->height): im->width=%5d,  im->height=%5d,  im->stride=%5d \n",im->width,im->height,im->stride);
                    for (int y = 0; y < imc->height; y++) {
                        for (int x = 0; x < imc->width; x++) {
                            double r = imc->buf[y*imc->stride + 3*x + 0] / 255.0;   //  each jpeg in-buffer is a concatenation of lines with r,g,b,r,g,b,r,g,b,...
                            double g = imc->buf[y*imc->stride + 3*x + 1] / 255.0;   // e.g. first iteration is 0*stride + 0*pixel + [1|2|3], and last iteration of an image row is at buffer position (y*stride) + (3*width) + [1|2|3] and then skips to the next image row; the
                            double b = imc->buf[y*imc->stride + 3*x + 2] / 255.0;
                            double alpha = 0.42;
                            double v = 0.5 + log(g) - alpha*log(b) - (1-alpha)*log(r);
                            int iv = v * 255;                                       // ... but then each is converted into an int in the image buffer ...
                            if (iv < 0)
                                iv = 0;
                            if (iv > 255)
                                iv = 255;
                            im->buf[y*im->stride + x] = iv;                         // ... which has size = height * stride
                        }
                    }
                    image_u8x3_destroy(imc);
                    if (tagDetector->debug) {
                        image_u8_write_pnm(im, "debug_invariant.pnm"); }
                }
                pjpeg_destroy(pjpeg);
            }

            if (im == NULL) {
                printf("ERROR: image is null: couldn't load %s\n", path);
                continue;
            } else { printf("INFO: image is not null: processing. \n"); }

            printf("INFO: about to detect AprilTags. \n");
            zarray_t *detections = apriltag_detector_detect(tagDetector, im);  ////   the meaty bit - run tag detection on a grayscale image given
            printf("INFO: detected AprilTags. \n");

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                if (!quiet) {
                    printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f, centre pixel (%8.3f,%8.3f)\n",
                           i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin
                          , det->c[0],det->c[1]);
                    printf( "\t\tcorners X = [%5.0f, %5.0f, %5.0f, %5.0f]\n" , det->p[0][0], det->p[1][0], det->p[2][0], det->p[3][0] );
                    printf( "\t\tcorners y = [%5.0f, %5.0f, %5.0f, %5.0f]\n" , det->p[0][1], det->p[1][1], det->p[2][1], det->p[3][1] );
              printf( "im=iread('%s'); idisp(im); hold on; \n",path);
              printf( " hold on; plot([%5.0f], [%5.0f],'+','LineWidth',2);\n" , det->c[0],det->c[1] );
              printf( " hold on; plot([%5.0f, %5.0f, %5.0f, %5.0f, %5.0f]" , det->p[0][0], det->p[1][0], det->p[2][0], det->p[3][0], det->p[0][0] );
              printf( ", [%5.0f, %5.0f, %5.0f, %5.0f, %5.0f],'--+','LineWidth',2);\n\n" , det->p[0][1], det->p[1][1], det->p[2][1], det->p[3][1], det->p[0][1] );
                }
                hamm_hist[det->hamming]++;
                total_hamm_hist[det->hamming]++;
            }

            apriltag_detections_destroy(detections);

            if (!quiet) {
                timeprofile_display(tagDetector->tp);
            }

            total_quads += tagDetector->nquads;

            if (!quiet)
                printf("hamm ");

            for (int i = 0; i < hamm_hist_max; i++)
                printf("%5d ", hamm_hist[i]);

            double t =  timeprofile_total_utime(tagDetector->tp) / 1.0E3;
            total_time += t;
            printf("%12.3f ", t);
            printf("%5d", tagDetector->nquads);

            printf("\n");

            image_u8_destroy(im);
        }


        printf("Summary\n");

        printf("hamm ");

        for (int i = 0; i < hamm_hist_max; i++)
            printf("%5d ", total_hamm_hist[i]);
        printf("%12.3f ", total_time);
        printf("%5d", total_quads);
        printf("\n");

    }

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(tagDetector);

    tag36h11_destroy(tagFamily);
    return 0;
}
