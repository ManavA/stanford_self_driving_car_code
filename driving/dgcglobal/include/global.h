/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#ifndef DGC_GLOBAL_H_
#define DGC_GLOBAL_H_

#include <sys/stat.h>
#include <cmath>
#include <comp_stdio.h>
#include <scaledTime.h>
#include <mathMisc.h>

namespace dgc {

typedef void (*dgc_usage_func)(char *fmt, ...);

extern int dgc_carp_verbose;

extern "C" {
float strtof(const char *nptr, char **endptr);
int strcasecmp(const char *s1, const char *s2);
int strncasecmp(const char *s1, const char *s2, size_t n);
}

#ifndef va_copy
#define va_copy __va_copy
#endif

void dgc_fatal_error(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_error(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_warning(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_info(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

void dgc_fatal_ferror(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_ferror(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_fwarning(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
void dgc_finfo(const char *fname, char *fmt, ...) __attribute__ ((format (printf, 2, 3)));

void dgc_verbose(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void dgc_die(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

void dgc_carp_set_verbose(int verbosity);
int dgc_carp_get_verbose(void);
void dgc_carp_set_output(FILE *output);
char *dgc_extract_filename(char *path);
char *dgc_hostname(void);
int dgc_sign(double num);
void dgc_rect_to_polar(double x, double y, double *r, double *theta);

unsigned int dgc_randomize(int *argc, char ***argv);
unsigned int dgc_generate_random_seed();
void dgc_set_random_seed(unsigned int seed);
int dgc_int_random(int max);
double dgc_uniform_random(double min, double max);
double dgc_gaussian_random(double mean, double std);

int dgc_file_exists(const char* filename);
char *dgc_file_extension(char *filename);
int dgc_strcasecmp(const char *s1, const char *s2);
int dgc_strncasecmp(const char *s1, const char *s2, size_t n);
char *dgc_new_string(const char *fmt, ...);
char *dgc_new_stringv(const char *fmt, va_list ap);

typedef struct {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

} dgc_pose_t, *dgc_pose_p;

#define dgc_time_code(code, str) { double time_code_t1, time_code_t2; time_code_t1 = dgc_get_time(); code; time_code_t2 = dgc_get_time(); fprintf(stderr, "%-20s :%7.2f ms.\n", str, (time_code_t2 - time_code_t1) * 1000.0); }

template <class T>
inline void dgc_test_alloc(T* X) {
    if (!X) {
        dgc_die("Out of memory in %s, (%s, line %d).\n", __FUNCTION__, __FILE__, __LINE__);
    }
}

/*
 The function is global.c is a backup function as the function in library/timesync/timesync.c does a better job
 */

extern inline int dgc_round(double X) {
    if (X >= 0) return (int) (X + 0.5);
    else return (int) (X - 0.5);
}

inline double dgc_clamp(double X, double Y, double Z) {
    if (Y < X) return X;
    else if (Y > Z) return Z;
    return Y;
}

inline int dgc_trunc(double X) {
    return (int) (X);
}

inline double dgc_hypot3(double a, double b, double c) {
    return sqrt(a * a + b * b + c * c);
}

inline double dgc_r2d(double theta) {
    return (theta * 180.0 / M_PI);
}

inline double dgc_d2r(double theta) {
    return (theta * M_PI / 180.0);
}

inline float dgc_r2df(float theta) {
    return (theta * 180.0 / M_PI);
}

inline float dgc_d2rf(float theta) {
    return (theta * M_PI / 180.0);
}

inline double dgc_mph2ms(double mph) {
    return (mph * 0.44704);
}

inline double dgc_ms2mph(double ms) {
    return (ms * 2.23693629);
}

inline double dgc_kph2ms(double kph) {
    return (kph * 0.277777778);
}

inline double dgc_ms2kph(double ms) {
    return (ms * 3.6);
}

inline double dgc_meters2feet(double meters) {
    return (meters * 3.2808399);
}

inline double dgc_feet2meters(double feet) {
    return (feet * 0.3048);
}

inline double dgc_meters2miles(double meters) {
    return (meters / 1609.344);
}

inline double dgc_miles2meters(double miles) {
    return (miles * 1609.344);
}

inline double dgc_mph2kph(double mph) {
    return (mph * 1.609344);
}

inline double dgc_kph2mph(double kph) {
    return (kph * 0.621371192);
}

inline double dgc_surveyor_feet2meters(double x) {
    return x * 0.304800609601219;
}

inline double dgc_meters2surveyor_feet(double x) {
    return x / 0.304800609601219;
}

inline double dgc_square(double val) {
    return (val * val);
}

inline int my_isblank(char c) {
    if (c == ' ' || c == '\t') return 1;
    else return 0;
}

char* dgc_next_word(char *str);
char *dgc_next_n_words(char *str, int n);

void dgc_point_to_line_distance(double x, double y, double x1, double y1, double x2, double y2, double *parallel_dist,
        double *perp_dist, double *x_match, double *y_match);

void dgc_point_to_segment_distance(double x, double y, double x1, double y1, double x2, double y2, double *perp_dist,
        double *parallel_dist, double *x_match, double *y_match);

void dgc_point_to_segment_distance2(double x, double y, double yaw, double x1, double y1, double x2, double y2,
        double xfar, double yfar, double *perp_dist, double *parallel_dist, double *x_match, double *y_match);

double dgc_average_angle(double theta1, double theta2);

off64_t dgc_file_size(const char* filename);

char *dgc_run_program(char *command_line, char *input, double timeout);

int dgc_complete_filename(const char* filename, const char* extension, char** new_filename);
char* dgc_expand_filename(const char* filename);
char* dgc_unique_filename(char* name);
char dgc_ascii_rotor();

double dgc_get_time();
float dgc_avg_time(float t, int cnt);
float dgc_avg_times(float t, int cnt, int num);

char* find_matching_logfile(char *ipc_filename, char *extension, double max_dt);
char* dgc_timestamped_filename(char *basename, char *extension);

} // namespace dgc

namespace vlr {

template<class T, void*(T::*mem_fn)()>
void* threadCBWrapper(void* ptr) {
  return (static_cast<T*>(ptr)->*mem_fn)();
  }

} // namespace vlr

#endif
