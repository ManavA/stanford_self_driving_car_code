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


#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <dirent.h>
#include <wordexp.h>
#include <libgen.h>
#include <sys/dir.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#include <global.h>

namespace dgc {

int dgc_carp_verbose = 0;

double dgc_get_time() {
    struct timeval tv;
    double t;

    if (gettimeofday(&tv, NULL) < 0) dgc_warning("dgc_get_time encountered error in gettimeofday : %s\n", strerror(
            errno));
    t = tv.tv_sec + tv.tv_usec / 1000000.0;
    return t;
}

static char* get_from_bin_host() {
    FILE *bin_Host;
    char hostname[255];

    if (getenv("HOST") == NULL) {
        if (getenv("HOSTNAME") != NULL) setenv("HOST", getenv("HOSTNAME"), 1);
        else if (getenv("host") != NULL) setenv("HOST", getenv("host"), 1);
        else if (getenv("hostname") != NULL) setenv("HOST", getenv("hostname"), 1);
        else {
            bin_Host = popen("/bin/hostname", "r");
            if (bin_Host == NULL) return NULL;
            if (fscanf(bin_Host, "%s", hostname) == 0) dgc_warning(
                    "/bin/hostname didn't return a hostname, HOST will be set to \"\"");
            setenv("HOST", hostname, 1);
            pclose(bin_Host);
        }
    }
    return getenv("HOST");
}

char* dgc_hostname() {
    char *Host;
    char *mark;
    static char hostname[10] = "";

    if (strlen(hostname) == 0) {
        Host = get_from_bin_host();
        if (!Host) dgc_die("\n\tCan't get machine name from $HOST, $host, $hostname or /bin/hostname.\n"
            "\tPlease set one of these environment variables properly.\n\n");
        if (strlen(Host) >= 10) {
            strncpy(hostname, Host, 9);
            hostname[9] = '\0';
        }
        else strcpy(hostname, Host);
        mark = strchr(hostname, '.');
        if (mark) mark = '\0';
    }

    return hostname;
}

char *
dgc_extract_filename(char *path) {
    /* remove any leading path from the program name */
    if (strrchr(path, '/') != NULL) {
        path = strrchr(path, '/');
        path++;
    }

    return path;
}

static FILE *dgc_carp_output = NULL;

void print_error_string(const char *header, const char *fname, char *message) {
    unsigned int i, j, k, header_len, message_len, count;
    char *dest_str;

    header_len = strlen(header);
    message_len = strlen(message);
    count = header_len;
    if (fname != NULL) count += strlen(fname) + 3;
    for (i = 0; i < message_len - 1; i++)
        if (message[i] == '\n') count += header_len + 1;
        else count++;
    count++;

    dest_str = (char*) calloc(count + 5, 1);
    dgc_test_alloc(dest_str);

    strcpy(dest_str, header);
    j = header_len;
    if (fname != NULL) {
        strcat(dest_str, fname);
        strcat(dest_str, " : ");
        j += strlen(fname) + 3;
    }
    for (i = 0; i < message_len; i++) {
        if (i == message_len - 1 || message[i] != '\n') {
            dest_str[j] = message[i];
            j++;
            dest_str[j] = '\0';
        }
        else {
            dest_str[j] = '\n';
            j++;
            for (k = 0; k < header_len; k++) {
                dest_str[j] = ' ';
                j++;
            }
            dest_str[j] = '\0';
        }
    }
    if (message[message_len - 1] != '\n') strcat(dest_str, "\n");

    if (dgc_carp_output == NULL) dgc_carp_output = stderr;

    fprintf(dgc_carp_output, "%s", dest_str);
    fflush(dgc_carp_output);
    free(dest_str);
}

void dgc_fatal_error(const char* fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# ERROR: ", NULL, message);
    exit(-1);
}

void dgc_error(const char* fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# ERROR: ", NULL, message);
}

void dgc_warning(const char* fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# WARNING: ", NULL, message);
}

void dgc_info(const char* fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# INFO: ", NULL, message);
}

void dgc_fatal_ferror(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# ERROR: ", fname, message);
    exit(-1);
}

void dgc_ferror(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# ERROR: ", fname, message);
}

void dgc_fwarning(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# WARNING: ", fname, message);
}

void dgc_finfo(const char *fname, char *fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# INFO: ", fname, message);
}

void dgc_die(const char* fmt, ...) {
    char message[1024];
    va_list args;

    va_start(args, fmt);
    vsnprintf(message, 1024, fmt, args);
    va_end(args);
    message[1023] = '\0';
    print_error_string("# ERROR: ", NULL, message);
    exit(-1);
}

void dgc_verbose(const char *fmt, ...) {
    va_list args;

    if (!dgc_carp_verbose) return;

    if (dgc_carp_output == NULL) dgc_carp_output = stderr;

    va_start(args, fmt);
    vfprintf(dgc_carp_output, fmt, args);
    va_end(args);
    fflush(dgc_carp_output);
}

void dgc_carp_set_output(FILE *output) {
    dgc_carp_output = output;
}

void dgc_carp_set_verbose(int verbosity) {
    dgc_carp_verbose = verbosity;
}

int dgc_carp_get_verbose(void) {
    return dgc_carp_verbose;
}

double dgc_average_angle(double theta1, double theta2) {
    double x, y;

    x = cos(theta1) + cos(theta2);
    y = sin(theta1) + sin(theta2);
    if (x == 0 && y == 0) {return 0;}
    return atan2(y, x);
}


int dgc_sign(double num) {
    if (num >= 0) return 1;
    return -1;
}

void dgc_rect_to_polar(double x, double y, double *r, double *theta) {
    *r = hypot(x, y);
    *theta = atan2(y, x);
}

unsigned int dgc_generate_random_seed(void) {
    FILE *random_fp;
    unsigned int seed;
    int ints;

    random_fp = fopen("/dev/random", "r");
    if (random_fp == NULL) {
        dgc_warning("Could not open /dev/random for reading: %s\n"
            "Using time ^ PID\n", strerror(errno));
        seed = time(NULL) ^ getpid();
        srandom(seed);
        return seed;
    }

    ints = fread(&seed, sizeof(int), 1, random_fp);
    if (ints != 1) {
        dgc_warning("Could not read an int from /dev/random: %s\n"
            "Using time ^ PID\n", strerror(errno));
        seed = time(NULL) ^ getpid();
        srandom(seed);
        return seed;
    }
    fclose(random_fp);
    srandom(seed);
    return seed;
}

unsigned int dgc_randomize(int *argc, char ***argv) {
    long long int user_seed;
    unsigned int seed;
    int bytes_to_move;
    int i;
    char *endptr;

    for (i = 0; i < *argc - 1; i++) {
        if (strcmp((*argv)[i], "--seed") == 0) {
            user_seed = strtoll((*argv)[i + 1], &endptr, 0);
            seed = (unsigned int) user_seed;
            if (endptr && *endptr != '\0') {
                dgc_warning("Bad random seed %s.\n", (*argv)[i + 1]);
                seed = dgc_generate_random_seed();
            }
            else if (seed != user_seed) {
                dgc_warning("Random seed too large: %s.\n", (*argv)[i + 1]);
                seed = dgc_generate_random_seed();
            }
            else {
                if (i < *argc - 2) {
                    bytes_to_move = (*argc - 2 - i) * sizeof(char *);
                    memmove((*argv) + i, (*argv) + i + 2, bytes_to_move);
                }
                (*argc) -= 2;
                srandom(seed);
            }
            return seed;
        }
    }
    seed = dgc_generate_random_seed();
    return seed;
}

void dgc_set_random_seed(unsigned int seed) {
    srand(seed);
}

/*
 From the rand(3) man page:

 In Numerical Recipes in C: The Art of Scientific Computing
 (William H. Press, Brian P.  Flannery, Saul A. Teukolsky,
 William T. Vetterling; New York: Cambridge University Press,
 1992 (2nd ed., p. 277)), the following comments are made:
 "If you want to generate a random integer  between  1  and  10,
 you  should always do it by using high-order bits, as in

 j=1+(int) (10.0*rand()/(RAND_MAX+1.0));

 and never by anything resembling

 j=1+(rand() % 10);

 (which uses lower-order bits)."
 */

int dgc_int_random(int max) {
    return (int) (max * (rand() / (RAND_MAX + 1.0)));
}

double dgc_uniform_random(double min, double max) {
    return min + (rand() / (double) RAND_MAX) * (max - min);
}

double dgc_gaussian_random(double mean, double std) {
    const double norm = 1.0 / (RAND_MAX + 1.0);
    double u = 1.0 - rand() * norm; /* can't let u == 0 */
    double v = rand() * norm;
    double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
    return mean + std * z;
}

int dgc_file_exists(const char* filename) {
    FILE *fp;

    fp = fopen64(filename, "r");
    if (fp == NULL) return 0;
    else {
        fclose(fp);
        return 1;
    }
}

off64_t dgc_file_size(const char* filename) {
    struct stat64 file_stat;

    stat64(filename, &file_stat);
    return file_stat.st_size;
}

char *dgc_file_extension(char *filename) {
    return strrchr(filename, '.');
}

int dgc_strcasecmp(const char *s1, const char *s2) {
    const unsigned char *p1 = (const unsigned char *) s1;
    const unsigned char *p2 = (const unsigned char *) s2;
    unsigned char c1, c2;

    if (p1 == p2) return 0;

    do {
        c1 = tolower(*p1++);
        c2 = tolower(*p2++);
        if (c1 == '\0') break;
    } while (c1 == c2);

    return c1 - c2;
}

int dgc_strncasecmp(const char *s1, const char *s2, size_t n) {
    const unsigned char *p1 = (const unsigned char *) s1;
    const unsigned char *p2 = (const unsigned char *) s2;
    unsigned char c1, c2;

    if (p1 == p2 || n == 0) return 0;

    do {
        c1 = tolower(*p1++);
        c2 = tolower(*p2++);
        if (c1 == '\0' || c1 != c2) return c1 - c2;
    } while (--n > 0);

    return c1 - c2;
}

char *dgc_new_stringv(const char *fmt, va_list args) {
    va_list ap;
    int n, size = 128;
    char *s;

    if (fmt == NULL) return NULL;

    s = (char *) calloc(size, sizeof(char));
    dgc_test_alloc(s);

    while (1) {
        va_copy(ap, args);
        n = vsnprintf(s, size, fmt, ap);
        va_end(ap);
        if (n < 0) {
            free(s);
            return NULL;
        }
        if (n >= 1 && n < size) return s;
        if (n >= 1) // glibc 2.1
        size = n + 1;
        else // glibc 2.0
        size *= 2;
        s = (char *) realloc(s, size * sizeof(char));
        dgc_test_alloc(s);
    }

    return NULL;
}

char *dgc_new_string(const char *fmt, ...) {
    va_list ap;
    char *s;

    va_start(ap, fmt);
    s = dgc_new_stringv(fmt, ap);
    va_end(ap);

    return s;
}

char* dgc_next_word(char *str) {
    char *mark = str;

    if (str == NULL) return NULL;
    while (*mark != '\0' && !(*mark == ' ' || *mark == '\t'))
        mark++;
    while (*mark != '\0' && (*mark == ' ' || *mark == '\t'))
        mark++;
    return mark;
}

char* dgc_next_n_words(char *str, int n) {
    int i;
    char *result;

    result = str;
    for (i = 0; i < n; i++)
        result = dgc_next_word(result);
    return result;
}

void dgc_point_to_line_distance(double x, double y, double x1, double y1, double x2, double y2, double *parallel_dist,
        double *perp_dist, double *x_match, double *y_match) {
    double v1x, v1y, v2x, v2y;
    double v2len, temp;

    v1x = x - x1;
    v1y = y - y1;
    v2x = x2 - x1;
    v2y = y2 - y1;
    v2len = hypot(v2x, v2y);
    *parallel_dist = (v1x * v2x + v1y * v2y) / v2len;
    temp = v1x * v1x + v1y * v1y - (*parallel_dist) * (*parallel_dist);
    if (temp < 0) *perp_dist = 0;
    else *perp_dist = sqrt(temp);
    *parallel_dist /= v2len;
    *x_match = x1 + v2x * (*parallel_dist);
    *y_match = y1 + v2y * (*parallel_dist);
}

void dgc_point_to_segment_distance(double x, double y, double x1, double y1, double x2, double y2, double *perp_dist,
        double *parallel_dist, double *x_match, double *y_match) {

    dgc_point_to_line_distance(x, y, x1, y1, x2, y2, parallel_dist, perp_dist, x_match, y_match);
    if (*parallel_dist < 0) {
        *perp_dist = hypot(x1 - x, y1 - y);
        *x_match = x1;
        *y_match = y1;
    }
    else if (*parallel_dist > 1) {
        *perp_dist = hypot(x2 - x, y2 - y);
        *x_match = x2;
        *y_match = y2;
    }

}

void dgc_point_to_segment_distance2(double x, double y, double yaw, double x1, double y1, double x2, double y2,
        double xfar, double yfar, double *perp_dist, double *parallel_dist, double *x_match, double *y_match) {

    dgc_point_to_line_distance(x, y, x1, y1, x2, y2, parallel_dist, perp_dist, x_match, y_match);
    if (*parallel_dist < 0) {
        *perp_dist = hypot(x1 - x, y1 - y);
        *x_match = x1;
        *y_match = y1;
    }
    else if (*parallel_dist > 1) {
        *perp_dist = hypot(x2 - x, y2 - y);
        *x_match = x2;
        *y_match = y2;
    }
    yaw = yaw - (2 * M_PI * (int) (yaw / (2 * M_PI)));
    float yawdiff = atan2(yfar - y1, xfar - x1) - yaw;
    if (yawdiff < -M_PI) yawdiff += 2 * M_PI;
    if (yawdiff > M_PI) yawdiff -= 2 * M_PI;
    *perp_dist += 2 * fabs(yawdiff);

}

#define      MAX_NUM_ARGS            64
#define      MAX_LINE_LENGTH         1000

char *dgc_run_program(char *command_line, char *input, double timeout) {
    int n, spawned_pid, num_chars, max_chars, ctr = 0, mark;
    char *path = NULL, *running, *ptr, *output = NULL;
    char buf[MAX_LINE_LENGTH], *arg[MAX_NUM_ARGS];
    int readpipe[2], writepipe[2], err;
    struct timeval t;
    fd_set set;

    strcpy(buf, command_line);
    running = buf;
    ctr = 0;
    while ((ptr = strtok(ctr == 0 ? running : NULL, " ")) != NULL) {
        arg[ctr] = (char *) malloc((strlen(ptr) + 1) * sizeof(char));
        dgc_test_alloc(arg[ctr]);
        strcpy(arg[ctr], ptr);
        if (ctr == 0) {
            path = (char *) malloc((strlen(ptr) + 1) * sizeof(char));
            dgc_test_alloc(path);
            strcpy(path, ptr);
        }
        ctr++;
    }
    /* Fourth argument will be NULL as required by the function. */
    arg[ctr++] = (char*) 0;

    if (pipe(readpipe) != 0) dgc_error("Error opening readpipe");
    if (pipe(writepipe) != 0) dgc_error("Error opening writepipe");

#define	PARENT_READ	readpipe[0]
#define	CHILD_WRITE	readpipe[1]
#define CHILD_READ	writepipe[0]
#define PARENT_WRITE	writepipe[1]

    if ((spawned_pid = fork()) == 0) {
        /* I am the child */
        close(PARENT_WRITE);
        close(PARENT_READ);
        dup2(CHILD_READ, 0);
        close(CHILD_READ);
        dup2(CHILD_WRITE, 1);
        close(CHILD_WRITE);

        /* run the process */
        execv(path, arg);

        /* if it works, execv never returns.  otherwise exit */
        printf("%c", '\0');
        exit(-1);
    }
    else {
        /* I am the parent */
        close(CHILD_READ);
        close(CHILD_WRITE);

        /* write input to standard input of child process */
        if (input != NULL) {
            mark = 0;
            do {
                n = write(PARENT_WRITE, (unsigned char *) input + mark, strlen(input) - mark);
                if (n > 0) mark += n;
            } while (mark < (int) strlen(input));
        }

        /* read standard output of child */
        num_chars = 0;
        max_chars = 1000;
        output = (char *) calloc(max_chars, 1);
        dgc_test_alloc(output);

        do {
            if (num_chars + 100 > max_chars) {
                max_chars += 1000;
                output = (char *) realloc(output, max_chars);
                dgc_test_alloc(output);
            }

            if (timeout != -1) {
                t.tv_sec = (int) floor(timeout);
                t.tv_usec = (timeout - t.tv_sec) * 1e6;
            }
            FD_ZERO(&set);
            FD_SET(PARENT_READ, &set);
            if (timeout == -1) err = select(PARENT_READ + 1, &set, NULL, NULL, NULL);
            else err = select(PARENT_READ + 1, &set, NULL, NULL, &t);
            if (err == 0) break;

            n = read(PARENT_READ, output + num_chars, 100);
            if (n >= 0) {
                num_chars += n;
                output[num_chars] = '\0';
            }
        } while (n > 0);

        close(PARENT_READ);
        close(PARENT_WRITE);
    }
    return output;
}

int dgc_complete_filename(const char* filename, const char* extension, char **new_filename) {
    int n, i, count = 0, last_match = -1;
    struct dirent **namelist;
    char *dirname, *name;
    int retval;

    if (dgc_file_exists(filename) && strlen(filename) >= strlen(extension) && strcmp(filename + strlen(filename)
            - strlen(extension), extension) == 0) {
        *new_filename = (char *) calloc(strlen(filename) + 1, 1);
        dgc_test_alloc(*new_filename);
        strcpy(*new_filename, filename);
        return 1;
    }

    dirname = (char *) calloc(strlen(filename) + 1, 1);
    dgc_test_alloc(dirname);
    name = (char *) calloc(strlen(filename) + 1, 1);
    dgc_test_alloc(name);

    const char* p = strrchr(filename, '/');
    if (p != NULL) {
        strcpy(name, p + 1);
        strncpy(dirname, filename, strlen(filename) - strlen(name));
        dirname[strlen(filename) - strlen(name)] = '\0';
    }
    else {
        strcpy(dirname, ".");
        strcpy(name, filename);
    }

    n = scandir(dirname, &namelist, NULL, NULL);

    for (i = 0; i < n; i++)
        if (strlen(namelist[i]->d_name) >= strlen(name) && strncmp(namelist[i]->d_name, name, strlen(name)) == 0
                && strlen(namelist[i]->d_name) >= strlen(extension) && strcmp(namelist[i]->d_name + strlen(
                namelist[i]->d_name) - strlen(extension), extension) == 0) {
            count++;
            last_match = i;
        }

    if (count == 1) {
        *new_filename = (char *) calloc(strlen(dirname) + strlen(namelist[last_match]->d_name) + 10, 1);
        dgc_test_alloc(*new_filename);

        strcpy(*new_filename, dirname);
        strcat(*new_filename, "/");
        strcat(*new_filename, namelist[last_match]->d_name);
        retval = 1;
    }
    else {
        /*    if(count == 0)
         fprintf(stderr, "Error: no filename matches\n");
         else if(count > 1)
         fprintf(stderr, "Error: more than one filename matches pattern.\n");*/
        retval = 0;
    }

    if (n > 0) {
        for (i = 0; i < n; i++)
            free(namelist[i]);
        free(namelist);
    }
    return retval;
}

char* dgc_expand_filename(const char* filename) {
    char *result = NULL, *output = NULL;
    wordexp_t p;

    wordexp(filename, &p, 0);
    if (p.we_wordc > 0) {
        result = (char *) realloc(result, strlen(p.we_wordv[0]) + 1);
        dgc_test_alloc(result);
        strcpy(result, p.we_wordv[0]);
        output = result;
    }
    p.we_offs = 0; // should avoid the non-aligned free under leopard (soka)
    wordfree(&p);
    return output;
}

char *dgc_unique_filename(char *name) {
    static char fname[256];
    struct tm * local_time;
    time_t current_time;
    char ename[256];
    char ext[256], *ptr;

    strncpy(ename, dgc_expand_filename(name), 256);

    ptr = strrchr(ename, '.');
    if (ptr == NULL) strncpy(ext, "", 256);
    else {
        strncpy(ext, ptr, 256);
        *ptr = '\0';
    }

    current_time = time(NULL);
    local_time = localtime(&current_time);
    snprintf(fname, 256, "%s-%02d-%02d-%04d--%02d-%02d-%02d%s", ename, local_time->tm_mon + 1, local_time->tm_mday,
            local_time->tm_year + 1900, local_time->tm_hour, local_time->tm_min, local_time->tm_sec, ext);
    return (fname);
}

// ********************************************************************
// *
// * spinning character
// *
// ********************************************************************

char dgc_ascii_rotor(void) {
    static int r = 0;
    int rotor_chr[4] = { 47, 45, 92, 124 };
    if (++r > 3) r = 0;
    return (rotor_chr[r]);
}

// ********************************************************************
// *
// * average throughput
// *
// ********************************************************************

#define AVG_OVER_N_VALUES     5

float dgc_avg_time(float t, int cnt) {
    static int init = 0;
    static int ctr = 0;
    static int sum[AVG_OVER_N_VALUES];
    static int totalsum = 0;
    static float times[AVG_OVER_N_VALUES];
    static float totaltime = 0.0;

    int idx = (ctr++) % AVG_OVER_N_VALUES;

    if (!init) {
        totalsum += cnt;
        sum[idx] = cnt;
        totaltime += t;
        times[idx] = t;
        if (ctr == AVG_OVER_N_VALUES) init = 1;
    }
    else {
        totalsum += (cnt - sum[idx]);
        sum[idx] = cnt;
        totaltime += (t - times[idx]);
        times[idx] = t;
    }

    return (totalsum / totaltime);
}

#define MAX_NUM_AVG_COUNTERS     10

float dgc_avg_times(float t, int cnt, int num) {
    static int init[MAX_NUM_AVG_COUNTERS];
    static int ctr[MAX_NUM_AVG_COUNTERS];
    static int sum[MAX_NUM_AVG_COUNTERS][AVG_OVER_N_VALUES];
    static int totalsum[MAX_NUM_AVG_COUNTERS];
    static float times[MAX_NUM_AVG_COUNTERS][AVG_OVER_N_VALUES];
    static float totaltime[MAX_NUM_AVG_COUNTERS];

    if (num >= MAX_NUM_AVG_COUNTERS) return 0.0;

    int idx = (ctr[num]++) % AVG_OVER_N_VALUES;

    if (!init[num]) {
        totalsum[num] += cnt;
        sum[num][idx] = cnt;
        totaltime[num] += t;
        times[num][idx] = t;
        if (ctr[num] == AVG_OVER_N_VALUES) init[num] = 1;
    }
    else {
        totalsum[num] += (cnt - sum[num][idx]);
        sum[num][idx] = cnt;
        totaltime[num] += (t - times[num][idx]);
        times[num][idx] = t;
    }
    return (totalsum[num] / totaltime[num]);
}

time_t extract_date_time_from_logfile(char *filename) {
    int month, day, year, hour, minute, second;
    char *mark, *end_mark, *filename2;
    struct tm t;
    int i, n;

    if (filename == NULL || strlen(filename) == 0) return -1;
    mark = filename + strlen(filename) - 1;

    while (mark >= filename && !isdigit(*mark))
        mark--;
    if (mark < filename) return (time_t) -1;

    end_mark = mark;

    for (i = 0; i < 2; i++) {
        /* find previous - */
        mark--;
        while (mark >= filename && *mark != '-')
            mark--;
        if (mark < filename) return (time_t) -1;
    }

    /* find previous _ */
    mark--;
    while (mark >= filename && *mark != '_')
        mark--;
    if (mark < filename) return (time_t) -1;

    for (i = 0; i < 2; i++) { // fix by Toyota
        /* find previous - */
        mark--;
        while (mark >= filename && *mark != '-')
            mark--;
        if (mark < filename) return (time_t) -1;
    }
    /* find previous - */// fix by Toyota
    mark--;
    while (mark >= filename && isdigit(*mark))
        mark--;
    if (mark < filename) return (time_t) -1;

    mark++;

    filename2 = strdup(mark);
    filename2[end_mark - mark + 1] = '\0';

    for (i = 0; i < (int) strlen(filename2); i++)
        if (!isdigit(filename2[i])) filename2[i] = ' ';

    n = sscanf(filename2, "%d %d %d %d %d %d", &month, &day, &year, &hour, &minute, &second);

    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_year = year - 1900;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;
    t.tm_isdst = 0;

    free(filename2);
    if (n != 6) return (time_t) -1;
    else return mktime(&t);
}

char *find_matching_logfile(char *ipc_filename, char *extension, double max_dt) {
    char filename[500], best_filename[500];
    char *dirc, *basec, *dname, *bname;
    int i, n, min_dt = 1000, min_n;
    struct dirent **namelist;
    time_t ipc_t, vlf_t, dt;
    struct stat file_stat;

    dirc = strdup(ipc_filename);
    basec = strdup(ipc_filename);
    dname = dirname(dirc);
    bname = basename(basec);

    ipc_t = extract_date_time_from_logfile(ipc_filename);
    if (ipc_t == -1) {
        free(dirc);
        free(basec);
        return NULL;
    }

    n = scandir(dname, &namelist, NULL, NULL);
    if (n < 0) {
        free(dirc);
        free(basec);
        return NULL;
    }
    else {
        for (i = 0; i < n; i++)
            if (strlen(namelist[i]->d_name) >= strlen(extension) && strcmp(namelist[i]->d_name + strlen(
                    namelist[i]->d_name) - strlen(extension), extension) == 0) {
                sprintf(filename, "%s/%s", dname, namelist[i]->d_name);
                stat(filename, &file_stat);
                if (S_ISREG(file_stat.st_mode)) {
                    vlf_t = extract_date_time_from_logfile(filename);
                    if (vlf_t != -1) {
                        dt = abs(vlf_t - ipc_t);
                        if (dt < min_dt) {
                            min_dt = dt;
                            min_n = n;
                            strcpy(best_filename, filename);
                        }
                    }
                }
            }
    }

    for (i = 0; i < n; i++)
        free(namelist[i]);
    free(namelist);

    free(dirc);
    free(basec);

    if (min_dt < max_dt) return strdup(best_filename);
    else return NULL;
}

char *dgc_timestamped_filename(char *basename, char *extension) {
    struct tm *local_time;
    time_t current_time;
    char *s;
    int l;

    current_time = time(NULL);
    local_time = localtime(&current_time);

    l = strlen(basename) + strlen(extension) + 30;
    s = (char *) calloc(l, 1);
    dgc_test_alloc(s);

    snprintf(s, l, "%s-%02d-%02d-%04d_%02d-%02d-%02d%s", basename, local_time->tm_mon + 1, local_time->tm_mday,
            local_time->tm_year + 1900, local_time->tm_hour, local_time->tm_min, local_time->tm_sec, extension);

    /* make sure the string always has a \0 in it */
    s[l - 1] = '\0';
    return s;
}

} // namespace dgc

