#include "rt_utils.h"

#include <cstdio>
#include <cstring>
#include <sched.h>
#include <sys/mman.h>

bool rt_setup_process(const int *cores, size_t n_cores) {
    // Lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }

    // Set CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (size_t i = 0; i < n_cores; i++) {
        CPU_SET(cores[i], &cpuset);
    }
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity");
        return false;
    }

    printf("Running on CPU cores");
    for (size_t i = 0; i < n_cores; i++) {
        printf(" %d", cores[i]);
    }
    printf("\n");
    return true;
}

bool rt_thread_create(pthread_t *thread, const char *name, int priority, size_t stack_size,
                      void *(*func)(void *), void *arg) {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, stack_size);

    struct sched_param param = {};
    param.sched_priority = priority;
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);

    int ret = pthread_create(thread, &attr, func, arg);
    pthread_attr_destroy(&attr);
    if (ret != 0) {
        printf("Error: Could not create thread %s (FIFO %d): %s\n", name, priority, strerror(ret));
        return false;
    }

    pthread_setname_np(*thread, name);
    printf("Thread %s started (SCHED_FIFO %d)\n", name, priority);
    return true;
}

void add_timespec(struct timespec *ts, int64_t addtime) {
    int64_t nsec = addtime % NSEC_PER_SEC;
    int64_t sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= NSEC_PER_SEC) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

void dc_sync(int64_t reftime, int64_t cycletime, int64_t *offsettime) {
    static int64_t integral = 0;
    int64_t delta = reftime % cycletime;
    if (delta > (cycletime / 2)) {
        delta = delta - cycletime;
    }
    if (delta > 0) {
        integral++;
    }
    if (delta < 0) {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
}
