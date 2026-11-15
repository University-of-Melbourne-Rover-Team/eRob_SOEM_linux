/**
 * Real-time helper functions
 * Process scheduling setup and DC clock synchronisation
 */

#ifndef URT_RT_UTILS_H
#define URT_RT_UTILS_H

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <pthread.h>

#define NSEC_PER_SEC 1000000000

/**
 * Locks memory and sets process to CPU cores
 * Scheduling priority is set per thread with rt_thread_create()
 * @param {const int *} cores: CPU core list, {size_t} n_cores
 * @return {bool}: false if CPU affinity could not be set
 */
bool rt_setup_process(const int *cores, size_t n_cores);

/**
 * Creates a named SCHED_FIFO thread with an explicit priority
 * @param {pthread_t *} thread, {const char *} name, {int} priority: SCHED_FIFO priority,
 * {size_t} stack_size, {void *(*)(void *)} func, {void *} arg
 * @return {bool}: false if the thread could not be created
 */
bool rt_thread_create(pthread_t *thread, const char *name, int priority, size_t stack_size,
                      void *(*func)(void *), void *arg);

/**
 * Adds nanoseconds to a struct timespec and normalizes the result
 * so that tv_nsec remains within valid bounds (doesn't overflow).
 * @param {timespec *} ts, {int64_t} addtime: nanoseconds to add
 * @return {void}
 */
void add_timespec(struct timespec *ts, int64_t addtime);

/**
 * Aligns PI controller with Distributed Clock (DC) cycle
 * @param {int64_t} reftime: DC time, {int64_t} cycletime: cycle (ns), {int64_t *} offsettime: output offset (ns)
 * @return {void}
 */
void dc_sync(int64_t reftime, int64_t cycletime, int64_t *offsettime);

#endif
