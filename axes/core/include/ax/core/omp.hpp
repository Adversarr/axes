#pragma once
#include "macros.hpp"
#ifdef AX_HAS_OPENMP
#include <omp.h>
#define AX_OMP_PARALLEL_FOR AX_PRAGMA(omp parallel for)
#define AX_OMP_PARALLEL AX_PRAGMA(omp parallel)
#define AX_OMP_CRITICAL(name) AX_PRAGMA(omp critical(name))
#define AX_OMP_BARRIER AX_PRAGMA(omp barrier)
#define AX_OMP_SINGLE AX_PRAGMA(omp single)
#define AX_OMP_MASTER AX_PRAGMA(omp master)
#define AX_OMP_SCHEDULE_DYNAMIC AX_PRAGMA(omp schedule(dynamic))
#define AX_OMP_SCHEDULE_STATIC AX_PRAGMA(omp schedule(static))
#define AX_OMP_SCHEDULE_GUIDED AX_PRAGMA(omp schedule(guided))
#define AX_OMP_SCHEDULE_AUTO AX_PRAGMA(omp schedule(auto))
#define AX_OMP_SCHEDULE_RUNTIME AX_PRAGMA(omp schedule(runtime))
#define AX_OMP_SCHEDULE_CHUNK_SIZE(size) AX_PRAGMA(omp schedule(static, size))
#define AX_OMP_THREAD_NUM() omp_get_thread_num()
#define AX_OMP_NUM_THREADS() omp_get_num_threads()
#define AX_OMP_GET_WTIME() omp_get_wtime()
#else
#include <ctime>
#define AX_OMP_PARALLEL_FOR AX_PRAGMA()
#define AX_OMP_PARALLEL AX_PRAGMA()
#define AX_OMP_CRITICAL(name) AX_PRAGMA()
#define AX_OMP_BARRIER AX_PRAGMA()
#define AX_OMP_SINGLE AX_PRAGMA()
#define AX_OMP_MASTER AX_PRAGMA()
#define AX_OMP_SCHEDULE_DYNAMIC AX_PRAGMA()
#define AX_OMP_SCHEDULE_STATIC AX_PRAGMA()
#define AX_OMP_SCHEDULE_GUIDED AX_PRAGMA()
#define AX_OMP_SCHEDULE_AUTO AX_PRAGMA()
#define AX_OMP_SCHEDULE_RUNTIME AX_PRAGMA()
#define AX_OMP_SCHEDULE_CHUNK_SIZE(size) AX_PRAGMA()
#define AX_OMP_THREAD_NUM 0
#define AX_OMP_NUM_THREADS 1
#endif