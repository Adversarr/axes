#pragma once
#include <tbb/parallel_for.h>

#ifndef AX_FORCE_PARALLEL
#define AX_FORCE_PARALLEL 0
#endif


#if (defined (NDEBUG)) || (AX_FORCE_PARALLEL == 1)
#define AX_PARALLEL_FOR_BEGIN(beg, end, label) tbb::parallel_for<idx>((beg), (end), [&](idx  label ) {
#define AX_PARALLEL_FOR_END() })
#else
#define AX_PARALLEL_FOR_BEGIN(beg, end, label) for (ax::idx label = (beg); label < (end); ++label) {
#define AX_PARALLEL_FOR_END() }
#endif
