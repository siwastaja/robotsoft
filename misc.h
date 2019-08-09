#pragma once

#define ALWAYS_INLINE static inline __attribute__((always_inline))

//#define PR_FUNC() {}
#define PR_FUNC() {printf(__func__); printf("\n");}

#define TRACK_MIN(invar_, minvar_) do{if((invar_) < (minvar_)) (minvar_) = (invar_);}while(0)
#define TRACK_MAX(invar_, maxvar_) do{if((invar_) > (maxvar_)) (maxvar_) = (invar_);}while(0)

#define VECTSQ_I64(xa_, ya_, za_, xb_, yb_, zb_) (sq((int64_t)(xa_)-(int64_t)(xb_)) + sq((int64_t)(ya_)-(int64_t)(yb_)) + sq((int64_t)(za_)-(int64_t)(zb_)))
#define sq(x) ((x)*(x))
#define abso(x) (((x)<0)?(-(x)):(x))

#define SWAP(t_, a_, b_) do{t_ tmp_ = a_; a_ = b_; b_ = tmp_;}while(0)

#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
ALWAYS_INLINE double subsec_timestamp()
{
	struct timespec spec;
	clock_gettime(CLOCK_MONOTONIC, &spec);

	return (double)spec.tv_sec + (double)spec.tv_nsec/1.0e9;
}

