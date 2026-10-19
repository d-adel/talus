#include "sums.h"

#include <immintrin.h>

#include <cstddef>

namespace talus::destruction {

namespace {

size_t at(int32_t column, int32_t stride, int32_t row) {
  return static_cast<size_t>(column) * static_cast<size_t>(stride) + static_cast<size_t>(row);
}

const double* inputOf(const SumSource& source, int32_t k, int32_t r) {
  return source.rows + static_cast<ptrdiff_t>(k) * source.stride + r;
}

const double* weightOf(const SumSource& source, int32_t k) {
  return source.pivot + static_cast<ptrdiff_t>(k) * source.stride;
}

}

void sumWide(double* product, int32_t span, const SumSources& sources, int32_t count) {
  int32_t r = 0;

  for (; r + 8 <= span; r += 8) {
    __m256d sum[12];

    for (__m256d& lane : sum) {
      lane = _mm256_setzero_pd();
    }

    for (int32_t which = 0; which != count; ++which) {
      const SumSource& source = sources[static_cast<size_t>(which)];

      for (int32_t k = 0; k != 6; ++k) {
        const double* in = inputOf(source, k, r);
        const double* weight = weightOf(source, k);
        const __m256d low = _mm256_loadu_pd(in);
        const __m256d high = _mm256_loadu_pd(in + 4);

        for (int32_t q = 0; q != 6; ++q) {
          const __m256d w = _mm256_broadcast_sd(weight + q);

          sum[2 * q] = _mm256_add_pd(sum[2 * q], _mm256_mul_pd(low, w));
          sum[2 * q + 1] = _mm256_add_pd(sum[2 * q + 1], _mm256_mul_pd(high, w));
        }
      }
    }

    for (int32_t q = 0; q != 6; ++q) {
      _mm256_storeu_pd(product + at(q, span, r), sum[2 * q]);
      _mm256_storeu_pd(product + at(q, span, r) + 4, sum[2 * q + 1]);
    }
  }

  for (; r + 4 <= span; r += 4) {
    __m256d sum[6];

    for (__m256d& lane : sum) {
      lane = _mm256_setzero_pd();
    }

    for (int32_t which = 0; which != count; ++which) {
      const SumSource& source = sources[static_cast<size_t>(which)];

      for (int32_t k = 0; k != 6; ++k) {
        const double* weight = weightOf(source, k);
        const __m256d in = _mm256_loadu_pd(inputOf(source, k, r));

        for (int32_t q = 0; q != 6; ++q) {
          sum[q] = _mm256_add_pd(sum[q], _mm256_mul_pd(in, _mm256_broadcast_sd(weight + q)));
        }
      }
    }

    for (int32_t q = 0; q != 6; ++q) {
      _mm256_storeu_pd(product + at(q, span, r), sum[q]);
    }
  }

  for (; r != span; r += 2) {
    __m128d sum[6];

    for (__m128d& lane : sum) {
      lane = _mm_setzero_pd();
    }

    for (int32_t which = 0; which != count; ++which) {
      const SumSource& source = sources[static_cast<size_t>(which)];

      for (int32_t k = 0; k != 6; ++k) {
        const double* weight = weightOf(source, k);
        const __m128d in = _mm_loadu_pd(inputOf(source, k, r));

        for (int32_t q = 0; q != 6; ++q) {
          sum[q] = _mm_add_pd(sum[q], _mm_mul_pd(in, _mm_set1_pd(weight[q])));
        }
      }
    }

    for (int32_t q = 0; q != 6; ++q) {
      _mm_storeu_pd(product + at(q, span, r), sum[q]);
    }
  }
}

}
