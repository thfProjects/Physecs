#pragma once

#include "immintrin.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace physecs {

    struct vec3W {
         __m128 x;
         __m128 y;
         __m128 z;

        vec3W(__m128 x, __m128 y, __m128 z) : x(x), y(y), z(z) {}

        explicit vec3W(glm::vec3* v) {
            const float* f = glm::value_ptr(v[0]);

            const auto r0 = _mm_load_ps(f);
            const auto r1 = _mm_load_ps(f + 4);
            const auto r2 = _mm_load_ps(f + 8);

            const auto xy = _mm_shuffle_ps(r1, r2, _MM_SHUFFLE(2, 1, 3, 2));
            const auto yz = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(1, 0, 2, 1));

            x = _mm_shuffle_ps(r0, xy, _MM_SHUFFLE(2, 0, 3, 0));
            y = _mm_shuffle_ps(yz, xy, _MM_SHUFFLE(3, 1, 2, 0));
            z = _mm_shuffle_ps(yz, r2, _MM_SHUFFLE(3, 0, 3, 1));
        }

        void store(glm::vec3* v) const {
            const auto xy = _mm_unpackhi_ps(x, y);
            const auto yz = _mm_unpacklo_ps(y, z);

            auto r0 = _mm_shuffle_ps(x, yz, _MM_SHUFFLE(1, 0, 1, 0));
            r0 = _mm_shuffle_ps(r0, r0, _MM_SHUFFLE(1, 3, 2, 0));

            const auto r1 = _mm_shuffle_ps(yz, xy, _MM_SHUFFLE(1, 0, 3, 2));

            auto r2 = _mm_shuffle_ps(xy, z, _MM_SHUFFLE(3, 2, 3, 2));
            r2 = _mm_shuffle_ps(r2, r2, _MM_SHUFFLE(3, 1, 0, 2));

            float* f = glm::value_ptr(v[0]);

            _mm_store_ps(f, r0);
            _mm_store_ps(f + 4, r1);
            _mm_store_ps(f + 8, r2);
        }
    };

    inline __m128 dotW(const vec3W& v1, const vec3W& v2) {
        const auto x = _mm_mul_ps(v1.x, v2.x);
        const auto y = _mm_mul_ps(v1.y, v2.y);
        const auto z = _mm_mul_ps(v1.z, v2.z);

        auto result = _mm_add_ps(x, y);
        result = _mm_add_ps(result, z);

        return result;
    }

    inline __m128 operator+ (__m128 a, __m128 b) {
        return _mm_add_ps(a, b);
    }

    inline __m128 operator- (__m128 a, __m128 b) {
        return _mm_sub_ps(a, b);
    }

    inline void operator+=(__m128& a, __m128 b) {
        a = _mm_add_ps(a, b);
    }

    inline void operator-=(__m128& a, __m128 b) {
        a = _mm_sub_ps(a, b);
    }

    inline __m128 operator* (__m128 a, __m128 b) {
        return _mm_mul_ps(a, b);
    }

    inline __m128 operator/ (__m128 a, __m128 b) {
        return _mm_div_ps(a, b);
    }

    inline __m128 operator- (__m128 a) {
        const auto zero = _mm_setzero_ps();
        return _mm_sub_ps(zero, a);
    }

    inline vec3W operator* (__m128 a, const vec3W &b) {
        return vec3W(a * b.x, a * b.y, a * b.z);
    }

    inline void operator+= (vec3W& a,vec3W b) {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
    }

    inline void operator-=(vec3W& a,vec3W b) {
        a.x -= b.x;
        a.y -= b.y;
        a.z -= b.z;
    }
}
