#pragma once

#include "immintrin.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace physecs {
    struct Vec3W {
        __m128 x;
        __m128 y;
        __m128 z;

        Vec3W(__m128 x, __m128 y, __m128 z) : x(x), y(y), z(z) {}

        explicit Vec3W(glm::vec3* v) {
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

    struct QuatW {
        __m128 x;
        __m128 y;
        __m128 z;
        __m128 w;

        QuatW(__m128 w, __m128 x, __m128 y, __m128 z) : x(x), y(y), z(z), w(w) {}

        explicit QuatW(__m128 s, const Vec3W& v) : x(v.x), y(v.y), z(v.z), w(s) {}

        explicit QuatW(glm::quat* q) {
            const float* f = glm::value_ptr(q[0]);

            const auto r0 = _mm_load_ps(f);
            const auto r1 = _mm_load_ps(f + 4);
            const auto r2 = _mm_load_ps(f + 8);
            const auto r3 = _mm_load_ps(f + 12);

            const auto xyLo = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(1, 0, 1, 0));
            const auto zwLo = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(3, 2, 3, 2));
            const auto xyHi = _mm_shuffle_ps(r2, r3, _MM_SHUFFLE(1, 0, 1, 0));
            const auto zwHi = _mm_shuffle_ps(r2, r3, _MM_SHUFFLE(3, 2, 3, 2));

            x = _mm_shuffle_ps(xyLo, xyHi, _MM_SHUFFLE(2, 0, 2, 0));
            y = _mm_shuffle_ps(xyLo, xyHi, _MM_SHUFFLE(3, 1, 3, 1));
            z = _mm_shuffle_ps(zwLo, zwHi, _MM_SHUFFLE(2, 0, 2, 0));
            w = _mm_shuffle_ps(zwLo, zwHi, _MM_SHUFFLE(3, 1, 3, 1));
        }

        void store(glm::quat* q) const {
            const auto xyLo = _mm_unpacklo_ps(x, y);
            const auto zwLo = _mm_unpacklo_ps(z, w);
            const auto xyHi = _mm_unpackhi_ps(x, y);
            const auto zwHi = _mm_unpackhi_ps(z, w);

            const auto r0 = _mm_shuffle_ps(xyLo, zwLo, _MM_SHUFFLE(1, 0, 1, 0));
            const auto r1 = _mm_shuffle_ps(xyLo, zwLo, _MM_SHUFFLE(3, 2, 3, 2));
            const auto r2 = _mm_shuffle_ps(xyHi, zwHi, _MM_SHUFFLE(1, 0, 1, 0));
            const auto r3 = _mm_shuffle_ps(xyHi, zwHi, _MM_SHUFFLE(3, 2, 3, 2));

            float* f = glm::value_ptr(q[0]);

            _mm_store_ps(f, r0);
            _mm_store_ps(f + 4, r1);
            _mm_store_ps(f + 8, r2);
            _mm_store_ps(f + 12, r3);
        }
    };

    inline __m128 dotW(const Vec3W& v1, const Vec3W& v2) {
        const auto x = _mm_mul_ps(v1.x, v2.x);
        const auto y = _mm_mul_ps(v1.y, v2.y);
        const auto z = _mm_mul_ps(v1.z, v2.z);

        auto result = _mm_add_ps(x, y);
        result = _mm_add_ps(result, z);

        return result;
    }

    inline __m128 dotW(const QuatW& q1, const QuatW& q2) {
        const auto x = _mm_mul_ps(q1.x, q2.x);
        const auto y = _mm_mul_ps(q1.y, q2.y);
        const auto z = _mm_mul_ps(q1.z, q2.z);
        const auto w = _mm_mul_ps(q1.w, q2.w);

        auto result = _mm_add_ps(x, y);
        result = _mm_add_ps(result, z);
        result = _mm_add_ps(result, w);

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

    inline Vec3W operator* (__m128 a, const Vec3W &b) {
        return Vec3W(a * b.x, a * b.y, a * b.z);
    }

    inline void operator+= (Vec3W& a,Vec3W b) {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
    }

    inline void operator-=(Vec3W& a,Vec3W b) {
        a.x -= b.x;
        a.y -= b.y;
        a.z -= b.z;
    }

    inline void operator+= (QuatW& p, const QuatW& q) {
        p.x += q.x;
        p.y += q.y;
        p.z += q.z;
        p.w += q.w;
    }

    inline void operator-=(QuatW& p, const QuatW& q) {
        p.x -= q.x;
        p.y -= q.y;
        p.z -= q.z;
        p.w -= q.w;
    }

    inline QuatW operator* (const QuatW& p, const QuatW& q) {
        return {
            p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
            p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
            p.w * q.y + p.y * q.w + p.z * q.x - p.x * q.z,
            p.w * q.z + p.z * q.w + p.x * q.y - p.y * q.x
        };
    }

    inline QuatW operator* (__m128 a, const QuatW& q) {
        return {
            a * q.w,
            a * q.x,
            a * q.y,
            a * q.z
        };
    }

    inline QuatW normalize(const QuatW& q) {
        const auto one = _mm_set1_ps(1);
        const auto len = _mm_sqrt_ps(dotW(q, q));
        const auto oneOverLen = one / len;

        return {
            oneOverLen * q.w,
            oneOverLen * q.x,
            oneOverLen * q.y,
            oneOverLen * q.z
        };
    }
}
