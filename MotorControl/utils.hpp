
#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>
#include <limits>
#include <algorithm>
#include <array>
#include <tuple>
#include <cmath>

/**
 * @brief Flash size register address
 */
#define ID_FLASH_ADDRESS (0x1FFF7A22)

/**
 * @brief Device ID register address
 */
#define ID_DBGMCU_IDCODE (0xE0042000)

/**
 * "Returns" the device signature
 *
 * Possible returns:
 *    - 0x0413: STM32F405xx/07xx and STM32F415xx/17xx)
 *    - 0x0419: STM32F42xxx and STM32F43xxx
 *    - 0x0423: STM32F401xB/C
 *    - 0x0433: STM32F401xD/E
 *    - 0x0431: STM32F411xC/E
 *
 * Returned data is in 16-bit mode, but only bits 11:0 are valid, bits 15:12 are always 0.
 * Defined as macro
 */
#define STM_ID_GetSignature() ((*(uint16_t *)(ID_DBGMCU_IDCODE)) & 0x0FFF)

/**
 * "Returns" the device revision
 *
 * Revisions possible:
 *    - 0x1000: Revision A
 *    - 0x1001: Revision Z
 *    - 0x1003: Revision Y
 *    - 0x1007: Revision 1
 *    - 0x2001: Revision 3
 *
 * Returned data is in 16-bit mode.
 */
#define STM_ID_GetRevision() (*(uint16_t *)(ID_DBGMCU_IDCODE + 2))

/**
* "Returns" the Flash size
*
* Returned data is in 16-bit mode, returned value is flash size in kB (kilo bytes).
*/
#define STM_ID_GetFlashSize() (*(uint16_t *)(ID_FLASH_ADDRESS))

#ifdef M_PI
#undef M_PI
#endif

#define MACRO_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MACRO_MIN(x, y) (((x) < (y)) ? (x) : (y))

// Math Constants
constexpr float M_PI = 3.14159265358979323846f;
constexpr float one_by_sqrt3 = 0.57735026919f;
constexpr float two_by_sqrt3 = 1.15470053838f;
constexpr float sqrt3_by_2 = 0.86602540378f;

// ----------------
// Inline functions
template<typename T>
constexpr T SQ(const T& x){
    return x * x;
}

// To allow use of -ffast-math we need to have a special check for nan
// that bypasses the "ignore nan" flag
__attribute__((optimize("-fno-finite-math-only")))
inline bool is_nan(float x) {
    return __builtin_isnan(x);
}

// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
inline float wrap_pm(float x, float pm_range) {
    while (x >= pm_range) x -= (2.0f * pm_range);
    while (x < -pm_range) x += (2.0f * pm_range);
    return x;
}

// Same as fmodf but result is positive and y must be positive
inline float fmodf_pos(float x, float y) {
    float res = wrap_pm(x, y);
    if (res < 0) res += y;
    return res;
}

inline float wrap_pm_pi(float x) {
    return wrap_pm(x, 2 * M_PI);
}

// Evaluate polynomials in an efficient way
// coeffs[0] is highest order, as per numpy.polyfit
// p(x) = coeffs[0] * x^deg + ... + coeffs[deg], for some degree "deg"
inline float horner_poly_eval(float x, const float *coeffs, size_t count) {
    float result = 0.0f;
    for (size_t idx = 0; idx < count; ++idx)
        result = (result * x) + coeffs[idx];
    return result;
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
inline int mod(const int dividend, const int divisor){
    int r = dividend % divisor;
    if (r < 0) r += divisor;
    return r;
}

// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
// as per the magnitude invariant clarke transform
// The magnitude of the alpha-beta vector may not be larger than sqrtf(3)/2
// Returns 0 on success, and -1 if the input was out of range
int SVM(float valpha, float vbeta, float vbus, float* tA, float* tB, float* tC);
void simple_svm(float alph, float beta, float vbus, float * A, float * B, float * C);

uint32_t micros(void);
void delay_us(uint32_t us);

extern "C" {
float our_arm_sin_f32(float x);
float our_arm_cos_f32(float x);
}

#define FAST_MATH_TABLE_SIZE 512
extern const float sinTable_f32[FAST_MATH_TABLE_SIZE + 1];


#endif  //__UTILS_H
