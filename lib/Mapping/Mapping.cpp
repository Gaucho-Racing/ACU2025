#include "Mapping.h"
#include <Arduino.h>

float mapCurve(float x[], float y[], uint16_t size, float in) {
    // binary search
    uint16_t left, mid, right;
    left = 0; right = size - 1;
    if (in <= x[0]) return y[0];
    if (in >= x[size - 1]) return y[size - 1];
    while (right - left > 1) {
        mid = left + ((right - left) >> 1);
        if (x[mid] < in) {
            left = mid;
        }
        else if (x[mid] > in) {
            right = mid;
        }
        else {
            return y[mid];
        }
    }
    return map(in, x[left], x[right], y[left], y[right]);
}