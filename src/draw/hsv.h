#ifndef HSV_H
#define HSV_H

#include <basic/color.h>

namespace draw {

struct RGBColor { uint8_t r, g, b; };
struct HSVColor { uint8_t h, s, v; };


RGBColor hsv2rgb(HSVColor hsv);
HSVColor rgb2hsv(RGBColor rgb);

} /* namespace draw */

#endif /* HSV_H */
