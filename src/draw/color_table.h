#ifndef COLOR_TABLE_H_INCLUDED
#define COLOR_TABLE_H_INCLUDED

#include <vector>
#include <random>
#include <algorithm>
#include <basic/color.h>


class ColorTable
{
    std::vector<Color4> colors;
    unsigned int num_variations;
    unsigned int max_colors;
    float get(unsigned int x) { return (x * 1.0)/(num_variations-1); }

public:
    ColorTable(unsigned int num_variations, bool initialize_randomized = false)
    : colors()
    , num_variations(num_variations)
    , max_colors(num_variations * num_variations * num_variations)
    {
        assert(num_variations > 1);
        unsigned int i=0;
        colors.reserve(max_colors);
        for (unsigned int r = num_variations; r-- > 0; )
            for (unsigned int g = num_variations; g-- > 0; )
                for (unsigned int b = num_variations; b-- > 0; ) {
                    colors.emplace_back(get(r), get(g), get(b), 1.0);
                    dbg_msg("%2u: %1.2f %1.2f %1.2f",i++, get(r), get(g), get(b));
                }
        if (initialize_randomized) randomize();
    }

    const Color4& get_color(unsigned int index) const { return colors[index % max_colors]; }

    void randomize() {
        auto engine = std::default_random_engine{};
        std::shuffle(std::begin(colors), std::end(colors), engine);
    }
};

#endif // COLOR_TABLE_H_INCLUDED
