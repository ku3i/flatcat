#define CATCH_CONFIG_MAIN
#include <tests/catch.hpp>
#include <common/modules.h>
/* Framework tests
 * Di, 8.November 2016
 * Hillary oder Donald? */

const double tolerance = 0.0001; // DO NOT CHANGE!


#include <common/incremental_average.h>
TEST_CASE( "Incremental Average", "[math]" ) {
    incremental_average inc;

    REQUIRE( inc.get() == 0.0 );
    REQUIRE( inc.get_num_samples() == 0 );

    inc.sample(-0.01337);

    REQUIRE( inc.get() == -0.01337 );
    REQUIRE( inc.get_num_samples() == 1 );

    inc.sample(-0.01337);

    REQUIRE( inc.get() == -0.01337 );
    REQUIRE( inc.get_num_samples() == 2 );

    inc.reset();
    for (unsigned i = 0; i < 10; ++i)
        inc.sample(1.0*i);

    REQUIRE( inc.get_num_samples() == 10 );
    REQUIRE( inc.get() == 4.5 );

    inc.reset();
    REQUIRE( inc.get() == 0.0 );
    REQUIRE( inc.get_num_samples() == 0 );

    double sum = .0;
    for (unsigned int i = 0; i < 10000; ++i) {
        double r = random_value(0.0, 23.42);
        sum += r;
        inc.sample(r);
    }
    sum /= 10000;
    REQUIRE( close(inc.get(), sum, tolerance) );
}

#include <common/modules.h>
TEST_CASE( "unwrap", "[math]" ) {

    /* Test of wrap2 and unwrap */
    double step = M_PI_4;
    double angle = -4*M_PI;
    double unwrapped = angle;

    for (unsigned int i = 0; i < 32; ++i)
    {
        double mod_angle = wrap2(angle);
        REQUIRE( close(mod_angle, wrap(angle), tolerance) ); // check 2nd version
        unwrapped = unwrap(mod_angle, unwrapped);
        REQUIRE( close(angle, unwrapped, tolerance) );
        angle += step;
    }
    for (unsigned int i = 0; i < 32; ++i)
    {
        double mod_angle = wrap2(angle);
        REQUIRE( close(mod_angle, wrap(angle), tolerance) ); // check 2nd version
        unwrapped = unwrap(mod_angle, unwrapped);
        REQUIRE( close(angle, unwrapped, tolerance) );
        angle -= step;
    }
}

#include <evolution/pool_strategy.h>
TEST_CASE( "biased_random_index", "[math]") {
    srand(2342);
    /* test case for biased random index */
    double selection_bias = 1.0;
    unsigned N = 7;
    std::vector<unsigned> bins(N,0);
    std::size_t total = 50000;

    for (unsigned i = 0; i < total; ++i){
        unsigned k = biased_random_index_inv(N, selection_bias);
        REQUIRE( k < bins.size() );
        ++bins[k];
    }

    auto draw_bar = [](unsigned len) {
        std::string foo{};
        for (unsigned i = 0; i < len;++i) foo.append("=");
        return foo;
    };

    unsigned counter = total;
    for (unsigned p = 0; p < bins.size(); ++p) {
        dbg_msg("%2u: %5u %5.2f %s", p, bins[p], 100.0*bins[p]/total, draw_bar(50*bins[p]/total).c_str());
        REQUIRE( bins[p] < counter );
        counter = bins[p];
    }
    srand((unsigned) time(0));
}


TEST_CASE( "random_index", "[math]") {
    srand((unsigned) time(0));

    /* check that random index does not overshoot range */
    REQUIRE( random_index(0) == 0 );
    unsigned counter = 0;
    for (unsigned i = 1; i < 1337; ++i) {
        unsigned rnd_idx = random_index(i);
        if (rnd_idx >= i) ++counter;
    }
    REQUIRE( counter == 0);

    /* weakly check uniform distribution */
    const unsigned num_bins = 13;
    std::vector<unsigned> bins(num_bins);
    for (unsigned i = 0; i < 13000; ++i) {
        ++bins.at(random_index(num_bins));
    }

    for (auto& b : bins) {
        REQUIRE( in_range(b, 900u, 1100u));
        dbg_msg("%u", b);
    }
}
