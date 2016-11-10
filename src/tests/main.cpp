#define CATCH_CONFIG_MAIN
#include <tests/catch.hpp>

/* Framework tests
 * Di, 8.November 2016
 * Hillary oder Donald? */

static bool close(double value, double refval, double maxdiff) { return (fabs(value - refval) < maxdiff); }

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
