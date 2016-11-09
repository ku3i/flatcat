#define CATCH_CONFIG_MAIN
#include <tests/catch.hpp>

/* Framework tests
 * Di, 8.November 2016
 * Hillary oder Donald? */

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

