#define CATCH_CONFIG_MAIN
#include <tests/catch.hpp>

/* Framework tests
 * Di, 8.November 2016
 * Hillary oder Donald? */

#include <common/incremental_average.h>
TEST_CASE( "Incremental Average", "[math]" ) {
    incremental_average inc;

    REQUIRE( inc.mean == 0.0 );
    REQUIRE( inc.num_samples == 0 );

    for (unsigned i = 0; i < 10; ++i)
        inc.sample(1.0*i);

    REQUIRE( inc.num_samples == 10 );
    REQUIRE( inc.mean == 4.5 );

    inc.reset();
    REQUIRE( inc.mean == 0.0 );
    REQUIRE( inc.num_samples == 0 );

    inc.reset(13.37, 42);
    REQUIRE( inc.mean == 13.37 );
    REQUIRE( inc.num_samples == 42 );
}

