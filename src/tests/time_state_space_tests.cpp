#include <tests/catch.hpp>
#include <tests/test_robot.h>

#include <common/modules.h>
#include <control/sensorspace.h>
#include <learning/time_state_space.h>


namespace local_tests {
namespace time_state_space {


TEST_CASE( "TES Construction with length 1", "[time_embedded_signal]")
{
    double j = 13.37;
    time_embedded_signal<1> signal("foo", [&j](){ return j; });

    REQUIRE( signal.buffer.size() == 1 );

    REQUIRE( signal() == 0.0 );

    signal.execute_cycle();
    REQUIRE( signal() == 13.37 );

    j = 23.42;

    REQUIRE( signal() == 13.37 );
    signal.execute_cycle();

    REQUIRE( signal() == 23.42 );
}

TEST_CASE( "TES Construction with length N", "[time_embedded_signal]")
{
    double j = 13.37;
    const unsigned BUFLEN = 3;
    time_embedded_signal<BUFLEN> signal("foo", [&j](){ return j; });
    signal.execute_cycle();

    REQUIRE( signal.buffer.size() == BUFLEN );

    REQUIRE( signal() == 13.37 );

    j = 23.42;

    REQUIRE( signal[0] == 13.37 );

    signal.execute_cycle();

    REQUIRE( signal[0] == 23.42 );
    REQUIRE( signal[1] == 13.37 );
    REQUIRE( signal[2] ==  0.00 );

    j = 77.99;
    signal.execute_cycle();

    REQUIRE( signal[0] == 77.99 );
    REQUIRE( signal[1] == 23.42 );
    REQUIRE( signal[2] == 13.37 );

    signal.execute_cycle();
    signal.execute_cycle();

    REQUIRE( signal[0] == 77.99 );
    REQUIRE( signal[1] == 77.99 );
    REQUIRE( signal[2] == 77.99 );
}

TEST_CASE( "TSS Construction" , "[Time_State_Space]")
{
    Test_Robot robot(3,0);
    Time_State_Space<7> inputs{robot};

    //robot.set_joint

}

} /* time_state_space */
} /* local_tests */

