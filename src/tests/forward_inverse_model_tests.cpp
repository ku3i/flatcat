#include <tests/catch.hpp>
#include <tests/test_robot.h>

#include <common/modules.h>
#include <learning/forward_inverse_model.hpp>


namespace local_tests {

namespace forward_inverse_model_tests {

typedef learning::BidirectionalModel<learning::LinearTransfer<>> BidirectionalLinearModelType;
typedef learning::BidirectionalModel<learning::TanhTransfer<>> BidirectionalNonlinearModelType;


TEST_CASE( "twopart_vector access and manipulation" , "[twopart_vector]")
{
    typedef std::vector<double> Vector_t;

    Vector_t v1(3);
    Vector_t v2(5);

    learning::twopart_vector<Vector_t> vec(v1,v2);

    // check sizes
    REQUIRE( vec      .size() == 8 );
    REQUIRE( vec.part0.size() == 3 );
    REQUIRE( vec.part1.size() == 5 );

    for (std::size_t i = 0; i < vec.size(); ++i)
        REQUIRE( vec[i] == 0 );

    auto& A = vec.part0;
    auto& B = vec.part1;

    for (std::size_t i = 0; i < A.size(); ++i)
        REQUIRE( A[i] == 0 );

    for (std::size_t i = 0; i < B.size(); ++i)
        REQUIRE( B[i] == 0 );

    A[0] = 0;
    A[1] = 1;
    A[2] = 2;

    B[0] = 3;
    B[1] = 4;
    B[2] = 5;
    B[3] = 6;
    B[4] = 7;


    REQUIRE( v1 == A );
    REQUIRE( v2 == B );

    // elements can be read
    for (std::size_t i = 0; i < vec.size(); ++i) {
        dbg_msg("%u = %f", i, vec[i]);
        REQUIRE( close(vec[i], i, 0.00001) );
    }

    // elements can be written
    vec[4] = 23;
    REQUIRE( B[1] == 23 );

    vec[1] = 17;
    REQUIRE( A[1] == 17 );
}

TEST_CASE( "forward_inverse_model construction" , "[forward_inverse_model]")
{
    srand(time(0)); // set random seed

    double random_range = 0.01;

    BidirectionalLinearModelType model(13, 7, random_range);

    /* check weights are not zero, but randomized */
    auto const& weights = model.get_weights();
    double sum = .0;
    int diff = 0;
    for (std::size_t i = 0; i < weights.size(); ++i)
        for (std::size_t j = 0; j < weights[i].size(); ++j) {
            diff += ( weights[i][j] != .0 )? 0 : 1;
            sum += weights[i][j];
        }

    /* check randomize_weight_matrix() is executed */
    REQUIRE( diff == 0 );
    const double max_range = 0.5* random_range * weights.size()*weights[0].size();
    dbg_msg("Max rand: %e < %e", std::abs(sum), max_range);
    REQUIRE( std::abs(sum) <= max_range ); // check small
    REQUIRE( std::abs(sum) != 0. ); // but not zero

    // check matrix size
    REQUIRE( weights   .size() == 7   );
    REQUIRE( weights[0].size() == 13  );

    auto const& inputs  = model.get_inputs ();
    auto const& outputs = model.get_outputs();

    // check vector size
    REQUIRE( inputs.size() == 13 );
    REQUIRE( outputs.size() == 7  );

    // check in and outputs are zero on initialization
    for (std::size_t i = 0; i < outputs.size(); ++i)
        REQUIRE( outputs[i] == .0 );

    for (std::size_t i = 0; i < inputs.size(); ++i)
        REQUIRE( inputs[i] == .0 );

    model.propagate_forward(inputs);
    for (std::size_t i = 0; i < outputs.size(); ++i)
        REQUIRE( outputs[i] == .0 );

    model.propagate_inverse(outputs);
    for (std::size_t i = 0; i < outputs.size(); ++i)
        REQUIRE( inputs[i] == .0 );

    BidirectionalLinearModelType model2 = model;
}


TEST_CASE( "forward_inverse_model learning (linear)", "[forward_inverse_model]")
{
    dbg_msg("linear");
    srand(time(0)); // set random seed

    const double learning_rate = 0.005;

    std::vector<double> X = {1,0.5,1,-1,0,1,1,0,-1,-1,1,0.75,1,0,1,0.5,-1,1};
    std::vector<double> Y = {1,0,-1,0,-1,1,0,-1,-1,0.5};

    BidirectionalLinearModelType model(X.size(), Y.size(), 0.01);

    /* check error is decreasing (forward) */
    auto const& Y_ = model.propagate_forward(X);
    auto const& X_ = model.propagate_inverse(Y); // inverse direction

    double ery0 = squared_distance(Y, model.get_outputs());
    double erx0 = squared_distance(X, model.get_inputs ()); // inverse error

    double ery1, erx1;

    for (std::size_t trials = 0; trials < 500; ++trials) {
        // adapt
        model.adapt(X, Y, learning_rate);

        // verify
        model.propagate_forward(X);
        model.propagate_inverse(Y);

        ery1 = squared_distance(Y, model.get_outputs());
        erx1 = squared_distance(X, model.get_inputs ());
        //dbg_msg("E_fw: %+e | E_bw: %+e",ery1, erx1);
        REQUIRE( ery0 > ery1 );
        REQUIRE( erx0 > erx1 );
        ery0 = ery1;
        erx0 = erx1;
    }

    model.propagate_forward(X);
    print_vector(Y);
    print_vector(Y_);

    REQUIRE( close(Y_, Y, 0.01) );

    model.propagate_inverse(Y);
    print_vector(X);
    print_vector(X_);

    REQUIRE( close(X_, X, 0.01) );

}

TEST_CASE( "forward_inverse_model learning (non-linear)", "[forward_inverse_model]")
{
    dbg_msg("non-linear");
    srand(time(0)); // set random seed

    const double learning_rate = 0.01;

    std::vector<double> X = {1,0.5,1,-1,0,1,1,0,-1,-1,1,0.75,1,0,1,0.5,-1,1};
    std::vector<double> Y = {1,0,-1,0,-1,1,0,-1,-1,0.5};

    BidirectionalNonlinearModelType model(X.size(), Y.size(), 0.01);

    /* check error is decreasing (forward) */
    auto const& Y_ = model.propagate_forward(X);
    auto const& X_ = model.propagate_inverse(Y); // inverse direction

    double ery0 = squared_distance(Y, model.get_outputs());
    double erx0 = squared_distance(X, model.get_inputs ()); // inverse error

    double ery1, erx1;

    for (std::size_t trials = 0; trials < 2500; ++trials) {
        // adapt
        model.adapt(X, Y, learning_rate);

        // verify
        model.propagate_forward(X);
        model.propagate_inverse(Y);

        ery1 = squared_distance(Y, model.get_outputs());
        erx1 = squared_distance(X, model.get_inputs ());
        //dbg_msg("E_fw: %+e | E_bw: %+e",ery1, erx1);
        REQUIRE( ery0 > ery1 );
        REQUIRE( erx0 > erx1 );
        ery0 = ery1;
        erx0 = erx1;
    }

    model.propagate_forward(X);
    print_vector(Y);
    print_vector(Y_);
    for (unsigned i = 0; i < Y_.size(); ++i)
        REQUIRE( close(Y_[i], Y[i], 0.03) );

    dbg_msg("---");
    model.propagate_inverse(Y);
    print_vector(X);
    print_vector(X_);

    REQUIRE( close(X_, X, 0.03) );

}



}} // namespace local_tests::forward_inverse_model_tests


