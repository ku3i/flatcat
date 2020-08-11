#include <tests/catch.hpp>
#include <tests/test_robot.h>

#include <limits>
#include <common/modules.h>
#include <learning/forward_inverse_model.hpp>


namespace local_tests {

namespace neural_model_tests {

typedef learning::NeuralModel<learning::TanhTransfer<>> NeuralModelType;
typedef learning::InverseNeuralModel InverseModelType;


TEST_CASE( "neural_model construction" , "[neural_model]")
{
    srand(time(0)); // set random seed

    double random_range = 0.01;

    NeuralModelType model(13, 7, random_range);

    learning::model::vector_t inputs(13);

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

    //auto const& inputs  = model.get_inputs ();
    auto const& outputs = model.get_outputs();

    // check vector size
    //REQUIRE( inputs.size() == 13 );
    REQUIRE( outputs.size() == 7  );

    // check in and outputs are zero on initialization
    for (std::size_t i = 0; i < outputs.size(); ++i)
        REQUIRE( outputs[i] == .0 );

//    for (std::size_t i = 0; i < inputs.size(); ++i)
//        REQUIRE( inputs[i] == .0 );

    model.propagate(inputs);
    for (std::size_t i = 0; i < outputs.size(); ++i)
        REQUIRE( outputs[i] == .0 );

    NeuralModelType model2 = model;
}


TEST_CASE( "neural_model learning (non-linear)", "[neural_model]")
{
    srand(time(0)); // set random seed

    const double learning_rate = 0.005;

    std::vector<double> X = {1,0.5,1,-1,0,1,1,0,-1,-1,1,0.75,1,0,1,0.5,-1,1};
    std::vector<double> Y = {0.7,0,-0.8,0,-0.5,0.4,0,-0.5,-0.7,0.5};

    NeuralModelType model(X.size(), Y.size(), 0.01);

    /* check error is decreasing (forward) */
    auto const& Y_ = model.propagate(X);

    double err0 = squared_distance(Y, model.get_outputs());
    double err1;

    for (std::size_t trials = 0; trials < 500; ++trials) {
        // adapt
        model.adapt(X, Y, learning_rate);

        // verify
        model.propagate(X);
        err1 = squared_distance(Y, model.get_outputs());
        REQUIRE( err0 > err1 );
        err0 = err1;

    }

    model.propagate(X);
    print_vector(Y);
    print_vector(Y_);

    REQUIRE( close(Y_, Y, 0.01) );

}

TEST_CASE( "inverse-neural model learning (non-linear)", "[inverse_neural_model]")
{
    srand(time(0)); // set random seed

    const double learning_rate = 0.0025;

    // still a reasonable number
    double f = InverseModelType::G(0.999999999999999943);
    sts_msg("%f", f);
    REQUIRE( close(f, 18.714974, 0.00001) );

    // infinity
    double g = InverseModelType::G(1.0);
    REQUIRE( g == std::numeric_limits<double>::infinity() );

    // not a number anymore
    double h = InverseModelType::G(2.0);
    REQUIRE( std::isnan(h) );

    std::vector<double> X = {0.9,0.5,0.9,-0.9,0,0.9,0.9,0,-0.9,-0.9};
    std::vector<double> Y = {0.7,0,-0.8,0,-0.5,0.4,0,-0.5,-0.7,0.5};

    InverseModelType model(X.size(), Y.size(), 0.01);
     NeuralModelType proof(Y.size(), X.size(), 0.01); // for comparison


    /* check error is decreasing (forward) */
    auto const& Y_ = model.propagate(X);

    double err0 = squared_distance(Y, model.get_outputs());
    double err1;

    for (std::size_t trials = 0; trials < 800; ++trials) {
        // adapt
        model.adapt(X, Y, learning_rate);
        proof.adapt(Y, X, learning_rate*20); // for comparison

        // verify
        model.propagate(X);
        proof.propagate(Y);
        auto const& O = model.get_outputs();
        //print_vector(O,"o");
        err1 = squared_distance(Y, O);
        REQUIRE( err0 > err1 );
        err0 = err1;
    }

    /* Y_ = */ model.propagate(X);
    auto const& X_ = proof.propagate(Y_);

    print_vector(Y ,"target output");
    print_vector(Y_,"___prediction");
    REQUIRE( close(Y_, Y, 0.01) );

    print_vector(X ,"_____original");
    print_vector(X_,"reconstructed");
    REQUIRE( close(X_, X, 0.01) );
}


} /* namespace local_tests */
} /* namespace neural_model_tests */

