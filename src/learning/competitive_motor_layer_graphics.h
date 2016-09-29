#ifndef COMPETITIVE_MOTOR_LAYER_GRAPHICS_H_INCLUDED
#define COMPETITIVE_MOTOR_LAYER_GRAPHICS_H_INCLUDED

#include <draw/draw.h>
#include <draw/axes3D.h>
#include <draw/plot3D.h>
#include <draw/network3D.h>
#include <draw/graphics.h>
#include <draw/display.h>

#include <learning/competitive_motor_layer.h>

class CompetitiveMotorLayer_Graphics : Graphics_Interface {
public:
    CompetitiveMotorLayer_Graphics( const CompetitiveMotorLayer& motor_layer
                                  , const float posx
                                  , const float posy
                                  , const float width )
    : motor_layer(motor_layer)
    , axis(posx, posy, 0.0, width, width, width, 0)
    , graph(motor_layer.get_number_of_motor_units(), axis, white)
    {
        dbg_msg("Creating Graphics Module for Competitive Motor Layer.");
    }

    void execute_cycle(void)
    {
        const std::vector<double>& weights = motor_layer.get_weights(motor_layer.last_selected_idx);
        assert(weights.size() >= 4); /**TODO make general */

        graph.update_node( motor_layer.last_selected_idx
                         , weights[1] // zero omitted
                         , weights[2]
                         , weights[3]
                         , fmin(2.0, motor_layer.get_unit(motor_layer.last_selected_idx).learning_capacity / MotorLayerConstants::initial_learning_capacity) );

        graph.update_node( motor_layer.recipient_idx
                         , fmin(2.0, motor_layer.get_unit(motor_layer.recipient_idx).learning_capacity / MotorLayerConstants::initial_learning_capacity) );

        graph.activated(motor_layer.last_selected_idx);
        //graph.special(gmes.get_to_insert());

        //TODO use for current variate weight vector
        //const std::vector<double>& variate = motor_layer.get_variate_weights();
        //plot.add_sample((float) variate[1], (float) variate[2], (float) variate[3]);

        /**TODO display motor weights in the 'cube' but display weights belonging to different joints with different colors. */
    }

    void draw(const pref& p) const
    {
        /* plots and drawings */
        glLineWidth(2.0f);
        glColor4f(1.0, 1.0, 1.0, 0.2);
        axis.draw(p.x_angle, p.y_angle);

        glColor4f(1.0, 1.0, 1.0, 1.0);
        graph.draw(p.x_angle, p.y_angle);

        /* text */
        for (std::size_t i = 0; i < motor_layer.motor_units.size(); ++i) {

            if (motor_layer.last_selected_idx == i) glColor3f(1.0, 0.5, 1.0);
            else if (motor_layer.motor_units[i].exists)
                glColor3f(.9f, .9f, .9f);
            else
                glColor3f(.3f,.3f,.3f);

            glprintf(-1.1, -1.2 - 0.05*i, 0.0, 0.04, "%2u" , i);
            draw::hbar(-1.4, -1.2 - 0.05*i, 0.3, 0.02, 0.5* motor_layer.motor_units[i].learning_capacity, MotorLayerConstants::initial_learning_capacity);
            draw_vector2(-1.0, -1.2 - 0.05*i, 0.045, 2.0, motor_layer.motor_units[i].weights.get_parameter(), 3.0);
        }


        if (motor_layer.is_adaption_enabled()) glColor3f(.9f,.9f,.9f);
        else                                   glColor3f(.3f,.3f,.3f);

        glprintf(-1.0, -1.1, 0.0, 0.05, "learning: %s", motor_layer.is_adaption_enabled() ? "enabled" : "disabled");
    }

private:

    const CompetitiveMotorLayer& motor_layer;

    /*draw*/
    const axes3D axis;
    network3D    graph;
};


#endif // COMPETITIVE_MOTOR_LAYER_GRAPHICS_H_INCLUDED
