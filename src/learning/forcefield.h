#ifndef FORCEFIELD_H_INCLUDED
#define FORCEFIELD_H_INCLUDED

#include <basic/vector3.h>
#include <draw/axes3D.h>
#include <draw/network3D.h>
#include <draw/graphics.h>

#include <learning/gmes.h>
#include <learning/expert.h>

namespace ff_constants
{
    const double mass           = 0.1; // mass of one mass point 10g
    const double k_spring       = 0.05;
    const double damper         = 2*2*sqrt(mass * k_spring); // constant for the dampers
    const double distance_0     = 0.2;
    const double fluid_friction = 1.0 * mass;
    const double center_gravity = 0.0001;


    /* wall collision and damping */
    const double wall_damping = 0.5;
    const double wall_decay   = 0.8;
    const double sticktion    = 0.1;
}

class Particle
{
public:
    Particle()
    : position(random_value(-0.50,0.50),random_value(-0.50,0.50),random_value(-0.50,0.50))
    , velocity(random_value(-0.10,0.10),random_value(-0.10,0.10),random_value(-0.10,0.10))
    , force(.0)
    , mass(ff_constants::mass)
    {}

    void reset(void)
    {
        position = 0.0;
        velocity = 0.0;
        force    = 0.0;
    }

    Vector3 position;
    Vector3 velocity;
    Vector3 force;
    const double mass;
};


inline void wall_contact(Particle& p)
{

    if (p.position.x < -1.0 || p.position.x > +1.0) {
        p.velocity.x *= -ff_constants::wall_decay;
        p.velocity.y *=  (fabs(p.velocity.y) > ff_constants::sticktion) * ff_constants::wall_damping;
        p.velocity.z *=  (fabs(p.velocity.z) > ff_constants::sticktion) * ff_constants::wall_damping;
    }

    if (p.position.y < -1.0 || p.position.y > +1.0) {
        p.velocity.x *=  (fabs(p.velocity.x) > ff_constants::sticktion) * ff_constants::wall_damping;
        p.velocity.y *= -ff_constants::wall_decay;
        p.velocity.z *=  (fabs(p.velocity.z) > ff_constants::sticktion) * ff_constants::wall_damping;
    }

    if (p.position.x < -1.0 || p.position.x > +1.0) {
        p.velocity.x *=  (fabs(p.velocity.x) > ff_constants::sticktion) * ff_constants::wall_damping;
        p.velocity.y *=  (fabs(p.velocity.y) > ff_constants::sticktion) * ff_constants::wall_damping;
        p.velocity.z *= -ff_constants::wall_decay;
    }
    p.position.clip(1.0);
}

Vector3 gravity(const Vector3& vec1, const Vector3& vec2, double g_const, double min_distance)
{
    Vector3 force(0.0);
    double r = distance(vec1, vec2);

    if (r < min_distance)
        force = -g_const * (vec1 - vec2);
    else
        force = -g_const * (vec1 - vec2) / (r * r);

    return force;
}

Vector3 repell(const Vector3& vec1, const Vector3& vec2, double g_const, double min_distance)
{
    Vector3 force(0.0);
    double r = distance(vec1, vec2);
    double k = g_const * clip(2*min_distance - fabs(r), 0.0, 2*min_distance);// + 0.0001;
    force = k * ((vec1 - vec2)/r);
    return force;
}

class Force_Field : public Graphics_Interface {
public:
    Force_Field(const GMES& gmes)
    : gmes(gmes)
    , expert(gmes.expert)
    , activations(gmes.get_activations())
    , particle(expert.get_max_number_of_experts())
    , dt(0.001)
    , ff_axis(.0, .0, .0, 1., 1., 1., 0)
    , ff_graph(expert.get_max_number_of_experts(), ff_axis, white)
    {
        dbg_msg("Creating Forcefield");
    }

    void calculate_new_node_position(void) {
        std::size_t winner = gmes.get_winner();
        particle[winner].reset();

        std::size_t count_transitions = 0;
        Vector3 new_position = 0.0;
        new_position.random(-0.5*ff_constants::distance_0, +0.5*ff_constants::distance_0);

        for (std::size_t k = 0; k < expert[winner].transition.size(); ++k) {
            if (expert[winner].exists_transition(k)) {
                new_position += particle[k].position;
                ++count_transitions;
            }
        }
        new_position /= count_transitions;
        particle[winner].position = new_position;
    }

    void execute_cycle(uint64_t cycle)
    {
        if (gmes.has_new_node())
            calculate_new_node_position();

        for (unsigned int i = 0; i < expert.get_max_number_of_experts(); ++i)
        {
            particle[i].force  = .0; //clear
            particle[i].force += -ff_constants::fluid_friction * particle[i].velocity;

            if (expert[i].exists) //TODO check also if node is isolated
            {
                particle[i].force += gravity(particle[i].position, Vector3(.0), ff_constants::center_gravity, ff_constants::distance_0);

                for (unsigned int k = 0; k < expert.get_max_number_of_experts(); ++k)
                {
                    if (i != k && expert[k].exists)
                    {
                        double dx = distance(particle[i].position, particle[k].position);

                        if (expert[i].exists_transition(k) || expert[k].exists_transition(i)) {
                            particle[i].force += -ff_constants::k_spring * clip(dx - ff_constants::distance_0, ff_constants::distance_0) * ((particle[i].position - particle[k].position)/dx);
                            //TODO add damper forces
                        }
                        else
                            particle[i].force += repell(particle[i].position, particle[k].position, 1.0, ff_constants::distance_0);

                    }
                }
            }

            particle[i].velocity += particle[i].force / particle[i].mass * dt;
            particle[i].position += particle[i].velocity * dt;

            wall_contact(particle[i]);
        }

        /* update drawings */
        for (unsigned int i = 0; i < expert.get_max_number_of_experts(); ++i)
        {
            ff_graph.update_node(i,
                                 particle[i].position.x,
                                 particle[i].position.y,
                                 particle[i].position.z,
                                 fmin(2.0, expert[i].learning_capacity));

            for (unsigned int k = 0; k < expert.get_max_number_of_experts(); ++k)
                ff_graph.update_edge(i, k, (expert[i].exists_transition(k) ? expert[i].transition[k]*192 : 0)); //TODO use color of edge to display eligibility traces
        }
        ff_graph.activated(gmes.get_winner());
        ff_graph.special(gmes.get_to_insert());
    }

    void draw(const pref& p) const
    {
        ff_axis .draw(p.x_angle, p.y_angle);
        ff_graph.draw(p.x_angle, p.y_angle);
        glprintf(0.8, 0.7, 0.0, 0.03, "%u/%u", gmes.get_number_of_experts(), gmes.get_max_number_of_experts());
    }

    const GMES&            gmes;
    const Expert_Vector&   expert;
    const VectorN&         activations;

    std::vector<Particle>  particle;
    const double           dt;

    /* drawing */
    axes3D    ff_axis;
    network3D ff_graph;

};

#endif // FORCEFIELD_H_INCLUDED
