#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (int i = 0; i < num_nodes; i++) {
            Mass *mass = new Mass(start + (end - start) * (i / (float) num_nodes), node_mass, false);
            masses.push_back(mass);
            if (i > 0)
            {
                Spring *spring = new Spring(masses[i - 1], mass, k);
                springs.push_back(spring);
            }
        }

        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D m2Tom1 = s->m1->position - s->m2->position;
            Vector2D force = s->k * (m2Tom1.norm() - s->rest_length) * m2Tom1.unit();
            s->m1->forces -= force;
            s->m2->forces += force;
        }

        float dampingFactor = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                Vector2D acceleration = m->forces / m->mass;
                //m->position += m->velocity * delta_t; // explicit method

                m->velocity += acceleration * delta_t;
                m->position = m->position + m->velocity * delta_t; // semi-explicit method
                // TODO (Part 2): Add global damping
                m->velocity *= (1 - dampingFactor);
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        float dampingFactor = 0.00005;
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D m2Tom1 = s->m1->position - s->m2->position;
            Vector2D force = s->k * (m2Tom1.norm() - s->rest_length) * m2Tom1.unit();
            s->m1->forces -= force;
            s->m2->forces += force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D acceleration = m->forces / m->mass;

                m->position += (1- dampingFactor) * (m->position - m->last_position) + acceleration * delta_t * delta_t;
                m->last_position = temp_position;

                // TODO (Part 4): Add global Verlet damping

            }
            m->forces = Vector2D(0, 0);
        }
    }
}
