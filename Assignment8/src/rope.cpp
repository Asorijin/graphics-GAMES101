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
			Vector2D position = start + (end - start) * ((float)i / (num_nodes - 1));
			Mass* mass = new Mass(position, node_mass, false);
			masses.push_back(mass);
        }
//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
        for (int i = 0; i < num_nodes - 1; i++) {
			Spring* spring = new Spring(masses[i], masses[i + 1], k);
			springs.push_back(spring);
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Mass* m1 = s->m1, *m2 = s->m2;
			Vector2D dir = m1->position - m2->position;
			Vector2D force = s->k * (dir.norm() - s->rest_length) * dir.unit();
			m2->forces += force;
			m1->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
				m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
			     Vector2D v_t1 = m->velocity + m->forces / m->mass * delta_t;
				m->position += v_t1 * delta_t;
				m->velocity = v_t1;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Mass* m1 = s->m1, * m2 = s->m2;
            Vector2D dir = m1->position - m2->position;
            Vector2D force = s->k * (dir.norm() - s->rest_length) * dir.unit();
            m2->forces += force;
            m1->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
				m->forces += gravity * m->mass;
				m->position += (m->position - m->last_position)*(1-0.00005) + (m->forces / m->mass * delta_t * delta_t);
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
            }
			m->forces = Vector2D(0, 0);
			
        }
    }
}
