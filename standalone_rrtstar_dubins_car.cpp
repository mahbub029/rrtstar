// Standard header files
#include<iostream>
#include <fstream>

using namespace std;


// SMP HEADER FILES ------
#include <smp/components/samplers/uniform.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/dubins.hpp>
#include <smp/components/collision_checkers/standard.hpp>
#include <smp/components/multipurpose/minimum_time_reachability.hpp>

#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>


// SMP TYPE DEFINITIONS -------
using namespace smp;

// State, input, vertex_data, and edge_data definitions
typedef state_dubins state_t;
typedef input_dubins input_t;
typedef minimum_time_reachability_vertex_data vertex_data_t;
typedef minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure

typedef struct _typeparams {
    typedef state_t state;
    typedef input_t input;
    typedef vertex_data_t vertex_data;
    typedef edge_data_t edge_data;
} typeparams;

// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef sampler_uniform<typeparams, 3> sampler_t;
typedef distance_evaluator_kdtree<typeparams, 3> distance_evaluator_t;
typedef extender_dubins<typeparams> extender_t;
typedef collision_checker_standard<typeparams, 2> collision_checker_t;
typedef minimum_time_reachability<typeparams, 2> min_time_reachability_t;

// Define all algorithm types
typedef rrtstar<typeparams> rrtstar_t;

typedef vertex<typeparams> vertex_t;
typedef edge<typeparams> edge_t;

void nearCapture(rrtstar_t planner, distance_evaluator_t distance_evaluator, state_t *state_extended) {
    //list<void*> list_vertices_in_ball;
    //double radius = planner.parameters.get_gamma() * pow (log(planner.get_num_vertices())/planner.get_num_vertices(),  1.0 /( (double)(planner.parameters.get_dimension()) )  );


    //distance_evaluator.find_near_vertices_r (state_extended, radius, &list_vertices_in_ball);

    // for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
    //vertex_t *vertex_curr = (vertex_t*)(*iter);
    //}

}

int
main() {




    // 1. CREATE PLANNING OBJECTS

    // 1.a Create the components
    sampler_t sampler1, sampler2;
    distance_evaluator_t distance_evaluator1, distance_evaluator2;
    extender_t extender1, extender2;
    collision_checker_t collision_checker1, collision_checker2;
    min_time_reachability_t min_time_reachability1, min_time_reachability2;

    // 1.b Create the planner algorithm -- Note that the min_time_reachability variable acts both
    //                                       as a model checker and a cost evaluator.
    rrtstar_t planner1(sampler1, distance_evaluator1, extender1, collision_checker1,
            min_time_reachability1, min_time_reachability1);
    rrtstar_t planner2(sampler2, distance_evaluator2, extender2, collision_checker2,
            min_time_reachability2, min_time_reachability2);


    planner1.parameters.set_phase(2); // The phase parameter can be used to run the algorithm as an RRT,
    planner2.parameters.set_phase(2); // See the documentation of the RRG algorithm for more information.

    planner1.parameters.set_gamma(35.0); // Set this parameter should be set at least to the side length of
    planner2.parameters.set_gamma(35.0); //   the (bounded) state space. E.g., if the state space is a box
    //   with side length L, then this parameter should be set to at
    //   least L for rapid and efficient convergence in trajectory space.
    planner1.parameters.set_dimension(3);

    planner1.parameters.set_max_radius(20.0); // This parameter should be set to a high enough value. In practice,
    //   one can use smaller values of this parameter to get a good
    planner2.parameters.set_dimension(3);
    planner2.parameters.set_max_radius(20.0); //   solution quickly, while preserving the asymptotic optimality.






    // 2. INITALIZE PLANNING OBJECTS

    // 2. Initialize the sampler component
    region<3> sampler_support;
    sampler_support.center[0] = 0.0;
    sampler_support.center[1] = 0.0;
    sampler_support.center[2] = 0.0;
    sampler_support.size[0] = 20.0;
    sampler_support.size[1] = 20.0;
    sampler_support.size[2] = 2.0 * M_PI;
    sampler1.set_support(sampler_support);
    sampler2.set_support(sampler_support);


    // 2.b Initialize the distance evaluator
    //     Nothing to initialize. One could change the kdtree weights.


    // 2.c Initialize the extender
    ofstream obstacles, units;
    obstacles.open("obstacles.txt");
    units.open("units.txt");



    // 2.d Initialize the collision checker
    region<2> obstacle_new;
    for (int i = 0; i < 2; i++) {
        obstacle_new.center[i] = 5.0;
        obstacle_new.size[i] = 5.0;
    }
    collision_checker1.add_obstacle(obstacle_new);
    collision_checker2.add_obstacle(obstacle_new);
    obstacles << obstacle_new.center[0] << "," << obstacle_new.center[1] << "," << obstacle_new.size[0] << "," << obstacle_new.size[1] << "\n";
    // 2.d add landmarks

    region<2> landmark_new1, landmark_new2;
    for (int i = 0; i < 2; i++) {
        landmark_new1.center[i] = -5.0;
        landmark_new1.size[i] = 2.0;
    }
    landmark_new1.center[1] = 5.0;
    landmark_new2.center[0] = 8.0;
    landmark_new2.center[1] = 8.0;
    landmark_new2.size[0] = 2.0;
    landmark_new2.size[1] = 2.0;
    collision_checker1.add_landmark(landmark_new1);
    collision_checker1.add_landmark(landmark_new2);
    collision_checker2.add_landmark(landmark_new1);
    collision_checker2.add_landmark(landmark_new2);
    units << landmark_new1.center[0] << "," << landmark_new1.center[1] << "\n";
    units << landmark_new2.center[0] << "," << landmark_new2.center[1] << "\n";

    // 2.e Initialize the model checker and the cost evaluator
    region<2> region_goal1;
    region_goal1.center[0] = 8.0;
    region_goal1.center[1] = 8.0;
    region_goal1.size[0] = 2.0;
    region_goal1.size[1] = 2.0;
    region<2> region_goal2;
    region_goal2.center[0] = -5.0;
    region_goal2.center[1] = 5.0;
    region_goal2.size[0] = 2.0;
    region_goal2.size[1] = 2.0;
    min_time_reachability1.set_goal_region(region_goal1);
    min_time_reachability2.set_goal_region(region_goal2);


    // 2.f Initialize the planner1
    state_t *state_initial1 = new state_t;
    state_t *state_initial2 = new state_t;

    state_initial1->state_vars[0] = -10.0;
    state_initial1->state_vars[1] = 0.0;
    state_initial1->state_vars[2] = 0.0;
    state_initial2->state_vars[0] = 8.0;
    state_initial2->state_vars[1] = 0.0;
    state_initial2->state_vars[2] = 0.0;
    ////////////////////////////////////////////////////////////////////////////////////////
    //        region<2> landmark_cooperative1,landmark_cooperative2;
    //        landmark_cooperative1.center[0] = state_initial2->operator [](0);landmark_cooperative1.center[1] =state_initial2->operator [](1);
    //    landmark_cooperative1.size[0] = 2.0;landmark_cooperative1.size[1] = 2.0;    
    //    landmark_cooperative2.center[0] = state_initial1->operator [](0);landmark_cooperative2.center[1] = state_initial1->operator [](1);
    //    landmark_cooperative2.size[0] = 2.0;landmark_cooperative2.size[1] = 2.0;
    //    collision_checker1.add_landmark(landmark_cooperative1);
    //    collision_checker2.add_landmark(landmark_cooperative2);
    //    






    planner1.initialize(state_initial1);
    planner2.initialize(state_initial2);






    // 3. RUN THE PLANNER
    vertex_t *z_e_new, *z_p_new;
    for (int i = 0; i < 20; i++) {

        planner1.iteration_multi_cost(&z_e_new);
        //planner1.iteration_additive_cost(&z_e_new);
        //planner1.iteration_evader(&z_e_new);


        planner2.iteration_multi_cost(&z_p_new);
        //planner2.iteration_additive_cost(&z_p_new);
        //planner2.iteration_evader(&z_p_new);

    }





    ofstream tree1, tree2, trajectory1, trajectory2;
    tree1.open("tree1.txt");
    tree2.open("tree2.txt");
    trajectory1.open("trajectory1.txt");
    trajectory2.open("trajectory2.txt");




    for (typename std::list<vertex_t*>::iterator iter = planner1.list_vertices.begin(); iter != planner1.list_vertices.end(); iter++) {//vertex_t *vertex_curr = (vertex_t*)(*iter);
        //SingleIntegrator::State *astate=*((*iter)->getState());
        double x = ((*iter)->state)->operator[](0);
        double y = ((*iter)->state)->operator[](1);
        double z = ((*iter)->state)->operator[](2);


        if (x != 0 && y != 0 && z != 0) {
            for (typename list<edge_t*>::iterator iter_edge = (*iter)->incoming_edges.begin();
                    iter_edge != (*iter)->incoming_edges.end(); iter_edge++) {
                edge_t *edge_curr = *iter_edge;
                vertex_t *vertex_parent = edge_curr->vertex_src;

                double px = (vertex_parent->state)->operator[](0);
                double py = (vertex_parent->state)->operator[](1);
                double pz = (vertex_parent->state)->operator[](2);
                //cout << x << "," << y << "," << z << "," << px << "," << py << "," << pz << "\n";




                for (typename std::list<state_t*>::iterator intstates = edge_curr->trajectory_edge->list_states.begin(); intstates != edge_curr->trajectory_edge->list_states.end(); intstates++) {
                    state_t *astate = (*intstates);
                    double x1 = astate->operator[](0);
                    double y1 = astate->operator[](1);
                    double z1 = astate->operator[](2);
                    tree1 << x1 << "," << y1 << "," << z1 << "," << px << "," << py << "," << pz << "\n";

                    px = x1;
                    py = y1;
                    pz = z1;


                }
                tree1 << x << "," << y << "," << z << "," << px << "," << py << "," << pz << "\n";



            }

        }


    }



    for (typename std::list<vertex_t*>::iterator iter = planner2.list_vertices.begin(); iter != planner2.list_vertices.end(); iter++) {//vertex_t *vertex_curr = (vertex_t*)(*iter);
        //SingleIntegrator::State *astate=*((*iter)->getState());
        double x = ((*iter)->state)->operator[](0);
        double y = ((*iter)->state)->operator[](1);
        double z = ((*iter)->state)->operator[](2);

        // cout<<x<<","<<y;

        if (x != 0 && y != 0 && z != 0) {
            for (typename list<edge_t*>::iterator iter_edge = (*iter)->incoming_edges.begin();
                    iter_edge != (*iter)->incoming_edges.end(); iter_edge++) {
                edge_t *edge_curr = *iter_edge;
                vertex_t *vertex_parent = edge_curr->vertex_src;
                double px = (vertex_parent->state)->operator[](0);
                double py = (vertex_parent->state)->operator[](1);
                double pz = (vertex_parent->state)->operator[](2);
                //cout << x << "," << y << "," << z << "," << px << "," << py << "," << pz << "\n";




                for (typename std::list<state_t*>::iterator intstates = edge_curr->trajectory_edge->list_states.begin(); intstates != edge_curr->trajectory_edge->list_states.end(); intstates++) {
                    state_t *astate = (*intstates);
                    double x1 = astate->operator[](0);
                    double y1 = astate->operator[](1);
                    double z1 = astate->operator[](2);
                    tree2 << x1 << "," << y1 << "," << z1 << "," << px << "," << py << "," << pz << "\n";

                    px = x1;
                    py = y1;
                    pz = z1;


                }
                tree2 << x << "," << y << "," << z << "," << px << "," << py << "," << pz << "\n";

            }
            //  double px=(((*iter)->getParent()).getState()).coordinateValue (0);
            //double py=(((*iter)->getParent()).getState()).coordinateValue (1);
            //double pz=(((*iter)->getParent()).getState()).coordinateValue (2);

        }


    }




    // 4. GET THE RESULTS
    trajectory_t trajectory_final1, trajectory_final2;
    min_time_reachability1.get_solution(trajectory_final1);
    min_time_reachability2.get_solution(trajectory_final2);

    double px = 0;
    double py = 0;
    double pz = 0;
    int i = 0;


    for (typename std::list<state_t*>::iterator iter = trajectory_final1.list_states.begin(); iter != trajectory_final1.list_states.end(); iter++) {
        state_t *astate = (*iter);
        double x = astate->operator[](0);
        double y = astate->operator[](1);
        double z = astate->operator[](2);
        cout << collision_checker1.calculate_visibility_cost(astate) << "\n";
        if (i > 0) trajectory1 << x << "," << y << "," << z << "," << px << "," << py << "," << pz << "\n";
        px = x;
        py = y;
        pz = z;
        i++;


    }
    i = 0;
    cout << "trajectory2" << "\n";
    for (typename std::list<state_t*>::iterator iter = trajectory_final2.list_states.begin(); iter != trajectory_final2.list_states.end(); iter++) {
        state_t *astate = (*iter);
        double x = astate->operator[](0);
        double y = astate->operator[](1);
        double z = astate->operator[](2);
        cout << collision_checker2.calculate_visibility_cost(astate) << "\n";
        if (i > 0)trajectory2 << x << "," << y << "," << z << "," << px << "," << py << "," << pz << "\n";
        px = x;
        py = y;
        pz = z;
        i++;
    }


    tree1.close();
    tree2.close();
    trajectory1.close();
    trajectory2.close();
    obstacles.close();
    units.close();


    return 1;

}






