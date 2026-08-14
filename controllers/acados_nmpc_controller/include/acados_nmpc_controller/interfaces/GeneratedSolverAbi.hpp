#ifndef GENERATED_SOLVER_ABI_HPP
#define GENERATED_SOLVER_ABI_HPP

#include "acados_c/ocp_nlp_interface.h"

extern "C" {
using generated_solver_create_capsule_t = void *(*)();
using generated_solver_free_capsule_t = int (*)(void *);
using generated_solver_create_with_discretization_t = int (*)(void *, int, double *);
using generated_solver_reset_t = int (*)(void *, int);
using generated_solver_free_t = int (*)(void *);
using generated_solver_solve_t = int (*)(void *);
using generated_solver_update_params_t = int (*)(void *, int, double *, int);
using generated_solver_set_p_global_t = int (*)(void *, double *, int);
using generated_solver_get_nlp_config_t = ocp_nlp_config *(*)(void *);
using generated_solver_get_nlp_dims_t = ocp_nlp_dims *(*)(void *);
using generated_solver_get_nlp_in_t = ocp_nlp_in *(*)(void *);
using generated_solver_get_nlp_out_t = ocp_nlp_out *(*)(void *);
using generated_solver_get_nlp_solver_t = ocp_nlp_solver *(*)(void *);
using generated_solver_get_nlp_opts_t = void *(*)(void *);
using generated_solver_get_control_t = int (*)(void *, double *);
using generated_solver_get_dims_t = void (*)(int *, int *, int *, int *, int *, int *, int *, int *);
using generated_solver_int_t = int (*)();
}

#endif // GENERATED_SOLVER_ABI_HPP
