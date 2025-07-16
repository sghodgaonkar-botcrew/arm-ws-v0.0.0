#include <iostream>
extern "C" {
#include "../../../c_code/ik_solver_acados/acados_solver_ik_model.h"
#include "acados/utils/math.h"
}

// solveIK:
//   target_p  : 7×1 [x,y,z,qx,qy,qz,qw] pose
//   q0        : 6×1 current joint angles, used as x⁰ warm start
//   q_opt_out : 6×1 buffer for the returned solution
// returns acados status
int solveIK(const double target_p[7], double q0[6], double q_opt_out[6]) {
    // 1) create & init solver capsule once (you can also hoist
    //    these two lines into your program init, not every call)
    static ik_model_solver_capsule *capsule = nullptr;
    if (!capsule) {
        capsule = ik_model_acados_create_capsule();
        ik_model_acados_create(capsule);
    }

    // 2) update the target‐pose parameter
    ik_model_acados_update_params(capsule, 0, const_cast<double *>(target_p),
                                  7);

    // 3) warm‐start the NLP with your current joint angles
    //    this seeds the SQP with x⁰ = q0
    ocp_nlp_out_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out,
                    capsule->nlp_in, 0, "x", q0);

    // 4) solve
    int status = ik_model_acados_solve(capsule);
    if (status != 0) {
        std::cerr << "[IK] solver failed with status " << status << "\n";
        return status;
    }

    // 5) fetch the optimized joint angles
    ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0,
                    "x", q_opt_out);

    return 0;
}

int main() {
    // example usage
    double target_p[7] = {0.5,     0.5, 0.5,   /*qx, qy, qz, qw=*/0,
                          -0.7071, 0,   0.7071};
    // feed in your real motor feedback here
    double q0[6] = {0.1, -0.5, 0.2, 0.0, 1.0, -0.3};
    double q_opt[6];

    int stat = solveIK(target_p, q0, q_opt);
    if (stat == 0) {
        std::cout << "IK solution:\n";
        for (int i = 0; i < 6; ++i)
            std::cout << "  q[" << i << "] = " << q_opt[i] << "\n";
    }
    return stat;
}
