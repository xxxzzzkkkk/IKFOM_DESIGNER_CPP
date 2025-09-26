#include "BaseDesigner.hpp"

BaseDesigner::BaseDesigner(const std::string& name, const Dict& opts)
{
    construct(name, opts);
}

void BaseDesigner::build() {
    p   = define_parameters();
    x   = define_states();
    dx  = define_states_perturbation();
    u   = define_inputs();
    w   = define_process_noises();
    v   = define_measurement_noises();

    dx_vec = vertcat(dx);
    w_vec  = vertcat(w);
    v_vec  = vertcat(v);

    expr_dyn  = dyn(x, u, w, p);
    expr_meas = meas(x, v, p);

    init_functions();
}

void BaseDesigner::init_functions() {
    f = Function("f", {vertcat(x), vertcat(u), vertcat(w), vertcat(p)}, {expr_dyn});
    h = Function("h", {vertcat(x), vertcat(v), vertcat(p)}, {expr_meas});

    MX perturbed_x_mx = perturb_states(x, dx);

    std::vector<MX> perturbed_x;
    for (casadi_int i = 0; i < perturbed_x_mx.size1(); ++i) {
        perturbed_x.push_back(perturbed_x_mx(i));
    }

    MX expr_dyn_perturbed = dyn(perturbed_x, u, std::vector<MX>(w.size(), MX::zeros(w[0].sparsity())), p);
    jac_df_dx = MX::jacobian(expr_dyn_perturbed, dx_vec);

    jac_df_dw = MX::jacobian(expr_dyn, w_vec);

    MX meas_perturbed = meas(perturbed_x, std::vector<MX>(v.size(), MX::zeros(v[0].sparsity())), p);
    MX meas_nominal   = meas(x, std::vector<MX>(v.size(), MX::zeros(v[0].sparsity())), p);
    jac_dh_dx = MX::jacobian(meas_perturbed - meas_nominal, dx_vec);

    MX meas_with_noise = meas(x, v, p);
    MX meas_without_noise = meas(x, std::vector<MX>(v.size(), MX::zeros(v[0].sparsity())), p);
    jac_dh_dv = MX::jacobian(meas_with_noise - meas_without_noise, v_vec);

    df_dx = Function("df_dx", {vertcat(x), vertcat(u), vertcat(w), vertcat(p)}, {jac_df_dx});
    df_dw = Function("df_dw", {vertcat(x), vertcat(u), vertcat(w), vertcat(p)}, {jac_df_dw});
    dh_dx = Function("dh_dx", {vertcat(x), vertcat(v), vertcat(p)}, {jac_dh_dx});
    dh_dv = Function("dh_dv", {vertcat(x), vertcat(v), vertcat(p)}, {jac_dh_dv});
}


