#ifndef ACADOS_NMPC_CONTROLLER_INTERFACES_ACADOS_INTERFACE_HPP
#define ACADOS_NMPC_CONTROLLER_INTERFACES_ACADOS_INTERFACE_HPP

#include "acados_nmpc_controller/control/CtrlComponent.h"
#include "acados_nmpc_controller/interfaces/GeneratedSolverAbi.hpp"
#include "acados_nmpc_controller/utils/TargetTrajectories.hpp"
#include "acados_nmpc_controller/utils/Types.hpp"

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rcpputils/shared_library.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class AcadosInterface {
public:
    explicit AcadosInterface(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) :
        node_(node) {
        solver_id_ = declareOrGet<std::string>("solver_id", "bluerov2_heavy");
        const std::string solver_library_override =
            declareOrGet<std::string>("solver_library", "");
        const std::string library_path =
            solver_library_override.empty() ? defaultSolverLibraryPath(solver_id_) : solver_library_override;

        library_ = std::make_unique<rcpputils::SharedLibrary>(library_path);
        loadSymbols();
        readDimensions();
        readSolverParameters();

        capsule_ = create_capsule_();
        if (capsule_ == nullptr) {
            throw std::runtime_error("Failed to create acados solver capsule for solver_id '" + solver_id_ + "'");
        }

        const int create_status = create_with_discretization_(capsule_, n_, nullptr);
        if (create_status != 0) {
            free_capsule_(capsule_);
            capsule_ = nullptr;
            throw std::runtime_error("acados create_with_discretization failed for solver_id '" + solver_id_ + "' with status " + std::to_string(create_status));
        }

        nlp_config_ = get_nlp_config_(capsule_);
        nlp_dims_ = get_nlp_dims_(capsule_);
        nlp_in_ = get_nlp_in_(capsule_);
        nlp_out_ = get_nlp_out_(capsule_);
        nlp_solver_ = get_nlp_solver_(capsule_);
        nlp_opts_ = get_nlp_opts_(capsule_);
        (void)nlp_opts_;

        control_buffer_.assign(static_cast<size_t>(nu_), 0.0);
        u0_ = vector_t::Zero(nu_);

        RCLCPP_INFO(node_->get_logger(),
                    "Loaded acados solver '%s' from %s (N=%d, NX=%d, NU=%d, NP=%d, NP_GLOBAL=%d)",
                    solver_id_.c_str(),
                    library_path.c_str(),
                    n_,
                    nx_,
                    nu_,
                    np_,
                    np_global_);
    }

    ~AcadosInterface() {
        if (capsule_ != nullptr) {
            free_(capsule_);
            free_capsule_(capsule_);
            capsule_ = nullptr;
        }
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "Acados solver '%s' resources freed successfully", solver_id_.c_str());
        }
    }

    int horizon() const {
        return n_;
    }

    int stateDim() const {
        return nx_;
    }

    int inputDim() const {
        return nu_;
    }

    void setPathConstraintsEnabled(bool enabled) {
        path_constraints_enabled_ = enabled;
    }

    void create(SystemObservation &initPos, TargetTrajectories &targetTraj) {
        copyToBuffer(initPos.state, state_buffer_, nx_);
        copyToBuffer(initPos.state, x0_buffer_, nbx0_);

        if (static_cast<int>(initPos.input.size()) != nu_) {
            initPos.input = vector_t::Zero(nu_);
        }

        if (nbx0_ > 0) {
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", x0_buffer_.data());
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", x0_buffer_.data());
        }

        const auto &state_traj = targetTraj.state();
        initializeStateGuess();

        for (int stage = 0; stage < n_; ++stage) {
            setStageInputBounds(stage);
            setStagePathBounds(stage);
            updateParameters(stage, state_traj, initPos.state);
        }

        updateGlobalParameters();
        updateParameters(n_, state_traj, initPos.state);
    }

    bool solve() {
        const int status = solve_(capsule_);
        if (status != 0) {
            RCLCPP_ERROR(node_->get_logger(),
                         "acados solver '%s' failed with status %d",
                         solver_id_.c_str(),
                         status);
            return false;
        }

        get_control_(capsule_, control_buffer_.data());
        u0_ = clampControl(Eigen::Map<vector_t>(control_buffer_.data(), nu_));
        ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time_);
        elapsed_time_ = std::min(elapsed_time_, 1e12);
        return true;
    }

    vector_t getControl() const {
        return u0_;
    }

    vector_t clampControl(const vector_t &control) const {
        vector_t clamped = vector_t::Zero(nu_);
        const int count = std::min<int>(nu_, static_cast<int>(control.size()));
        for (int i = 0; i < count; ++i) {
            double value = control(i);
            const size_t index = static_cast<size_t>(i);
            if (index < input_lower_.size() && std::isfinite(input_lower_[index])) {
                value = std::max(value, input_lower_[index]);
            }
            if (index < input_upper_.size() && std::isfinite(input_upper_[index])) {
                value = std::min(value, input_upper_[index]);
            }
            clamped(i) = value;
        }
        return clamped;
    }

private:
    template <typename T>
    T declareOrGet(const std::string &name, const T &default_value) const {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<T>(name, default_value);
        }
        T value = default_value;
        node_->get_parameter(name, value);
        return value;
    }

    std::string defaultSolverLibraryPath(const std::string &solver_id) const {
        const std::string package_prefix = ament_index_cpp::get_package_prefix("acados_nmpc_controller");
        const std::string library_name = rcpputils::get_platform_library_name(solver_id + "_solver");
        return package_prefix + "/lib/acados_nmpc_controller/solvers/" + library_name;
    }

    template <typename T>
    T loadSymbol(const std::string &name) {
        return reinterpret_cast<T>(library_->get_symbol(name));
    }

    void loadSymbols() {
        create_capsule_ = loadSymbol<generated_solver_create_capsule_t>("generated_solver_create_capsule");
        free_capsule_ = loadSymbol<generated_solver_free_capsule_t>("generated_solver_free_capsule");
        create_with_discretization_ =
            loadSymbol<generated_solver_create_with_discretization_t>("generated_solver_create_with_discretization");
        reset_ = loadSymbol<generated_solver_reset_t>("generated_solver_reset");
        free_ = loadSymbol<generated_solver_free_t>("generated_solver_free");
        solve_ = loadSymbol<generated_solver_solve_t>("generated_solver_solve");
        update_params_ = loadSymbol<generated_solver_update_params_t>("generated_solver_update_params");
        set_p_global_ =
            loadSymbol<generated_solver_set_p_global_t>("generated_solver_set_p_global_and_precompute_dependencies");
        get_nlp_config_ = loadSymbol<generated_solver_get_nlp_config_t>("generated_solver_get_nlp_config");
        get_nlp_dims_ = loadSymbol<generated_solver_get_nlp_dims_t>("generated_solver_get_nlp_dims");
        get_nlp_in_ = loadSymbol<generated_solver_get_nlp_in_t>("generated_solver_get_nlp_in");
        get_nlp_out_ = loadSymbol<generated_solver_get_nlp_out_t>("generated_solver_get_nlp_out");
        get_nlp_solver_ = loadSymbol<generated_solver_get_nlp_solver_t>("generated_solver_get_nlp_solver");
        get_nlp_opts_ = loadSymbol<generated_solver_get_nlp_opts_t>("generated_solver_get_nlp_opts");
        get_control_ = loadSymbol<generated_solver_get_control_t>("generated_solver_get_control");
        get_dims_ = loadSymbol<generated_solver_get_dims_t>("generated_solver_get_dims");
        n_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_n");
        nx_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_nx");
        nu_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_nu");
        np_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_np");
        np_global_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_np_global");
        nbx0_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_nbx0");
        nbu_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_nbu");
        nh_fn_ = loadSymbol<generated_solver_int_t>("generated_solver_nh");
        (void)reset_;
    }

    void readDimensions() {
        get_dims_(&n_, &nx_, &nu_, &np_, &np_global_, &nbx0_, &nbu_, &nh_);
        if (n_ <= 0) {
            n_ = n_fn_();
        }
        if (nx_ <= 0) {
            nx_ = nx_fn_();
        }
        if (nu_ <= 0) {
            nu_ = nu_fn_();
        }
        if (np_ < 0) {
            np_ = np_fn_();
        }
        if (np_global_ < 0) {
            np_global_ = np_global_fn_();
        }
        if (nbx0_ < 0) {
            nbx0_ = nbx0_fn_();
        }
        if (nbu_ < 0) {
            nbu_ = nbu_fn_();
        }
        if (nh_ < 0) {
            nh_ = nh_fn_();
        }
        if (nx_ <= 0 || nu_ <= 0) {
            throw std::runtime_error("Invalid acados dimensions for solver_id '" + solver_id_ + "'");
        }
    }

    std::vector<double> readSizedVector(const std::string &name, int expected_size, double fallback) const {
        if (expected_size <= 0) {
            return {};
        }
        const std::vector<double> defaults(static_cast<size_t>(expected_size), fallback);
        auto values = declareOrGet<std::vector<double>>(name, defaults);
        if (static_cast<int>(values.size()) != expected_size) {
            RCLCPP_WARN(node_->get_logger(),
                        "Parameter '%s' has %zu values, expected %d. Using default %.3f.",
                        name.c_str(),
                        values.size(),
                        expected_size,
                        fallback);
            return defaults;
        }
        return values;
    }

    std::vector<double> readOptionalSizedVector(const std::string &name, int expected_size) const {
        if (expected_size <= 0) {
            return {};
        }
        auto values = declareOrGet<std::vector<double>>(name, std::vector<double>{});
        if (values.empty()) {
            return {};
        }
        if (static_cast<int>(values.size()) != expected_size) {
            RCLCPP_WARN(node_->get_logger(),
                        "Parameter '%s' has %zu values, expected %d. Ignoring this bound vector.",
                        name.c_str(),
                        values.size(),
                        expected_size);
            return {};
        }
        return values;
    }

    void readSolverParameters() {
        input_lower_ = readSizedVector("solver.input_lower", nbu_, -40.0);
        input_upper_ = readSizedVector("solver.input_upper", nbu_, 40.0);
        path_lower_ = readOptionalSizedVector("solver.path_lower", nh_);
        path_upper_ = readOptionalSizedVector("solver.path_upper", nh_);
        path_disabled_lower_.assign(static_cast<size_t>(std::max(0, nh_)), -1e12);
        path_disabled_upper_.assign(static_cast<size_t>(std::max(0, nh_)), 1e12);
        global_parameters_ = readSizedVector("solver.global_parameters", np_global_, 1.0);

        state_buffer_.assign(static_cast<size_t>(nx_), 0.0);
        x0_buffer_.assign(static_cast<size_t>(std::max(0, nbx0_)), 0.0);
        param_buffer_.assign(static_cast<size_t>(std::max(0, np_)), 0.0);
        stage_input_buffer_.assign(static_cast<size_t>(nu_), 0.0);
    }

    static void copyToBuffer(const vector_t &source, std::vector<double> &target, int expected_size) {
        target.assign(static_cast<size_t>(std::max(0, expected_size)), 0.0);
        const int count = std::min<int>(expected_size, static_cast<int>(source.size()));
        for (int i = 0; i < count; ++i) {
            target[static_cast<size_t>(i)] = source(i);
        }
        if (expected_size >= 7 && std::abs(target[3]) + std::abs(target[4]) + std::abs(target[5]) + std::abs(target[6]) < std::numeric_limits<double>::epsilon()) {
            target[3] = 1.0;
        }
    }

    void fillStageInput(const vector_t *source) {
        std::fill(stage_input_buffer_.begin(), stage_input_buffer_.end(), 0.0);
        if (source == nullptr) {
            return;
        }
        const int count = std::min<int>(nu_, static_cast<int>(source->size()));
        for (int i = 0; i < count; ++i) {
            stage_input_buffer_[static_cast<size_t>(i)] = (*source)(i);
        }
    }

    void initializeStateGuess() {
        for (int stage = 0; stage <= n_; ++stage) {
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "x", state_buffer_.data());
        }
    }

    void clearInteriorInputGuess() {
        for (int stage = 1; stage < n_; ++stage) {
            fillStageInput(nullptr);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "u", stage_input_buffer_.data());
        }
    }

    void setStageInputBounds(int stage) {
        if (nbu_ <= 0 || input_lower_.empty() || input_upper_.empty()) {
            return;
        }
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "lbu", input_lower_.data());
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "ubu", input_upper_.data());
    }

    void setStagePathBounds(int stage) {
        if (nh_ <= 0) {
            return;
        }
        if (!path_constraints_enabled_) {
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "lh", path_disabled_lower_.data());
            ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "uh", path_disabled_upper_.data());
            return;
        }
        if (path_lower_.empty() || path_upper_.empty()) {
            return;
        }
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "lh", path_lower_.data());
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "uh", path_upper_.data());
    }

    void updateParameters(int stage, const vector_array_t &state_traj, const vector_t &fallback_state) {
        if (np_ <= 0) {
            return;
        }

        const vector_t *source = &fallback_state;
        if (!state_traj.empty()) {
            source = &state_traj[std::min<size_t>(static_cast<size_t>(stage), state_traj.size() - 1)];
        }

        std::fill(param_buffer_.begin(), param_buffer_.end(), 0.0);
        const int count = std::min<int>(np_, static_cast<int>(source->size()));
        for (int i = 0; i < count; ++i) {
            param_buffer_[static_cast<size_t>(i)] = (*source)(i);
        }

        const int status = update_params_(capsule_, stage, param_buffer_.data(), np_);
        if (status != 0) {
            RCLCPP_WARN(node_->get_logger(),
                        "acados_update_params failed for solver '%s' at stage %d with status %d",
                        solver_id_.c_str(),
                        stage,
                        status);
        }
    }

    void updateGlobalParameters() {
        if (np_global_ <= 0) {
            return;
        }
        const int status = set_p_global_(capsule_, global_parameters_.data(), np_global_);
        if (status != 0) {
            RCLCPP_WARN(node_->get_logger(),
                        "acados_set_p_global failed for solver '%s' with status %d",
                        solver_id_.c_str(),
                        status);
        }
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::string solver_id_;
    std::unique_ptr<rcpputils::SharedLibrary> library_;
    void *capsule_{nullptr};

    generated_solver_create_capsule_t create_capsule_{nullptr};
    generated_solver_free_capsule_t free_capsule_{nullptr};
    generated_solver_create_with_discretization_t create_with_discretization_{nullptr};
    generated_solver_reset_t reset_{nullptr};
    generated_solver_free_t free_{nullptr};
    generated_solver_solve_t solve_{nullptr};
    generated_solver_update_params_t update_params_{nullptr};
    generated_solver_set_p_global_t set_p_global_{nullptr};
    generated_solver_get_nlp_config_t get_nlp_config_{nullptr};
    generated_solver_get_nlp_dims_t get_nlp_dims_{nullptr};
    generated_solver_get_nlp_in_t get_nlp_in_{nullptr};
    generated_solver_get_nlp_out_t get_nlp_out_{nullptr};
    generated_solver_get_nlp_solver_t get_nlp_solver_{nullptr};
    generated_solver_get_nlp_opts_t get_nlp_opts_{nullptr};
    generated_solver_get_control_t get_control_{nullptr};
    generated_solver_get_dims_t get_dims_{nullptr};
    generated_solver_int_t n_fn_{nullptr};
    generated_solver_int_t nx_fn_{nullptr};
    generated_solver_int_t nu_fn_{nullptr};
    generated_solver_int_t np_fn_{nullptr};
    generated_solver_int_t np_global_fn_{nullptr};
    generated_solver_int_t nbx0_fn_{nullptr};
    generated_solver_int_t nbu_fn_{nullptr};
    generated_solver_int_t nh_fn_{nullptr};

    ocp_nlp_config *nlp_config_{nullptr};
    ocp_nlp_dims *nlp_dims_{nullptr};
    ocp_nlp_in *nlp_in_{nullptr};
    ocp_nlp_out *nlp_out_{nullptr};
    ocp_nlp_solver *nlp_solver_{nullptr};
    void *nlp_opts_{nullptr};

    int n_{0};
    int nx_{0};
    int nu_{0};
    int np_{0};
    int np_global_{0};
    int nbx0_{0};
    int nbu_{0};
    int nh_{0};

    std::vector<double> input_lower_;
    std::vector<double> input_upper_;
    std::vector<double> path_lower_;
    std::vector<double> path_upper_;
    std::vector<double> path_disabled_lower_;
    std::vector<double> path_disabled_upper_;
    std::vector<double> global_parameters_;
    std::vector<double> state_buffer_;
    std::vector<double> x0_buffer_;
    std::vector<double> param_buffer_;
    std::vector<double> stage_input_buffer_;
    std::vector<double> control_buffer_;
    vector_t u0_;
    double elapsed_time_{0.0};
    bool path_constraints_enabled_{true};
};

#endif // ACADOS_NMPC_CONTROLLER_INTERFACES_ACADOS_INTERFACE_HPP
