#ifndef MYCTRL_H
#define MYCTRL_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Quaternion.h>
#include <array>
#include <random>
#include <string>
#include <vector>


namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class ComboBox;
        class Label;
        class Vector3DSpinBox;
    }
    namespace filter {
        // If you prefer to use a custom controller class, you can define it here.
        // ...
    }
}

namespace flair {
    namespace filter {
        class MyController : public ControlLaw
        {
            public :
                MyController(const flair::gui::LayoutPosition *position, const std::string &name);
                ~MyController();
                void UpdateFrom(const flair::core::io_data *data);
                void Reset(void);
                void SetValues(flair::core::Vector3Df pos_error, flair::core::Vector3Df vel_error, flair::core::Vector3Df current_position, flair::core::Quaternion currentQuaternion, flair::core::Vector3Df omega, float yaw_ref, float xppd, float yppd, float zppd);
                void applyMotorConstant(flair::core::Vector3Df &signal);
                void applyMotorConstant(float &signal);
                void plotEstimatedMass(const flair::gui::LayoutPosition *position);
                void plotXYWeights(const flair::gui::LayoutPosition *position);

            private :
                float delta_t, initial_time;
                float g = 9.81;
                bool first_update;
                bool kf_initialized;

                static constexpr size_t kStateBaseEntries = 20;
                static constexpr size_t kLoggedNeurons = 6;
                static constexpr size_t kWeightsPerLoggedNeuron = 4;
                static constexpr size_t kXYWeightsPerLoggedNeuron = 4;

                flair::core::Matrix *state;
                flair::gui::Vector3DSpinBox *Kp_pos, *Kd_pos, *Kp_att, *Kd_att;
                flair::gui::DoubleSpinBox *deltaT_custom, *k_motor, *sat_pos, *sat_att, *sat_thrust;
                flair::gui::DoubleSpinBox *nn_hidden_neurons, *nn_weight_std, *nn_learning_rate, *nn_regularization;
                flair::gui::DoubleSpinBox *nn_eps0, *nn_u_nom, *nn_nu_nom, *nn_mass_min, *nn_mass_max, *nn_use_nlms;
                flair::gui::ComboBox *mass_source_selection, *orientation_mode_selection;
                flair::gui::DoubleSpinBox *nominal_mass;
                flair::gui::DoubleSpinBox *kf_q, *kf_sigma_e, *kf_sigma_edot, *kf_sigma_eddot;
                flair::gui::DoubleSpinBox *xy_nn_enable, *xy_nn_hidden_neurons, *xy_nn_weight_std;
                flair::gui::DoubleSpinBox *xy_nn_eta_w1_b1, *xy_nn_eta_w2, *xy_nn_eta_b2, *xy_nn_regularization;
                flair::gui::DoubleSpinBox *xy_nn_w1_clip, *xy_nn_w2_clip, *xy_nn_b1_clip, *xy_nn_b2_clip;
                flair::gui::DoubleSpinBox *ext_dist_enable, *ext_dist_amp_x, *ext_dist_amp_y, *ext_dist_wx, *ext_dist_wy;
                flair::gui::DoubleSpinBox *xy_nn_pos_scale, *xy_nn_vel_scale, *xy_nn_nominal_scale;

                std::mt19937 rng;
                bool network_ready;
                size_t hidden_neurons;
                float last_weight_std;
                std::vector<std::array<float, 2>> W1;
                std::vector<float> b1;
                std::vector<float> w2;
                std::vector<float> hidden_layer;
                float b2;
                float theta_hat;
                float mass_hat;
                float u_prev_z;
                float kf_state[3];
                float kf_cov[3][3];

                bool xy_network_ready;
                size_t xy_hidden_neurons;
                float xy_last_weight_std;
                std::vector<std::array<float, 6>> xy_W1;
                std::vector<float> xy_b1;
                std::vector<std::array<float, 2>> xy_W2;
                std::vector<float> xy_hidden_layer;
                std::array<float, 2> xy_b2;
                std::array<float, 2> xy_dhat;
                std::array<float, 2> xy_dhat_applied;
                std::array<float, 2> xy_external_disturbance;
                std::array<float, 2> xy_prev_nominal;
                std::array<float, 2> xy_prev_vel_error;
                float kf_residual_norm;
                float mass_residual;
                bool xy_nn_last_enabled;

                void ensureNetworkSize(void);
                void initializeNetwork(void);
                void ensureXYNetworkSize(void);
                void initializeXYNetwork(void);
                void updateMassEstimator(float dt, float thrust_input, float nuz, float actual_acc_z);
                float clipScalar(float value, float limit) const;
                void updateXYDisturbanceCompensator(float dt,
                                                   const flair::core::Vector3Df &pos_error,
                                                   const flair::core::Vector3Df &vel_error,
                                                   const flair::core::Vector3Df &Kp_pos_val,
                                                   const flair::core::Vector3Df &Kd_pos_val,
                                                   float nominal_x,
                                                   float nominal_y);
                float clampMass(float mass);
                void massBounds(float &min_mass, float &max_mass) const;
                float initialMassGuess(void) const;
                float safeSoftplus(float x) const;
                float sigmoid(float x) const;
        };
    }
}

#endif // MYCTRL_H