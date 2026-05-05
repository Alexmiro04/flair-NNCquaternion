#include "myCtrl.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <TabWidget.h>
#include <Tab.h>
#include <ComboBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <algorithm>
#include <Euler.h>
#include <iostream>
#include <Label.h>
#include <Vector3DSpinBox.h>
#include <Pid.h>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

MyController::MyController(const LayoutPosition *position, const string &name) : ControlLaw(position->getLayout(),name,4)
{
    first_update = true;
    kf_initialized = false;
    rng.seed(0);
    network_ready = false;
    hidden_neurons = 0;
    last_weight_std = 0.0f;
    b2 = 0.0f;
    theta_hat = 0.0f;
    mass_hat = 0.0f;
    u_prev_z = 0.0f;
    xy_network_ready = false;
    xy_hidden_neurons = 0;
    xy_last_weight_std = 0.0f;
    xy_b2 = {0.0f, 0.0f};
    xy_dhat = {0.0f, 0.0f};
    xy_dhat_applied = {0.0f, 0.0f};
    xy_external_disturbance = {0.0f, 0.0f};
    xy_prev_nominal = {0.0f, 0.0f};
    xy_prev_vel_error = {0.0f, 0.0f};
    kf_residual_norm = 0.0f;
    mass_residual = 0.0f;
    xy_nn_last_enabled = true;
    for(int i = 0; i < 3; ++i)
    {
        kf_state[i] = 0.0f;
        for(int j = 0; j < 3; ++j)
        {
            kf_cov[i][j] = 0.0f;
        }
    }

    // Input matrix
    input = new Matrix(this, 4, 7, floatType, name);

    // Matrix descriptor for logging. It should be always a nx1 matrix.
    const size_t xy_weights_offset = kStateBaseEntries + kLoggedNeurons * kWeightsPerLoggedNeuron;
    const size_t state_rows = xy_weights_offset + kLoggedNeurons * kXYWeightsPerLoggedNeuron;
    MatrixDescriptor *log_labels = new MatrixDescriptor(state_rows, 1);
    log_labels->SetElementName(0, 0, "x_pos");
    log_labels->SetElementName(1, 0, "y_pos");
    log_labels->SetElementName(2, 0, "z_pos");
    log_labels->SetElementName(3, 0, "x_error");
    log_labels->SetElementName(4, 0, "y_error");
    log_labels->SetElementName(5, 0, "z_error");
    log_labels->SetElementName(6, 0, "x_vel_error");
    log_labels->SetElementName(7, 0, "y_vel_error");
    log_labels->SetElementName(8, 0, "z_vel_error");
    log_labels->SetElementName(9, 0, "mass_hat");
    log_labels->SetElementName(10, 0, "thrust");
    log_labels->SetElementName(11, 0, "acc_z_kf");
    log_labels->SetElementName(12, 0, "xy_dhat_x");
    log_labels->SetElementName(13, 0, "xy_dhat_y");
    log_labels->SetElementName(14, 0, "xy_s_x");
    log_labels->SetElementName(15, 0, "xy_s_y");
    log_labels->SetElementName(16, 0, "dist_x");
    log_labels->SetElementName(17, 0, "dist_y");
    log_labels->SetElementName(18, 0, "kf_residual_norm");
    log_labels->SetElementName(19, 0, "mass_residual");
    for(size_t i = 0; i < kLoggedNeurons; ++i)
    {
        size_t base = kStateBaseEntries + i * kWeightsPerLoggedNeuron;
        std::string prefix = "nn_h" + std::to_string(i + 1) + "_";
        std::string label_mu0 = prefix + "w1_mu0";
        std::string label_mu1 = prefix + "w1_mu1";
        std::string label_b1 = prefix + "b1";
        std::string label_w2 = prefix + "w2";
        log_labels->SetElementName(base + 0, 0, label_mu0.c_str());
        log_labels->SetElementName(base + 1, 0, label_mu1.c_str());
        log_labels->SetElementName(base + 2, 0, label_b1.c_str());
        log_labels->SetElementName(base + 3, 0, label_w2.c_str());
    }
    for(size_t i = 0; i < kLoggedNeurons; ++i)
    {
        size_t base = xy_weights_offset + i * kXYWeightsPerLoggedNeuron;
        std::string prefix = "xy_nn_h" + std::to_string(i + 1) + "_";
        std::string label_w1x = prefix + "w1_x";
        std::string label_w1y = prefix + "w1_y";
        std::string label_w2x = prefix + "w2_x";
        std::string label_w2y = prefix + "w2_y";
        log_labels->SetElementName(base + 0, 0, label_w1x.c_str());
        log_labels->SetElementName(base + 1, 0, label_w1y.c_str());
        log_labels->SetElementName(base + 2, 0, label_w2x.c_str());
        log_labels->SetElementName(base + 3, 0, label_w2y.c_str());
    }
    state = new Matrix(this, log_labels, floatType, name);
    delete log_labels;

    // GUI for custom PID
    auto *controller_tabs = new TabWidget(position, "Controller setup");
    Tab *setup_pos_tab = new Tab(controller_tabs, "Setup Pos");
    Tab *setup_orientation_tab = new Tab(controller_tabs, "Setup Orientation");

    GroupBox *gui_customPID = new GroupBox(setup_pos_tab->At(0,0), name);
    GroupBox *general_parameters = new GroupBox(gui_customPID->NewRow(), "General parameters");
    deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(), "Custom dt [s]", 0, 1, 0.001, 4);
    k_motor = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Motor constant", 0, 50, 0.01, 4, 29.5870);
    sat_pos = new DoubleSpinBox(general_parameters->NewRow(), "Saturation pos", 0, 10, 0.01, 3);
    sat_att = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation att", 0, 10, 0.01, 3);
    sat_thrust = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation thrust", 0, 10, 0.01, 3);

    // Neural network mass estimator parameters
    GroupBox *nn_group = new GroupBox(gui_customPID->NewRow(), "NN mass estimator");
    nn_hidden_neurons = new DoubleSpinBox(nn_group->NewRow(), "Hidden neurons", 1, 64, 1, 0, 6);
    nn_weight_std = new DoubleSpinBox(nn_group->LastRowLastCol(), "Weight std", 0, 1, 0.001, 4, 0.1);
    nn_learning_rate = new DoubleSpinBox(nn_group->NewRow(), "Learning rate", 0, 10, 0.0001, 4, 2.0);
    nn_regularization = new DoubleSpinBox(nn_group->LastRowLastCol(), "L2 regularization", 0, 0.1, 0.0001, 6, 0.0004);
    nn_eps0 = new DoubleSpinBox(nn_group->NewRow(), "Softplus eps0", 0, 1, 0.000001, 6, 0.001);
    nn_use_nlms = new DoubleSpinBox(nn_group->LastRowLastCol(), "Use NLMS (0/1)", 0, 1, 1, 0, 1);
    nn_u_nom = new DoubleSpinBox(nn_group->NewRow(), "u_{nom}", 0.1, 100, 0.1, 2, 10.0);
    nn_nu_nom = new DoubleSpinBox(nn_group->LastRowLastCol(), "nu_{nom}", 0.1, 100, 0.1, 2, 15.0);
    nn_mass_min = new DoubleSpinBox(nn_group->NewRow(), "Min mass [kg]", 0.1, 20, 0.01, 2, 0.1);
    nn_mass_max = new DoubleSpinBox(nn_group->LastRowLastCol(), "Max mass [kg]", 0.1, 20, 0.01, 2, 20.0);
    mass_source_selection = new ComboBox(nn_group->NewRow(), "Mass for control");
    mass_source_selection->AddItem("Enable NN");
    mass_source_selection->AddItem("Disable NN");
    nominal_mass = new DoubleSpinBox(nn_group->LastRowLastCol(), "Nominal mass [kg]", 0.1, 20, 0.01, 2, 1.2);

    GroupBox *kf_group = new GroupBox(gui_customPID->NewRow(), "Kalman filter (z acc reconstruction)");
    kf_q = new DoubleSpinBox(kf_group->NewRow(), "Jerk q", 0.0, 100.0, 0.001, 6, 1.0);
    kf_sigma_e = new DoubleSpinBox(kf_group->LastRowLastCol(), "Sigma e_z", 0.0, 10.0, 0.0001, 6, 0.01);
    kf_sigma_edot = new DoubleSpinBox(kf_group->NewRow(), "Sigma e_dot_z", 0.0, 10.0, 0.0001, 6, 0.05);
    kf_sigma_eddot = new DoubleSpinBox(kf_group->LastRowLastCol(), "Sigma e_ddot_z (init)", 0.0, 10.0, 0.0001, 6, 0.1);
    

    GroupBox *xy_nn_group = new GroupBox(gui_customPID->NewRow(), "NN xy disturbance compensator");
    xy_nn_enable = new DoubleSpinBox(xy_nn_group->NewRow(), "Enable xy NN (0/1)", 0, 1, 1, 0, 1);
    xy_nn_hidden_neurons = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "Hidden neurons", 1, 64, 1, 0, 12);
    xy_nn_weight_std = new DoubleSpinBox(xy_nn_group->NewRow(), "Weight std", 0, 1, 0.0001, 4, 0.2);
    xy_nn_eta_w1_b1 = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "Eta W1+b1", 0, 20, 0.0001, 4, 0.005);
    xy_nn_eta_w2 = new DoubleSpinBox(xy_nn_group->NewRow(), "Eta W2", 0, 20, 0.0001, 4, 0.005);
    xy_nn_eta_b2 = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "Eta b2", 0, 20, 0.0001, 4, 0.005);
    xy_nn_regularization = new DoubleSpinBox(xy_nn_group->NewRow(), "L2 regularization", 0, 1, 0.0001, 6, 0.0005);
    xy_nn_w1_clip = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "W1 clip (fro)", 0.1, 1000, 0.1, 2, 20.0);
    xy_nn_w2_clip = new DoubleSpinBox(xy_nn_group->NewRow(), "W2 clip (fro)", 0.1, 1000, 0.1, 2, 20.0);
    xy_nn_b1_clip = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "b1 clip", 0.1, 1000, 0.1, 2, 10.0);
    xy_nn_b2_clip = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "b2 clip", 0.1, 1000, 0.1, 2, 10.0);
    xy_nn_pos_scale = new DoubleSpinBox(xy_nn_group->NewRow(), "Input pos scale", 0.01, 1000, 0.01, 2, 1.0);
    xy_nn_vel_scale = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "Input vel scale", 0.01, 1000, 0.01, 2, 1.0);
    xy_nn_nominal_scale = new DoubleSpinBox(xy_nn_group->LastRowLastCol(), "Input nominal scale", 0.01, 1000, 0.01, 2, 1.0);

    GroupBox *dist_group = new GroupBox(gui_customPID->NewRow(), "External disturbance");
    ext_dist_enable = new DoubleSpinBox(dist_group->NewRow(), "Enable disturbance (0/1)", 0, 1, 1, 0, 0);
    ext_dist_amp_x = new DoubleSpinBox(dist_group->LastRowLastCol(), "Amp x", -10, 10, 0.001, 3, 0.15);
    ext_dist_amp_y = new DoubleSpinBox(dist_group->LastRowLastCol(), "Amp y", -10, 10, 0.001, 3, 0.12);
    ext_dist_wx = new DoubleSpinBox(dist_group->NewRow(), "Freq x [rad/s]", 0, 10, 0.001, 3, 0.22);
    ext_dist_wy = new DoubleSpinBox(dist_group->LastRowLastCol(), "Freq y [rad/s]", 0, 10, 0.001, 3, 0.20);

    // Custom cartesian position controller
    GroupBox *custom_position = new GroupBox(gui_customPID->NewRow(), "Custom position controller");
    Kp_pos = new Vector3DSpinBox(custom_position->NewRow(), "Kp_pos", 0, 100, 0.1, 3);
    Kd_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Kd_pos", 0, 100, 0.1, 3);
    
    GroupBox *orientation_selector_group = new GroupBox(setup_orientation_tab->At(0,0), "Orientation mode");
    orientation_mode_selection = new ComboBox(orientation_selector_group->NewRow(), "Orientation controller");
    orientation_mode_selection->AddItem("PD Euler");
    orientation_mode_selection->AddItem("PD Quaternion");

    // Custom attitude controller (PD Euler)
    GroupBox *custom_attitude = new GroupBox(orientation_selector_group->NewRow(), "PD Euler attitude");
    Kp_att = new Vector3DSpinBox(custom_attitude->NewRow(), "Kp_att", 0, 100, 0.1, 3);
    Kd_att = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "Kd_att", 0, 100, 0.1, 3);

    AddDataToLog(state);
    initializeNetwork();
    initializeXYNetwork();
    xy_nn_last_enabled = (xy_nn_enable->Value() >= 0.5f);
}

MyController::~MyController()
{
    delete state;
}

void MyController::UpdateFrom(const io_data *data)
{
    ensureNetworkSize();
    ensureXYNetworkSize();

    float thrust = 0.0f;
    Vector3Df u, tau;

    if(deltaT_custom->Value() == 0)
    {
        delta_t = (float)(data->DataDeltaTime())/1000000000;
    }
    else
    {
        delta_t = deltaT_custom->Value();
    }

    if(first_update)
    {
        initial_time = double(GetTime())/1000000000;
        first_update = false;
    }

    // Obtain state
    input->GetMutex();
    Vector3Df pos_error(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    Vector3Df vel_error(input->Value(0, 1), input->Value(1, 1), input->Value(2, 1));
    float xppd = input->Value(0, 5);
    float yppd = input->Value(1, 5);
    float zppd = input->Value(2, 5);
    Quaternion q(input->Value(0, 2), input->Value(1, 2), input->Value(2, 2), input->Value(3, 2));
    Vector3Df omega(input->Value(0, 3), input->Value(1, 3), input->Value(2, 3));
    float yaw_ref = input->Value(0, 4);
    Vector3Df current_pos(input->Value(0, 6), input->Value(1, 6), input->Value(2, 6));
    input->ReleaseMutex();

    // Get tunning parameters from GUI
    Vector3Df Kp_pos_val(Kp_pos->Value().x, Kp_pos->Value().y, Kp_pos->Value().z);
    Vector3Df Kd_pos_val(Kd_pos->Value().x, Kd_pos->Value().y, Kd_pos->Value().z);
    Vector3Df Kp_att_val(Kp_att->Value().x, Kp_att->Value().y, Kp_att->Value().z);
    Vector3Df Kd_att_val(Kd_att->Value().x, Kd_att->Value().y, Kd_att->Value().z);

    // Mass used for control
    float nominal_mass_value = clampMass(static_cast<float>(nominal_mass->Value()));
    if(nominal_mass_value <= 0.0f)
    {
        nominal_mass_value = initialMassGuess();
    }

    bool use_nn_mass = (mass_source_selection->CurrentIndex() == 0);
    float mass_for_control = use_nn_mass ? clampMass(mass_hat) : nominal_mass_value;

    float nux_nom = xppd+Kp_pos_val.x*pos_error.x + Kd_pos_val.x*vel_error.x;
    float nuy_nom = yppd+Kp_pos_val.y*pos_error.y + Kd_pos_val.y*vel_error.y;
    float nuz = zppd + Kp_pos_val.z*pos_error.z + Kd_pos_val.z*vel_error.z - g;


    updateXYDisturbanceCompensator(delta_t, pos_error, vel_error, Kp_pos_val, Kd_pos_val, nux_nom, nuy_nom);
    float xy_enable = static_cast<float>(xy_nn_enable->Value());
    xy_dhat_applied[0] = (xy_enable >= 0.5f) ? xy_dhat[0] : 0.0f;
    xy_dhat_applied[1] = (xy_enable >= 0.5f) ? xy_dhat[1] : 0.0f;
    float nux = nux_nom - xy_dhat_applied[0] + xy_external_disturbance[0];
    float nuy = nuy_nom - xy_dhat_applied[1] + xy_external_disturbance[1];

    // Cartesian custom controller
    Vector3Df control_effort(mass_for_control * nux,
                             mass_for_control * nuy,
                             mass_for_control * nuz);
    float ctrl_z = control_effort.z; // This is the thrust needed to control the z position before saturation
    control_effort.Saturate(sat_pos->Value());
    u = control_effort;

    Euler rpy = q.ToEuler();

    // Attitude controller selection
    if(orientation_mode_selection->CurrentIndex() == 1)
    {
        /*
        * Quaternion PD compatible with the previous Euler PD convention.
        *
        * Previous Euler controller:
        * tau.x = Kp_roll  * (roll  + u.y) + Kd_roll  * omega.x
        * tau.y = Kp_pitch * (pitch - u.x) + Kd_pitch * omega.y
        * tau.z = Kp_yaw   * yaw_error     + Kd_yaw   * omega.z
        *
        * Therefore:
        * roll_ref  = -u.y
        * pitch_ref =  u.x
        * yaw_ref   = yaw_ref
        */

        float max_tilt = 0.35f; // about 20 degrees. Adjust later.

        float roll_ref  = std::max(-max_tilt, std::min(max_tilt, -u.y));
        float pitch_ref = std::max(-max_tilt, std::min(max_tilt,  u.x));
        float yaw_des   = std::atan2(std::sin(yaw_ref), std::cos(yaw_ref));

        Euler desiredEuler;
        desiredEuler.roll  = roll_ref;
        desiredEuler.pitch = pitch_ref;
        desiredEuler.yaw   = yaw_des;

        Quaternion qd_flair = desiredEuler.ToQuaternion();

        Eigen::Quaternionf qd(qd_flair.q0, qd_flair.q1, qd_flair.q2, qd_flair.q3);
        Eigen::Quaternionf q_curr(q.q0, q.q1, q.q2, q.q3);

        if(qd.norm() > 1e-6f) {
            qd.normalize();
        }

        if(q_curr.norm() > 1e-6f) {
            q_curr.normalize();
        }

        Eigen::Quaternionf qe = qd.conjugate() * q_curr;

        if(qe.w() < 0.0f) {
            qe.coeffs() *= -1.0f;
        }

        Eigen::Vector3f e_q = 2.0f * qe.vec();

        /*
        * Important:
        * Keep the same sign convention as your Euler PD.
        * Do NOT use the standard textbook negative sign unless you also
        * verify the FLAIR torque convention.
        */
        tau.x = Kp_att_val.x * e_q(0) + Kd_att_val.x * omega.x;
        tau.y = Kp_att_val.y * e_q(1) + Kd_att_val.y * omega.y;
        tau.z = Kp_att_val.z * e_q(2) + Kd_att_val.z * omega.z;

        applyMotorConstant(tau);
        tau.Saturate(sat_att->Value());
    }
    else
    {
        tau.x = Kp_att_val.x*(rpy.roll + u.y) + Kd_att_val.x*omega.x;
        tau.y = Kp_att_val.y*(rpy.pitch - u.x) + Kd_att_val.y*omega.y;
        tau.z = Kp_att_val.z*(rpy.YawDistanceFrom(yaw_ref)) + Kd_att_val.z*omega.z;
        applyMotorConstant(tau);
        tau.Saturate(sat_att->Value());
    }

    // Compute custom thrust
    thrust = ctrl_z; // This is the thrust needed to counteract gravity and control the z position
    applyMotorConstant(thrust);
    float thr_sat = sat_thrust->Value();
    if(thrust < -thr_sat) {
      thrust = -thr_sat;
    } else if(thrust >= 0) {
        thrust = 0;
    }

    // Mass estimator update
    
    float actual_acc_z = zppd;
    kf_residual_norm = 0.0f;
    if(delta_t > 1e-6f && std::isfinite(delta_t))
    {
        float dt = delta_t;
        float q = std::max(0.0f, static_cast<float>(kf_q->Value()));
        float sigma_e = std::max(0.0f, static_cast<float>(kf_sigma_e->Value()));
        float sigma_edot = std::max(0.0f, static_cast<float>(kf_sigma_edot->Value()));
        float sigma_eddot = std::max(0.0f, static_cast<float>(kf_sigma_eddot->Value()));

        if(!kf_initialized)
        {
            kf_state[0] = pos_error.z;
            kf_state[1] = vel_error.z;
            kf_state[2] = 0.0f;
            kf_cov[0][0] = sigma_e * sigma_e;
            kf_cov[1][1] = sigma_edot * sigma_edot;
            kf_cov[2][2] = sigma_eddot * sigma_eddot;
            kf_cov[0][1] = 0.0f;
            kf_cov[0][2] = 0.0f;
            kf_cov[1][0] = 0.0f;
            kf_cov[1][2] = 0.0f;
            kf_cov[2][0] = 0.0f;
            kf_cov[2][1] = 0.0f;
            kf_initialized = true;
        }
        else
        {
            float F[3][3] = {
                {1.0f, dt, 0.5f * dt * dt},
                {0.0f, 1.0f, dt},
                {0.0f, 0.0f, 1.0f}
            };
            float Q[3][3] = {
                {q * dt * dt * dt * dt * dt / 20.0f, q * dt * dt * dt * dt / 8.0f, q * dt * dt * dt / 6.0f},
                {q * dt * dt * dt * dt / 8.0f, q * dt * dt * dt / 3.0f, q * dt * dt / 2.0f},
                {q * dt * dt * dt / 6.0f, q * dt * dt / 2.0f, q * dt}
            };

            float x_pred[3] = {
                F[0][0] * kf_state[0] + F[0][1] * kf_state[1] + F[0][2] * kf_state[2],
                F[1][0] * kf_state[0] + F[1][1] * kf_state[1] + F[1][2] * kf_state[2],
                F[2][0] * kf_state[0] + F[2][1] * kf_state[1] + F[2][2] * kf_state[2]
            };

            float FP[3][3] = {};
            for(int i = 0; i < 3; ++i)
            {
                for(int j = 0; j < 3; ++j)
                {
                    FP[i][j] = 0.0f;
                    for(int k = 0; k < 3; ++k)
                    {
                        FP[i][j] += F[i][k] * kf_cov[k][j];
                    }
                }
            }
            float P_pred[3][3] = {};
            for(int i = 0; i < 3; ++i)
            {
                for(int j = 0; j < 3; ++j)
                {
                    P_pred[i][j] = 0.0f;
                    for(int k = 0; k < 3; ++k)
                    {
                        P_pred[i][j] += FP[i][k] * F[j][k];
                    }
                    P_pred[i][j] += Q[i][j];
                }
            }

            float S00 = P_pred[0][0] + sigma_e * sigma_e;
            float S01 = P_pred[0][1];
            float S10 = P_pred[1][0];
            float S11 = P_pred[1][1] + sigma_edot * sigma_edot;
            float detS = S00 * S11 - S01 * S10;

            float K[3][2] = {};
            if(std::fabs(detS) > 1e-9f)
            {
                float invS00 = S11 / detS;
                float invS01 = -S01 / detS;
                float invS10 = -S10 / detS;
                float invS11 = S00 / detS;
                for(int i = 0; i < 3; ++i)
                {
                    K[i][0] = P_pred[i][0] * invS00 + P_pred[i][1] * invS10;
                    K[i][1] = P_pred[i][0] * invS01 + P_pred[i][1] * invS11;
                }

                float y0 = pos_error.z - x_pred[0];
                float y1 = vel_error.z - x_pred[1];
                kf_residual_norm = std::sqrt(y0 * y0 + y1 * y1);
                for(int i = 0; i < 3; ++i)
                {
                    kf_state[i] = x_pred[i] + K[i][0] * y0 + K[i][1] * y1;
                }

                for(int i = 0; i < 3; ++i)
                {
                    for(int j = 0; j < 3; ++j)
                    {
                        kf_cov[i][j] = P_pred[i][j] - K[i][0] * P_pred[0][j] - K[i][1] * P_pred[1][j];
                    }
                }
            }
            else
            {
                kf_residual_norm = 0.0f;
                for(int i = 0; i < 3; ++i)
                {
                    kf_state[i] = x_pred[i];
                    for(int j = 0; j < 3; ++j)
                    {
                        kf_cov[i][j] = P_pred[i][j];
                    }
                }
            }
        }
        actual_acc_z = zppd + kf_state[2];
    }
    if(!std::isfinite(actual_acc_z))
    {
        actual_acc_z = zppd;
    }
    // Ya tienes: Euler rpy = q.ToEuler();  (lo calculas arriba para actitud)
    float R33 = std::cos(rpy.roll) * std::cos(rpy.pitch);  // proyección vertical

    float motor_const = static_cast<float>(k_motor->Value());
    if(motor_const < 1e-6f) motor_const = 1e-6f;

    float estimator_thrust = -thrust * motor_const; // magnitud (o proporcional)
    float u_eff = estimator_thrust * R33;           // empuje vertical efectivo

    updateMassEstimator(delta_t, u_eff, nuz, actual_acc_z);

    // Debug thrust value
    std::cout << " error_x: " << pos_error.x << " error_y: " << pos_error.y << " error_z: " << pos_error.z << " mass_hat: " << mass_hat <<std::endl;
    
    // Send controller output
    output->SetValue(0, 0, tau.x);
    output->SetValue(1, 0, tau.y);
    output->SetValue(2, 0, tau.z);
    output->SetValue(3, 0, thrust);
    output->SetDataTime(data->DataTime());
    
    // Log state (example).
    // Modify the log_labels matrix in the constructor to add more variables.
    state->GetMutex();
    state->SetValue(0, 0, current_pos.x);
    state->SetValue(1, 0, current_pos.y);
    state->SetValue(2, 0, current_pos.z);
    state->SetValue(3, 0, pos_error.x);
    state->SetValue(4, 0, pos_error.y);
    state->SetValue(5, 0, pos_error.z);
    state->SetValue(6, 0, vel_error.x);
    state->SetValue(7, 0, vel_error.y);
    state->SetValue(8, 0, vel_error.z);
    state->SetValue(9, 0, mass_hat);
    state->SetValue(10, 0, thrust);
    state->SetValue(11, 0, actual_acc_z);
    state->SetValue(12, 0, xy_dhat[0]);
    state->SetValue(13, 0, xy_dhat[1]);
    float sx_log = Kd_pos_val.x * vel_error.x + Kp_pos_val.x * pos_error.x;
    float sy_log = Kd_pos_val.y * vel_error.y + Kp_pos_val.y * pos_error.y;
    state->SetValue(14, 0, sx_log);
    state->SetValue(15, 0, sy_log);
    state->SetValue(16, 0, xy_external_disturbance[0]);
    state->SetValue(17, 0, xy_external_disturbance[1]);
    state->SetValue(18, 0, kf_residual_norm);
    state->SetValue(19, 0, mass_residual);
    size_t row = kStateBaseEntries;
    for(size_t i = 0; i < kLoggedNeurons; ++i)
    {
        float w_mu0 = 0.0f;
        float w_mu1 = 0.0f;
        float b_hidden = 0.0f;
        float w_out = 0.0f;
        if(i < W1.size())
        {
            w_mu0 = W1[i][0];
            w_mu1 = W1[i][1];
        }
        if(i < b1.size())
        {
            b_hidden = b1[i];
        }
        if(i < w2.size())
        {
            w_out = w2[i];
        }
        state->SetValue(row + 0, 0, w_mu0);
        state->SetValue(row + 1, 0, w_mu1);
        state->SetValue(row + 2, 0, b_hidden);
        state->SetValue(row + 3, 0, w_out);
        row += kWeightsPerLoggedNeuron;
    }

    size_t xy_row = kStateBaseEntries + kLoggedNeurons * kWeightsPerLoggedNeuron;
    for(size_t i = 0; i < kLoggedNeurons; ++i)
    {
        float xy_w1_x = 0.0f;
        float xy_w1_y = 0.0f;
        float xy_w2_x = 0.0f;
        float xy_w2_y = 0.0f;
        if(i < xy_W1.size())
        {
            xy_w1_x = xy_W1[i][0];
            xy_w1_y = xy_W1[i][1];
        }
        if(i < xy_W2.size())
        {
            xy_w2_x = xy_W2[i][0];
            xy_w2_y = xy_W2[i][1];
        }
        state->SetValue(xy_row + 0, 0, xy_w1_x);
        state->SetValue(xy_row + 1, 0, xy_w1_y);
        state->SetValue(xy_row + 2, 0, xy_w2_x);
        state->SetValue(xy_row + 3, 0, xy_w2_y);
        xy_row += kXYWeightsPerLoggedNeuron;
    }
    state->ReleaseMutex();

    ProcessUpdate(output);
}

void MyController::Reset(void)
{
    first_update = true;
    u_prev_z = 0.0f;
    if(!std::isfinite(mass_hat) || mass_hat <= 0.0f)
    {
        mass_hat = initialMassGuess();
    }
    if(!std::isfinite(theta_hat) || theta_hat <= 0.0f)
    {
        theta_hat = 1.0f / mass_hat;
    }
    kf_initialized = false;

    xy_prev_nominal = {0.0f, 0.0f};
    xy_prev_vel_error = {0.0f, 0.0f};
    xy_dhat = {0.0f, 0.0f};
    xy_dhat_applied = {0.0f, 0.0f};
    xy_external_disturbance = {0.0f, 0.0f};
    kf_residual_norm = 0.0f;
    mass_residual = 0.0f;
    xy_nn_last_enabled = (xy_nn_enable->Value() >= 0.5f);
    for(int i = 0; i < 3; ++i)
    {
        kf_state[i] = 0.0f;
        for(int j = 0; j < 3; ++j)
        {
            kf_cov[i][j] = 0.0f;
        }
    }
}

void MyController::SetValues(Vector3Df pos_error, Vector3Df vel_error, Vector3Df current_position, Quaternion currentQuaternion, Vector3Df omega, float yaw_ref, float xppd, float yppd, float zppd)
{
    // Set the input values for the controller. 
    // This function is called from the main controller to set the input values.
    input->GetMutex();
    input->SetValue(0, 0, pos_error.x);
    input->SetValue(1, 0, pos_error.y);
    input->SetValue(2, 0, pos_error.z);

    input->SetValue(0, 1, vel_error.x);
    input->SetValue(1, 1, vel_error.y);
    input->SetValue(2, 1, vel_error.z);

    input->SetValue(0, 2, currentQuaternion.q0);
    input->SetValue(1, 2, currentQuaternion.q1);
    input->SetValue(2, 2, currentQuaternion.q2);
    input->SetValue(3, 2, currentQuaternion.q3);

    input->SetValue(0, 3, omega.x);
    input->SetValue(1, 3, omega.y);
    input->SetValue(2, 3, omega.z);

    // Set yaw reference
    input->SetValue(0, 4, yaw_ref);

    input->SetValue(0, 5, xppd);
    input->SetValue(1, 5, yppd);
    input->SetValue(2, 5, zppd);

    input->SetValue(0, 6, current_position.x);
    input->SetValue(1, 6, current_position.y);
    input->SetValue(2, 6, current_position.z);

    input->ReleaseMutex();
}

void MyController::applyMotorConstant(Vector3Df &signal)
{
    float motor_constant = k_motor->Value();
    signal.x = signal.x/motor_constant;
    signal.y = signal.y/motor_constant;
    signal.z = signal.z/motor_constant;
}

void MyController::applyMotorConstant(float &signal)
{
    float motor_constant = k_motor->Value();
    signal = signal/motor_constant;
}

float MyController::clipScalar(float value, float limit) const
{
    float safe_limit = std::max(0.0f, limit);
    if(safe_limit <= 0.0f)
    {
        return value;
    }
    return std::max(-safe_limit, std::min(safe_limit, value));
}

void MyController::ensureXYNetworkSize(void)
{
    double hidden_value = xy_nn_hidden_neurons->Value();
    if(hidden_value < 1.0)
    {
        hidden_value = 1.0;
    }
    size_t desired = static_cast<size_t>(hidden_value + 0.5);
    float current_std = static_cast<float>(xy_nn_weight_std->Value());
    if(!xy_network_ready || desired != xy_hidden_neurons || std::fabs(current_std - xy_last_weight_std) > 1e-6f)
    {
        initializeXYNetwork();
    }
}

void MyController::initializeXYNetwork(void)
{
    double hidden_value = xy_nn_hidden_neurons->Value();
    if(hidden_value < 1.0)
    {
        hidden_value = 1.0;
    }
    xy_hidden_neurons = static_cast<size_t>(hidden_value + 0.5);

    xy_W1.assign(xy_hidden_neurons, std::array<float, 6>{{0,0,0,0,0,0}});
    xy_b1.assign(xy_hidden_neurons, 0.0f);
    xy_W2.assign(xy_hidden_neurons, std::array<float, 2>{{0,0}});
    xy_hidden_layer.assign(xy_hidden_neurons, 0.0f);

    std::normal_distribution<float> dist(0.0f, static_cast<float>(xy_nn_weight_std->Value()));
    for(size_t i = 0; i < xy_hidden_neurons; ++i)
    {
        for(size_t j = 0; j < 6; ++j)
        {
            xy_W1[i][j] = dist(rng);
        }
        xy_W2[i][0] = dist(rng);
        xy_W2[i][1] = dist(rng);
    }

    xy_b2 = {0.0f, 0.0f};
    xy_dhat = {0.0f, 0.0f};
    xy_prev_nominal = {0.0f, 0.0f};
    xy_prev_vel_error = {0.0f, 0.0f};
    xy_last_weight_std = static_cast<float>(xy_nn_weight_std->Value());
    xy_network_ready = true;
}

void MyController::updateXYDisturbanceCompensator(float dt,
                                                  const Vector3Df &pos_error,
                                                  const Vector3Df &vel_error,
                                                  const Vector3Df &Kp_pos_val,
                                                  const Vector3Df &Kd_pos_val,
                                                  float nominal_x,
                                                  float nominal_y)
{
    bool xy_enabled = (xy_nn_enable->Value() >= 0.5f);
    if(xy_enabled != xy_nn_last_enabled)
    {
        initializeXYNetwork();
    }
    xy_nn_last_enabled = xy_enabled;

    float now = static_cast<float>(GetTime()) / 1000000000.0f;
    if(ext_dist_enable->Value() >= 0.5f)
    {
        xy_external_disturbance[0] = static_cast<float>(ext_dist_amp_x->Value()) * std::sin(static_cast<float>(ext_dist_wx->Value()) * now);
        xy_external_disturbance[1] = static_cast<float>(ext_dist_amp_y->Value()) * std::cos(static_cast<float>(ext_dist_wy->Value()) * now);
    }
    else
    {
        xy_external_disturbance[0] = 0.0f;
        xy_external_disturbance[1] = 0.0f;
    }

    if(!xy_network_ready || xy_hidden_neurons == 0 || !xy_enabled)
    {
        xy_dhat = {0.0f, 0.0f};
        xy_prev_nominal = {nominal_x, nominal_y};
        xy_prev_vel_error = {vel_error.x, vel_error.y};
        return;
    }

    const float pos_scale = std::max(1e-6f, static_cast<float>(xy_nn_pos_scale->Value()));
    const float vel_scale = std::max(1e-6f, static_cast<float>(xy_nn_vel_scale->Value()));
    const float nominal_scale = std::max(1e-6f, static_cast<float>(xy_nn_nominal_scale->Value()));
    float z_in[6] = {pos_error.x / pos_scale,
                     pos_error.y / pos_scale,
                     vel_error.x / vel_scale,
                     vel_error.y / vel_scale,
                     xy_prev_nominal[0] / nominal_scale,
                     xy_prev_nominal[1] / nominal_scale};
    for(size_t i = 0; i < xy_hidden_neurons; ++i)
    {
        float sum = xy_b1[i];
        for(size_t j = 0; j < 6; ++j)
        {
            sum += xy_W1[i][j] * z_in[j];
        }
        xy_hidden_layer[i] = std::tanh(sum);
    }

    float dhat_x = xy_b2[0];
    float dhat_y = xy_b2[1];
    for(size_t i = 0; i < xy_hidden_neurons; ++i)
    {
        dhat_x += xy_W2[i][0] * xy_hidden_layer[i];
        dhat_y += xy_W2[i][1] * xy_hidden_layer[i];
    }
    xy_dhat[0] = dhat_x;
    xy_dhat[1] = dhat_y;

    if(dt > 1e-6f && std::isfinite(dt))
    {
        float sx = Kd_pos_val.x * vel_error.x + Kp_pos_val.x * pos_error.x;
        float sy = Kd_pos_val.y * vel_error.y + Kp_pos_val.y * pos_error.y;

        float eta_w1_b1 = static_cast<float>(xy_nn_eta_w1_b1->Value());
        float eta_w2 = static_cast<float>(xy_nn_eta_w2->Value());
        float eta_b2 = static_cast<float>(xy_nn_eta_b2->Value());
        float lambda = static_cast<float>(xy_nn_regularization->Value());

        float delta2_x = sx;
        float delta2_y = sy;

        for(size_t i = 0; i < xy_hidden_neurons; ++i)
        {
            float h = xy_hidden_layer[i];
            float old_w2x = xy_W2[i][0];
            float old_w2y = xy_W2[i][1];
            xy_W2[i][0] -= dt * eta_w2 * (delta2_x * h + lambda * xy_W2[i][0]);
            xy_W2[i][1] -= dt * eta_w2 * (delta2_y * h + lambda * xy_W2[i][1]);

            float delta1 = (old_w2x * delta2_x + old_w2y * delta2_y) * (1.0f - h * h);
            for(size_t j = 0; j < 6; ++j)
            {
                xy_W1[i][j] -= dt * eta_w1_b1 * (delta1 * z_in[j] + lambda * xy_W1[i][j]);
            }
            xy_b1[i] -= dt * eta_w1_b1 * (delta1 + lambda * xy_b1[i]);
        }

        xy_b2[0] -= dt * eta_b2 * (delta2_x + lambda * xy_b2[0]);
        xy_b2[1] -= dt * eta_b2 * (delta2_y + lambda * xy_b2[1]);

        float w1_sq = 0.0f;
        float w2_sq = 0.0f;
        for(size_t i = 0; i < xy_hidden_neurons; ++i)
        {
            for(size_t j = 0; j < 6; ++j)
            {
                w1_sq += xy_W1[i][j] * xy_W1[i][j];
            }
            w2_sq += xy_W2[i][0] * xy_W2[i][0] + xy_W2[i][1] * xy_W2[i][1];
            xy_b1[i] = clipScalar(xy_b1[i], static_cast<float>(xy_nn_b1_clip->Value()));
        }
        float w1_norm = std::sqrt(w1_sq);
        float w2_norm = std::sqrt(w2_sq);
        float w1_max = std::max(1e-6f, static_cast<float>(xy_nn_w1_clip->Value()));
        float w2_max = std::max(1e-6f, static_cast<float>(xy_nn_w2_clip->Value()));
        if(w1_norm > w1_max)
        {
            float scale = w1_max / w1_norm;
            for(size_t i = 0; i < xy_hidden_neurons; ++i)
            {
                for(size_t j = 0; j < 6; ++j)
                {
                    xy_W1[i][j] *= scale;
                }
            }
        }
        if(w2_norm > w2_max)
        {
            float scale = w2_max / w2_norm;
            for(size_t i = 0; i < xy_hidden_neurons; ++i)
            {
                xy_W2[i][0] *= scale;
                xy_W2[i][1] *= scale;
            }
        }
        xy_b2[0] = clipScalar(xy_b2[0], static_cast<float>(xy_nn_b2_clip->Value()));
        xy_b2[1] = clipScalar(xy_b2[1], static_cast<float>(xy_nn_b2_clip->Value()));
    }

    xy_prev_nominal = {nominal_x, nominal_y};
    xy_prev_vel_error = {vel_error.x, vel_error.y};
}


void MyController::ensureNetworkSize(void)
{
    double hidden_value = nn_hidden_neurons->Value();
    if(hidden_value < 1.0)
    {
        hidden_value = 1.0;
    }
    size_t desired = static_cast<size_t>(hidden_value + 0.5);
    float current_std = static_cast<float>(nn_weight_std->Value());
    if(!network_ready || desired != hidden_neurons || std::fabs(current_std - last_weight_std) > 1e-6f)
    {
        initializeNetwork();
    }
}

void MyController::initializeNetwork(void)
{
    double hidden_value = nn_hidden_neurons->Value();
    if(hidden_value < 1.0)
    {
        hidden_value = 1.0;
    }
    hidden_neurons = static_cast<size_t>(hidden_value + 0.5);

    W1.assign(hidden_neurons, std::array<float, 2>{{0.0f, 0.0f}});
    b1.assign(hidden_neurons, 0.0f);
    w2.assign(hidden_neurons, 0.0f);
    hidden_layer.assign(hidden_neurons, 0.0f);

    std::normal_distribution<float> dist(0.0f, static_cast<float>(nn_weight_std->Value()));
    for(size_t i = 0; i < hidden_neurons; ++i)
    {
        for(size_t j = 0; j < 2; ++j)
        {
            W1[i][j] = dist(rng);
        }
        w2[i] = dist(rng);
    }

    float min_mass, max_mass;
    massBounds(min_mass, max_mass);
    float nominal_mass_value = clampMass(static_cast<float>(nominal_mass->Value()));
    float theta_init = 1.0f / nominal_mass_value;
    float theta_min = 1.0f / max_mass;
    float theta_max = 1.0f / min_mass;
    theta_init = std::max(theta_min, std::min(theta_max, theta_init));

    float hidden_sum = 0.0f;
    for(size_t i = 0; i < hidden_neurons; ++i)
    {
        hidden_sum += w2[i] * std::tanh(b1[i]);
    }

    float eps0 = static_cast<float>(nn_eps0->Value());
    float y = std::max(theta_init - eps0, 1e-6f);
    float inv_softplus = (y > 20.0f) ? y : std::log(std::expm1(y));
    b2 = inv_softplus - hidden_sum;
    last_weight_std = static_cast<float>(nn_weight_std->Value());

    mass_hat = nominal_mass_value;
    theta_hat = theta_init;
    u_prev_z = 0.0f;
    kf_initialized = false;
    network_ready = true;
}

void MyController::massBounds(float &min_mass, float &max_mass) const
{
    min_mass = static_cast<float>(nn_mass_min->Value());
    max_mass = static_cast<float>(nn_mass_max->Value());
    if(min_mass < 1e-3f)
    {
        min_mass = 1e-3f;
    }
    if(max_mass < min_mass + 1e-3f)
    {
        max_mass = min_mass + 1e-3f;
    }
}

float MyController::clampMass(float mass)
{
    float min_mass, max_mass;
    massBounds(min_mass, max_mass);
    mass = std::max(min_mass, std::min(max_mass, mass));
    return mass;
}

float MyController::initialMassGuess(void) const
{
    float min_mass, max_mass;
    massBounds(min_mass, max_mass);
    float guess = static_cast<float>(nominal_mass->Value());
    if(!std::isfinite(guess) || guess <= 0.0f)
    {
        guess = 0.5f * (min_mass + max_mass);
    }
    if(guess <= 0.0f)
    {
        guess = std::max(min_mass, 1e-3f);
    }
    return std::max(min_mass, std::min(max_mass, guess));
}

float MyController::safeSoftplus(float x) const
{
    if(x > 20.0f)
    {
        return x;
    }
    if(x < -20.0f)
    {
        return std::exp(x);
    }
    return std::log1p(std::exp(x));
}

float MyController::sigmoid(float x) const
{
    if(x >= 0.0f)
    {
        float exp_neg = std::exp(-x);
        return 1.0f / (1.0f + exp_neg);
    }
    float exp_pos = std::exp(x);
    return exp_pos / (1.0f + exp_pos);
}

void MyController::updateMassEstimator(float dt, float thrust_input, float nuz, float actual_acc_z)
{
    if(!network_ready || hidden_neurons == 0)
    {
        u_prev_z = thrust_input;
        mass_residual = 0.0f;
        return;
    }

    float min_mass, max_mass;
    massBounds(min_mass, max_mass);
    float theta_min = 1.0f / max_mass;
    float theta_max = 1.0f / min_mass;

    float u_nom = std::max(static_cast<float>(nn_u_nom->Value()), 1e-6f);
    float nu_nom = std::max(static_cast<float>(nn_nu_nom->Value()), 1e-6f);
    float mu0 = u_prev_z / u_nom; // Normalized NN input
    float mu1 = nuz / nu_nom; // Normalized NN input 

    // float mu0 = u_prev_z; // Unnormalized NN input
    // float mu1 = nuz; // Unormalized NN input

    for(size_t i = 0; i < hidden_neurons; ++i)
    {
        float sum = W1[i][0] * mu0 + W1[i][1] * mu1 + b1[i];
        hidden_layer[i] = std::tanh(sum);
    }

    float z2 = b2;
    for(size_t i = 0; i < hidden_neurons; ++i)
    {
        z2 += w2[i] * hidden_layer[i];
    }

    float theta_raw = static_cast<float>(nn_eps0->Value()) + safeSoftplus(z2);
    theta_hat = std::max(theta_min, std::min(theta_raw, theta_max));
    
    float estimated_mass = 1.0f / theta_hat;
    if(!std::isfinite(estimated_mass))
    {
        estimated_mass = min_mass;
    }
    mass_hat = std::max(min_mass, std::min(max_mass, estimated_mass));

    float uz = thrust_input;
    float y_sup = actual_acc_z + g;
    float eps = y_sup - theta_hat * uz;
    mass_residual = eps;

    float eta = static_cast<float>(nn_learning_rate->Value());
    if(eta > 0.0f && dt > 0.0f)
    {
        float lambda = static_cast<float>(nn_regularization->Value());
        float uz_sq = uz * uz;
        float g_norm;
        if(nn_use_nlms->Value() >= 0.5f)
        {
            g_norm = (uz_sq > 0.0f) ? (eps * uz) / (1.0f + uz_sq) : 0.0f;
        }
        else
        {
            g_norm = eps * uz;
        }
        
        float sig = sigmoid(z2);
        float delta_common = eta * g_norm * sig;

        for(size_t i = 0; i < hidden_neurons; ++i)
        {
            float h_val = hidden_layer[i];
            float dh = 1.0f - h_val * h_val;
            float w2_i = w2[i];
            w2[i] += dt * (delta_common * h_val - lambda * w2[i]);
            float grad_common = delta_common * w2_i*dh;
            W1[i][0] += dt * (grad_common * mu0 - lambda * W1[i][0]);
            W1[i][1] += dt * (grad_common * mu1 - lambda * W1[i][1]);
            b1[i] += dt * (grad_common - lambda * b1[i]);
        }
        b2 += dt * (delta_common - lambda * b2);
    }

    u_prev_z = uz;
}

void MyController::plotEstimatedMass(const LayoutPosition *position)
{
    GroupBox *plots_group = new GroupBox(position, "NN monitoring");

    DataPlot1D *mass_plot = new DataPlot1D(plots_group->NewRow(), "Estimated Mass", 0, 4);
    mass_plot->AddCurve(state->Element(9), DataPlot::Blue);

    DataPlot1D *thrust_plot = new DataPlot1D(plots_group->LastRowLastCol(), "Thrust command", -1, 1);
    thrust_plot->AddCurve(state->Element(10), DataPlot::Red);

    DataPlot1D *weights_plot = new DataPlot1D(plots_group->NewRow(), "NN weights (first neuron)", -1, 1);
    size_t row = kStateBaseEntries;
    for(size_t i = 0; i < kLoggedNeurons; ++i)
    {
        size_t base = row + i * kWeightsPerLoggedNeuron;
        weights_plot->AddCurve(state->Element(base + 0), DataPlot::Green);
        weights_plot->AddCurve(state->Element(base + 1), DataPlot::Blue);
        weights_plot->AddCurve(state->Element(base + 2), DataPlot::Red);
        weights_plot->AddCurve(state->Element(base + 3), DataPlot::Yellow);
    }

    DataPlot1D *xy_dhat_plot = new DataPlot1D(plots_group->LastRowLastCol(), "XY dhat", -5, 5);
    xy_dhat_plot->AddCurve(state->Element(12), DataPlot::Blue);
    xy_dhat_plot->AddCurve(state->Element(13), DataPlot::Red);

    DataPlot1D *kf_and_residual_plot = new DataPlot1D(plots_group->NewRow(), "Kalman + residuals", -5, 5);
    kf_and_residual_plot->AddCurve(state->Element(11), DataPlot::Green);
    kf_and_residual_plot->AddCurve(state->Element(18), DataPlot::Yellow);
    kf_and_residual_plot->AddCurve(state->Element(19), DataPlot::Red);
}

void MyController::plotXYWeights(const LayoutPosition *position)
{
    GroupBox *xy_plots_group = new GroupBox(position, "XY NN weights");
    DataPlot1D *xy_weights_plot = new DataPlot1D(xy_plots_group->NewRow(), "XY NN weights", -2, 2);
    size_t xy_weights_row = kStateBaseEntries + kLoggedNeurons * kWeightsPerLoggedNeuron;
    for(size_t i = 0; i < kLoggedNeurons; ++i)
    {
        size_t base = xy_weights_row + i * kXYWeightsPerLoggedNeuron;
        xy_weights_plot->AddCurve(state->Element(base + 0), DataPlot::Green);
        xy_weights_plot->AddCurve(state->Element(base + 1), DataPlot::Blue);
        xy_weights_plot->AddCurve(state->Element(base + 2), DataPlot::Red);
        xy_weights_plot->AddCurve(state->Element(base + 3), DataPlot::Yellow);
    }
}
