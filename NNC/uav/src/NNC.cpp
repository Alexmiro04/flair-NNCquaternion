//  created:    2025/03/02
//  filename:   NNC.cpp
//
//  author:     ateveraz
//
//  version:    $Id: 0.1$
//
//  purpose:    Custom control template
//
//
/*********************************************************************/

#include "NNC.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <GroupBox.h>
#include <ComboBox.h>
#include <CheckBox.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <iostream>
#include <Vector2D.h>
#include <TabWidget.h>
#include <Bldc.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

NNC::NNC(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false), controlMode_t(ControlMode_t::Default) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        targetVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    }
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
																 
    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"start_circle");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_circle");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");

    // Groupbox for control selection
    controlModeBox = new GroupBox(GetButtonsLayout()->NewRow(),"Control mode");
    on_customController = new PushButton(controlModeBox->NewRow(), "Activate");
    off_customController = new PushButton(controlModeBox->LastRowLastCol(), "Deactivate");
    control_selection = new ComboBox(controlModeBox->NewRow(), "Control selection");
    control_selection->AddItem("Default");
    control_selection->AddItem("Custom controller");
    control_selection->AddItem("Sliding + myCtrl PD");

    esc_fault_enabled = new CheckBox(controlModeBox->NewRow(), "Enable ESC fault", false);
    esc_fault_motor_index = new DoubleSpinBox(controlModeBox->NewRow(), "Failed motor index (1-4)", 1, 4, 1, 0, 1);
    esc_fault_percentage = new DoubleSpinBox(controlModeBox->LastRowLastCol(), "Failure percentage [%]", 0, 100, 1, 0, 0);

    // Custom tasks
    GroupBox *task_selection_box = new GroupBox(GetButtonsLayout()->LastRowLastCol(), "Custom task");
    task_selection = new ComboBox(task_selection_box->NewRow(), "Custom task");
    task_selection->AddItem("Hovering at zero");
    task_selection->AddItem("Regulation task");
    task_selection->AddItem("Circle tracking");

    desired_position = new Vector3DSpinBox(task_selection_box->NewRow(), "Desired position", -3, 3, 0.1, 3);
    desired_yaw = new DoubleSpinBox(task_selection_box->LastRowLastCol(), "Desired yaw", -M_PI, M_PI, 0.1, 3);

    
    circle=new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),"circle");
    uavVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    uavVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    uavVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    uavVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    uavVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");

    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

    // Custom control law
    Tab *gui_custom_controller = new Tab(getFrameworkManager()->GetTabWidget(),"NNC");
    auto *tabs_custom_controller = new TabWidget(gui_custom_controller->At(0,0),"Setup");
    setup_custom_controller = new Tab(tabs_custom_controller,"Setup");
    graphs_custom_controller = new Tab(tabs_custom_controller,"Graphs");
    xy_weights_custom_controller = new Tab(tabs_custom_controller,"XY Weights");
    setup_sliding_controller = new Tab(tabs_custom_controller,"Sliding Setup");
    graphs_sliding_controller = new Tab(tabs_custom_controller,"Sliding Graphs");

    myCtrl = new MyController(setup_custom_controller->At(0,0),"NNC");
    myCtrl->plotEstimatedMass(graphs_custom_controller->At(0,0));
    myCtrl->plotXYWeights(xy_weights_custom_controller->At(0,0));

    slidingCtrl = new Sliding(setup_sliding_controller->At(0,0),"Sliding");
    slidingCtrl->UseDefaultPlot(graphs_sliding_controller->At(0,0));
    slidingCtrl->UseDefaultPlot2(graphs_sliding_controller->At(0,1));
    slidingCtrl->UseDefaultPlot3(graphs_sliding_controller->At(1,0));
    slidingCtrl->UseDefaultPlot4(graphs_sliding_controller->At(1,1));
    slidingCtrl->UseDefaultPlot5(graphs_sliding_controller->At(2,0));

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);
    AddDeviceToControlLawLog(myCtrl);
    AddDeviceToControlLawLog(slidingCtrl);

    customOrientation=new AhrsData(this,"orientation");
}

NNC::~NNC() {
}

void NNC::ComputeCustomTorques(Euler &torques)
{
    // Implement your custom control law here or call a controller class. 
    ComputeDefaultTorques(torques);
    thrust = ComputeDefaultThrust();

    // Selection of the control mode based on the control_selection combobox. 
    switch (control_selection->CurrentIndex())
    {
        case 1:
            controlMode_t = ControlMode_t::Custom;
            computeMyCtrl(torques);
            ComputeCustomThrust();
            applyEscMotorFailure();
            break;

        case 2:
            controlMode_t = ControlMode_t::Sliding;
            computeSlidingCtrl(torques);
            ComputeCustomThrust();
            applyEscMotorFailure();
            break;

        default:
            controlMode_t = ControlMode_t::Default;
            Thread::Warn("NNC: default control law started. Check custom torque definition. \n");
            EnterFailSafeMode();
            break;
    }

    // Independent from BLDC backend support, inject a virtual motor-loss effect
    // directly in the commanded torques/thrust so the vehicle behaves as if one
    // motor is slower.
    applyVirtualMotorFailureToControl(torques);
}

void NNC::computeMyCtrl(Euler &torques)
{
    // Get position, velocity and quaternion from the VRPN object in its coordinate system
    Vector3Df uav_pos, uav_vel; 
    Quaternion vrpn_quaternion;
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(vrpn_quaternion);

    // Get current orientation and angular speed from the AHRS object (IMU)
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();

    // Use yaw from VRPN and roll, pitch from IMU
    Euler ahrsEuler = currentQuaternion.ToEuler();
    ahrsEuler.yaw = vrpn_quaternion.ToEuler().yaw;
    Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

    // Compute the position and velocity errors in the UAV frame
    Vector2Df pos_error2D, vel_error2D, acceleration2D;
    // Example of desired altitude [m] => (ALWAYS A POSITIVE VALUE) 
    // Because the AltitudeValues function returns a positive value also. However, the UAV's altitude is negative in the VRPN coordinate system.
    float altittude_desired = desired_position->Value().z; 
    float yaw_ref;
    float z, dz;
    AltitudeValues(z, dz);
    PositionValues(pos_error2D, vel_error2D, yaw_ref, acceleration2D);
    // Notice that the error definition is current - desired for x,y and z. 
    Vector3Df pos_error = Vector3Df(pos_error2D.x, pos_error2D.y, z-altittude_desired);
    Vector3Df vel_error = Vector3Df(vel_error2D.x, vel_error2D.y, dz);

    // Set the values of the custom controller and update it
    //myCtrl->SetValues(pos_error, vel_error, mixQuaternion, currentAngularSpeed, yaw_ref, acceleration2D.x, acceleration2D.y, 0.0f);
    myCtrl->SetValues(pos_error, vel_error, Vector3Df(uav_pos.x, uav_pos.y, -uav_pos.z), mixQuaternion, currentAngularSpeed, yaw_ref, acceleration2D.x, acceleration2D.y, 0.0f);
    myCtrl->Update(GetTime());

    // Apply the computed torques and thrust
    torques.roll = myCtrl->Output(0);
    torques.pitch = myCtrl->Output(1);
    torques.yaw = myCtrl->Output(2);
    thrust = myCtrl->Output(3); 
    // If you prefer, you can also use the ComputeDefaultThrust() function. E.g.:
    // thrust = ComputeDefaultThrust();
    // The desired take-off altitude will be used as a reference. 
}

void NNC::computeSlidingCtrl(Euler &torques)
{
    // Hybrid mode: keep myCtrl position PD (mainly thrust and XY guidance),
    // but close orientation loop with Sliding in quaternion form.
    // Get position, velocity and quaternion from the VRPN object in its coordinate system
    Vector3Df uav_pos, uav_vel; 
    Quaternion vrpn_quaternion;
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(vrpn_quaternion);

    // Get current orientation and angular speed from the AHRS object (IMU)
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();

    // Use yaw from VRPN and roll, pitch from IMU
    Euler ahrsEuler = currentQuaternion.ToEuler();
    ahrsEuler.yaw = vrpn_quaternion.ToEuler().yaw;
    Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

    // Compute the position and velocity errors in the UAV frame
    Vector2Df pos_error2D, vel_error2D, acceleration2D;
    // Example of desired altitude [m] => (ALWAYS A POSITIVE VALUE) 
    // Because the AltitudeValues function returns a positive value also. However, the UAV's altitude is negative in the VRPN coordinate system.
    float altittude_desired = desired_position->Value().z; 
    float yaw_ref;
    float z, dz;
    AltitudeValues(z, dz);
    PositionValues(pos_error2D, vel_error2D, yaw_ref, acceleration2D);
    // Notice that the error definition is current - desired for x,y and z. 
    float ze = z - altittude_desired;
    Vector3Df pos_error = Vector3Df(pos_error2D.x, pos_error2D.y, ze);
    Vector3Df vel_error = Vector3Df(vel_error2D.x, vel_error2D.y, dz);

    // Set the values of the custom controller and update it
    //myCtrl->SetValues(pos_error, vel_error, mixQuaternion, currentAngularSpeed, yaw_ref, acceleration2D.x, acceleration2D.y, 0.0f);
    myCtrl->SetValues(pos_error, vel_error, Vector3Df(uav_pos.x, uav_pos.y, -uav_pos.z), mixQuaternion, currentAngularSpeed, yaw_ref, acceleration2D.x, acceleration2D.y, 0.0f);
    myCtrl->Update(GetTime());

    const float roll_ref = std::max(-0.5f, std::min(0.5f, myCtrl->Output(3)));
    const float pitch_ref = std::max(-0.5f, std::min(0.5f, myCtrl->Output(4)));
    const Euler desiredEuler(roll_ref, pitch_ref, desired_yaw->Value());

    // const Quaternion desiredQuaternion = desiredEuler.ToQuaternion();
    const Quaternion desiredQuaternion(1, 0, 0, 0); // For testing the sliding controller in attitude only, we set a fixed desired quaternion.
    const Vector3Df wd(0.0f, 0.0f, 0.0f);

    slidingCtrl->SetValues(ze, dz, currentAngularRates, wd, currentQuaternion, desiredQuaternion);
    slidingCtrl->Update(GetTime());

    torques.roll = slidingCtrl->Output(0);
    torques.pitch = slidingCtrl->Output(1);
    torques.yaw = slidingCtrl->Output(2);
    thrust = myCtrl->Output(3); 
    // keep thrust from myCtrl position loop
}

void NNC::applyEscMotorFailure(void)
{
    Uav* uav = GetUav();
    if(uav == nullptr || uav->GetBldc() == nullptr)
    {
        return;
    }

    if(!esc_fault_enabled->Value())
    {
        clearEscMotorFailure();
        return;
    }

    float failure = static_cast<float>(esc_fault_percentage->Value()) / 100.0f;
    failure = std::max(0.0f, std::min(1.0f, failure));
    if(failure <= 0.0f)
    {
        clearEscMotorFailure();
        return;
    }

    int failed_motor = static_cast<int>(esc_fault_motor_index->Value() + 0.5f) - 1;
    if(failed_motor < 0 || failed_motor > 3)
    {
        clearEscMotorFailure();
        return;
    }

    if(esc_fault_applied && esc_fault_last_motor != failed_motor)
    {
        if(esc_fault_last_motor >= 0 && esc_fault_last_motor <= 3)
        {
            uav->GetBldc()->SetPower(esc_fault_last_motor, esc_fault_base_power[esc_fault_last_motor]);
        }
    }

    // Capture a base value once and always apply the fault relative to that base.
    // If BLDC readback is not available here, we fallback to nominal power 1.0.
    if(!esc_fault_base_power_captured[failed_motor])
    {
        esc_fault_base_power[failed_motor] = 1.0f;
        esc_fault_base_power_captured[failed_motor] = true;
    }

    float target_power = esc_fault_base_power[failed_motor] * (1.0f - failure);
    target_power = std::max(0.0f, std::min(1.0f, target_power));
    uav->GetBldc()->SetPower(failed_motor, target_power);
    esc_fault_last_motor = failed_motor;
    esc_fault_applied = true;
}

void NNC::applyVirtualMotorFailureToControl(Euler &torques)
{
    if(!esc_fault_enabled->Value())
    {
        return;
    }

    float failure = static_cast<float>(esc_fault_percentage->Value()) / 100.0f;
    failure = std::max(0.0f, std::min(1.0f, failure));
    if(failure <= 0.0f)
    {
        return;
    }

    int failed_motor = static_cast<int>(esc_fault_motor_index->Value() + 0.5f) - 1;
    if(failed_motor < 0 || failed_motor > 3)
    {
        return;
    }

    // Quad-X approximation of motor-loss effects:
    // - less collective lift
    // - roll/pitch bias toward failed corner
    // - yaw imbalance from CW/CCW pair mismatch
    const float lift_loss = 0.35f * failure;   // 35% at full failure
    const float att_bias = 0.25f * failure;    // rad-equivalent control bias
    const float yaw_bias = 0.15f * failure;

    // In this codebase thrust is typically <= 0 in flight (see myCtrl saturation).
    if(thrust < 0.0f)
    {
        thrust *= (1.0f - lift_loss);
    }

    switch (failed_motor)
    {
        case 0: // front-left
            torques.roll  += att_bias;
            torques.pitch += att_bias;
            torques.yaw   += yaw_bias;
            break;
        case 1: // front-right
            torques.roll  -= att_bias;
            torques.pitch += att_bias;
            torques.yaw   -= yaw_bias;
            break;
        case 2: // rear-left
            torques.roll  += att_bias;
            torques.pitch -= att_bias;
            torques.yaw   -= yaw_bias;
            break;
        case 3: // rear-right
            torques.roll  -= att_bias;
            torques.pitch -= att_bias;
            torques.yaw   += yaw_bias;
            break;
        default:
            break;
    }
}

void NNC::clearEscMotorFailure(void)
{
    Uav* uav = GetUav();
    if(!esc_fault_applied || uav == nullptr || uav->GetBldc() == nullptr)
    {
        return;
    }

    if(esc_fault_last_motor >= 0 && esc_fault_last_motor <= 3)
    {
        uav->GetBldc()->SetPower(esc_fault_last_motor, esc_fault_base_power[esc_fault_last_motor]);
    }
    esc_fault_last_motor = -1;
    esc_fault_applied = false;
}

float NNC::ComputeCustomThrust(void)
{
    // Implement your custom thrust computation here or asign its value from another function, because it is a global variable.
    if (thrust == 0)
    {
        // For safety reasons, the default thrust is computed if the custom thrust is not defined.
        thrust = ComputeDefaultThrust();
        std::cout << "Custom thrust not defined, using default thrust: " << thrust << std::endl;
    }
    return thrust;
}

void NNC::StartCustomTorques(void)
{
    if (control_selection->CurrentIndex() == 0)
    {
        StartDefaultTorques();
        Start_task();
        Thread::Info("NNC: default control law started\n");
    }
    else
    {
        if(SetTorqueMode(TorqueMode_t::Custom) && SetThrustMode(ThrustMode_t::Custom) && control_selection->CurrentIndex() != 0)
        {
            controlMode_t = ControlMode_t::Custom;
            myCtrl->Reset();
            Start_task();
            Thread::Info("NNC: custom control law started\n");
        }
        else
        {
            StopCustomTorques();
            Thread::Err("NNC: could not start custom control law\n");
        }
    }
    
}

void NNC::StopCustomTorques(void)
{
    StartDefaultTorques();
    controlMode_t = ControlMode_t::Default;
    clearEscMotorFailure();
    Thread::Info("NNC: custom control law stopped\n");
}

void NNC::StartDefaultTorques(void)
{
    if (controlMode_t == ControlMode_t::Default)
    {
        Thread::Warn("NNC: already in default control law\n");
        return;
    }

    if(SetTorqueMode(TorqueMode_t::Default) && SetThrustMode(ThrustMode_t::Default) )
    {
        controlMode_t = ControlMode_t::Default;
        Thread::Info("NNC: default control law started\n");
    }
    else
    {
        Thread::Err("NNC: could not start default control law\n");
    }
}

const AhrsData *NNC::GetOrientation(void) const {
    //get yaw from vrpn
    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);

    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    // yaw from vrpn and roll, pitch from imu
    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    return customOrientation;
}

void NNC::AltitudeValues(float &z,float &dz) const{
    Vector3Df uav_pos,uav_vel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}

AhrsData *NNC::GetReferenceOrientation(void) {
    Vector2Df pos_err, vel_err, circle_acc; // in Uav coordinate system
    float yaw_ref;
    Euler refAngles;

    PositionValues(pos_err, vel_err, yaw_ref, circle_acc);

    refAngles.yaw=yaw_ref;

    uX->SetValues(pos_err.x, vel_err.x);
    uX->Update(GetTime());
    refAngles.pitch=uX->Output();

    uY->SetValues(pos_err.y, vel_err.y);
    uY->Update(GetTime());
    refAngles.roll=-uY->Output();

    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));

    return customReferenceOrientation;
}

void NNC::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref, Vector2Df &circle_acc) {
    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    if (behaviourMode==BehaviourMode_t::PositionHold) {
        pos_error=uav_2Dpos-posHold;
        vel_error=uav_2Dvel;
        yaw_ref=yawHold;
        circle_acc.x = 0.0f; // Circle acceleration is not used in PositionHold mode (Remove if not work)
        circle_acc.y = 0.0f;
    } 
    else if (behaviourMode==BehaviourMode_t::Hover) {
        pos_error=uav_2Dpos;
        vel_error=uav_2Dvel;
        yaw_ref=0;
        circle_acc.x = 0.0f; // Circle acceleration is not used in Hover mode (Remove if not work)
        circle_acc.y = 0.0f;
    } 
    else if (behaviourMode==BehaviourMode_t::Regulation) {
        Vector2Df desired_position_xy(desired_position->Value().x, desired_position->Value().y);
        pos_error=uav_2Dpos - desired_position_xy;
        vel_error=uav_2Dvel;
        yaw_ref=(float)desired_yaw->Value();
        circle_acc.x = 0.0f;
        circle_acc.y = 0.0f;
    }
    else { //Circle
        Vector3Df target_pos;
        Vector2Df circle_pos,circle_vel, circle_acc_ref;
        Vector2Df target_2Dpos;

        targetVrpn->GetPosition(target_pos);
        target_pos.To2Dxy(target_2Dpos);
        circle->SetCenter(target_2Dpos);

        //circle reference
        circle->Update(GetTime());
        circle->GetPosition(circle_pos);
        circle->GetSpeed(circle_vel);
        circle->GetAcceleration(circle_acc); // Acceleration for circle

        //error in optitrack frame
        pos_error=uav_2Dpos-circle_pos;
        vel_error=uav_2Dvel-circle_vel;
        yaw_ref = 0;
        circle_acc = circle_acc_ref;
    }

    //error in uav frame
    Quaternion currentQuaternion=GetCurrentQuaternion();
    Euler currentAngles;//in vrpn frame
    currentQuaternion.ToEuler(currentAngles);
    pos_error.Rotate(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);
    circle_acc.Rotate(-currentAngles.yaw);
}

void NNC::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode==BehaviourMode_t::Circle) && (!circle->IsRunning())) {
            VrpnPositionHold();
        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        clearEscMotorFailure();
        break;
    }
}

void NNC::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        // if (!targetVrpn->IsTracked(500)) {
        //     Thread::Err("VRPN, target lost\n");
        //     vrpnLost=true;
        //     EnterFailSafeMode();
        //     Land();
        // }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Hover) || (behaviourMode==BehaviourMode_t::Regulation))) {
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void NNC::ExtraCheckPushButton(void) {
    if(startCircle->Clicked()) {
        Start_task();
    }
    if(stopCircle->Clicked()) {
        StopCircle();
    }
    if(positionHold->Clicked()) {
        VrpnPositionHold();
    }
    if(on_customController->Clicked()) {
        StartCustomTorques();
    } 
    if(off_customController->Clicked()) {
        StopCustomTorques();
    }
}

void NNC::ExtraCheckJoystick(void) {
    /*     Do not use cross, start nor select buttons!!
    0: "start"       1: "select"      2: "square"      3: "triangle"
    4: "circle"      5: "cross";      6: "left 1"      7: "left 2"
    8: "left 3"      9: "right 1"     10: "right 2"    11: "right 3"
    12: "up"         13: "down"       14: "left"       15: "right"
    */

    //R1 and Circle
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        Start_task();
    }

    //R1 and Cross
    if(GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
        StopCircle();
    }
    
    //R1 and Square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        VrpnPositionHold();
    }
}

void NNC::Start_task(void) {
    if( behaviourMode==BehaviourMode_t::Circle) {
        Thread::Warn("NNC: already in circle mode\n");
        return;
    }
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("NNC: start circle\n");
    } else {
        Thread::Warn("NNC: could not start circle\n");
        return;
    }

    // Defining desired task. 
    if (task_selection->CurrentIndex() == 0) {
        behaviourMode=BehaviourMode_t::Hover;
        Thread::Info("NNC: hovering at zero\n");
    }
    else if (task_selection->CurrentIndex() == 1) {
        behaviourMode=BehaviourMode_t::Regulation;
        Thread::Info("NNC: regulation task\n");
    }
    else if (task_selection->CurrentIndex() == 2) {
        Vector3Df uav_pos;
        Vector2Df uav_2Dpos;

        circle->SetCenter(Vector2Df(0.0, 0.0)); //circle->SetCenter(target_2Dpos);

        uavVrpn->GetPosition(uav_pos);
        uav_pos.To2Dxy(uav_2Dpos);
        circle->StartTraj(uav_2Dpos, 1); //circle->StartTraj(uav_2Dpos, 1); // One lap

        uX->Reset();
        uY->Reset();
        behaviourMode=BehaviourMode_t::Circle;
        Thread::Info("NNC: circle tracking\n");
    }
    else {
        Thread::Err("NNC: unknown task\n");
        return;
    }
}

void NNC::StopCircle(void) {
    if( behaviourMode!=BehaviourMode_t::Circle) {
        Thread::Warn("NNC: not in circle mode\n");
        return;
    }
    circle->FinishTraj();
    //GetJoystick()->Rumble(0x70);
    Thread::Info("NNC: finishing circle\n");
}

void NNC::VrpnPositionHold(void) {
    if( behaviourMode==BehaviourMode_t::PositionHold) {
        Thread::Warn("NNC: already in vrpn position hold mode\n");
        return;
    }
	Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
	yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("NNC: holding position\n");
}