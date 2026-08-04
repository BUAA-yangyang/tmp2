/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"
#include "FSM/NavigationEarlyContact.h"
#include "FSM/NavigationGaitProfile.h"
#include <boost/bind/bind.hpp>
#include <cmath>
#include <limits>

namespace {
constexpr double kFootForceThresholdN = 5.0;
constexpr double kFootForceFreshnessS = 0.20;
constexpr double kMaximumTiltRad = 0.20;
constexpr double kMaximumGyroRadS = 0.15;
constexpr double kStableDurationS = 0.50;
constexpr double kGyroFilterTimeConstantS = 0.10;
constexpr double kGyroFilterMaximumStepS = 0.05;
static_assert(
    kFootForceThresholdN
        == navigation_early_contact::kFootForceThresholdN,
    "early-contact and safe-stand force evidence must use one threshold");
static_assert(
    kFootForceFreshnessS
        == navigation_early_contact::kFootForceFreshnessS,
    "early-contact and safe-stand freshness must remain aligned");
}

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp),
     _defaultGaitPeriodS(ctrlComp->waveGen->getT()),
     _safeStandRequested(false),
     _safeStandGyroFilter(
         kGyroFilterTimeConstantS, kGyroFilterMaximumStepS),
     _vx(0.0), _vy(0.0), _wz(0.0){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    setGaitHeight(navigation_gait::kMoveBaseSwingHeightM);
    _footForceZ.fill(0.0);
    _footForceStamp.fill(ros::Time());
    _swingUnloadObserved.fill(false);
    initRecv();
}

void State_move_base::enter(){
    // Never carry an old velocity across an FSM transition.  In particular,
    // these members were uninitialized in the upstream implementation, so the
    // first move_base control cycle could receive arbitrary values before the
    // first /cmd_vel callback.
    _vx = 0.0;
    _vy = 0.0;
    _wz = 0.0;
    _safeStandRequested = false;
    _safeStandRequestStamp = ros::Time();
    _stableSince = ros::Time();
    _safeStandGyroFilter.reset();
    _footForceZ.fill(0.0);
    _footForceStamp.fill(ros::Time());
    _swingUnloadObserved.fill(false);
    _ctrlComp->clearContactOverrides();
    _ctrlComp->waveGen->setPeriod(
        navigation_gait::kMoveBaseGaitPeriodS);
    setForceAllStance(false);
    State_Trotting::enter();
    publishSafeStandReady(false);
    publishReady(true);
}

void State_move_base::exit(){
    // Publish false before the base exit path joins worker threads.  If the
    // controller stalls or dies instead, the guard's heartbeat timeout closes
    // the output automatically.
    publishReady(false);
    State_Trotting::exit();
    // State_Trotting::exit() requests SWING_ALL. Undo that transient request
    // for the guarded fixed-stand transition; FixedStand::enter() also asserts
    // STANCE_ALL before the next control update.
    if(_safeStandRequested){
        _ctrlComp->setAllStance();
    }
    _ctrlComp->clearContactOverrides();
    _swingUnloadObserved.fill(false);
    _ctrlComp->waveGen->setPeriod(_defaultGaitPeriodS);
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        beginSafeStand();
    }
    if(_safeStandRequested){
        if(safeStandReady()){
            publishSafeStandReady(true);
            return FSMStateName::FIXEDSTAND;
        }
        return FSMStateName::MOVE_BASE;
    }
    return FSMStateName::MOVE_BASE;
}

void State_move_base::getUserCmd(){
    ros::spinOnce();
    if(_safeStandRequested){
        _vx = 0.0;
        _vy = 0.0;
        _wz = 0.0;
        setHighCmd(0.0, 0.0, 0.0);
        setForceAllStance(true);
        return;
    }
    const ros::Time now = ros::Time::now();
    if(_lastReadyPublish.isZero() ||
       now < _lastReadyPublish ||
       (now - _lastReadyPublish).toSec() >= 0.1){
        publishReady(true);
    }
    setHighCmd(_vx, _vy, _wz);
}

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
    if(_safeStandRequested){
        return;
    }
    if(!std::isfinite(msg.linear.x) ||
       !std::isfinite(msg.linear.y) ||
       !std::isfinite(msg.angular.z)){
        _vx = 0.0;
        _vy = 0.0;
        _wz = 0.0;
        ROS_WARN_THROTTLE(1.0, "Ignoring non-finite /cmd_vel command");
        return;
    }
    _vx = msg.linear.x;
    _vy = msg.linear.y;
    _wz = msg.angular.z;
}

void State_move_base::initRecv(){
    _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_move_base::twistCallback, this);
    _readyPub = _nm.advertise<std_msgs::Bool>("/a1/controller_ready", 1);
    _safeStandReadyPub =
        _nm.advertise<std_msgs::Bool>("/a1/safe_stand_ready", 1, true);
    const std::array<std::string, 4> topics = {{
        "/visual/FR_foot_contact/the_force",
        "/visual/FL_foot_contact/the_force",
        "/visual/RR_foot_contact/the_force",
        "/visual/RL_foot_contact/the_force",
    }};
    for(std::size_t index = 0; index < topics.size(); ++index){
        _footForceSub[index] = _nm.subscribe<geometry_msgs::WrenchStamped>(
            topics[index],
            5,
            boost::bind(
                &State_move_base::footForceCallback,
                this,
                boost::placeholders::_1,
                index));
    }
}

void State_move_base::footForceCallback(
    const geometry_msgs::WrenchStamped::ConstPtr& msg, std::size_t index){
    const double force = msg->wrench.force.z;
    if(index >= _footForceZ.size() || !std::isfinite(force)){
        return;
    }
    _footForceZ[index] = std::abs(force);
    _footForceStamp[index] = ros::Time::now();
}

void State_move_base::adjustContactPhase(){
    const ros::Time now = ros::Time::now();
    for(std::size_t index = 0; index < _footForceZ.size(); ++index){
        double sampleAge = std::numeric_limits<double>::infinity();
        if(!now.isZero() && !_footForceStamp[index].isZero()){
            sampleAge = (now - _footForceStamp[index]).toSec();
        }
        const double phase = (*_ctrlComp->phase)(index);
        const bool wasLatched = _ctrlComp->contactOverrideActive(index);
        const navigation_early_contact::State next =
            navigation_early_contact::updateState(
                _ctrlComp->plannedContact(index),
                {_swingUnloadObserved[index], wasLatched},
                phase,
                _footForceZ[index],
                sampleAge);
        _swingUnloadObserved[index] = next.unloadObserved;
        _ctrlComp->setContactOverride(index, next.touchdownLatched);
        if(!wasLatched && next.touchdownLatched){
            ROS_INFO(
                "Navigation early touchdown after swing unload: leg=%zu "
                "phase=%.3f force=%.1f N; holding measured contact until "
                "the planned stance phase",
                index,
                phase,
                _footForceZ[index]);
        }
    }
}

void State_move_base::beginSafeStand(){
    if(_safeStandRequested){
        return;
    }
    _safeStandRequested = true;
    _vx = 0.0;
    _vy = 0.0;
    _wz = 0.0;
    _safeStandRequestStamp = ros::Time::now();
    _stableSince = ros::Time();
    _safeStandGyroFilter.reset();
    setForceAllStance(true);
    publishReady(false);
    publishSafeStandReady(false);
    ROS_INFO(
        "Safe stand requested: zeroing command and waiting for four-foot "
        "contact plus stable IMU before FIXEDSTAND");
}

bool State_move_base::safeStandReady(){
    const ros::Time now = ros::Time::now();
    if(now.isZero() || now < _safeStandRequestStamp){
        _stableSince = ros::Time();
        ROS_ERROR_THROTTLE(
            1.0, "Safe stand blocked: ROS/simulation clock moved backwards");
        return false;
    }

    bool measuredContact = true;
    for(std::size_t index = 0; index < _footForceZ.size(); ++index){
        measuredContact = measuredContact
            && !_footForceStamp[index].isZero()
            && now >= _footForceStamp[index]
            && (now - _footForceStamp[index]).toSec() <= kFootForceFreshnessS
            && _footForceZ[index] >= kFootForceThresholdN;
    }
    bool commandedContact = true;
    for(int index = 0; index < 4; ++index){
        commandedContact =
            commandedContact && (*_ctrlComp->contact)(index) == 1;
    }
    const Vec3 rpy = rotMatToRPY(_lowState->getRotMat());
    const Vec3 gyro = _lowState->getGyro();
    const std::array<double, 3> gyroSample = {{
        gyro(0), gyro(1), gyro(2),
    }};
    const SafeStandGyroFilterResult filteredGyro =
        _safeStandGyroFilter.update(now.toSec(), gyroSample);
    if(filteredGyro.discontinuity){
        _stableSince = ros::Time();
    }
    const bool imuStable =
        std::isfinite(rpy(0))
        && std::isfinite(rpy(1))
        && filteredGyro.valid
        && std::isfinite(filteredGyro.norm)
        && std::abs(rpy(0)) <= kMaximumTiltRad
        && std::abs(rpy(1)) <= kMaximumTiltRad
        && filteredGyro.norm <= kMaximumGyroRadS;
    if(!measuredContact || !commandedContact || !imuStable){
        _stableSince = ros::Time();
        ROS_WARN_THROTTLE(
            1.0,
            "Safe stand waiting: measured_contact=%d commanded_contact=%d "
            "roll=%.3f pitch=%.3f raw_gyro=%.3f filtered_gyro=%.3f",
            measuredContact,
            commandedContact,
            rpy(0),
            rpy(1),
            gyro.norm(),
            filteredGyro.norm);
        return false;
    }
    if(_stableSince.isZero()){
        _stableSince = now;
        return false;
    }
    if(now < _stableSince){
        _stableSince = ros::Time();
        return false;
    }
    return (now - _stableSince).toSec() >= kStableDurationS;
}

void State_move_base::publishReady(bool ready){
    std_msgs::Bool message;
    message.data = ready;
    _readyPub.publish(message);
    _lastReadyPublish = ros::Time::now();
}

void State_move_base::publishSafeStandReady(bool ready){
    std_msgs::Bool message;
    message.data = ready;
    _safeStandReadyPub.publish(message);
}

#endif  // COMPILE_WITH_MOVE_BASE
