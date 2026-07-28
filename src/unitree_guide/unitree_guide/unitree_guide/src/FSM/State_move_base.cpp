/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"
#include <cmath>

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp), _vx(0.0), _vy(0.0), _wz(0.0){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
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
    State_Trotting::enter();
    publishReady(true);
}

void State_move_base::exit(){
    // Publish false before the base exit path joins worker threads.  If the
    // controller stalls or dies instead, the guard's heartbeat timeout closes
    // the output automatically.
    publishReady(false);
    State_Trotting::exit();
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    if(_lastReadyPublish.isZero() ||
       now < _lastReadyPublish ||
       (now - _lastReadyPublish).toSec() >= 0.1){
        publishReady(true);
    }
    setHighCmd(_vx, _vy, _wz);
}

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
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
}

void State_move_base::publishReady(bool ready){
    std_msgs::Bool message;
    message.data = ready;
    _readyPub.publish(message);
    _lastReadyPublish = ros::Time::now();
}

#endif  // COMPILE_WITH_MOVE_BASE
