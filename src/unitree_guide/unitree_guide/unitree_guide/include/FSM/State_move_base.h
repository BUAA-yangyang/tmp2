/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#ifndef STATE_MOVE_BASE_H
#define STATE_MOVE_BASE_H

#include "FSM/State_Trotting.h"
#include "FSM/NavigationEarlyContact.h"
#include "FSM/SafeStandGyroFilter.h"
#include "ros/ros.h"
#include <array>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>

class State_move_base : public State_Trotting{
public:
    State_move_base(CtrlComponents *ctrlComp);
    ~State_move_base(){}
    void enter();
    void exit();
    FSMStateName checkChange();
private:
    void getUserCmd();
    void initRecv();
    void twistCallback(const geometry_msgs::Twist& msg);
    void footForceCallback(
        const geometry_msgs::WrenchStamped::ConstPtr& msg, std::size_t index);
    void adjustContactPhase() override;
    void beginSafeStand();
    bool safeStandReady();
    void publishReady(bool ready);
    void publishSafeStandReady(bool ready);
    ros::NodeHandle _nm;
    ros::Subscriber _cmdSub;
    std::array<ros::Subscriber, 4> _footForceSub;
    ros::Publisher _readyPub;
    ros::Publisher _safeStandReadyPub;
    ros::Time _lastReadyPublish;
    std::array<double, 4> _footForceZ;
    std::array<ros::Time, 4> _footForceStamp;
    std::array<bool, 4> _swingUnloadObserved;
    double _defaultGaitPeriodS;
    bool _safeStandRequested;
    ros::Time _safeStandRequestStamp;
    ros::Time _stableSince;
    SafeStandGyroFilter _safeStandGyroFilter;
    double _vx, _vy;
    double _wz;
};

#endif  // STATE_MOVE_BASE_H

#endif  // COMPILE_WITH_MOVE_BASE
