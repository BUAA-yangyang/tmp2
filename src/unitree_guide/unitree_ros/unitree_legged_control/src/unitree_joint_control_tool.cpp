#include "unitree_joint_control_tool.h"

float clamp(float &val, float min_val, float max_val)
{
    val = std::min(std::max(val, min_val), max_val);
    return val;
}

double clamp(double &val, double min_val, double max_val)
{
    val = std::min(std::max(val, min_val), max_val);
    return val;
}

double canonicalizeRevolutePosition(double position, double lower, double upper)
{
    if(!std::isfinite(position) ||
       !std::isfinite(lower) ||
       !std::isfinite(upper) ||
       lower > upper){
        return position;
    }

    // Gazebo may report a bounded revolute joint in an equivalent 2*pi
    // representation outside its URDF interval.  Select the equivalent angle
    // nearest the interval midpoint without clamping real out-of-limit motion.
    const double twoPi = 6.28318530717958647692;
    const double midpoint = lower + 0.5 * (upper - lower);
    return midpoint + std::remainder(position - midpoint, twoPi);
}

double computeVel(double currentPos, double lastPos, double lastVel, double period)
{
    return lastVel*0.35f + 0.65f*(currentPos-lastPos)/period;
}

double computeTorque(double currentPos, double currentVel, ServoCmd &cmd)
{
    double targetPos, targetVel, targetTorque, posStiffness, velStiffness, calcTorque;
    targetPos = cmd.pos;
    targetVel = cmd.vel;
    targetTorque = cmd.torque;
    posStiffness = cmd.posStiffness;
    velStiffness = cmd.velStiffness;
    if(fabs(targetPos-posStopF) < 1e-6) posStiffness = 0;
    if(fabs(targetVel-velStopF) < 1e-6) velStiffness = 0;
    calcTorque = posStiffness*(targetPos-currentPos) + velStiffness*(targetVel-currentVel) + targetTorque;
    return calcTorque;
}
