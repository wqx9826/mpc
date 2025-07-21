#ifndef CHHROBOTICS_CPP_KINEMATICMODEL_H
#define CHHROBOTICS_CPP_KINEMATICMODEL_H

#include "data_struct.h"
#include "../param.h"

class KinematicModel {
public:
    VehicleState vehicle;

public:
    KinematicModel();

    KinematicModel(VehicleState robot_state);

    VehicleState getState();

    void updateState(double accel, double delta_f);

private:
    double ts_ = 0.0;
    double WHEEL_BASE_ = 0.0;


};
#endif
