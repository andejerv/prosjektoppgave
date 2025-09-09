/*----------------------------------------------------------------------------*\

gRPC client that gets set-points speed and heading from a server.

Originally used to implement simple autocross functionality as a Python gRPC
server specifically for the Flakk-Rorvik pre-project (Python was used for the
autocross functionality in that project to get a flexible solution up and
running in a short time).

\*----------------------------------------------------------------------------*/

#ifndef SET_POINTS_CLIENT_H
#define SET_POINTS_CLIENT_H

#include <grpcpp/grpcpp.h>

#include <string>

#include "set_points.grpc.pb.h"

using grpc::Channel;
using set_points::SetPoints;

class SetPointsClient
{
   public:
    SetPointsClient(std::shared_ptr<Channel>);

    void update_set_points(double t, double x, double y, double u, double v, double heading, double loading);

    double get_speed_set_point();
    double get_heading_set_point();

    bool get_speed_pid_on();
    bool get_heading_pid_on();

    double get_speed_gain_adjust();
    double get_heading_gain_adjust();

   private:
    std::unique_ptr<SetPoints::Stub> m_stub;

    double m_speed_set_point;
    double m_heading_set_point;

    bool m_speed_pid_on;
    bool m_heading_pid_on;

    double m_speed_gain_adjust;
    double m_heading_gain_adjust;
};

#endif