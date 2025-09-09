#include "set_points_client.hpp"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using set_points::SetPoints;
using set_points::SetPointsRequest;
using set_points::SetPointsResponse;

SetPointsClient::SetPointsClient(std::shared_ptr<Channel> channel) : m_stub(SetPoints::NewStub(channel))
{
    m_speed_set_point = 0.0;
    m_heading_set_point = 0.0;
}

void SetPointsClient::update_set_points(double t, double x, double y, double u, double v, double heading,
                                        double loading)
{
    SetPointsRequest request;

    request.set_t(t);

    request.set_x(x);
    request.set_y(y);

    request.set_u(u);
    request.set_v(v);

    request.set_heading(heading);

    request.set_loading(loading);

    SetPointsResponse response;

    ClientContext context;

    Status status = m_stub->GetSetPoints(&context, request, &response);

    if (status.ok()) {
        m_speed_set_point = response.speed_set_point();
        m_heading_set_point = response.heading_set_point();

        m_speed_pid_on = response.speed_pid_on();
        m_heading_pid_on = response.heading_pid_on();

        m_speed_gain_adjust = response.speed_gain_adjust();
        m_heading_gain_adjust = response.heading_gain_adjust();
    }
    else {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
}

double SetPointsClient::get_speed_set_point()
{
    return m_speed_set_point;
}

double SetPointsClient::get_heading_set_point()
{
    return m_heading_set_point;
}

bool SetPointsClient::get_speed_pid_on()
{
    return m_speed_pid_on;
}

bool SetPointsClient::get_heading_pid_on()
{
    return m_heading_pid_on;
}

double SetPointsClient::get_speed_gain_adjust()
{
    return m_speed_gain_adjust;
}

double SetPointsClient::get_heading_gain_adjust()
{
    return m_heading_gain_adjust;
}