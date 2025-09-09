#include "set_points_client_fmu.hpp"

#include <grpcpp/grpcpp.h>

#include <sstream>

SetPointsClientFMU::SetPointsClientFMU(const std::string& instance_name) : m_instance_name(instance_name)
{
    m_string_signals.insert(std::make_pair(m_CONNECTION_STRING, "127.0.0.1:50051"));

    m_real_signals.insert(std::make_pair(m_X, 0.0));
    m_real_signals.insert(std::make_pair(m_Y, 0.0));
    m_real_signals.insert(std::make_pair(m_U, 0.0));
    m_real_signals.insert(std::make_pair(m_V, 0.0));
    m_real_signals.insert(std::make_pair(m_HEADING, 0.0));
    m_real_signals.insert(std::make_pair(m_LOADING, 0.0));

    m_real_signals.insert(std::make_pair(m_SPEED_SET_POINT, 0.0));
    m_real_signals.insert(std::make_pair(m_HEADING_SET_POINT, 0.0));

    m_boolean_signals.insert(std::make_pair(m_SPEED_PID_ON, true));
    m_boolean_signals.insert(std::make_pair(m_HEADING_PID_ON, true));

    m_real_signals.insert(std::make_pair(m_SPEED_GAIN_ADJUST, 1.0));
    m_real_signals.insert(std::make_pair(m_HEADING_GAIN_ADJUST, 1.0));
}

void SetPointsClientFMU::ExitInitializationMode()
{
    std::string connection_string = m_string_signals.at(m_CONNECTION_STRING);

    auto channel = grpc::CreateChannel(connection_string, grpc::InsecureChannelCredentials());

    m_client = std::make_unique<SetPointsClient>(channel);
}

bool SetPointsClientFMU::DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean newStep,
                                cppfmu::FMIReal& endOfStep)
{
    double x = m_real_signals.at(m_X);
    double y = m_real_signals.at(m_Y);
    double u = m_real_signals.at(m_U);
    double v = m_real_signals.at(m_V);
    double heading = m_real_signals.at(m_HEADING);
    double loading = m_real_signals.at(m_LOADING);

    m_client->update_set_points(t, x, y, u, v, heading, loading);

    m_real_signals.at(m_SPEED_SET_POINT) = m_client->get_speed_set_point();
    m_real_signals.at(m_HEADING_SET_POINT) = m_client->get_heading_set_point();

    m_boolean_signals.at(m_SPEED_PID_ON) = m_client->get_speed_pid_on();
    m_boolean_signals.at(m_HEADING_PID_ON) = m_client->get_heading_pid_on();

    m_real_signals.at(m_SPEED_GAIN_ADJUST) = m_client->get_speed_gain_adjust();
    m_real_signals.at(m_HEADING_GAIN_ADJUST) = m_client->get_heading_gain_adjust();

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instanceName, cppfmu::FMIString fmuGUID, cppfmu::FMIString fmuResourceLocation,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger logger)
{
    if (strcmp(fmuGUID, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<SetPointsClientFMU>(memory, instanceName);
}
