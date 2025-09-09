#include "traffic_sim_lib_wrapper.hpp"

#include <fstream>
#include <iostream>

TrafficSimLibWrapper::TrafficSimLibWrapper(std::shared_ptr<TrafficSimulator> traffic_sim)
: m_traffic_manager_api(traffic_sim)
{
}

void TrafficSimLibWrapper::configure_scene_from_file(const std::string& file_name)
{
    m_traffic_manager_api->configure_scene_from_file(file_name);
}

nlohmann::json TrafficSimLibWrapper::retrieve_full_scenario_description(const std::string& filename)
{
    std::ifstream ifs(filename);
    if (ifs.fail()) {
        throw std::invalid_argument(" Error opening file " + filename);
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return nlohmann::json::parse(oss.str());
}

void TrafficSimLibWrapper::step_simulation(const std::chrono::milliseconds& step_size)
{
    m_traffic_manager_api->step_simulation(step_size);
}

void TrafficSimLibWrapper::update_traffic_sim_vessels()
{
    m_traffic_sim_vessels = m_traffic_manager_api->get_target_vessels_data();
}

std::vector<VesselData> TrafficSimLibWrapper::get_traffic_sim_vessels() const
{
    return m_traffic_sim_vessels;
}

void TrafficSimLibWrapper::update_ego_pose(const EtaData& ego_eta)
{
    m_ego_eta = ego_eta;
    m_ego_vessel_eta_received = true;
    update_ego_vessel_data();
}

void TrafficSimLibWrapper::update_ego_twist(const NuData& ego_nu)
{
    m_ego_nu = ego_nu;
    m_ego_vessel_nu_received = true;
    update_ego_vessel_data();
}

void TrafficSimLibWrapper::update_ego_vessel_data()
{
    if (m_ego_vessel_eta_received && m_ego_vessel_nu_received) {
        std::vector<EgoVesselData> ego_vessel;
        ego_vessel.emplace_back(1, m_ego_eta, m_ego_nu, M_EGO_VESSEL_DIMENSIONS.first, M_EGO_VESSEL_DIMENSIONS.second);
        m_traffic_manager_api->set_ego_vessels_data(ego_vessel);
        m_ego_vessel_eta_received = false;
        m_ego_vessel_nu_received = false;
    }
}

Eigen::Vector3d TrafficSimLibWrapper::get_three_dof_ego_vessel_position()
{
    return Eigen::Vector3d{m_ego_eta.NorthEastHeading(0), m_ego_eta.NorthEastHeading(1), m_ego_eta.NorthEastHeading(2)};
}

void TrafficSimLibWrapper::reset_simulation()
{
    m_traffic_manager_api->reset_simulation();
}
