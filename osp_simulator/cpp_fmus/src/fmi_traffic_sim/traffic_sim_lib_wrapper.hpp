#ifndef ROS_SIMULATION_TOOLS_TRAFFIC_SIM_TRAFFIC_SIM_LIB_WRAPPER_HPP
#define ROS_SIMULATION_TOOLS_TRAFFIC_SIM_TRAFFIC_SIM_LIB_WRAPPER_HPP

#include <memory>
#include <nlohmann/json.hpp>
#include <zb_traffic_sim/traffic_simulator.hpp>

class TrafficSimLibWrapper final
{
   public:
    explicit TrafficSimLibWrapper(std::shared_ptr<TrafficSimulator> traffic_sim);

    void configure_scene_from_file(const std::string& file_name);
    nlohmann::json retrieve_full_scenario_description(const std::string& filename);
    void step_simulation(const std::chrono::milliseconds& step_size);
    void reset_simulation();
    void update_traffic_sim_vessels();

    void update_ego_pose(const EtaData& ego_eta);
    void update_ego_twist(const NuData& ego_nu);
    void update_ego_vessel_data();

    std::vector<VesselData> get_traffic_sim_vessels() const;
    Eigen::Vector3d get_three_dof_ego_vessel_position();
    void set_traffic_sim_vessels(const std::vector<VesselData>& vesssls);

   private:
    // TODO: What should these dimensions be?
    static constexpr std::pair<double, double> M_EGO_VESSEL_DIMENSIONS{8.5, 3.75};
    std::string m_traffic_scenario_path{};
    bool m_ego_vessel_eta_received{};
    bool m_ego_vessel_nu_received{};
    std::vector<VesselData> m_traffic_sim_vessels{};
    nlohmann::json m_current_scenario{};
    std::unordered_map<std::string, nlohmann::json> m_available_scenarios{};
    EtaData m_ego_eta{};
    NuData m_ego_nu{};
    std::shared_ptr<TrafficSimulator> m_traffic_manager_api;
};
#endif