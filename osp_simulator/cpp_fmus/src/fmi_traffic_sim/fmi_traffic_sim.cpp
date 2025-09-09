
#include <cppfmu_cs.hpp>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <zb_traffic_sim/traffic_simulator.hpp>
#include <zb_traffic_sim/utilities/data_types.hpp>
#include <zeabuz/common/utilities/math.hpp>

using zeabuz::common::utilities::math::heading_to_rotation_matrix;
using zeabuz::common::utilities::math::to_rad;

struct Vector3ValueRef
{
    Vector3ValueRef() : x(-1), y(-1), z(-1){};
    cppfmu::FMIValueReference x;
    cppfmu::FMIValueReference y;
    cppfmu::FMIValueReference z;
};

struct PoseValueRef
{
    Vector3ValueRef position;
    Vector3ValueRef orientation;
};

struct twistValueRef
{
    Vector3ValueRef linear;
    Vector3ValueRef angular;
};

struct ParameterValueRef
{
    cppfmu::FMIValueReference traffic_scene_path;
};

struct ValueReferences
{
    ParameterValueRef parameters;
    PoseValueRef pose;
    twistValueRef twistBody;
    twistValueRef twistNED;
};

class FmuTrafficSim : public cppfmu::SlaveInstance
{
   public:
    FmuTrafficSim(cppfmu::FMIString fmuResourceLocation) : _resource_location(fmuResourceLocation)
    {
        initializeValueReferences();
        initializeSignalMaps();
    }

    TrafficSimulator m_traffic_manager_api{};
    std::string m_traffic_scenario_path = "";

    void initializeValueReferences()
    {
        m_ValueRefs.parameters.traffic_scene_path = 0;

        m_ValueRefs.pose.position.x = 1;
        m_ValueRefs.pose.position.y = 2;
        m_ValueRefs.pose.position.z = 3;
        m_ValueRefs.pose.orientation.x = 4;
        m_ValueRefs.pose.orientation.y = 5;
        m_ValueRefs.pose.orientation.z = 6;

        m_ValueRefs.twistBody.linear.x = 7;
        m_ValueRefs.twistBody.linear.y = 8;
        m_ValueRefs.twistBody.linear.z = 9;
        m_ValueRefs.twistBody.angular.x = 10;
        m_ValueRefs.twistBody.angular.y = 11;
        m_ValueRefs.twistBody.angular.z = 12;

        m_ValueRefs.twistNED.linear.x = 13;
        m_ValueRefs.twistNED.linear.y = 14;
        m_ValueRefs.twistNED.linear.z = 15;
        m_ValueRefs.twistNED.angular.x = 16;
        m_ValueRefs.twistNED.angular.y = 17;
        m_ValueRefs.twistNED.angular.z = 18;
    }

    void initializeSignalMaps()
    {
        // Parameters
        m_StringSignals[m_ValueRefs.parameters.traffic_scene_path] = m_traffic_scenario_path;

        // output signals
        m_RealSignals[m_ValueRefs.pose.position.x] = 0;
        m_RealSignals[m_ValueRefs.pose.position.y] = 0;
        m_RealSignals[m_ValueRefs.pose.position.z] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.x] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.y] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.z] = 0;

        m_RealSignals[m_ValueRefs.twistBody.linear.x] = 0;
        m_RealSignals[m_ValueRefs.twistBody.linear.y] = 0;
        m_RealSignals[m_ValueRefs.twistBody.linear.z] = 0;
        m_RealSignals[m_ValueRefs.twistBody.angular.x] = 0;
        m_RealSignals[m_ValueRefs.twistBody.angular.y] = 0;
        m_RealSignals[m_ValueRefs.twistBody.angular.z] = 0;

        m_RealSignals[m_ValueRefs.twistNED.linear.x] = 0;
        m_RealSignals[m_ValueRefs.twistNED.linear.y] = 0;
        m_RealSignals[m_ValueRefs.twistNED.linear.z] = 0;
        m_RealSignals[m_ValueRefs.twistNED.angular.x] = 0;
        m_RealSignals[m_ValueRefs.twistNED.angular.y] = 0;
        m_RealSignals[m_ValueRefs.twistNED.angular.z] = 0;
    }

    void ExitInitializationMode() override
    {
        update_inputs();
    }

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[])
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_StringSignals.at(vr[i]) = value[i];
        }
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[])
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_RealSignals.at(vr[i]) = value[i];
        }
    }

    void SetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIInteger value[]) override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_IntegerSignals.at(vr[i]) = value[i];
        }
    }

    void SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIBoolean value[])
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_BooleanSignals.at(vr[i]) = value[i];
        }
    };

    void GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_StringSignals.at(vr[i]).c_str();
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_RealSignals.at(vr[i]);
        }
    }

    void GetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIInteger value[]) const override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_IntegerSignals.at(vr[i]);
        }
    }

    void GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIBoolean value[]) const
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_BooleanSignals.at(vr[i]);
        }
    }

    void update_inputs()
    {
        auto new_traffic_scene = m_StringSignals.at(m_ValueRefs.parameters.traffic_scene_path);
        if (new_traffic_scene != m_traffic_scenario_path) {
            std::cout << "Loading new traffic scenario file: " << new_traffic_scene << std::endl;
            std::filesystem::path currentPath = std::filesystem::current_path();
            std::cout << "Current Path: " << currentPath << std::endl;
            m_traffic_scenario_path = new_traffic_scene;
            try {
                m_traffic_manager_api.configure_scene_from_file(m_traffic_scenario_path);
            }
            catch (std::exception& e) {
                std::cout << "Failed to load traffic scenario " << m_traffic_scenario_path << e.what() << std::endl;
            }
        }
    }

    void update_outputs()
    {
        auto vessel_data = m_traffic_manager_api.get_target_vessels_data();
        if (vessel_data.size() == 0) {
            return;
        }
        const auto ego_id = 0;
        const auto& ego_vessel = vessel_data[ego_id];
        auto heading = ego_vessel.eta.NorthEastHeading.z();
        Eigen::Matrix3d rotMat = heading_to_rotation_matrix(heading);
        Eigen::Vector3d eta_dot = rotMat * ego_vessel.nu.SurgeSwayYaw;

        m_RealSignals.at(m_ValueRefs.pose.position.x) = ego_vessel.eta.NorthEastHeading.x();
        m_RealSignals.at(m_ValueRefs.pose.position.y) = ego_vessel.eta.NorthEastHeading.y();
        m_RealSignals.at(m_ValueRefs.pose.position.z) = 0;

        m_RealSignals.at(m_ValueRefs.pose.orientation.x) = 0;
        m_RealSignals.at(m_ValueRefs.pose.orientation.y) = 0;
        m_RealSignals.at(m_ValueRefs.pose.orientation.z) = ego_vessel.eta.NorthEastHeading.z();

        m_RealSignals.at(m_ValueRefs.twistBody.linear.x) = ego_vessel.nu.SurgeSwayYaw.x();
        m_RealSignals.at(m_ValueRefs.twistBody.linear.y) = ego_vessel.nu.SurgeSwayYaw.y();
        m_RealSignals.at(m_ValueRefs.twistBody.linear.z) = 0;

        m_RealSignals.at(m_ValueRefs.twistBody.angular.x) = 0;
        m_RealSignals.at(m_ValueRefs.twistBody.angular.y) = 0;
        m_RealSignals.at(m_ValueRefs.twistBody.angular.z) = ego_vessel.nu.SurgeSwayYaw.z();

        m_RealSignals.at(m_ValueRefs.twistNED.linear.x) = eta_dot.x();
        m_RealSignals.at(m_ValueRefs.twistNED.linear.y) = eta_dot.y();
        m_RealSignals.at(m_ValueRefs.twistNED.linear.z) = 0;

        m_RealSignals.at(m_ValueRefs.twistNED.angular.x) = 0;
        m_RealSignals.at(m_ValueRefs.twistNED.angular.y) = 0;
        m_RealSignals.at(m_ValueRefs.twistNED.angular.z) = eta_dot.z();

        //         m_ValueRefs.twistBody.linear.x = 7;
        // m_ValueRefs.twistBody.linear.y = 8;
        // m_ValueRefs.twistBody.linear.z = 9;
        // m_ValueRefs.twistBody.angular.x = 10;
        // m_ValueRefs.twistBody.angular.y = 11;
        // m_ValueRefs.twistBody.angular.z = 12;

        // m_ValueRefs.twistNED.linear.x = 13;
        // m_ValueRefs.twistNED.linear.y = 14;
        // m_ValueRefs.twistNED.linear.z = 15;
        // m_ValueRefs.twistNED.angular.x = 16;
        // m_ValueRefs.twistNED.angular.y = 17;
        // m_ValueRefs.twistNED.angular.z = 18;
    }

    bool DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal& /*endOfStep*/)
    {
        update_inputs();
        auto step_size = std::chrono::duration<double>(dt);
        m_traffic_manager_api.step_simulation(step_size);
        update_outputs();
        return true;
    }

   private:
    std::string _resource_location;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_RealSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_IntegerSignals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_StringSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_BooleanSignals;

    ValueReferences m_ValueRefs;
};

struct component
{
    std::vector<int> codes;
};

struct mtllc
{
    int cardId;
    std::unordered_map<int, component> components;
};

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString fmuGUID, cppfmu::FMIString fmuResourceLocation,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger logger)
{
    // if (std::strcmp(fmuGUID, FMU_UUID) != 0) {
    //     throw std::runtime_error("FMU GUID mismatch");
    // }
    return cppfmu::AllocateUnique<FmuTrafficSim>(memory, fmuResourceLocation);
}