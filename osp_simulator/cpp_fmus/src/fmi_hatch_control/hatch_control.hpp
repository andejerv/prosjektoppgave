#ifndef HATCH_CONTROL_HPP
#define HATCH_CONTROL_HPP

#include <atomic>
#include <functional>
#include <zeabuz/common/io/messages/custom/wago_hatch.hpp>
#include <zeabuz/common/io/messages/custom/wago_hatch_fb.hpp>

namespace HatchTypes
{
enum class HatchSensorStates : std::uint16_t { DI_OPENED = 0, DI_CLOSED = 1, DI_50 = 2, DI_EM_STOP = 3, UNDEFINED = 4 };
enum class State : std::uint16_t { STATE_UNDEFINED = 0, STATE_OPEN = 1, STATE_CLOSED = 2, STATE_50 = 3 };
enum class Command : std::uint16_t {
    COMMAND_OPEN = 0,
    COMMAND_CLOSE = 1,
    COMMAND_50 = 2,
    COMMAND_STOP = 3,
    COMMAND_RESET_FAULT = 4
};
enum class Action : std::uint16_t {
    ACTION_IDLE = 0,
    ACTION_OPENING = 1,     // Hatch moving to open state
    ACTION_CLOSING = 2,     // Hatch moving to closed state
    ACTION_50_CLOSING = 3,  // Hatch moving to 50% state from open state
    ACTION_50_OPENING = 4   // Hatch moving to 50% state from closed state
};

enum class Fault : std::uint16_t {
    FAULT_NO_FAULT = 0,            // No fault active
    FAULT_OPEN_TIMEOUT = 1,        // Hatch did not open in time
    FAULT_CLOSE_TIMEOUT = 2,       // Hatch did not close in time
    FAULT_50_CLOSING_TIMEOUT = 3,  // Hatch did not reach 50 % from open state in time
    FAULT_50_OPENING_TIMEOUT = 4,  // Hatch did not reach 50 % from closed state in time
    FAULT_GENERAL_FAULT = 5,       // Hatch general fault
    FAULT_TIMEOUT = 6              // Hatch general timeout fault
};
struct HatchCommand
{
    uint16_t hatch_id{};
    Command command{};
    bool operator==(const HatchCommand& rhs) const
    {
        return (hatch_id == rhs.hatch_id) && (command == rhs.command);
    }

    bool operator!=(const HatchCommand& rhs) const
    {
        return !(*this == rhs);
    }
};
struct HatchStatus
{
    uint16_t hatch_id{};
    State state{};
    Action action{};
    Fault fault{};
    bool operator==(const HatchStatus& rhs) const
    {
        return ((state == rhs.state) && (action == rhs.action) && (fault == rhs.fault));
    }

    bool operator!=(const HatchStatus& rhs) const
    {
        return !(*this == rhs);
    }
};
struct HatchStatuses
{
    bool auto_manual_control{};
    bool emergency_stop_active{};
    std::vector<HatchStatus> hatch_statuses{};

    bool operator==(const HatchStatuses& rhs) const
    {
        return ((hatch_statuses == rhs.hatch_statuses) && (auto_manual_control == rhs.auto_manual_control) &&
                (emergency_stop_active == rhs.emergency_stop_active));
    }

    bool operator!=(const HatchStatuses& rhs) const
    {
        return !(*this == rhs);
    }
};

}  // namespace HatchTypes

class HatchControl
{
   public:
    explicit HatchControl(const double hatch_timeout, const double start_position);

    void execute(const double time_step);
    void hatch_command_callback(const HatchTypes::Command command);
    HatchTypes::HatchStatus get_hatch_status() const;
    void iterate_time_taken_by_action(const double time_step);
    HatchTypes::HatchSensorStates get_hatch_sensor_status() const;
    void set_emergency_stop_state(const bool emergency_stop_active);
    double get_hatch_position() const;

   private:
    void determine_initial_state();
    void setup_command_callbacks();
    void set_hatch_action(const HatchTypes::Command command);
    void command_open();
    void command_close();
    void command_50();
    void command_50_open();
    void command_50_close();
    void command_stop();
    void command_reset_fault();

    bool check_hatch_ready_for_action(const HatchTypes::Command command) const;
    void perform_stepping_of_hatch_position(const double step_size);
    bool check_hatch_position(const double target_position);
    bool check_time_spent_performing_action() const;
    bool check_success_criterias(const double target_position);

    void calculate_hatch_step_size(const double time_step);

    static constexpr double M_HATCH_POSITION_SUCCESS_THRESHOLD{2.0};
    double m_hatch_step_size{};
    double m_hatch_timeout;
    // 0.0 is OPEN, 100.0 is CLOSED
    std::atomic<double> m_hatch_position;
    HatchTypes::HatchSensorStates m_hatch_sensor_previous_status{HatchTypes::HatchSensorStates::UNDEFINED};
    std::atomic<HatchTypes::HatchSensorStates> m_hatch_sensor_status{HatchTypes::HatchSensorStates::UNDEFINED};
    std::atomic<double> m_hatch_command_time_spent{0.0};
    std::atomic<HatchTypes::State> m_current_hatch_state{HatchTypes::State::STATE_UNDEFINED};
    std::atomic<HatchTypes::Action> m_current_hatch_action{HatchTypes::Action::ACTION_IDLE};
    std::atomic<HatchTypes::Fault> m_current_hatch_fault{HatchTypes::Fault::FAULT_NO_FAULT};
    std::atomic<HatchTypes::Command> m_current_hatch_command{HatchTypes::Command::COMMAND_CLOSE};
    std::function<void(HatchControl&)> m_hatch_execute_function;
    std::unordered_map<HatchTypes::Command, std::function<void(HatchControl&)>> m_execute_functions_map;
};
#endif