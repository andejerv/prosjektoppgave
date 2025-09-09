#include "physics_fmu_base_class.hpp"

#include <sstream>
#include <stdexcept>  // std::out_of_range

FMUBaseClass::FMUBaseClass(const std::string& instance_name) : m_model_name(instance_name)
{
}

void FMUBaseClass::SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        try {
            m_real_signals.at(vr[i]) = value[i];
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in SetReal. Invalid real_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}

void FMUBaseClass::GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const
{
    for (size_t i = 0; i < nvr; ++i) {
        try {
            value[i] = m_real_signals.at(vr[i]);
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in GetReal. Invalid real_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}

void FMUBaseClass::SetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIInteger value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        try {
            m_integer_signals.at(vr[i]) = value[i];
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in SetInteger. Invalid integer_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}

void FMUBaseClass::GetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIInteger value[]) const
{
    for (size_t i = 0; i < nvr; ++i) {
        try {
            value[i] = m_integer_signals.at(vr[i]);
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in GetInteger. Invalid integer_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}

void FMUBaseClass::SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIBoolean value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        try {
            m_boolean_signals.at(vr[i]) = value[i];
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in SetBoolean. Invalid boolean_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}

void FMUBaseClass::GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIBoolean value[]) const
{
    for (size_t i = 0; i < nvr; ++i) {
        try {
            value[i] = m_boolean_signals.at(vr[i]);
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in GetBoolean. Invalid boolean_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}

void FMUBaseClass::SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        try {
            m_string_signals.at(vr[i]) = std::string(value[i]);
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in SetString. Invalid string_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}

void FMUBaseClass::GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const
{
    for (size_t i = 0; i < nvr; ++i) {
        try {
            value[i] = m_string_signals.at(vr[i]).c_str();
        }
        catch (const std::out_of_range& oor) {
            std::stringstream error_message;
            error_message << "Error in GetString. Invalid string_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
    }
}
