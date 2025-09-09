/*----------------------------------------------------------------------------*\
Class that standarize the handling of variables for interfacing external c++
models through the fmi-standard. The added functionality relative to the osp
cppfmu::SlaveInstance is a fixed way to represent the variables that are
accesed through the fmi set/get methods.

All fmi-variable types (real, integer, strings and boolean) are stored in
seperate unordered maps with integers as references / keys. The mapping of
index values to variable values and how to update the underlaying model must be
implemented in derived fmi-interface classes.

The intent of this class is to reduce the amount of boiler plate code for each
new c++ model, by not having to write fmi set/get methods each time.
\*----------------------------------------------------------------------------*/

#ifndef FMUBASECLASS_H
#define FMUBASECLASS_H

#include <unordered_map>

#include "cppfmu_cs.hpp"

class FMUBaseClass : public cppfmu::SlaveInstance
{
   public:
    FMUBaseClass(const std::string& instance_name);
    FMUBaseClass() = default;

    // Set get methods used by fmi
    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override;
    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override;

    void SetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIInteger value[]) override;
    void GetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIInteger value[]) const override;

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[]) override;
    void GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const override;

    void SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIBoolean value[]) override;
    void GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIBoolean value[]) const override;

   protected:
    const std::string m_model_name;

    // Maps for storing paramaters / input / ouput from fmi
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_real_signals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_integer_signals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_string_signals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_boolean_signals;
};

#endif  // end header guard