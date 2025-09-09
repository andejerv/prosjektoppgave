# Create FMU target
add_fmu(fmi_dp_mock
    "fmi_dp_mock.hpp"
    "fmi_dp_mock.cpp"
    "dynamic_positioning.hpp"
    "dynamic_positioning.cpp"
    "data_types.hpp"
    "models/models.hpp"
    "models/models.cpp"
)
