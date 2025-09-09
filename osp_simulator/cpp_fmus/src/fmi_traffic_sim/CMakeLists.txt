
add_fmu(fmi_zb_traffic_sim "fmi_traffic_sim.cpp")

get_target_property(target_options fmi_zb_traffic_sim COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-Werror")
set_property(TARGET fmi_zb_traffic_sim PROPERTY COMPILE_OPTIONS ${target_options})
