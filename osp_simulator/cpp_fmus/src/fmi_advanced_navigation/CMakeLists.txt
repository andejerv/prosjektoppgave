# Create FMU target
add_fmu(fmi_advanced_navigation
    "fmi_advanced_navigation.cpp"
    "adnav_lib/an_packet_protocol.h"
    "adnav_lib/an_packet_protocol.c"
    "adnav_lib/anpp_encode_decode.h"
    "adnav_lib/anpp_encode_decode.c"
)
