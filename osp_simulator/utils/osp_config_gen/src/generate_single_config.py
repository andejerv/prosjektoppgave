from dataclasses import dataclass
import numpy as np
from pathlib import Path
from typing import List
import xml.etree.ElementTree as ET
import yaml


@dataclass(frozen=True)
class DockConfig:
    p0_x: float
    p0_y: float
    p1_x: float
    p1_y: float
    depth: float


class GenerateSingleConfig:
    def __init__(self, input_config: dict, config_path: Path, xml: ET.Element, cfg: dict) -> None:
        self.__gen_input = input_config
        self.__config_path = config_path
        self.__osp_xml = xml
        self.__cfg = cfg

        with open(self.__config_path.joinpath("config/crossing_behavior.yaml"), "r") as ymlfile:
            self.__crossing_behavior_yaml = yaml.safe_load(ymlfile)
        
        with open(self.__config_path.joinpath("config/vessel_config.yaml"), "r") as ymlfile:
            self.__vessel_config_yaml = yaml.safe_load(ymlfile)

    def create_config(self) -> None:
        self.handle_parameters()
        self.update_ned_frame_origin()
        self.update_quay_coords()
        self.initial_pos_select()
        self.sim_type_select()
        self.set_fmu_path()
        self.hatch_position_select()
        self.wave_load_select()
        self.wind_load_select()
        
    def handle_parameters(self) -> None:
        try:
            init_pos = self.__gen_input['initial_position']
        except KeyError:
            self.__gen_input['initial_position'] = 'ravnkloa'
        else:
            if type(init_pos) == str:
                try:
                    if (init_pos != 'ravnkloa' and init_pos != 'fosenkaia'):
                        self.__cfg[self.__gen_input['vessel']]['initial_position'][init_pos]
                except KeyError:
                    raise Exception('The initial_position {} is not found in the config'.format(init_pos))
            elif type(init_pos) == list:
                if len(init_pos) != 3:
                    raise Exception('Manually specified initial_positions must be of length 3')
                for val in init_pos:
                    if type(val) != float:
                        raise Exception('All initial_position values must be floats')
            else:
                raise Exception('initial_position parameter must either be a string or a list of floats')

        try:
            sim_type = self.__gen_input['sim_type']
        except KeyError:
            self.__gen_input['sim_type'] = 'SIL'
        else:
            if type(sim_type) != str:
                raise Exception('sim_type parameter must be a string')
            if sim_type != 'SIL' and sim_type != 'HIL-NO-DP':
                raise Exception('{} is not a supported sim_type.'.format(sim_type))

        try:
            hatch_status = self.__gen_input['hatch_position']
        except KeyError:
            self.__gen_input['hatch_position'] = 'closed'
        else:
            if type(hatch_status) != str:
                raise Exception('hatch_status parameter must be a string')
            if hatch_status != 'open' and \
               hatch_status != 'closed' and \
               hatch_status != 'fore_open' and \
               hatch_status != 'aft_open' and \
               hatch_status != 'both_open':
                raise Exception('{} is not a supported hatch_status.'.format(hatch_status))

        try:
            invert_transit = self.__gen_input['invert_transit']
        except KeyError:
            self.__gen_input['invert_transit'] = False
        else:
            if type(invert_transit) != bool:
                raise Exception('invert_transit parameter must be a boolean')
        # For some reason dock positions on ravnkloa side are defined with inverted heading, this is to correct for that
        if self.__gen_input['initial_position'] == 'ravnkloa':
            self.__gen_input['invert_transit'] = not self.__gen_input['invert_transit']

        try:
            wave_loads = self.__gen_input['wave_loads']
        except KeyError:
            self.__gen_input['wave_loads'] = 'none'
        else:
            if type(wave_loads) == str:
                if wave_loads != 'none' and \
                   wave_loads != 'low' and \
                   wave_loads != 'medium' and \
                   wave_loads != 'high':
                    raise Exception('{} is not a supported wave_loads preset.'.format(wave_loads))
            elif type(wave_loads) == list:
                if len(wave_loads) != 6:
                    raise Exception('Wave loads must be defined with 6 frequency amplitudes')
                for val in wave_loads:
                    if type(val) != float:
                        raise Exception('All wave_loads amplitude values must be floats')
            else:
                raise Exception('wave_loads parameter must be either a string or list of floats')
                

        try:
            wind_speed = self.__gen_input['wind_speed']
        except KeyError:
            self.__gen_input['wind_speed'] = 0.0
        else:
            if type(wind_speed) != float:
                raise Exception('wind_speed parameter must be a float')

        try:
            wind_direction = self.__gen_input['wind_direction']
        except KeyError:
            self.__gen_input['wind_direction'] = 0.0
        else:
            if type(wind_direction) != float:
                raise Exception('wind_direction parameter must be a float')
            if wind_direction < 0.0 or wind_direction > 360.0:
                raise Exception('wind_direction must be between 0 and 360 degrees')
            
        try:
            wind_direction_noise = self.__gen_input['wind_direction_noise']
        except KeyError:
            self.__gen_input['wind_direction_noise'] = 0.0
        else:
            if type(wind_direction_noise) != float:
                raise Exception('wind_direction_noise parameter must be a float')
            if wind_direction_noise < 0.0:
                raise Exception('wind_direction_noise cannot be negative')

    def update_ned_frame_origin(self) -> None:
        latitude = self.__vessel_config_yaml['/**']['ros__parameters']['frames']['latitude_0']
        longitude = self.__vessel_config_yaml['/**']['ros__parameters']['frames']['longitude_0']
        for module in self.__osp_xml:
            match module.items()[0][1]:
                case 'advanced_navigation' | \
                     'advanced_navigation_gps_compass' | \
                     'dp_interface':
                    for var in module[0]:
                        if var.attrib['variable'] == 'latitude0':
                            var[0].attrib['value'] = str(latitude)
                        elif var.attrib['variable'] == 'longitude0':
                            var[0].attrib['value'] = str(longitude)

    def initial_pos_select(self) -> None:
        initial_position = self.__gen_input['initial_position']
        if type(initial_position) == str:
            if (initial_position == 'ravnkloa') or (initial_position == 'fosenkaia'):
                init_pos = self.__crossing_behavior_yaml['coordinates'][initial_position]['dock']
            else:
                # Custom initial positions defined in config.yml
                init_pos = self.__cfg[self.__gen_input['vessel']]['initial_position'][initial_position]
        else:
            init_pos = initial_position

        invert_transit = int(self.__gen_input['invert_transit']) * 3.141592

        for module in self.__osp_xml:
            match module.items()[0][1]:
                case 'Zeabuz 1' | \
                     'milliAmpere 2' | \
                     'Vessel':
                    module[0][0][0].attrib['value'] = str(init_pos[0]) # North
                    module[0][1][0].attrib['value'] = str(init_pos[1]) # East
                    module[0][5][0].attrib['value'] = str(init_pos[2] + invert_transit) # Yaw
                    break
                # No default case, as no action is required on remaining modules
                    
    def sim_type_select(self) -> None:
        for module in self.__osp_xml:
            match module.items()[0][1]:
                case 'hatch_control':
                    if self.__gen_input['vessel'] == 'ma2':
                        for var in module[0]:
                            if var.attrib['variable'] == 'remote_ip':
                                var[0].attrib['value'] = self.__cfg['sim_type'][self.__gen_input['sim_type']]['autonomy_ip']
                                break
                    elif self.__gen_input['vessel'] == 'estelle':
                        for var in module[0]:
                            if var.attrib['variable'] == 'modbus_server_ip':
                                var[0].attrib['value'] = self.__cfg['sim_type'][self.__gen_input['sim_type']]['osp_ip']
                                break
                case 'zb_sim_nav' | \
                     'zb_sim_clock' | \
                     'ultrasound_distance_communication' | \
                     'ultrasound_distance_communication_zb1'| \
                     'dp_interface':
                    for var in module[0]:
                        if var.attrib['variable'] == 'remote_ip':
                            var[0].attrib['value'] = self.__cfg['sim_type'][self.__gen_input['sim_type']]['autonomy_ip']
                            break
                case 'advanced_navigation':
                    for var in module[0]:
                        if var.attrib['variable'] == 'autonomy_machine_remote_ip':
                            var[0].attrib['value'] = self.__cfg['sim_type'][self.__gen_input['sim_type']]['autonomy_ip']
                        elif var.attrib['variable'] == 'mt_dp_remote_ip':
                            var[0].attrib['value'] = self.__cfg['sim_type'][self.__gen_input['sim_type']]['mt_dp_ip']
                case 'advanced_navigation_gps_compass':
                    for var in module[0]:
                        if var.attrib['variable'] == 'autonomy_machine_remote_ip':
                            var[0].attrib['value'] = self.__cfg['sim_type'][self.__gen_input['sim_type']]['autonomy_ip']
                            break
                # No default case, as no action is required on remaining modules


    def set_fmu_path(self) -> None:
        default_path = "../../fmus/"
        for module in self.__osp_xml:
            fmu_path = module.attrib['source']
            if default_path in fmu_path:
                module.attrib['source'] = fmu_path.replace(default_path, self.__cfg['fmu_path'])


    def hatch_position_select(self) -> None:
        # Default both hatches to closed (100)
        fore_status = 100.0
        aft_status = 100.0

        hatch_status = self.__gen_input['hatch_position']
        match hatch_status:
            case 'fore_open':
                fore_status = 0.0
            case 'aft_open':
                aft_status = 0.0
            case 'both_open':
                fore_status = 0.0
                aft_status = 0.0
            case 'open':
                init_pos = self.__gen_input['initial_position']
                inverted = self.__gen_input['invert_transit']
                if (init_pos == 'ravnkloa' and not inverted) or \
                   (init_pos == 'fosenkaia' and inverted):
                    fore_status = 0.0
                elif (init_pos == 'ravnkloa' and inverted) or \
                     (init_pos == 'fosenkaia' and not inverted):
                    aft_status = 0.0
                else:
                    raise Exception('Hatch position cannot be inferred if vessel is not docked')
            case 'closed':
                pass

        for module in self.__osp_xml:
            match module.items()[0][1]:
                case 'hatch_control':
                    for var in module[0]:
                        if var.attrib['variable'] == 'hatch_start_position_fore':
                            var[0].attrib['value'] = str(fore_status)
                        elif var.attrib['variable'] == 'hatch_start_position_aft':
                            var[0].attrib['value'] = str(aft_status)
                case 'mooring':
                    for var in module[0]:
                        if var.attrib['variable'] == 'hatch_position_fore':
                            var[0].attrib['value'] = str(fore_status)
                        elif var.attrib['variable'] == 'hatch_position_aft':
                            var[0].attrib['value'] = str(aft_status)

    def wave_load_select(self) -> None:
        wave_loads = self.__gen_input['wave_loads']
        if type(wave_loads) == list:
            wave_amplitude = wave_loads
        elif type(wave_loads) == str:
            wave_amplitude = self.__cfg['wave_loads'][wave_loads]
        else: 
            raise Exception('Wave loads must either be a preset or list of 6 frequency amplitudes')
        
        for module in self.__osp_xml:
            match module.items()[0][1]:
                case 'Wave forces':
                    for i in range(6):
                        module[0][i][0].attrib['value'] = str(wave_amplitude[i])
                    break
                # No default case, as no action is required on remaining modules

    def wind_load_select(self) -> None:
        wind_direction = self.__gen_input['wind_direction']
        wind_speed = self.__gen_input['wind_speed']
        wind_direction_noise = self.__gen_input['wind_direction_noise']
        for module in self.__osp_xml:
            match module.items()[0][1]:
                case 'Wind model':
                    for var in module[0]:
                        if var.attrib['variable'] == 'mean_wind_direction':
                            var[0].attrib['value'] = str(wind_direction)
                        if var.attrib['variable'] == 'mean_wind_speed':
                            var[0].attrib['value'] = str(wind_speed)
                        if var.attrib['variable'] == 'wind_direction_noise_power':
                            var[0].attrib['value'] = str(wind_direction_noise)

    def update_quay_coords(self) -> None:
        docks = self.calculate_quay_coords()

        for module in self.__osp_xml:
            fmu_name = module.items()[0][1]
            if (fmu_name == 'Zeabuz 1') | \
               (fmu_name == 'milliAmpere 2') | \
               (fmu_name == 'Vessel') | \
               (fmu_name.startswith('Distance Sensor ')):
                for i, var in enumerate(module[0]):
                    # Loop through variables until we reach quay config
                    if var.attrib['variable'] == 'quay_1_en':
                        for j, dock in enumerate(docks):
                            assert module[0][6*j + i].attrib['variable'] == f'quay_{j+1}_en'
                            module[0][6*j + i + 1][0].attrib['value'] = str(dock.p0_x)
                            module[0][6*j + i + 2][0].attrib['value'] = str(dock.p0_y)
                            module[0][6*j + i + 3][0].attrib['value'] = str(dock.p1_x)
                            module[0][6*j + i + 4][0].attrib['value'] = str(dock.p1_y)
                            module[0][6*j + i + 5][0].attrib['value'] = str(dock.depth)
    
    def calculate_quay_coords(self) -> List[DockConfig]:
        north_dock_pos = self.__crossing_behavior_yaml['coordinates']['fosenkaia']['dock']
        south_dock_pos = self.__crossing_behavior_yaml['coordinates']['ravnkloa']['dock']

        vessel_l = self.__crossing_behavior_yaml['vessel_params']['length']
        vessel_b = self.__crossing_behavior_yaml['vessel_params']['beam']

        north_dock_cfg = self.__cfg[self.__gen_input['vessel']]['fosenkaia']
        south_dock_cfg = self.__cfg[self.__gen_input['vessel']]['ravnkloa']

        try:
            if north_dock_cfg['outrigger']:
                has_outrigger = True
        except KeyError:
            has_outrigger = False

        docks = []
        for dock_cfg, dock_pos in zip([north_dock_cfg, south_dock_cfg], [north_dock_pos, south_dock_pos]):
            docks.extend(calculate_single_quay_coord(dock_cfg=dock_cfg,
                                                     dock_pos=dock_pos,
                                                     vessel_l=vessel_l,
                                                     vessel_b=vessel_b,
                                                     has_outrigger=has_outrigger))
        return docks
    
    
def calculate_single_quay_coord(dock_cfg: dict, \
                                dock_pos: List[float], \
                                vessel_l: float, \
                                vessel_b: float, \
                                has_outrigger: bool) -> List[DockConfig]:
    docks = []

    # Main quay polygons
    p0_main = np.array([0,0])
    p1_main = np.array([dock_cfg['width'], 0])

    if has_outrigger:
        outrigger_cfg = dock_cfg['outrigger']
        edge_offset = outrigger_cfg['depth'] + outrigger_cfg['offset']
        if outrigger_cfg['docking_side'] == 'left':
            if outrigger_cfg['alignment'] == 'left':
                x_val = vessel_b + edge_offset
            elif outrigger_cfg['alignment'] == 'right':
                x_val = dock_cfg['width'] - edge_offset
        elif outrigger_cfg['docking_side'] == 'right':
            if outrigger_cfg['alignment'] == 'left':
                x_val = edge_offset
            elif outrigger_cfg['alignment'] == 'right':
                x_val = dock_cfg['width'] - vessel_b - edge_offset

        p0_outrigger = np.array([x_val, outrigger_cfg['length']])
        p1_outrigger = np.array([x_val, 0])

        p_vessel_center = p1_outrigger + np.array([vessel_b/2, vessel_l/2])
    
    else:
        p_vessel_center = p0_main + np.array([vessel_b/2, vessel_l/2])

    # Rotate dock angle into local quay frames
    # South dock heading is given 180 degrees rotated
    ang_quay = dock_pos[2] - np.pi/2
    # Rotation matrices for transforming between NED and local quay frames
    R = np.array([(np.cos(ang_quay), -np.sin(ang_quay)), (np.sin(ang_quay), np.cos(ang_quay))])

    # Position and orientation of local quay frames in NED frame
    p_origin_quay = np.array([dock_pos[0], dock_pos[1]]) - R @ p_vessel_center

    # Transform
    p0_main_ned = p_origin_quay + R @ p0_main
    p1_main_ned = p_origin_quay + R @ p1_main
    docks.append(DockConfig(p0_x=p0_main_ned[0],
                            p0_y=p0_main_ned[1],
                            p1_x=p1_main_ned[0],
                            p1_y=p1_main_ned[1],
                            depth=dock_cfg['depth'])) 

    if has_outrigger:
        p0_outrigger_ned = p_origin_quay + R @ p0_outrigger
        p1_outrigger_ned = p_origin_quay + R @ p1_outrigger
        docks.append(DockConfig(p0_x=p0_outrigger_ned[0],
                                p0_y=p0_outrigger_ned[1],
                                p1_x=p1_outrigger_ned[0],
                                p1_y=p1_outrigger_ned[1],
                                depth=outrigger_cfg['depth']))
    return docks
