import json
import pytest
import numpy as np
import xml.etree.ElementTree as ET
import yaml

from src.generate_single_config import GenerateSingleConfig, calculate_single_quay_coord


@pytest.fixture
def gen_config_constructor():
    with open('config.yml', "r") as ymlfile:
        cfg = yaml.safe_load(ymlfile)

    with open('tests/resources/test_generation_plan.json', "r") as jsonfile:
        gen_plan = json.load(jsonfile)
    
    config_path = 'stockholm_config_test_system_dependencies'

    estelle_tree = ET.parse('presets/estelle_preset.xml')
    estelle_root = estelle_tree.getroot()
    estelle_sim = estelle_root[1]
    
    generator = GenerateSingleConfig(gen_plan['configs'][0], config_path, estelle_sim, cfg)
    return generator


def test_calculate_quay_coordinates():
    dock_cfg = {'width': 10,
                'depth': 5,
                'outrigger':
                    {'docking_side': "right",
                        'alignment': "left",
                        'length': 10,
                        'depth': 1,
                        'offset': 1
                    }
               }
    
    dock_pos = [20, 20, np.pi/2]
    vessel_l = 10
    vessel_b = 5
    has_outrigger = True
    docks = calculate_single_quay_coord(dock_cfg=dock_cfg,
                                        dock_pos=dock_pos,
                                        vessel_l=vessel_l,
                                        vessel_b=vessel_b,
                                        has_outrigger=has_outrigger)
    assert docks[0].p0_x == 15.5
    assert docks[0].p0_y == 15.0
    assert docks[0].p1_x == 25.5
    assert docks[0].p1_y == 15.0
    assert docks[0].depth == dock_cfg['depth']
    assert docks[1].p0_x == 17.5
    assert docks[1].p0_y == 25.0
    assert docks[1].p1_x == 17.5
    assert docks[1].p1_y == 15.0
    assert docks[1].depth == dock_cfg['outrigger']['depth']