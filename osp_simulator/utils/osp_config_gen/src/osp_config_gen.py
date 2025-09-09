import argparse
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from src.generate_single_config import GenerateSingleConfig

class OspConfigGen:
    def __init__(self, input_path: str, config_path: str, output_path: str, autonomy_config_path: str) -> None:
        """Constructor method.
        """
        abs_path = Path(os.path.dirname(os.path.realpath(__file__)))
        if abs_path.name == 'src':
            abs_path = abs_path.parent

        if config_path == '':
            config_path = abs_path.joinpath('config.yml')
        with open(config_path, "r") as ymlfile:
            cfg = yaml.safe_load(ymlfile)
        self.__cfg = cfg
        
        self.__resources_path = abs_path.joinpath('resources')
        ET.register_namespace('', 'http://opensimulationplatform.com/MSMI/OSPSystemStructure')
        ma2_tree = ET.parse(self.__resources_path.joinpath('presets/ma2_preset.xml'))
        ma2_root = ma2_tree.getroot()
        self.__ma2_xml_tree = ma2_tree
        self.__ma2_sim = ma2_root[1] # Selects 'Simulators' subtree

        estelle_tree = ET.parse(self.__resources_path.joinpath('presets/estelle_preset.xml'))
        estelle_root = estelle_tree.getroot()
        self.__estelle_xml_tree = estelle_tree
        self.__estelle_sim = estelle_root[1] # Selects 'Simulators' subtree

        froja_tree = ET.parse(self.__resources_path.joinpath('presets/froja_preset.xml'))
        froja_root = froja_tree.getroot()
        self.__froja_xml_tree = froja_tree
        self.__froja_sim = froja_root[1] # Selects 'Simulators' subtree

        with open(input_path, "r") as jsonfile:
            input = json.load(jsonfile)
        self.__gen_input = input

        self.__output_path = output_path

        if autonomy_config_path == '':
            self.__autonomy_config_path = self.__resources_path
        else:
            self.__autonomy_config_path = Path(autonomy_config_path)


    def generate_config(self) -> int:
        for config in self.__gen_input['configs']:
            try:
                match config['vessel']:
                    case 'ma2':
                        xml = self.__ma2_sim
                        xml_tree = self.__ma2_xml_tree
                        config_path = self.__autonomy_config_path.joinpath('ma2_config_test_system_dependencies')
                    case 'estelle':
                        xml = self.__estelle_sim
                        xml_tree = self.__estelle_xml_tree
                        config_path = self.__autonomy_config_path.joinpath('stockholm_config_test_system_dependencies')
                    case 'froja':
                        xml = self.__froja_sim
                        xml_tree = self.__froja_xml_tree
                        config_path = self.__autonomy_config_path.joinpath('isoleden_config_test_system_dependencies')
                    case other:
                        raise Exception('Vessel \"{}\" not recognized. \'estelle\', \'ma2\' and \'froja\'  are supported'.format(other))
            except KeyError:
                raise Exception('All configs must have specified the vessel parameter, either \'estelle\' or \'ma2\'')
            generator = GenerateSingleConfig(config, config_path, xml, self.__cfg)
            generator.create_config()

            try:
                if type(config['config_name']) != str:
                    raise Exception('config_name must be a string')
            except KeyError:
                raise Exception('config_name must be specified in the generation_plan')
            else:
                folder_path = Path.joinpath(Path(self.__output_path), config['config_name'])

            os.makedirs(folder_path, exist_ok=True)
            xml_tree.write(Path.joinpath(Path(folder_path), 'OspSystemStructure.xml'))

if __name__ == '__main__':
    """Generates OSP config files
    """
    parser = argparse.ArgumentParser(description='OSP Config Generator')
    parser.add_argument('input_path', help='Path to generation file', const=1, nargs='?', type=str)
    parser.add_argument('-c', '--config', dest='config', help='Path to config file', default='', const=1, nargs='?', type=str)
    parser.add_argument('-o', '--output_path', dest='output_directory', help='Path to output directory', default='osp_configs', const=1, nargs='?', type=str)
    parser.add_argument('-ac', '--autonomy_config_path', dest='autonomy_config_path', help='Path to folder of config repo artifacts', default='', const=1, nargs='?', type=str)
    args = parser.parse_args()

    if not args.input_path:
        raise Exception('An input file must be specified as a positional argument')
    input_path = args.input_path
    config_path = args.config
    output_path = args.output_directory
    autonomy_config_path = args.autonomy_config_path

    runner = OspConfigGen(input_path=input_path,
                          config_path=config_path,
                          output_path=output_path,
                          autonomy_config_path=autonomy_config_path)
    runner.generate_config()
