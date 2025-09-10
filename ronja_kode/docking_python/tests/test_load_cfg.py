from hydra import initialize, compose
from omegaconf import DictConfig, OmegaConf

def load_config(cfg_name):
    cfg_path = "../docking_algorithms/config"
    with initialize(config_path=cfg_path, version_base=None):  # Base directory for YAML files
        config = compose(config_name=cfg_name)  # Load the base configuration file
        return config
    
def test_load_pid_cfg():
    return load_config(cfg_name="pid")
    
def test_load_scenarios_cfg():
    return load_config(cfg_name="scenarios")
    

if __name__ == "__main__":
    pid_cfg = test_load_pid_cfg()
    print("pid config: ")
    print(OmegaConf.to_yaml(pid_cfg["scenario1"]))

    scenarios_cfg = test_load_scenarios_cfg()
    print("scenarios config: ")
    print(OmegaConf.to_yaml(scenarios_cfg["scenario1"]))