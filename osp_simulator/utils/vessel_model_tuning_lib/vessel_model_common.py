import numpy as np

knots_to_ms = 0.51444444444444
ms_to_knots = 1.0 / knots_to_ms

class physical_constants:
    def __init__(self):
        self.g = 9.8066 #Acceleration of gravity
        self.rho = 1025.87 #Water density
        self.nu = 1.19e-6 #Water kinematic viscosity


class OSPParameterGen:
    '''
    Class for generating XML formatted parameter files for vessel model FMUs that can be directly inserted into OSP configs
    '''

    def __init__(self, ship_dim, physical_constants, maneuvering_model = None, seakeeping_model = None, resistance_model = None, propeller_model = None):
        self.ship_dim = ship_dim
        self.physical_constants = physical_constants
        self.maneuvering_model = maneuvering_model
        self.seakeeping_model = seakeeping_model
        self.resistance_model = resistance_model
        self.propeller_model = propeller_model

    
    def print_params(self, params, rounding=6):
        ''' Print contents of parameter dictionary'''

        for k, v in params.items():
            if not(v is None):
                print(k + '\t', np.round(v, rounding))
            else:
                print(k + '\t', 'None')

    def write_to_xml(self, params, file_path, indent_spaces = 16):
        ''' 
        Write the value of paramaters to a format that can copied directly into OSP 
        xml files. Parameters must be given as a dict where the keys are the osp variable names
        '''

        f = open(file_path, 'w')

        for k, v in params.items():
            if not(v is None):
                f.write(indent_spaces * ' ' 
                + '<InitialValue variable="{0}"><Real value="{1}"/></InitialValue>\n'.format(k, np.round(v, 5)))

        f.close()

    def generate_hull_maneuvering_params(self):
        '''Generate parameters for `hull_maneuvering` FMU'''
        params = dict(self.maneuvering_model.mmg_coeffs)
        
        params["shape_factor"] = self.resistance_model.k
        params["CR_m"] = self.resistance_model.CR_m
        params["CR_p"] = self.resistance_model.CR_p
        params["CD_lateral"] = self.resistance_model.CD_lateral

        params["length"] = self.ship_dim.L
        params["depth"] = self.ship_dim.T
        params["wetted_surface"] = self.ship_dim.S
        
        #TODO Add low speed model

        return params
    
    def generate_hull_seakeeping_params(self):
        '''Generate parameters for `hull_seakeeping` FMU'''
        params = {}
        
        params["mass"] = self.seakeeping_model.inertias.mass
        params["waterplane_area"] = self.seakeeping_model.hydrostatics.waterplane_area
        params["GM_roll"] = self.seakeeping_model.hydrostatics.GM_roll
        params["GM_pitch"] = self.seakeeping_model.hydrostatics.GM_pitch
        
        params["heave_damping"] = self.seakeeping_model.d_heave
        params["roll_damping"] = self.seakeeping_model.d_roll
        params["pitch_damping"] = self.seakeeping_model.d_pitch

        return params
    
    def generate_rigid_body_params(self):
        '''Generate parameters for `rigid_body` FMU'''
        params = {}
        
        params["m"] = self.seakeeping_model.inertias.mass

        params["m_added[1]"] = self.seakeeping_model.inertias.added_mass_surge
        params["m_added[2]"] = self.seakeeping_model.inertias.added_mass_sway
        params["m_added[3]"] = self.seakeeping_model.inertias.added_mass_heave

        params['I[1]'] = self.seakeeping_model.inertias.I_roll
        params['I[2]'] = self.seakeeping_model.inertias.I_pitch
        params['I[3]'] = self.seakeeping_model.inertias.I_pitch

        params['I_added[1]'] = self.seakeeping_model.inertias.added_I_roll
        params['I_added[2]'] = self.seakeeping_model.inertias.added_I_pitch
        params['I_added[3]'] = self.seakeeping_model.inertias.added_I_yaw
        
        return params
