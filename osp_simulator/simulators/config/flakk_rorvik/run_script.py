import subprocess as sp
import shutil
import os

import time

end_time = 23 * 60

cosim_path = "C:/Users/JarleKramer/Software/cosim-v0.6.0-win64/bin/cosim.exe" # MUST BE UPDATED!
cosim_command = "run"

sim_path = 'flakk_rorvik.xml' # MUST BE UPDATED!
result_folder_path = 'C:/Users/JarleKramer/osp-results/flakk_rorvik'   # MUST BE UPDATED!

if os.path.isdir(result_folder_path):
    shutil.rmtree(result_folder_path)

cosim_options = {
    '--system_structure_path':sim_path,
    '-b':'0.0',
    '-e':'{:.6f}'.format(end_time),
    '--output-dir':result_folder_path,
}

execute_string = cosim_path + ' ' + cosim_command

for k, v in cosim_options.items():
    execute_string += ' ' + k + ' ' + v

t_start = time.time()
sp.call(execute_string, shell=True)
t_stop = time.time()

print('Simulation time:', t_stop - t_start)

