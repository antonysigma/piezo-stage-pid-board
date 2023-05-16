import os
import numpy as np
import matplotlib.pyplot as plt   # MATLAB plotting functions
from control import (feedback, tf, series, step_response, root_locus, zpk, step_info, rootlocus_pid_designer, pzmap)

post_filter = tf([952206], [1, 1916.6, 952206])

natural_freq = 250.0 * 2 * np.pi
#damping = 0.01
piezo_actuator = zpk([], [-10 + natural_freq*1j, -10 - natural_freq * 1j], gain=1.0)
print(f'Plant =\n{piezo_actuator}')

pole_branch = -450

controller = zpk([-800, -200], [0.0, pole_branch-50, pole_branch+50], gain=1.0)
print(f'Controller=\n{controller}')

# Compute the open-loop transfer function
#sys = series(controller, post_filter, piezo_actuator).sample(5e-3, 'bilinear')
sys = series(controller, post_filter, piezo_actuator)
print(f'Open loop transfer function=\n{sys}')


# Root lcous plot for the system
plt.figure(1)
root_locus(sys, xlim=[-1200, 20], ylim=[-2000, 2000])
plt.title('Root locus plot')
plt.show(block=False)

# Compute the close-loop transfer function
gain = 6e8
sys2 = feedback(sys * gain)
print(f'Close loop transfer function=\n{sys2}')

discretized_controller = (controller * gain).sample(5e-3, 'bilinear')
print(f'Discretized controller=\n{discretized_controller}')

# Step response for the system
plt.figure(2)
time, yout = step_response(sys2, T=100e-3)

info = step_info(sys2)
#print(info)
overshoot = info['Peak'] - 1.0

plt.plot(time * 1e3, yout)

plt.title(f'''Step response
Rise = {info["RiseTime"]:1.2g} s; Settle = {info["SettlingTime"]:1.2g} s; Overshoot = {overshoot * 100:0.2f} %''')
plt.xlabel('Time / ms')
plt.show()
