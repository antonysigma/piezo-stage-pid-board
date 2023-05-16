import os
import numpy as np
import matplotlib.pyplot as plt   # MATLAB plotting functions
from control import (feedback, tf, series, step_response, root_locus, zpk, step_info, rootlocus_pid_designer)

post_filter = tf([952206], [1, 1916.6, 952206])

natural_freq = 250.0 * 2 * np.pi
piezo_actuator = zpk([], [-natural_freq * 1j, natural_freq * 1j], gain=1.0)
controller = zpk([-800, -550, -500, -400], [0.0], gain=1.0)

# Compute the open-loop transfer function
sys = series(controller, post_filter, piezo_actuator)

# Root lcous plot for the system
plt.figure(1)
root_locus(sys, xlim=[-1200, 200])
plt.title('Root locus plot')
plt.show(block=False)

# Compute the close-loop transfer function
sys2 = feedback(sys * 8e-3)
#sys2 = rootlocus_pid_designer(sys, Kp0=1.0)

# Step response for the system
plt.figure(2)
time, yout = step_response(sys2, T=100e-3)

info = step_info(sys2)
print(info)
overshoot = info['Peak'] - 1.0

plt.plot(time * 1e3, yout)

plt.title(f'''Step response
Rise = {info["RiseTime"]:1.2g} s; Settle = {info["SettlingTime"]:1.2g} s; Overshoot = {overshoot * 100:0.2f} %''')
plt.xlabel('Time / ms')
plt.show()
