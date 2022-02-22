import sys
sys.path.append('/home/thomas/git/pybound_mathlib/build/')
from pybindings import SecondOrderReferenceModel
import numpy as np
import plotly.graph_objects as go


# simulate a random walk
num_steps = 1000
acc_step_set = [-2., 0., 2.]
acc_steps = np.random.choice(a = acc_step_set, size = num_steps)

time_step = 0.01  # [s]
times = np.array([ind * time_step for ind in range(num_steps)])
pos = np.zeros(num_steps)
vel = np.zeros(num_steps)

# simulate
for ind in range(1, num_steps):
    vel[ind] = vel[ind - 1] + time_step * acc_steps[ind - 1]
    pos[ind] = pos[ind-1] + time_step * vel[ind-1]

so_filter = SecondOrderReferenceModel()
natural_freq = 3.
damping_ratio = 0.7071
so_filter.setParameters(natural_freq, damping_ratio)

# filter without vel feedback
so_filter.reset(pos[0], vel[0])
pos_filt = np.zeros(num_steps)
vel_filt = np.zeros(num_steps)
pos_filt[0] = so_filter.getState()
vel_filt[0] = so_filter.getRate()
for ind in range(1, num_steps):
    so_filter.update(time_step, pos[ind], vel[ind])
    pos_filt[ind] = so_filter.getState()
    vel_filt[ind] = so_filter.getRate()

# filter with vel feedback
so_filter.reset(pos[0], vel[0])
pos_filt_fb = np.zeros(num_steps)
vel_filt_fb = np.zeros(num_steps)
pos_filt_fb[0] = so_filter.getState()
vel_filt_fb[0] = so_filter.getRate()
for ind in range(1, num_steps):
    so_filter.update(time_step, pos[ind])
    pos_filt_fb[ind] = so_filter.getState()
    vel_filt_fb[ind] = so_filter.getRate()

#
# plots
#

# acceleration steps
fig_acc = go.Figure()
fig_acc.add_trace(
    go.Scatter(
        x = times,
        y = acc_steps,
        name = 'Acceleration Steps'
    )
)
fig_acc['layout']['xaxis']['title'] = 'Time [s]'
fig_acc['layout']['yaxis']['title'] = 'Acceleration [m/s^2]'

# velocities
fig_speed = go.Figure()
fig_speed.add_trace(
    go.Scatter(
        x = times,
        y = vel,
        name = 'Raw'
    )
)
fig_speed.add_trace(
    go.Scatter(
        x = times,
        y = vel_filt,
        name = 'Filtered'
    )
)
fig_speed.add_trace(
    go.Scatter(
        x = times,
        y = vel_filt_fb,
        name = 'Filtered w/ speed feedback'
    )
)
fig_speed['layout']['xaxis']['title'] = 'Time [s]'
fig_speed['layout']['yaxis']['title'] = 'Speed [m/s]'

# positions
fig_pos = go.Figure()
fig_pos.add_trace(
    go.Scatter(
        x = times,
        y = pos,
        name = 'Raw'
    )
)
fig_pos.add_trace(
    go.Scatter(
        x = times,
        y = pos_filt,
        name = 'Filtered'
    )
)
fig_pos.add_trace(
    go.Scatter(
        x = times,
        y = pos_filt_fb,
        name = 'Filtered w/ speed feedback'
    )
)
fig_pos['layout']['xaxis']['title'] = 'Time [s]'
fig_pos['layout']['yaxis']['title'] = 'Position [m]'

fig_acc.show()
fig_speed.show()
fig_pos.show()



