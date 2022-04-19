# get the relative import path
import sys

sys.path.append('../build')  # assumes the readme was followed..

# other imports
import numpy as np
import plotly.graph_objects as go

# import the pybind module
from mathlib import SecondOrderReferenceModel, DiscretizationMethod


# simulate a random walk
num_steps = 1000
acc_step_set = [-3., 0., 3.]
acc_steps = np.random.choice(a=acc_step_set, size=num_steps)

time_step = 0.01  # [s]
times = np.array([ind * time_step for ind in range(num_steps)])
pos = np.zeros(num_steps)
vel = np.zeros(num_steps)

# simulate
for ind in range(1, num_steps):
    vel[ind] = vel[ind - 1] + time_step * acc_steps[ind - 1]
    pos[ind] = pos[ind - 1] + time_step * vel[ind - 1]

# setup the filters
natural_freq = 3.  # [rad/s]
damping_ratio = 0.7071

# filter without vel feedback
position_speed_filter = SecondOrderReferenceModel()
position_speed_filter.setParameters(natural_freq, damping_ratio)
position_speed_filter.setDiscretizationMethod(DiscretizationMethod.kForwardEuler)
position_speed_filter.reset(pos[0], vel[0])
pos_filt = np.zeros(num_steps)
vel_filt = np.zeros(num_steps)
pos_filt[0] = position_speed_filter.getState()
vel_filt[0] = position_speed_filter.getRate()

# filter with vel feedback
position_speed_filter_fb = SecondOrderReferenceModel()
position_speed_filter_fb.setParameters(natural_freq, damping_ratio)
position_speed_filter_fb.setDiscretizationMethod(DiscretizationMethod.kForwardEuler)
position_speed_filter_fb.reset(pos[0], vel[0])
pos_filt_fb = np.zeros(num_steps)
vel_filt_fb = np.zeros(num_steps)
pos_filt_fb[0] = position_speed_filter_fb.getState()
vel_filt_fb[0] = position_speed_filter_fb.getRate()

# run the filters
for ind in range(1, num_steps):
    # no feedback
    position_speed_filter.update(time_step, pos[ind])
    pos_filt[ind] = position_speed_filter.getState()
    vel_filt[ind] = position_speed_filter.getRate()
    # with feedback
    position_speed_filter_fb.update(time_step, pos[ind], vel[ind])
    pos_filt_fb[ind] = position_speed_filter_fb.getState()
    vel_filt_fb[ind] = position_speed_filter_fb.getRate()


#
# plots
#

# acceleration steps
fig_acc = go.Figure()
fig_acc.add_trace(
    go.Scatter(
        x=times,
        y=acc_steps,
        name='Acceleration Steps'
    )
)
fig_acc['layout']['xaxis']['title'] = 'Time [s]'
fig_acc['layout']['yaxis']['title'] = 'Acceleration [m/s^2]'

# velocities
fig_speed = go.Figure()
fig_speed.add_trace(
    go.Scatter(
        x=times,
        y=vel,
        name='Raw'
    )
)
fig_speed.add_trace(
    go.Scatter(
        x=times,
        y=vel_filt,
        name='Filtered'
    )
)
fig_speed.add_trace(
    go.Scatter(
        x=times,
        y=vel_filt_fb,
        name='Filtered w/ speed feedback'
    )
)
fig_speed['layout']['xaxis']['title'] = 'Time [s]'
fig_speed['layout']['yaxis']['title'] = 'Speed [m/s]'

# positions
fig_pos = go.Figure()
fig_pos.add_trace(
    go.Scatter(
        x=times,
        y=pos,
        name='Raw'
    )
)
fig_pos.add_trace(
    go.Scatter(
        x=times,
        y=pos_filt,
        name='Filtered'
    )
)
fig_pos.add_trace(
    go.Scatter(
        x=times,
        y=pos_filt_fb,
        name='Filtered w/ speed feedback'
    )
)
fig_pos['layout']['xaxis']['title'] = 'Time [s]'
fig_pos['layout']['yaxis']['title'] = 'Position [m]'

fig_acc.show()
fig_speed.show()
fig_pos.show()
