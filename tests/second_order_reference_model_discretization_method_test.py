# get the relative import path
import sys

sys.path.append('../build')  # assumes the readme was followed..

# other imports
import numpy as np
import plotly.graph_objects as go
import scipy.linalg as scp_linalg
# import sympy as sp

# import the pybind module
from pybound_mathlib import SecondOrderReferenceModel, DiscretizationMethod


# symbolic derivation of bilinear transform (for sanity check)
# spring_constant_sym = sp.symbols('K_x')
# damping_coefficient_sym = sp.symbols('K_v')
# sample_time_sym = sp.symbols('T')
# a_matrix_sym = sp.Matrix([[0, 1], [-spring_constant_sym, -damping_coefficient_sym]])
# b_matrix_sym = sp.Matrix([[0, 0], [spring_constant_sym, damping_coefficient_sym]])
# discrete_a_matrix_sym = sp.simplify((sp.eye(2) - 1 / 2 * a_matrix_sym * sample_time_sym)**-1 * (sp.eye(2) + 1 / 2 * a_matrix_sym * sample_time_sym))
# discrete_b_matrix_sym = sp.simplify(a_matrix_sym**-1 * (discrete_a_matrix_sym - sp.eye(2)) * b_matrix_sym)
# eval_discrete_a_matrix = sp.lambdify([sample_time_sym, spring_constant_sym, damping_coefficient_sym], discrete_a_matrix_sym)
# eval_discrete_b_matrix = sp.lambdify([sample_time_sym, spring_constant_sym, damping_coefficient_sym], discrete_b_matrix_sym)

# simulate a random walk
sim_step_time = 0.01  # [s]
sim_time = 10.  # [s]
num_sim_steps = int(sim_time / sim_step_time) + 1
sim_time_vector = np.array([i * sim_step_time for i in range(num_sim_steps)])
pos = np.zeros(num_sim_steps)
vel = np.zeros(num_sim_steps)

accel_step_set = [-5., -3., 0., 3., 5.]  # [m/s^2]
accel_steps = np.random.choice(a=accel_step_set, size=num_sim_steps)

# acceleration step is held for this long [s]
accel_hold_time = 0.5
ind_step_accel = int(accel_hold_time / sim_step_time)

# simulate
accel = accel_steps[0]
for ind in range(1, num_sim_steps):

    pos[ind] = pos[ind - 1] + sim_step_time * vel[ind - 1]
    vel[ind] = vel[ind - 1] + sim_step_time * accel

    # hold accel constant over some steps
    if np.mod(ind, ind_step_accel) == 0:
        accel = accel_steps[ind]

# continuous filter dynamics
natural_freq = 3.  # [rad/s]
damping_ratio = 0.7071

# filters are updated at this time interval [s]
filter_update_interval = 0.4
ind_step_filter = int(filter_update_interval / sim_step_time)
num_down_sampled_steps = int(np.floor(num_sim_steps / ind_step_filter))
filter_time_vector = np.array([i * filter_update_interval for i in range(num_down_sampled_steps)])

# filter with exact time discretization
spring_constant = natural_freq * natural_freq
damping_coefficient = 2.0 * damping_ratio * natural_freq
continuous_a_matrix = np.array([[0, 1], [-spring_constant, -damping_coefficient]])
continuous_b_matrix = np.array([[0, 0], [spring_constant, damping_coefficient]])
ab = np.concatenate([np.concatenate([continuous_a_matrix, continuous_b_matrix], axis=1), np.zeros([2, 4])], axis=0) * sim_step_time
abd = scp_linalg.expm(ab)
discrete_a_matrix_exact = abd[0:2, 0:2]
discrete_b_matrix_exact = abd[0:2, 2:]
pos_filt_exact = np.zeros(num_sim_steps)
vel_filt_exact = np.zeros(num_sim_steps)
pos_filt_exact[0] = pos[0]
vel_filt_exact[0] = vel[0]

# filter with forward-euler time discretization
position_speed_filter_euler = SecondOrderReferenceModel()
position_speed_filter_euler.setParameters(natural_freq, damping_ratio)
position_speed_filter_euler.setDiscretizationMethod(DiscretizationMethod.kForwardEuler)
position_speed_filter_euler.reset(pos[0], vel[0])
pos_filt_euler = np.zeros(num_down_sampled_steps)
vel_filt_euler = np.zeros(num_down_sampled_steps)
pos_filt_euler[0] = position_speed_filter_euler.getState()
vel_filt_euler[0] = position_speed_filter_euler.getRate()

# filter discretized with bilinear transform
position_speed_filter_bilinear = SecondOrderReferenceModel()
position_speed_filter_bilinear.setParameters(natural_freq, damping_ratio)
position_speed_filter_bilinear.setDiscretizationMethod(DiscretizationMethod.kBilinear)
position_speed_filter_bilinear.reset(pos[0], vel[0])
pos_filt_bilinear = np.zeros(num_down_sampled_steps)
vel_filt_bilinear = np.zeros(num_down_sampled_steps)
pos_filt_bilinear[0] = position_speed_filter_bilinear.getState()
vel_filt_bilinear[0] = position_speed_filter_bilinear.getRate()

# bilinear transform filter sanity check
# pos_filt_bilinear_sanity_check = np.zeros(num_down_sampled_steps)
# vel_filt_bilinear_sanity_check = np.zeros(num_down_sampled_steps)
# pos_filt_bilinear_sanity_check[0] = pos[0]
# vel_filt_bilinear_sanity_check[0] = vel[0]
# discrete_a_matrix_numerical = eval_discrete_a_matrix(filter_update_interval, spring_constant, damping_coefficient)
# discrete_b_matrix_numerical = eval_discrete_b_matrix(filter_update_interval, spring_constant, damping_coefficient)

# run the filters
down_sampled_ind = 1
last_down_sampled_pos = pos[0]
last_down_sampled_vel = vel[0]
for ind in range(1, num_sim_steps):

    # calculate the "exact" discretized solution for the continuous filter response
    updated_states = np.matmul(discrete_a_matrix_exact, np.array([pos_filt_exact[ind - 1], vel_filt_exact[ind - 1]])) + \
                     np.matmul(discrete_b_matrix_exact, np.array([pos[ind - 1], vel[ind - 1]]))
    pos_filt_exact[ind] = updated_states[0]
    vel_filt_exact[ind] = updated_states[1]

    # update the filters at the down-sampled interval
    if np.mod(ind, ind_step_filter) == 0 and down_sampled_ind < num_down_sampled_steps:

        # forward-euler
        position_speed_filter_euler.update(filter_update_interval, pos[ind], vel[ind])
        pos_filt_euler[down_sampled_ind] = position_speed_filter_euler.getState()
        vel_filt_euler[down_sampled_ind] = position_speed_filter_euler.getRate()

        # bilinear
        position_speed_filter_bilinear.update(filter_update_interval, pos[ind], vel[ind])
        pos_filt_bilinear[down_sampled_ind] = position_speed_filter_bilinear.getState()
        vel_filt_bilinear[down_sampled_ind] = position_speed_filter_bilinear.getRate()

        # bilinear sanity check
        # pos_filt_bilinear_sanity_check[down_sampled_ind] = \
        #     discrete_a_matrix_numerical[0][0] * pos_filt_bilinear_sanity_check[down_sampled_ind - 1] + \
        #     discrete_a_matrix_numerical[0][1] * vel_filt_bilinear_sanity_check[down_sampled_ind - 1] + \
        #     discrete_b_matrix_numerical[0][0] * last_down_sampled_pos + \
        #     discrete_b_matrix_numerical[0][1] * last_down_sampled_vel
        # vel_filt_bilinear_sanity_check[down_sampled_ind] = \
        #     discrete_a_matrix_numerical[1][0] * pos_filt_bilinear_sanity_check[down_sampled_ind - 1] + \
        #     discrete_a_matrix_numerical[1][1] * vel_filt_bilinear_sanity_check[down_sampled_ind - 1] + \
        #     discrete_b_matrix_numerical[1][0] * last_down_sampled_pos + \
        #     discrete_b_matrix_numerical[1][1] * last_down_sampled_vel
        #
        # last_down_sampled_pos = pos[ind]
        # last_down_sampled_vel = vel[ind]

        down_sampled_ind += 1

#
# plots
#

# acceleration steps
fig_acc = go.Figure()
fig_acc.add_trace(
    go.Scatter(
        x=sim_time_vector,
        y=accel_steps,
        name='Acceleration Steps'
    )
)
fig_acc['layout']['xaxis']['title'] = 'Time [s]'
fig_acc['layout']['yaxis']['title'] = 'Acceleration [m/s^2]'

# velocities
fig_speed = go.Figure()
fig_speed.add_trace(
    go.Scatter(
        x=sim_time_vector,
        y=vel,
        name='Raw measurements',
        line=dict({
            'color': 'rgb(0.7, 0.7, 0.7)'
        }),
    )
)
fig_speed.add_trace(
    go.Scatter(
        x=sim_time_vector,
        y=vel_filt_exact,
        name='Exact filter (continuous)'
    )
)
fig_speed.add_trace(
    go.Scatter(
        x=filter_time_vector,
        y=vel_filt_euler,
        name='Forward-Euler filter'
    )
)
fig_speed.add_trace(
    go.Scatter(
        x=filter_time_vector,
        y=vel_filt_bilinear,
        name='Bilinear transform filter'
    )
)
# fig_speed.add_trace(
#     go.Scatter(
#         x=filter_time_vector,
#         y=vel_filt_bilinear_sanity_check,
#         name='Bilinear transform filter sanity check'
#     )
# )
fig_speed['layout']['xaxis']['title'] = 'Time [s]'
fig_speed['layout']['yaxis']['title'] = 'Speed [m/s]'

# positions
fig_pos = go.Figure()
fig_pos.add_trace(
    go.Scatter(
        x=sim_time_vector,
        y=pos,
        name='Raw measurements',
        line=dict({
            'color': 'rgb(0.7, 0.7, 0.7)'
        }),
    )
)
fig_pos.add_trace(
    go.Scatter(
        x=sim_time_vector,
        y=pos_filt_exact,
        name='Exact filter (continuous)'
    )
)
fig_pos.add_trace(
    go.Scatter(
        x=filter_time_vector,
        y=pos_filt_euler,
        name='Forward-Euler filter'
    )
)
fig_pos.add_trace(
    go.Scatter(
        x=filter_time_vector,
        y=pos_filt_bilinear,
        name='Bilinear transform filter'
    )
)
# fig_pos.add_trace(
#     go.Scatter(
#         x=filter_time_vector,
#         y=pos_filt_bilinear_sanity_check,
#         name='Bilinear transform filter sanity check'
#     )
# )
fig_pos['layout']['xaxis']['title'] = 'Time [s]'
fig_pos['layout']['yaxis']['title'] = 'Position [m]'

# fig_acc.show()
fig_speed.show()
fig_pos.show()
