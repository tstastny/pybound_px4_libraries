# get the relative import path
import sys

sys.path.append('../build')  # assumes the readme was followed..

# other imports
import numpy as np
import plotly.graph_objects as go
from npfg_lib import NPFG


npfg = NPFG()

period = 5.
damping = 0.7071
time_const = npfg.timeConst(period, damping)
p_gain = npfg.pGain(period, damping)

print(p_gain)

ground_speeds = np.linspace(0., 6., 7)  # np.array([0., 1., 2., 4., 8., 16., 32.])
len_ground_speeds = len(ground_speeds)

len_track_errors = 11
track_errors = np.linspace(0., 3., len_track_errors)

unit_path_tangent = np.array([0, 1])  # east
track_error_bounds = np.zeros(len_track_errors)
bearing_vectors = np.zeros([len_ground_speeds, len_track_errors, 2])
for ind_gsp in range(len_ground_speeds):
    track_error_bounds[ind_gsp] = npfg.trackErrorBound(ground_speeds[ind_gsp], time_const)

    for ind_terr in range(len_track_errors):
        normalized_track_error = npfg.normalizedTrackError(track_errors[ind_terr], track_error_bounds[ind_gsp])
        look_ahead_ang = npfg.lookAheadAngle(normalized_track_error)
        bearing_vectors[ind_gsp, ind_terr, :] = npfg.bearingVec(unit_path_tangent, look_ahead_ang, -track_errors[ind_terr])


#
# plots
#

fig_bearing_map = go.Figure()

bearing_vec_magnitude = 1.
xaxis_scale = 1

for ind_gsp in range(len_ground_speeds):
    for ind_terr in range(len_track_errors):
        fig_bearing_map.add_trace(
            go.Scatter(
                x=np.array([ind_gsp * xaxis_scale, ind_gsp * xaxis_scale]) + bearing_vec_magnitude * np.array([0., bearing_vectors[ind_gsp, ind_terr, 1]]),
                y=np.array([track_errors[ind_terr], track_errors[ind_terr]]) + bearing_vec_magnitude * np.array([0., bearing_vectors[ind_gsp, ind_terr, 0]]),
                line=dict({
                    'color': 'rgb(0, 0, 0)'
                }),
                showlegend=False
            )
        )

fig_bearing_map['layout']['xaxis']['title'] = 'Ground speed [m/s]'
fig_bearing_map['layout']['yaxis']['title'] = 'Track error [m]'

# keep the axes equally scaled
fig_bearing_map.update_yaxes(scaleanchor='x')
fig_bearing_map.update_yaxes(scaleratio=1)

fig_bearing_map.update_layout(
    xaxis=dict(
        tickmode='array',
        tickvals=[x for x in range(len_ground_speeds)],
        ticktext=ground_speeds
    )
)


fig_bearing_map.show()


