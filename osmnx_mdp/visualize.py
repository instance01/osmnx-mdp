import numpy as np
import pickle
import matplotlib.pyplot as plt

from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axes_grid1.inset_locator import inset_axes


with open('simulation.pickle', 'rb') as f:
    results = pickle.load(f)


data = []
for results_ in results:
    x = [list(result.values()) for result in results_]
    x = np.rec.array(x, dtype=[
        ('algorithm', object),
        ('id', object),
        ('calculation_time', float),
        ('drive_time', float),
        ('n_nodes', int),
        ('path', list),
        ('diverge_policy', list)])

    data.append(x)


data = np.array(data)


mdp = data[:, np.where(data[0]['algorithm'] == 'MDP')]
brtdp = data[:, np.where(data[0]['algorithm'] == 'BRTDP')]
brtdp_replan = data[:, np.where(data[0]['algorithm'] == 'BRTDP_REPLAN')]
dstar = data[:, np.where(data[0]['algorithm'] == 'DStar_Lite')]


def draw(algorithm, arr, col, ax, ax2):
    arr = np.sort(arr, order='n_nodes')
    calc_time = np.average(arr[:, 0]['calculation_time'], axis=0)
    drive_time = np.average(arr[:, 0]['drive_time'], axis=0)
    n_nodes = arr[:, 0]['n_nodes'][0]

    X = arr[:, 0]['id'][0] + ' (' + n_nodes.astype('U') + ')'
    ax.plot(
        X,
        calc_time,
        color=col,
        label=algorithm + ' calculation time')
    ax2.plot(
        X,
        drive_time,
        color=col,
        linestyle='--',
        label=algorithm + ' drive time')


def draw_setin(*algorithms):
    ax = plt.gca()
    axins = inset_axes(
            ax,
            2, 1,
            loc='upper right',
            bbox_to_anchor=(1., .8),
            bbox_transform=ax.transAxes)

    axins.set_xlim(3, 6)
    axins.set_ylim(-.5, 1.5)

    for arr, col in algorithms:
        calc_time = np.average(arr[:, 0]['calculation_time'], axis=0)
        drive_time = np.average(arr[:, 0]['drive_time'], axis=0)

        axins.plot(arr[:, 0]['id'][0], drive_time, color=col)
        axins.plot(arr[:, 0]['id'][0], calc_time, color=col)

    mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.5")

    axins.get_xaxis().set_ticks([])


fig = plt.figure()
ax = fig.add_subplot(111)
ax2 = ax.twinx()

ax.set_xlabel('Map (number of nodes)')
ax.set_ylabel('Calculation time in seconds')
ax2.set_ylabel('Driving time in minutes')


# ffaa00 <<<<<<<<<<<<<<
draw('Value Iteration', mdp, 'r', ax, ax2)
draw('BRTDP', brtdp, '#ffaa00', ax, ax2)
draw('BRTDP_REPLAN', brtdp_replan, 'r', ax, ax2)
draw('D* Lite', dstar, 'g', ax, ax2)

ax.legend(loc='upper left')
ax2.legend(loc='upper right')

plt.title('Comparison of planners')

#draw_setin((brtdp, '#ffaa00'), (dstar, 'g'))

plt.show()
