import numpy as np
import pickle
import matplotlib.pyplot as plt


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
        ('diverge_policy', list),
        ('iterations', int)])

    data.append(x)


data = np.array(data)


mdp = data[:, np.where(data[0]['algorithm'] == 'MDP')]
brtdp = data[:, np.where(data[0]['algorithm'] == 'BRTDP')]
brtdp_replan = data[:, np.where(data[0]['algorithm'] == 'BRTDP_REPLAN')]
dstar = data[:, np.where(data[0]['algorithm'] == 'DStar_Lite')]


print(mdp.shape)
print(brtdp.shape)
print(brtdp_replan.shape)
print(dstar.shape)


def draw(algorithm, arr, col, ax, ax2):
    arr = np.sort(arr, order='n_nodes')
    calc_time = np.average(arr[:, 0]['calculation_time'], axis=0)
    drive_time = np.average(arr[:, 0]['drive_time'], axis=0)
    iterations = np.average(arr[:, 0]['iterations'], axis=0)
    n_nodes = arr[:, 0]['n_nodes'][0]

    print('Iterations (', algorithm, ')', iterations)

    X = arr[:, 0]['id'][0] + ' (' + n_nodes.astype('U') + ')'
    X = X.astype(str)
    X = np.char.replace(X, 'LONG', '')
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


fig = plt.figure()
ax = fig.add_subplot(111)
ax2 = ax.twinx()

ax.set_xlabel('Map (number of nodes)')
ax.set_ylabel('Calculation time in seconds')
ax2.set_ylabel('Driving time in minutes')


draw('Value Iteration', mdp, 'r', ax, ax2)
draw('BRTDP', brtdp, '#ffaa00', ax, ax2)
#draw('BRTDP_REPLAN', brtdp_replan, 'r', ax, ax2)
draw('D* Lite', dstar, 'g', ax, ax2)

ax.legend(loc='upper left')
ax2.legend(loc='upper right')

plt.title('Comparison of planners')
plt.show()
