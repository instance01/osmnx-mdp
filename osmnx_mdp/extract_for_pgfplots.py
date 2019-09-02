import numpy as np
import pickle


np.set_printoptions(linewidth=np.inf)


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
dstar = data[:, np.where(data[0]['algorithm'] == 'DStar_Lite')]


print(mdp.shape)
print(brtdp.shape)
print(dstar.shape)


def get_data(algorithm, arr, type_, style=''):
    arr = np.sort(arr, order='n_nodes')
    data = np.percentile(arr[:, 0][type_], [25, 50, 75], axis=0)

    if style:
        style = ',' + style

    for i, item in enumerate(data.T):
        print("\\boxplot{%d}{%f}{%f}{%f}" % (i+1, *item))
    print(f"\\addplot[color=black,mark=none{style}] coordinates {{")
    for i, item in enumerate(data.T):
        print("(%d,%f)" % (i+1, item[1]))
    print(f"}};\n\\addlegendentry{{{algorithm}}}")
    print('')


print("===== CALC")
get_data('Value Iteration', mdp, 'calculation_time', 'dotted,thick')
get_data('BRTDP', brtdp, 'calculation_time')
get_data('D* Lite', dstar, 'calculation_time', 'dashed')

print("===== DRIVE")
get_data('Value Iteration', mdp, 'drive_time', 'dotted,thick')
get_data('BRTDP', brtdp, 'drive_time')
get_data('D* Lite', dstar, 'drive_time', 'dashed')
