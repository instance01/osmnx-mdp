import scipy.stats
import numpy as np
import pickle


def get_data(results):
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

    brtdp = data[:, np.where(data[0]['algorithm'] == 'BRTDP')]
    dstar = data[:, np.where(data[0]['algorithm'] == 'DStar_Lite')]

    print(brtdp.shape)
    print(dstar.shape)

    brtdp_calc_time = brtdp[:, 0]['calculation_time']
    brtdp_drive_time = brtdp[:, 0]['drive_time']
    dstar_calc_time = dstar[:, 0]['calculation_time']
    dstar_drive_time = dstar[:, 0]['drive_time']

    return np.concatenate([
            brtdp_calc_time,
            brtdp_drive_time,
            dstar_calc_time,
            dstar_drive_time]).flatten()


with open('simulationSTAT1.pickle', 'rb') as f:
    results = pickle.load(f)

stat1_data = get_data(results)

with open('simulationSTAT2.pickle', 'rb') as f:
    results = pickle.load(f)

stat2_data = get_data(results)

statistic, pval = scipy.stats.ttest_ind(stat1_data, stat2_data)
print(statistic, pval)
