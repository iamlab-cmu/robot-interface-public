import numpy as np
import argparse

import h5py

from utils.data_utils import recursively_get_dict_from_group

def load_data(h5_path):
    assert os.path.exists(h5_path) and os.path.isfile(h5_path), \
            "h5 file does not exist {}".format(h5_path)
    h5f = h5py.File(h5_path, 'r')
    data = recursively_get_dict_from_group(h5f)
    return data

class DMPTrajectory(object):
    def __init__(self):
        self.tau = 1.05
        self.alpha = 25.0
        self.beta = self.alpha / 4.0

        self.num_basis = 10
        for i in range(self.num_basis)
            mean[i] = np.exp(-i * (0.5 / (self.num_basis - 1)))
        self.mean = [np.exp(-i * (0.5 / (self.num_basis - 1))) 
                for i in range(self.num_basis)]
        self.std = [0.5 / (0.65 * (mean[i+1]-mean[i])**2) 
                for i in range(self.num_basis - 1)]
        self.std += [self.std[1]]


    def get_x_values(self, dt, x_start=1.0):
        x_list = [x_start]
        for i in range(dt.shape[0] - 1):
            dx = -(self.tau * x_list[-1])
            x = x + dx * dt[i]
            x_list.append(x)
        return np.array(x_list)

    def convert_data_to_dmp_train_format(data_dict):
        time = data_dict['time']
        dt = time[1:] - time[:-1]
        assert np.min(dt) > 0.0 and np.max(dt) < 1.0, "Recorded time is far off"
        q = data_dict['q']
        dq = data_dict['dq']
        ddq = (dq[:, 1:] - dq[:, :-1]) / dt

        # Let's first calculate sum_j(phi_j * f(x;w_j)) (forcing function)
        y0 = q[0]
        force_val = (self.tau**2)*ddq - self.alpha*(self.beta*(y0-q) - self.tau*dq)
        # Get the x values
        x_arr = get_x_values(dt)


    
def main(args):
    expert_data = load_data(args.h5_path)
    for k in sorted(expert_data.keys()):
        convert_data_to_dmp_train_format(expert_data[k])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description='Convert data with csvs into h5 file')
    parser.add_argument('--h5_path', type=str, required=True,
                        help='Path to h5 file.')
    args = parser.parse_args()
    main(args)
