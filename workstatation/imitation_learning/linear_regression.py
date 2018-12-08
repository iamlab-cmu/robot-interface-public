import numpy as np
import argparse

import h5py
import os
import pdb

from sklearn.linear_model import RidgeCV

from utils.data_utils import recursively_get_dict_from_group
from utils.trajectory_utils import truncate_expert_data

def load_data(h5_path):
    assert os.path.exists(h5_path) and os.path.isfile(h5_path), \
            "h5 file does not exist {}".format(h5_path)
    h5f = h5py.File(h5_path, 'r')
    data = recursively_get_dict_from_group(h5f)
    return data

class DMPTrajectory(object):
    def __init__(self, num_dims, num_basis, num_sensors):
        self.tau = 1.05
        self.alpha = 25.0
        self.beta = self.alpha / 4.0

        self.num_dims = num_dims
        self.num_basis = num_basis
        self.num_sensors = num_sensors

        self.mean = np.array([np.exp(-i * (0.5 / (self.num_basis - 1))) 
                for i in range(self.num_basis)])
        self.std = [0.5 / (0.65 * (self.mean[i+1]-self.mean[i])**2) 
                for i in range(self.num_basis - 1)]
        self.std += [self.std[1]]
        self.std = np.array(self.std)


    def get_x_values(self, dt, x_start=1.0):
        x_list = [x_start]
        for i in range(dt.shape[0] - 1):
            dx = -(self.tau * x_list[-1])
            x = x_list[-1]
            x = x + dx * dt[i]
            x_list.append(x)
        return np.array(x_list)

    def convert_data_to_dmp_train_format(self, data_dict):
        time = data_dict['time']
        dt = time[1:] - time[:-1]
        assert np.min(dt) > 0.0 and np.max(dt) < 1.0, "Recorded time is far off"
        q = data_dict['q']
        dq = data_dict['dq']
        ddq = (dq[1:, :] - dq[:-1, :]) / dt
        # Repeat the last x to get equal length array.
        ddq = np.vstack([ddq, ddq[-1:, :]])

        # Let's first calculate sum_j(phi_j * f(x;w_j)) (forcing function)
        y0 = q[0]
        force_val = (self.tau**2)*ddq - self.alpha*(self.beta*(y0-q) - self.tau*dq)
        # Get the x values
        x_arr = self.get_x_values(dt)
        # Repeat the last x to get equal length array.
        x_arr = np.hstack([x_arr, x_arr[-1:]])[:, None]

        x_arr_rep = np.repeat(x_arr,  self.num_basis, axis=1)

        psi_k = np.exp(-self.std * (x_arr_rep - self.mean)**2)
        psi_k_sum = np.sum(psi_k[:, 0])
        feat = (psi_k * x_arr) / (psi_k_sum * self.mean)

        # Add min jerk trajectory to factor array
        min_jerk_t = np.minimum(-np.log(x_arr), np.ones(x_arr.shape))
        feat_min_jerk = (min_jerk_t**3)*(6*(min_jerk_t**2) - 15*min_jerk_t + 10)

        feat = np.hstack([feat_min_jerk, feat])

        # TODO(Mohit): Premultiply here with phi_j's if they are not 1.
        psi_jk = np.tile(feat, (1, self.num_sensors))
        psi_ijk = np.tile(psi_jk, (1, self.num_dims))

        y = force_val/(self.alpha * self.beta)
        X = psi_ijk.copy()
        assert X.shape[0] == y.shape[0], "X, y shape do not match"

        # X^T\beta = y (where we have to find \beta)
        return {'X': X, 'y': y}

    def train(self, X_train, y_train, X_test, y_test):
        clf = RidgeCV(alphas=[1e-3, 1e-2, 1e-1, 1]).fit(X_train, y_train)
        train_score = clf.score(X_train, y_train)
        test_score = clf.score(X_test, y_test)
        print("Score (max 1.0) Train: {:.3f}, Test{:.3f}".format(
            train_score, test_score))
        return clf.get_params()

    
def main(args):
    expert_data = load_data(args.h5_path)
    truncated_expert_data = truncate_expert_data(expert_data)
    num_dims, num_basis, num_sensors = 7, 9, 20
    dmp_traj = DMPTrajectory(num_dims, num_basis, num_sensors)
    X, y = [], []
    for k in sorted(expert_data.keys()):
        data = dmp_traj.convert_data_to_dmp_train_format(expert_data[k])
        assert type(data['X']) is np.ndarray \
            and type(data['y']) is np.ndarray, "Incorrect data type returned"
            
        X.append(data['X'])
        y.append(data['y'])

    # Get train and test data?
    X, y = np.vstack(X), np.vstack(y)
    train_size = int(X.shape[0] * 0.8)
    X_train, y_train = X[:train_size], y[:train_size]
    X_test, y_test = X[train_size:], y[train_size:]

    dmp_params = dmp_traj.train(X_train, y_train, X_test, y_test)
    pdb.set_trace()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description='Convert data with csvs into h5 file')
    parser.add_argument('--h5_path', type=str, required=True,
                        help='Path to h5 file.')
    args = parser.parse_args()
    main(args)
