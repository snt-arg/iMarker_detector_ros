import os
import numpy as np

# Sensors
sensorProjectRoot = f"{os.getcwd()}/src/csr_sensors/sensors/config"

# Processing
# Homography matrix for iDS cameras
# Structure: [[scaling x-axis, skewing x-axis, shift x-axis]
#             [skewing y-axis, scaling y-axis, shift y-axis]
#             [perspective x-axis, perspective y-axis, scaling all]]
homographyMatList = {
    'Settings1': np.array([[1.01621457e+00,  3.58445420e-02, -2.44065632e+01],
                          [-9.84581954e-03,  1.02765380e+00, -1.40367861e+01],
                          [4.41648502e-06,  3.15020103e-05,  1.00000000e+00]]),
    'Settings2': np.array([[1.00810034e+00, -8.35518266e-03,  8.24430129e+00],
                          [5.04865290e-03,  1.01568339e+00, -2.56132273e+00],
                          [-6.06091127e-06,  1.69549217e-05,  1.00000000e+00]]),
    'Settings3': np.array([[1.03330539e+00, -2.33383557e-02,  7.86579611e+00],
                           [2.72445070e-02,  1.01455844e+00, -1.21160907e+01],
                           [2.72468851e-05, -5.08436884e-06,  1.00000000e+00]]),
    'Settings4': np.array([[1.05178314e+00,  7.47381640e-03, -6.00956602e+00],
                           [2.26952543e-02,  1.08586602e+00, -3.55523734e+01],
                           [1.19584372e-05,  5.47181939e-05,  1.00000000e+00]]),
    'Settings5': np.array([[1.00599877e+00, -1.48766070e-02,  9.61639584e+00],
                           [2.28028148e-02,  9.99854316e-01, -9.24323584e+00],
                           [1.20355482e-05, -1.35088334e-05,  1.00000000e+00]])
}
