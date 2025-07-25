# Python Version of Kalman Filter for Lane Parameter Estimation

This is the Python implementation of the C++ project for lane parameter estimation using Kalman Filter.

## Project Structure

```
python_src/
├── kalman_filter.py      # Kalman Filter implementation
├── estimate_lane_param.py # Lane parameter estimation
├── main.py              # Main program
├── requirements.txt     # Python dependencies
└── README.md           # This file
```

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Usage

Run the main program:
```bash
python main.py
```

## Features

- **Kalman Filter**: Standard Kalman Filter implementation for state estimation
- **Lane Parameter Estimation**: Estimates lane line parameters (c0, c1, c2, c3) using polynomial model
- **Noise Handling**: Processes both measurement and process noise
- **Real-time Updates**: Supports continuous parameter updates based on new measurements

## Algorithm Overview

The system estimates lane line parameters using a 3rd-order polynomial model:
```
y = c0 + c1*x + c2*x² + c3*x³
```

Where:
- c0: lateral offset
- c1: heading angle
- c2: curvature
- c3: curvature rate

The Kalman Filter is used to:
1. Predict the next state based on vehicle motion
2. Update the state using new measurements
3. Handle noise and uncertainties in the estimation process

## Comparison with C++ Version

This Python implementation provides the same functionality as the original C++ code:
- Identical mathematical operations
- Same matrix structures and dimensions
- Equivalent noise parameters
- Same estimation algorithm

The main differences are:
- Uses NumPy for matrix operations instead of Eigen
- More Pythonic syntax and structure
- Easier to read and modify
- Better integration with Python ecosystem 


## Test log

```
$ python main.py 
matrix_P_ = 
[[0.001 0.    0.    0.   ]
 [0.    0.001 0.    0.   ]
 [0.    0.    0.001 0.   ]
 [0.    0.    0.    0.001]]
matrix_X_init = [1.8e+00 1.0e-01 1.0e-03 1.0e-06]
measurement matrix_Z = [1.95e+00 1.30e-01 6.00e-03 1.00e-06]
predict x_: [1.98162097e+00 1.01801620e-01 1.00180000e-03 1.00000000e-06]
matrix_X_res = [1.98080223e+00 1.02254359e-01 1.50389205e-03 2.27482064e-04]
--------------------------------------------------
measurement matrix_Z = [2.25e+00 1.40e-01 7.00e-03 1.00e-06]
predict x_: [2.16751749e+00 1.05329885e-01 1.91335977e-03 2.27482064e-04]
matrix_X_res = [2.21077458 0.13427997 0.01483463 0.0031528 ]
--------------------------------------------------
measurement matrix_Z = [2.55e+00 1.50e-01 8.00e-03 1.00e-06]
predict x_: [2.47957516 0.16608984 0.02050967 0.0031528 ]
matrix_X_res = [2.51864128 0.18065416 0.02295749 0.00289332]
--------------------------------------------------
measurement matrix_Z = [2.85e+00 1.60e-01 9.00e-03 1.00e-06]
predict x_: [2.88382222 0.22666483 0.02816547 0.00289332]
matrix_X_res = [ 2.84140250e+00  1.93629176e-01  1.29887058e-02 -5.54015366e-04]
--------------------------------------------------
measurement matrix_Z = [3.15e+00 1.70e-01 1.00e-02 1.00e-06]
predict x_: [ 3.21043822e+00  2.16111341e-01  1.19914781e-02 -5.54015366e-04]
matrix_X_res = [ 3.15655296e+00  1.83969288e-01  2.81619465e-04 -2.66440264e-03]
--------------------------------------------------
measurement matrix_Z = [3.45e+00 1.80e-01 1.10e-02 1.00e-06]
predict x_: [ 3.48556411e+00  1.80159871e-01 -4.51430529e-03 -2.66440264e-03]
matrix_X_res = [ 3.46208702e+00  1.73141418e-01 -4.69782623e-03 -2.21457805e-03]
--------------------------------------------------
measurement matrix_Z = [3.75e+00 1.90e-01 1.20e-02 1.00e-06]
predict x_: [ 3.76397852e+00  1.61097714e-01 -8.68406672e-03 -2.21457805e-03]
matrix_X_res = [ 3.76343456e+00  1.70829231e-01 -1.96886419e-03 -3.92238429e-04]
--------------------------------------------------
measurement matrix_Z = [4.05e+00 2.00e-01 1.30e-02 1.00e-06]
predict x_: [ 4.06735636e+00  1.66649849e-01 -2.67489336e-03 -3.92238429e-04]
matrix_X_res = [4.06535829e+00 1.76203310e-01 3.92533532e-03 1.35662717e-03]
--------------------------------------------------
measurement matrix_Z = [4.35e+00 2.10e-01 1.40e-02 1.00e-06]
predict x_: [4.39020194e+00 1.85466650e-01 6.36726423e-03 1.35662717e-03]
matrix_X_res = [4.36961204e+00 1.84400200e-01 8.99806670e-03 2.33827523e-03]
--------------------------------------------------
measurement matrix_Z = [4.65e+00 2.20e-01 1.50e-02 1.00e-06]
predict x_: [4.71838207e+00 2.04384726e-01 1.32069621e-02 2.33827523e-03]
matrix_X_res = [4.67562682e+00 1.91591979e-01 1.18401100e-02 2.63052479e-03]
--------------------------------------------------
(.venv) zyb@zyb-CORSAIR-VENGEANCE-i8100:/data/kalmanFilterForLaneParam/python_src$ 
```