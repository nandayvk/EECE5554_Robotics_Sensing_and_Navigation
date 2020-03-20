% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1677.203823772910027 ; 1676.251715162911978 ];

%-- Principal point:
cc = [ 755.627590219805143 ; 1013.045363453180016 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.216409175207847 ; -0.888358455653421 ; -0.003873629770098 ; -0.000698168569728 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 7.409738600841124 ; 7.592728630408922 ];

%-- Principal point uncertainty:
cc_error = [ 5.070408188950794 ; 5.397872684868134 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.012670748720024 ; 0.074418221990795 ; 0.001438820161471 ; 0.001258336062034 ; 0.000000000000000 ];

%-- Image size:
nx = 1536;
ny = 2048;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 27;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.470733e-02 ; -3.088757e+00 ; 3.657746e-01 ];
Tc_1  = [ 8.407070e+01 ; -1.032814e+02 ; 4.002128e+02 ];
omc_error_1 = [ 1.259976e-03 ; 3.952070e-03 ; 6.603938e-03 ];
Tc_error_1  = [ 1.217942e+00 ; 1.290675e+00 ; 1.835663e+00 ];

%-- Image #2:
omc_2 = [ 9.469507e-01 ; -2.759249e+00 ; 4.970277e-01 ];
Tc_2  = [ 1.082549e+02 ; -7.205074e+01 ; 3.921256e+02 ];
omc_error_2 = [ 1.185536e-03 ; 3.811119e-03 ; 5.827083e-03 ];
Tc_error_2  = [ 1.193524e+00 ; 1.275389e+00 ; 1.825032e+00 ];

%-- Image #3:
omc_3 = [ 1.559435e+00 ; -2.449668e+00 ; 1.958793e-01 ];
Tc_3  = [ 8.971835e+01 ; -1.238410e+01 ; 3.640795e+02 ];
omc_error_3 = [ 1.967759e-03 ; 3.549585e-03 ; 6.112046e-03 ];
Tc_error_3  = [ 1.103715e+00 ; 1.177701e+00 ; 1.720299e+00 ];

%-- Image #4:
omc_4 = [ 4.362266e-01 ; 2.888070e+00 ; -4.106365e-01 ];
Tc_4  = [ 7.369100e+01 ; -1.377764e+02 ; 4.048344e+02 ];
omc_error_4 = [ 1.178738e-03 ; 3.843990e-03 ; 5.738942e-03 ];
Tc_error_4  = [ 1.246127e+00 ; 1.319833e+00 ; 1.790250e+00 ];

%-- Image #5:
omc_5 = [ NaN ; NaN ; NaN ];
Tc_5  = [ NaN ; NaN ; NaN ];
omc_error_5 = [ NaN ; NaN ; NaN ];
Tc_error_5  = [ NaN ; NaN ; NaN ];

%-- Image #6:
omc_6 = [ 8.514400e-01 ; 2.828547e+00 ; -8.337454e-01 ];
Tc_6  = [ 2.907610e+01 ; -6.778321e+01 ; 3.874819e+02 ];
omc_error_6 = [ 1.058773e-03 ; 3.634166e-03 ; 5.246500e-03 ];
Tc_error_6  = [ 1.158297e+00 ; 1.242887e+00 ; 1.560659e+00 ];

%-- Image #7:
omc_7 = [ 3.102560e-01 ; 2.724853e+00 ; -1.068545e-01 ];
Tc_7  = [ 5.249781e+01 ; -1.169505e+02 ; 4.027359e+02 ];
omc_error_7 = [ 1.198862e-03 ; 3.660011e-03 ; 5.573765e-03 ];
Tc_error_7  = [ 1.226247e+00 ; 1.298875e+00 ; 1.713680e+00 ];

%-- Image #8:
omc_8 = [ -6.730827e-01 ; 2.838812e+00 ; 2.171881e-01 ];
Tc_8  = [ 9.576505e+01 ; -5.534479e+01 ; 3.637733e+02 ];
omc_error_8 = [ 1.421064e-03 ; 3.965075e-03 ; 6.859735e-03 ];
Tc_error_8  = [ 1.097521e+00 ; 1.175210e+00 ; 1.597267e+00 ];

%-- Image #9:
omc_9 = [ NaN ; NaN ; NaN ];
Tc_9  = [ NaN ; NaN ; NaN ];
omc_error_9 = [ NaN ; NaN ; NaN ];
Tc_error_9  = [ NaN ; NaN ; NaN ];

%-- Image #10:
omc_10 = [ 1.971964e-01 ; -3.034808e+00 ; -2.209756e-01 ];
Tc_10  = [ 8.186584e+01 ; -8.224653e+01 ; 3.149187e+02 ];
omc_error_10 = [ 8.273094e-04 ; 4.162751e-03 ; 7.052281e-03 ];
Tc_error_10  = [ 9.732719e-01 ; 1.034451e+00 ; 1.493527e+00 ];

%-- Image #11:
omc_11 = [ -8.253959e-01 ; -2.724806e+00 ; -2.237490e-01 ];
Tc_11  = [ 2.517886e+01 ; -1.235686e+02 ; 3.046419e+02 ];
omc_error_11 = [ 1.526554e-03 ; 3.572185e-03 ; 5.909456e-03 ];
Tc_error_11  = [ 9.475825e-01 ; 1.002868e+00 ; 1.490007e+00 ];

%-- Image #12:
omc_12 = [ NaN ; NaN ; NaN ];
Tc_12  = [ NaN ; NaN ; NaN ];
omc_error_12 = [ NaN ; NaN ; NaN ];
Tc_error_12  = [ NaN ; NaN ; NaN ];

%-- Image #13:
omc_13 = [ NaN ; NaN ; NaN ];
Tc_13  = [ NaN ; NaN ; NaN ];
omc_error_13 = [ NaN ; NaN ; NaN ];
Tc_error_13  = [ NaN ; NaN ; NaN ];

%-- Image #14:
omc_14 = [ NaN ; NaN ; NaN ];
Tc_14  = [ NaN ; NaN ; NaN ];
omc_error_14 = [ NaN ; NaN ; NaN ];
Tc_error_14  = [ NaN ; NaN ; NaN ];

%-- Image #15:
omc_15 = [ NaN ; NaN ; NaN ];
Tc_15  = [ NaN ; NaN ; NaN ];
omc_error_15 = [ NaN ; NaN ; NaN ];
Tc_error_15  = [ NaN ; NaN ; NaN ];

%-- Image #16:
omc_16 = [ NaN ; NaN ; NaN ];
Tc_16  = [ NaN ; NaN ; NaN ];
omc_error_16 = [ NaN ; NaN ; NaN ];
Tc_error_16  = [ NaN ; NaN ; NaN ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ -1.948157e+00 ; -2.215223e+00 ; 5.744414e-01 ];
Tc_18  = [ -5.762629e+01 ; -1.064039e+02 ; 3.046892e+02 ];
omc_error_18 = [ 2.938868e-03 ; 2.022205e-03 ; 5.145740e-03 ];
Tc_error_18  = [ 9.358081e-01 ; 9.826384e-01 ; 1.343147e+00 ];

%-- Image #19:
omc_19 = [ 4.695491e-01 ; -3.002040e+00 ; -3.234056e-01 ];
Tc_19  = [ 7.920102e+01 ; -5.367155e+01 ; 2.902514e+02 ];
omc_error_19 = [ 1.081047e-03 ; 3.742262e-03 ; 6.124529e-03 ];
Tc_error_19  = [ 8.927750e-01 ; 9.474328e-01 ; 1.359892e+00 ];

%-- Image #20:
omc_20 = [ -7.862946e-01 ; -2.935160e+00 ; -2.039651e-01 ];
Tc_20  = [ 3.757232e+01 ; -1.305463e+02 ; 3.061215e+02 ];
omc_error_20 = [ 1.524334e-03 ; 4.023399e-03 ; 7.183881e-03 ];
Tc_error_20  = [ 9.544165e-01 ; 1.011089e+00 ; 1.464674e+00 ];

%-- Image #21:
omc_21 = [ -2.798705e+00 ; 1.373109e+00 ; -1.262885e-01 ];
Tc_21  = [ 4.966670e+01 ; 1.053126e+02 ; 4.352722e+02 ];
omc_error_21 = [ 5.378469e-03 ; 2.379603e-03 ; 8.684190e-03 ];
Tc_error_21  = [ 1.323588e+00 ; 1.409282e+00 ; 2.083832e+00 ];

%-- Image #22:
omc_22 = [ 1.331441e+00 ; -2.816451e+00 ; 8.447619e-02 ];
Tc_22  = [ 1.053919e+02 ; -6.208953e+00 ; 3.797858e+02 ];
omc_error_22 = [ 1.851386e-03 ; 4.434232e-03 ; 8.223818e-03 ];
Tc_error_22  = [ 1.152200e+00 ; 1.229552e+00 ; 1.786528e+00 ];

%-- Image #23:
omc_23 = [ 2.325362e+00 ; -1.690773e+00 ; 9.166005e-01 ];
Tc_23  = [ 1.100374e+02 ; 3.917990e+01 ; 3.563508e+02 ];
omc_error_23 = [ 2.229484e-03 ; 3.500764e-03 ; 5.297939e-03 ];
Tc_error_23  = [ 1.080233e+00 ; 1.172434e+00 ; 1.711818e+00 ];

%-- Image #24:
omc_24 = [ NaN ; NaN ; NaN ];
Tc_24  = [ NaN ; NaN ; NaN ];
omc_error_24 = [ NaN ; NaN ; NaN ];
Tc_error_24  = [ NaN ; NaN ; NaN ];

%-- Image #25:
omc_25 = [ NaN ; NaN ; NaN ];
Tc_25  = [ NaN ; NaN ; NaN ];
omc_error_25 = [ NaN ; NaN ; NaN ];
Tc_error_25  = [ NaN ; NaN ; NaN ];

%-- Image #26:
omc_26 = [ NaN ; NaN ; NaN ];
Tc_26  = [ NaN ; NaN ; NaN ];
omc_error_26 = [ NaN ; NaN ; NaN ];
Tc_error_26  = [ NaN ; NaN ; NaN ];

%-- Image #27:
omc_27 = [ 1.626557e+00 ; -2.061511e+00 ; 5.922109e-01 ];
Tc_27  = [ 8.060656e+01 ; -6.515366e+01 ; 3.789802e+02 ];
omc_error_27 = [ 2.237214e-03 ; 3.559306e-03 ; 4.923209e-03 ];
Tc_error_27  = [ 1.146232e+00 ; 1.235569e+00 ; 1.839369e+00 ];

