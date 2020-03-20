% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1660.128139815274153 ; 1659.726012734718552 ];

%-- Principal point:
cc = [ 756.296805130625444 ; 1024.588216106716118 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.189063518275272 ; -0.702875471136265 ; -0.002126521544475 ; -0.000265367873858 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 7.281207457246242 ; 7.447549378916006 ];

%-- Principal point uncertainty:
cc_error = [ 4.585523706451439 ; 5.069777142276559 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.010276166863909 ; 0.053155456418338 ; 0.001389892325712 ; 0.001151231498562 ; 0.000000000000000 ];

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
omc_1 = [ -1.471148e-02 ; -3.090732e+00 ; 3.564670e-01 ];
Tc_1  = [ 8.390029e+01 ; -1.060128e+02 ; 3.956390e+02 ];
omc_error_1 = [ 1.241019e-03 ; 3.846481e-03 ; 6.458819e-03 ];
Tc_error_1  = [ 1.102534e+00 ; 1.210503e+00 ; 1.804755e+00 ];

%-- Image #2:
omc_2 = [ 9.486158e-01 ; -2.763642e+00 ; 4.877939e-01 ];
Tc_2  = [ 1.080899e+02 ; -7.473678e+01 ; 3.877501e+02 ];
omc_error_2 = [ 1.168926e-03 ; 3.627295e-03 ; 5.638762e-03 ];
Tc_error_2  = [ 1.080318e+00 ; 1.196256e+00 ; 1.798130e+00 ];

%-- Image #3:
omc_3 = [ 1.562739e+00 ; -2.453929e+00 ; 1.892454e-01 ];
Tc_3  = [ 8.958629e+01 ; -1.487926e+01 ; 3.602839e+02 ];
omc_error_3 = [ 1.926825e-03 ; 3.437568e-03 ; 5.950464e-03 ];
Tc_error_3  = [ 9.985045e-01 ; 1.106030e+00 ; 1.695789e+00 ];

%-- Image #4:
omc_4 = [ 4.377204e-01 ; 2.890412e+00 ; -4.010797e-01 ];
Tc_4  = [ 7.352596e+01 ; -1.405653e+02 ; 4.001708e+02 ];
omc_error_4 = [ 1.141133e-03 ; 3.676450e-03 ; 5.533624e-03 ];
Tc_error_4  = [ 1.129936e+00 ; 1.237009e+00 ; 1.760500e+00 ];

%-- Image #5:
omc_5 = [ -1.247531e+00 ; -2.791065e+00 ; 3.041633e-01 ];
Tc_5  = [ -4.613712e+00 ; -1.678283e+02 ; 3.577884e+02 ];
omc_error_5 = [ 2.511216e-03 ; 3.072466e-03 ; 6.143349e-03 ];
Tc_error_5  = [ 1.029842e+00 ; 1.100063e+00 ; 1.675074e+00 ];

%-- Image #6:
omc_6 = [ 8.521223e-01 ; 2.831758e+00 ; -8.251763e-01 ];
Tc_6  = [ 2.890468e+01 ; -7.041910e+01 ; 3.833913e+02 ];
omc_error_6 = [ 1.031635e-03 ; 3.463396e-03 ; 5.002630e-03 ];
Tc_error_6  = [ 1.048663e+00 ; 1.165878e+00 ; 1.534650e+00 ];

%-- Image #7:
omc_7 = [ 3.125105e-01 ; 2.727004e+00 ; -9.774797e-02 ];
Tc_7  = [ 5.238071e+01 ; -1.197391e+02 ; 3.982888e+02 ];
omc_error_7 = [ 1.168716e-03 ; 3.534908e-03 ; 5.395938e-03 ];
Tc_error_7  = [ 1.113162e+00 ; 1.218570e+00 ; 1.689473e+00 ];

%-- Image #8:
omc_8 = [ -6.722644e-01 ; 2.839206e+00 ; 2.250507e-01 ];
Tc_8  = [ 9.564368e+01 ; -5.782064e+01 ; 3.598745e+02 ];
omc_error_8 = [ 1.429220e-03 ; 3.870708e-03 ; 6.842779e-03 ];
Tc_error_8  = [ 9.941664e-01 ; 1.103697e+00 ; 1.578906e+00 ];

%-- Image #9:
omc_9 = [ -1.307737e+00 ; 2.664668e+00 ; 2.865753e-01 ];
Tc_9  = [ 9.314992e+01 ; 3.734872e+01 ; 3.404405e+02 ];
omc_error_9 = [ 1.578438e-03 ; 3.407851e-03 ; 6.343224e-03 ];
Tc_error_9  = [ 9.401338e-01 ; 1.039692e+00 ; 1.482666e+00 ];

%-- Image #10:
omc_10 = [ 1.979078e-01 ; -3.037466e+00 ; -2.255694e-01 ];
Tc_10  = [ 8.177895e+01 ; -8.447650e+01 ; 3.115075e+02 ];
omc_error_10 = [ 8.228603e-04 ; 4.044995e-03 ; 7.043599e-03 ];
Tc_error_10  = [ 8.823958e-01 ; 9.718545e-01 ; 1.466074e+00 ];

%-- Image #11:
omc_11 = [ -8.245129e-01 ; -2.726082e+00 ; -2.331076e-01 ];
Tc_11  = [ 2.511400e+01 ; -1.257060e+02 ; 3.008863e+02 ];
omc_error_11 = [ 1.513245e-03 ; 3.431101e-03 ; 5.867333e-03 ];
Tc_error_11  = [ 8.595051e-01 ; 9.404495e-01 ; 1.459826e+00 ];

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
omc_17 = [ 1.625132e+00 ; 2.386052e+00 ; -1.340547e-01 ];
Tc_17  = [ -3.407282e+01 ; -1.480022e+02 ; 3.162109e+02 ];
omc_error_17 = [ 1.869486e-03 ; 3.220595e-03 ; 5.348185e-03 ];
Tc_error_17  = [ 9.062094e-01 ; 9.708423e-01 ; 1.466141e+00 ];

%-- Image #18:
omc_18 = [ -1.946642e+00 ; -2.216428e+00 ; 5.646643e-01 ];
Tc_18  = [ -5.781320e+01 ; -1.084893e+02 ; 3.011642e+02 ];
omc_error_18 = [ 2.758216e-03 ; 1.909900e-03 ; 4.905039e-03 ];
Tc_error_18  = [ 8.469837e-01 ; 9.226725e-01 ; 1.304915e+00 ];

%-- Image #19:
omc_19 = [ NaN ; NaN ; NaN ];
Tc_19  = [ NaN ; NaN ; NaN ];
omc_error_19 = [ NaN ; NaN ; NaN ];
Tc_error_19  = [ NaN ; NaN ; NaN ];

%-- Image #20:
omc_20 = [ -7.862246e-01 ; -2.936503e+00 ; -2.070842e-01 ];
Tc_20  = [ 3.747803e+01 ; -1.327545e+02 ; 3.026608e+02 ];
omc_error_20 = [ 1.534161e-03 ; 3.906545e-03 ; 7.121197e-03 ];
Tc_error_20  = [ 8.652684e-01 ; 9.490179e-01 ; 1.433477e+00 ];

%-- Image #21:
omc_21 = [ -2.796546e+00 ; 1.372829e+00 ; -1.216538e-01 ];
Tc_21  = [ 4.952557e+01 ; 1.023078e+02 ; 4.310579e+02 ];
omc_error_21 = [ 5.315602e-03 ; 2.346816e-03 ; 8.578769e-03 ];
Tc_error_21  = [ 1.197895e+00 ; 1.324579e+00 ; 2.048995e+00 ];

%-- Image #22:
omc_22 = [ 1.332428e+00 ; -2.818917e+00 ; 7.913111e-02 ];
Tc_22  = [ 1.052411e+02 ; -8.825892e+00 ; 3.758181e+02 ];
omc_error_22 = [ 1.827340e-03 ; 4.400067e-03 ; 8.150201e-03 ];
Tc_error_22  = [ 1.042546e+00 ; 1.155169e+00 ; 1.757012e+00 ];

%-- Image #23:
omc_23 = [ 2.329312e+00 ; -1.697218e+00 ; 9.103871e-01 ];
Tc_23  = [ 1.099305e+02 ; 3.669305e+01 ; 3.528054e+02 ];
omc_error_23 = [ 2.155017e-03 ; 3.314092e-03 ; 5.038549e-03 ];
Tc_error_23  = [ 9.786961e-01 ; 1.102971e+00 ; 1.681237e+00 ];

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
omc_27 = [ 1.631263e+00 ; -2.067079e+00 ; 5.854280e-01 ];
Tc_27  = [ 8.045401e+01 ; -6.770901e+01 ; 3.746552e+02 ];
omc_error_27 = [ 2.163197e-03 ; 3.321402e-03 ; 4.708456e-03 ];
Tc_error_27  = [ 1.037630e+00 ; 1.159303e+00 ; 1.811978e+00 ];

