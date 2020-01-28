% This script will calculate the 3DOF repsonse of the CH-53D helicopter
% subject to collective and cyclic inputs

A = [-1.48831293e-02 -1.70258267e-19  9.09646012e-01 -9.79307874e+00;
    6.80011603e-18 -3.15898244e-01  9.54097912e-20 -1.22124533e-17;
     2.27139603e-03  2.72412238e-20 -1.45542833e-01  0.00000000e+00;
     0.00000000e+00  0.00000000e+00  1.00000000e+00  0.00000000e+00];
 B = [0.00000000e+00  9.79307874e+00;
     -1.63068793e+02  1.77635684e-17;
     0.00000000e+00 -1.56688691e+00;
     0.00000000e+00  0.00000000e+00];
 
 C = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];
  D = zeros(4,2);
  
  Ts = 0.1;
  sys = ss(A,B,C,D,'StateName',{'Horiz. Speed' 'Vert. Speed' 'Pitch Rate' 'Pitch Angle'},...
                    'InputName',{'Collective Pitch' 'Cyclic Pitch'});
  

  
  t = (0:0.1:10)';
  
  Kp = [0, 0, 0, 0;
         0, 0, 0, 0;
         0, 0, 0, 0];
  Ki = [0, 0, 0, 0;
         0, 0, 0, 0;
         0, 0, 0, 0];
  Kd = [0, 0, 0, 0;
         0, 0, 0, 0;
         0, 0, 0, 0];
  Tf = 0
  
pidtune(sys)
  
  
  
  
  