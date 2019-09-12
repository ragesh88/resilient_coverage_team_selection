% Sensor placement by greedy approach

global N sen_range com_range lambda

N = 10; % the number of sensors

%/*** sensor set up***/ 
sen_range = 50; % sensing range
com_range = 50; % communication range, to ensure connectivity
lambda = 0.1; % sensing decay parameter 


%/*** simulate the environment ***/
environment_setup();

%/*** greedy algorithm ***/
[set_gre, h_gre, prob_pos_gre] = gre_place();

%/*** plots ***/
plots(set_gre, prob_pos_gre);