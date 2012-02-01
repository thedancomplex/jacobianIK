close all
clear all


L1  	= 0.5;
L2 	= 0.5;

L 	= [ L1, L2 ];

deg1 	= pi/4;
deg2 	= pi/2;

deg 	= [ deg1, deg2 ];

%% Current Pos
pos	= jacobianGetPos(deg, L);
disp(['Current X pos = ', num2str(pos(1))]);
disp(['Current Y pos = ', num2str(pos(2))]);


%% Desired Pos
delta_pos = [0.01, 0.01];
des_pos = pos + delta_pos;
disp(['Des Pos X = ', num2str(des_pos(1))]);
disp(['Des Pos Y = ', num2str(des_pos(2))]);

[a, e ] =jacobianIk2Dof( des_pos, L, deg );

%% List error
disp(['Error X = ', num2str(e(1))])
disp(['Error Y = ', num2str(e(2))])
