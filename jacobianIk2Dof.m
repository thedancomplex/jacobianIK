function [ a, e ] =jacobianIk2Dof( pos, le, deg )
%% function [ a ] =jacobianIk2Dof( pos, le, deg )
%%
%% Send:
%%	pos 	= [x,y] desired pos
%%	le	= [length1, length2] of arms
%%	deg	= [deg1, deg2] angle of joints corrospondign to le
%%
%% Return
%%	a 	= [deg1_new, deg2_new] new degree for each joint
%%	e	= [ex, ey] error between the desired pos and the actuial pos

t1 	= sym('t1');
t2 	= sym('t2');

L1 	= le(1);
L2 	= le(2);

p1	= [L1 * cos(t1), 		L1 * sin(t1)]; 		% [i,j] components
p2	= [L2 * cos(t1+t2),	L2 * sin(t1+t2)];% [i,j] components
p 	= p1 + p2;

X 	= p(1);
Y	= p(2);

J 	= jacobian( p, [t1, t2] );	% get the jacobian

t0	= [ deg(1), deg(2) ];		% starting pos

%% starting pos
p0 	= jacobianGetPos(deg, le);

%% difference between current pos and new pos
dp	= pos - p0;

%% solve for change in deg from d_pos = J * d_deg
dd 	= inv(J) * dp';

%% sub in initial pos to get d_pos
dds	= subs(dd, [ t1, t2 ], p0);

%% get final pos
a	= deg+dds';
p1 	= subs(p, [t1, t2], a);

%% error
e 	= p1 - p0;

