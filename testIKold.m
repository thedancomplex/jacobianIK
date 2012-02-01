close all
clear all


x 	= sym('x');
y 	= sym('y');
z 	= sym('z');

t1 	= sym('t1');
t2	= sym('t2');
t3	= sym('t3'); 

L1 	= 0.5;
L2 	= 0.5;

p1 	=  [L1 * cos(t1) ,  L1 * sin(t1)];
p2	=  [L2 * cos(t1+t2 - pi/2) ,  L2 * sin(t1+t2 - pi/2)];

p 	= p1 + p2;
X	= p(1);
Y	= p(2);

J 	= 	[ 	diff(X,t1), 	diff(X,t2) ;
			diff(Y,t1),	diff(Y,t2)];
JJ	= jacobian(p,[t1,t2])

%% Initial pos - streight arm

t0 = [pi/4, pi/4];

r0 = subs(p1+p2,{t1,t2},{t0(1), t0(2)})

%r0 = [ x0, y0 ];

ddr0 = [0.001,0.0014];


r1 = r0+ddr0

dr = r1 - r0;		% error

dt = inv(J)*dr';
%dt = J^-1*dr';

dt1 = subs(dt,t1,t0(1));
dt2 = subs(dt1,t2,t0(2));


ddt1 = t0+dt2';
r1 = subs(p1+p2,{t1,t2},{ddt1(1), ddt1(2)})

r1 - r0
