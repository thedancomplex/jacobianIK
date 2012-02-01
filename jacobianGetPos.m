function [ pos_end, pos_elbow ] = jacobianGetPos( deg, L )
t1      = sym('t1');
t2      = sym('t2');

L1      = L(1);
L2      = L(2);

p1      = [L1 * cos(t1),                L1 * sin(t1)];          % [i,j] components
p2      = [L2 * cos(t1+t2),      L2 * sin(t1+t2)];% [i,j] components
p       = p1 + p2;

pos_end 	= subs(p,[t1,t2],deg);
pos_elbow	= subs(p1, t1, deg(1));
