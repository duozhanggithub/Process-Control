clear all
clc

syms A1 Kp u1 Kv1 rho g h1 G A2 Kv2 u2 h2

f1 = (Kp/A1) * u1 - ((Kv1/A1)*((rho*g)/G)^0.5)*h1^0.5;
f2 = ((Kv1/A2)*((rho*g)/G)^0.5)*h1^0.5 - ((Kv2/A2)*((rho*g)/G)^0.5)*u2*h2^0.5;
y1 = h1;
y2 = h2;

A = jacobian([f1;f2],[h1 h2])
B = jacobian([f1;f2],[u1 u2])
C = jacobian([y1;y2],[h1 h2])
D = jacobian([y1;y2],[u1 u2])