%Following are the values of w and q axes of the HA problem 3
syms L
w1 = [0, 0, 1];
q1 = [0, -2*L, -L];
v1 = -cross(w1,q1);

w2 = [1, 0, 0];
q2 = [0,-2*L,0];
v2 = -cross(w2,q2);

w3 = [0, 0, 1];
q3 = [0, -L, 0];
v3 = -cross(w3,q3);

w4 = [0, 0, 0];
q4 = [0, 1, 0];
v4 = -cross(w4,q4);

tab_body = [w1, v1;
       w2, v2;
       w3, v3;
       w4, v4];