%Observability
T = 169.53;
K = 0.007493;

A = [0 1 0;
     0 -1/T -K/T;
     0 0 0];

C = [1 0 0];

O = obsv(A,C);

rank_O = rank(O)