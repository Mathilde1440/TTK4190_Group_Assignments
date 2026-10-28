%% Problem 2b 

% Ship parameters
m = 17.0677e6;
Iz = 2.1732e10;
xg = -3.7;

Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot = 9.3677e5;
Nvdot = Yrdot;
Nrdot = -2.4283e10;

ud = 7;

% Mass matrices
MRB = [m, m*xg;
       m*xg, Iz];

MA = -[Yvdot, Yrdot;
       Nvdot, Nrdot];

M = MRB + MA;

% Linearized Coriolis matrices
CRB_star = [0, m*ud;
            0, m*xg*ud];

CA_star = [0, -Xudot*ud;
           (Xudot-Yvdot)*ud, -Yrdot*ud];

% Linear damping
T2 = 20;
T6 = 10;

Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;

D = -diag([Yv, Nr]);

% Combined matrix
N = CRB_star + CA_star + D;

disp('matrix M:')
disp(M)

disp('Matrix N:')
disp(N)

%% Rudder coefficients

rho = 1025;
L = 161;

bR = 2;
AR = 8;
CB = 0.8;

lambda = bR^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);

aH = 0.75;
xH = -0.4*L;
xR = -0.5*L;

Ydelta = 0.25*(1+aH)*rho*AR*CN;
Ndelta = 0.25*(xR+aH*xH)*rho*AR*CN;

% Input vector
b_input = 2*ud*[-Ydelta; -Ndelta];


disp('vector b:')
disp(b_input)

%% Problem 2b - Transfer function

A = -M\N;
B = M\b_input;
C = [0 1];
Dss = 0;

[num, den] = ss2tf(A, B, C, Dss);

G = tf(num, den);

disp('Transfer function:')
G