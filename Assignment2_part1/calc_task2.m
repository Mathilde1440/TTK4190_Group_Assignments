%task 2

% real ship parameters
rho = 1025; % density of water (kg/m^3)
m = 17.0677e6; % mass (kg)
Iz = 2.1732e10; % yaw moment of inertia about CO (kg m^3)
xg = -3.7; % x−coordinate of CG relative CO (m)
L = 161; % length (m)
B = 21.8; % beam (m)
T = 4.74; % draft (m)
g = 9.81;
H = 15.8;
% added mass about CO
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot = 9.3677e5;
Nvdot = Yrdot;
Nrdot = -2.4283e10;

delta = m/rho;
delta2 = L*B*T;

Awp = L*B;
Z_hs = - rho* g *Awp;
T3 = 2*pi *sqrt((2*T)/g);

I_T = (1/12) * B^3*L;
I_L = (1/12) * B*L^3;
BG = H/2 - T/2;
BM_T = I_T/delta2;
BM_L = I_L/delta2;


GM_T = BM_T - BG;
GM_L = BM_L -BG;