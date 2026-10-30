%open_loop_speed_controls

function n_c = open_loop_speed_control(U_ref)

m = 17.0677e6; 
Xudot = -8.9830e5;
T1 = 20;
Xu = -( m - Xudot ) / T1;

t_thr = 0.05; 
Dia = 3.3;
rho = 1025; 

KT = 0.6367;

T_u = U_ref * Xu / (t_thr-1);
nc_square = T_u/(Dia^4 * KT *rho);
n_c = sign(nc_square)*sqrt(nc_square); %need to accunt for the absolute value 

end