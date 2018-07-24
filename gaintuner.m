%%
R0 = ROTZ(x0(9))*ROTX(x0(7))*ROTY(x0(8));
psi_1 = error_so3(R0,eye(3))+.1;
%%
alphaa = sqrt(psi_1*(2-psi_1));
% ew0
w0 = x0(10:12);
wd0 = x0(10:12);
Rd0 = eye(3);
ew0 = w0 - R0'*Rd0*wd0;

m = model_param.mass;
g = model_param.grav;
J = model_param.I;
des_acc = [];
des_magacc = [];
t_int = linspace(ts(1), ts(end), 1000);
for i=1:length(t_int)
    des_state = desired_state(tau_vec,t_int(i),PATH,P);
    des_acc(:,i) = m*des_state.acc + m*g*[0 0 1]';
    des_magacc(i) = norm(des_acc(:,i));    
end
B = max(des_magacc)*1.0001;
ev0 = x0(4:6);
kx = 2000;
kv = 2500;
c1min = [kv*(1-alphaa) 4*m*kx*kv*(1-alphaa)/(kv^2*(1-alphaa)^2+4*m*kx) sqrt(kx*m)];
c1 = min(c1min)/16;
evmax = max([norm(ev0) B/(kv*(1-alphaa))]);
%
kr1 = 30000;
kw1 = kr1-5000;
kr = kr1:1:kr1+20;
kw = kw1:1:kw1+20;
[KR, KW] = meshgrid(kr, kw);
ggcc = zeros(length(kr),length(kw));
%
for i=1:length(kr)
    for ii=1:length(kw)
        c2 = c2_min(J, psi_1, kr(i), kw(ii))/1.5;
        if c2 < 0 
            error('c2 is negative')
        end
        ggcc(i,ii) = gain_cond_check(model_param, B, psi_1, evmax, c1, c2, kx, kv, kr(i), kw(ii));
    end
end
%
%figure(7);
mesh(KR,KW,ggcc'./10^6)
xlabel('K_R')
ylabel('K_\omega')