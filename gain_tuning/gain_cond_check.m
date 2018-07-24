function c = gain_cond_check(model_param, B, psi_1, evmax, c1, c2, kx, kv, kr, kw)

m = model_param.mass;
J = model_param.I;

alpha = sqrt(psi_1*(2-psi_1));
W1 = [ c1*kx/m -c1*kv/(2*m)*(1+alpha);-c1*kv/(2*m)*(1+alpha) kv*(1-alpha)-c1 ];
W12 = [kx*evmax+c1/m*B 0;B 0];
W2 = [c2*kr/max(eig(J)) -c2*kw/(2*min(eig(J)));-c2*kw/(2*min(eig(J))) kw-c2];

c = min(eig(W2))-4*norm(W12)^2/min(eig(W1));
