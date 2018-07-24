function c2_min = c2_min(J, psi1, kr, kw)

c2_min = min([kw 4*kr*kw*min(eig(J))^2/(kw^2*max(eig(J))+4*kr*min(eig(J))^2) sqrt(kr*min(eig(J))) sqrt(2/(2-psi1)*kr*max(eig(J)))]);