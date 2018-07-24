function error_so3 = error_so3(R,Rd)

error_so3 = 1/2 * trace(eye(3) - Rd'*R);