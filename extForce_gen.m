function Fmat = extForce_gen(tsave, start_T, duration, mag, direction)

% Direction vector of the external force (unit vector)
dir_F = direction/norm(direction);
F = mag * dir_F;
Fmat = zeros(length(tsave),3);

% time frame : tsave - duration - tsave

for i = 1:length(tsave)
    if tsave(i) >= start_T && tsave(i) < start_T + duration/6
        Fmat(i,:) = F*(sin(2*pi/(duration/3)*(tsave(i)-start_T)-pi/2) + 1)/2;
        
    elseif tsave(i) >= start_T + duration/6 && tsave(i) < start_T + duration*5/6
        Fmat(i,:) = F;
        
    elseif tsave(i) >= start_T + duration*5/6 && tsave(i) < start_T + duration
        Fmat(i,:) = F*(sin(2*pi/(duration/3)*(tsave(i)-(start_T+duration*5/6)) + pi/2) + 1)/2;
    end
end
Fmat = [tsave', Fmat];
        
end

