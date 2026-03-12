function R = Calc(geometry)
% Types of Heat Transfer: Conduction, Convection, Radiation
% Conduction: Q = q*A = k*A*dT/dx
% Convection: Q = q*A = hA(Ts-Tinf)
% Radiation:  Q = q*A = e*A*sigma*(Ts^4-Tinf^4)
% Resistances
% Conduction: R" = L/k
% Convection: R" = 1/h
% Radiation:  R" = 1/hr = 1/(e*sigma*(Ts^2+Tinf^2))
% The idea is to have the general expression Q = Rtot*ΔT and make the
% total resistance change.
valid = false ;
while~valid
    if R == 0 
        valid = true ;
    else 
        switch geometry
            case "flat plate"
                Rcond = k/L ;
                As = L*w ;
            case "cylinder"
                Rcond = log(ro/ri)/(2*pi*k*L) ;
                As = 2*pi*r*L ;
            case "sphere"
                Rcond = (ro-ri)/(4*pi*ro*ri*k) ;
                As = 4*pi*r^2 ;
        end
        switch transfer
            case "conduction"
            case "convection"
            case "radiation"
        end
        R = sigma*Rcond ;
    end
end
end