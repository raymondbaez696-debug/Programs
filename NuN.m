function [Nuss, Corr] = NuN(Ra, Gr, Pr, Geometry, Orientation, Surface, D, L)

arguments
    Ra (1,1) double {mustBePositive, mustBeFinite}
    Gr (1,1) double {mustBePositive, mustBeFinite}
    Pr (1,1) double {mustBePositive, mustBeFinite}
    Geometry (1,1) string {mustBeMember(Geometry, ["flat plate","cylinder", ...
        "sphere"])}
    Orientation (1,1) string {mustBeMember(Orientation, ["vertical",...
        "inclined","horizontal"])}
    Surface (1,1) string {mustBeMember(Surface, ["upper","lower"])}
    D (1,1) double {mustBePositive, mustBeFinite}
    L (1,1) double {mustBePositive, mustBeFinite}
end

Geometry = lower(Geometry) ; Orientation = lower(Orientation) ;
Surface = lower(Surface) ;

switch Geometry
    case "flat plate"
        switch Orientation
            case "vertical"
                    Nuss = (0.825+(0.387*Ra^(1/6))/(1+(0.492/Pr)^(9/16))^(8/27))^2 ;
                    Corr = sprintf("Correlation Used: Nu = (0.825+(0.387*Ra^(1/6))/(1+(0.492/Pr)^(9/16))^(8/27))^2") ;
            case "inclined"
                if Ra <= 10^9
                    Nuss = (0.825+(0.387*Ra^(1/6))/(1+(0.492/Pr)^(9/16))^(8/27))^2 ;
                    Corr = sprintf("Correlation Used: Nu = (0.825+(0.387*Ra^(1/6))/(1+(0.492/Pr)^(9/16))^(8/27))^2") ;
                else
                    error('No correlation for the specified Rayleigh number.') ;
                end
            case "horizontal"
                T = readtable("Nusselt Constants Horizontal Plate Natural Convection.xlsx",...
                    'Sheet',Surface) ;
                idx = (Ra >= T.Ramin) & (Ra <= T.Ramax) ;
                if ~any(idx)
                    error('No correlation for the specified Rayleigh number.');
                end
                r = find(idx,1,'first'); C = T.C(r) ; m = T.m(r) ;
                Nuss = C*Ra^(1/m) ;
        end 
    case "cylinder"
        switch Orientation
            case "vertical"
                if D >= 35*L/Gr^(1/4)
                    Nuss = (0.825+(0.387*Ra^(1/6))/(1+(0.492/Pr)^(9/16))^(8/27))^2 ;
                else
                    error('No correlation for the specified Rayleigh number.');
                end
            case "horizontal"
                if Ra <= 10^12
                    Nuss = (0.6+(0.387*Ra^(1/6))/(1+(0.559/Pr)^(9/16))^(8/27))^2 ;
                else
                    error('No correlation for the specified Rayleigh number.');
                end
        end
    case "sphere"
        if Pr < 0.7
            error('Sphere correlation requires Pr >= 0.7.');
        end
        if Ra <= 10^12
            Nuss = 2+(0.589*Ra^(1/4))/(1+(0.469/Pr)^(9/16))^(4/9) ;
        else
            error('No correlation for the specified Rayleigh number.');
        end
end
end