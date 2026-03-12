function [Nuss, Corr] = NuE(Re, Pr, condition, geometry, csection, opts)

arguments
    Re (1,1) double {mustBePositive, mustBeFinite}
    Pr (1,1) double {mustBePositive, mustBeFinite}
    condition (1,1) string
    geometry (1,1) string {mustBeMember(geometry, ["flat plate","cylinder","sphere"])}
    csection (1,1) string = ""
    opts.Tinf (1,1) double = NaN
    opts.Ts (1,1) double = NaN
    opts.fluid (1,1) string = ""
    opts.units (1,1) string {mustBeMember(opts.units,["SI","SE"])} = "SI"
    opts.phase (1,1) string {mustBeMember(opts.phase,["gas","liquid"])} = "gas"
end

geometry  = lower(geometry); condition = lower(condition);
csection  = lower(csection); Corr = "";
switch geometry
    case "flat plate"
        if Re < 5e5
            xlsxFile = "Nusselt Constants Flat Plate Laminar External Flow.xlsx" ;
        elseif Re > 5e5
            xlsxFile = "Nusselt Constants Flat Plate Turbulent External Flow.xlsx" ;
        else
            error("Re is exactly 5e5; handle transition region explicitly.") ;
        end
        if condition ~= "temperature" && condition ~= "flux"
            error('Condition must be "temperature" or "flux" for flat plate.') ;
        end
        T = readtable(xlsxFile,'Sheet',condition) ;
        C = T.C(1) ; m = T.m(1) ; b = T.b(1) ;
        Nuss = (C*Re^m + b) * Pr^(1/3) ;
        Corr = sprintf("Correlation Used: Nu = (%.3fRe^(%.3f) + %d)Pr^(1/3)", C, m, b);
    case "cylinder"
        xlsxFile = "Nusselt Constants Cylinder External Flow.xlsx";
        if strlength(csection) == 0
            error('For "cylinder", csection (sheet name) must be provided.') ;
        end
        T = readtable(xlsxFile,'Sheet',csection) ;
        idx = (Re >= T.Remin) & (Re <= T.Remax) ;
        if ~any(idx)
            error('No correlation found for Re = %.4g in sheet "%s".', Re, csection) ;
        end
        r = find(idx,1,'first') ;
        C = T.C(r) ; m = T.m(r) ; b = T.b(r) ;
        Nuss = (C*Re^m + b) * Pr^(1/3) ;
        Corr = sprintf("Correlation Used: Nu = (%.3fRe^(%.3f) + %d)Pr^(1/3)", C, m, b);
    case "sphere"
        if ~(Re >= 3.5 && Re <= 80e3 && Pr >= 0.7 && Pr <= 380)
            error('No correlation found for Re = %.4g.', Re) ;
        end
        if isnan(opts.Tinf) || isnan(opts.Ts) || strlength(opts.fluid)==0
            error('Sphere case requires Tinf, Ts, and Fluid.') ;
        end
        DB = loadPropDB();
        mu = getProp(DB, opts.phase, opts.units, opts.fluid, ...
                          "mu", opts.Tinf, "l", "linear") ;
        mus = getProp(DB, opts.phase, opts.units, opts.fluid, ...
                          "mu", opts.Ts, "l", "linear") ;
        Nuss = 2 + (0.4*sqrt(Re) + 0.06*Re^(2/3))*Pr^(0.4)*(mu/mus) ;
        Corr = sprintf("Correlation Used: Nu = 2 + (0.4Re^(1/2) + 0.06Re^(2/3))Pr^(0.4)*(%.3e/%.3e)",...
            mu,mus) ;
end
end