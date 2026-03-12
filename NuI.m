function [Nuss, Corr] = NuI(Re, Pr, fluid, state, D, L, pipe, parameter,...
    condition, csection, surface, material, units)

arguments
    Re (1,1) double {mustBePositive, mustBeFinite}
    Pr (1,1) double {mustBePositive, mustBeFinite}
    fluid (1,1) string {mustBeMember(fluid, ["", "gas","liquid",...
        "liquid metal"])}
    state (1,1) string {mustBeMember(state, ["", "developing","developed"])}
    D (1,1) double {mustBePositive, mustBeFinite}
    L (1,1) double {mustBePositive, mustBeFinite}
    pipe (1,1) string {mustBeMember(pipe, ["", "smooth","rough"])}
    parameter (1,1) double {mustBeNonnegative, mustBeFinite}
    condition (1,1) string {mustBeMember(condition, ["",...
        "isothermal","isoflux"])}
    csection (1,1) string {mustBeMember(csection, ["", "circle",...
        "rectangle","ellipse","triangle","annulus"])}
    surface (1,1) string {mustBeMember(surface, ["", "inside","outside"])}
    material (1,1) string = ""
    units (1,1) string = ""
end

condition = lower(condition); csection = lower(csection); fluid = lower(fluid);
pipe = lower(pipe); state = lower(state);
material = lower(material); units = lower(units);

if fluid == ""
    error('Fluid is required: use "gas", "liquid", or "liquid metal".');
end

switch fluid
    case {"gas","liquid"}
        if Re < 2300
            if csection == ""
                error('CSection is required for laminar flow: use "circle","rectangle","ellipse","triangle","annulus".');
            end
            switch csection
                case "circle"
                    if state == ""
                        error('State is required for laminar circular flow: use "developed" or "developing".');
                    end
                    if condition == ""
                        error('Condition is required for laminar circular flow: use "isothermal" or "isoflux".');
                    end
                    switch state
                        case "developing"
                            Nuss = 3.66 + 0.065*(D/L)*Re*Pr/(1 + 0.04*((D/L)*Re*Pr)^(2/3));
                            Corr = sprintf("Correlation Used: Nu = (3.66 + 0.065(%.3f/%.3f)RePr/(1+0.04((%.3f/%.3f)RePr)^(2/3))", D, L, D, L) ;
                        case "developed"
                            switch condition
                                case "isothermal"
                                    Nuss = 3.66 ;
                                case "isoflux"
                                    Nuss = 4.36 ;
                                otherwise
                                    error('Condition must be "isothermal" or "isoflux".') ;
                            end
                        Corr = sprintf("Nusselt number was found using tables.") ;
                        otherwise
                            error('State must be "developing" or "developed".') ;
                    end
                case {"rectangle","ellipse","triangle"}
                    if condition == ""
                        error('Condition is required for table-based laminar correlations: use "isothermal" or "isoflux".');
                    end
                    if condition == "isothermal"
                        T = readtable("Nusselt Isothermal Internal Flow Laminar.xlsx",'Sheet',csection,'VariableNamingRule',"preserve");
                    else
                        T = readtable("Nusselt Isoflux Internal Flow Laminar.xlsx",'Sheet',csection,'VariableNamingRule',"preserve");
                    end
                    x = T{:,1}; 
                    y = T{:,2};
                    mask = isfinite(x) & isfinite(y);
                    xf = x(mask);
                    yf = y(mask);
                    if isempty(xf)
                        error("Table %s has no finite Parameter values.", csection);
                    end
                    yInf = NaN;
                    if any(isinf(x))
                        yInf = y(find(isinf(x),1,'first'));
                    end
                    if parameter < min(xf)
                        error("Parameter %.4g outside %s table range [%.4g, %.4g].", parameter, csection, min(xf), max(xf));
                    elseif parameter > max(xf)
                        if any(isinf(x)) && isfinite(yInf)
                            Nuss = yInf;
                        else
                            error("Parameter %.4g outside %s table range [%.4g, %.4g].", parameter, csection, min(xf), max(xf));
                        end
                    else
                        Nuss = interp1(xf, yf, parameter, 'linear');
                    end
                    Corr = sprintf("Nusselt number was found using tables.") ;
                case "annulus"
                    if condition == ""
                        error('Condition is required for annulus table: use "isothermal" or "isoflux".');
                    end
                    if surface == ""
                        error('Surface is required for annulus: use "inside" or "outside".');
                    end
                    if condition == "isothermal"
                        T = readtable("Nusselt Isothermal Internal Flow Laminar.xlsx",'Sheet',"annulus",'VariableNamingRule',"preserve");
                    else
                        T = readtable("Nusselt Isoflux Internal Flow Laminar.xlsx",'Sheet',"annulus",'VariableNamingRule',"preserve");
                    end
                    x = T{:,1};
                    if parameter <= 1
                        switch surface
                            case "inside"
                                y = T.("Nui");
                            case "outside"
                                y = T.("Nuo");
                            otherwise
                                error('Surface must be "inside" or "outside".');
                        end
                    else
                        error('Geometric parameter out of known values.');
                    end
                    mask = isfinite(x) & isfinite(y);
                    xf = x(mask);
                    yf = y(mask);
                    if isempty(xf)
                        error("Annulus table has no finite Parameter values.");
                    end
                    yInf = NaN;
                    if any(isinf(x))
                        yInf = y(find(isinf(x),1,'first'));
                    end
                    if parameter < min(xf)
                        error("Parameter %.4g outside annulus table range [%.4g, %.4g].", parameter, min(xf), max(xf));
                    elseif parameter > max(xf)
                        if any(isinf(x)) && isfinite(yInf)
                            Nuss = yInf;
                        else
                            error("Parameter %.4g outside annulus table range [%.4g, %.4g].", parameter, min(xf), max(xf));
                        end
                    else
                        Nuss = interp1(xf, yf, parameter, 'linear');
                    end
                    Corr = sprintf("Nusselt number was found using tables.") ;
                otherwise
                    error("Unsupported cross-section.");
            end
        elseif Re >= 2300 && Re < 3000
            error("Transitional region: 2300 <= Re < 3000. Correlation not implemented.");
        elseif Re >= 3000 && Re <= 5e6
            if pipe == ""
                error('Pipe is required for turbulent flow: use "smooth" or "rough".');
            end
            if condition == ""
                error('Condition is required for turbulent flow: use "isothermal" or "isoflux".');
            end
            if csection == ""
                error('CSection is required for turbulent flow: use "circle","rectangle","ellipse","triangle","annulus".');
            end
            if material == ""
                error('Material is required for turbulent flow because FrictionFactor needs it.');
            end
            if units == ""
                error('Units is required for turbulent flow because FrictionFactor needs it.');
            end

            f = FrictionFactor(Re, pipe, csection, material, D, units, parameter);
            Nuss = ((f/8)*(Re-1000)*Pr) / (1 + 12.7*sqrt(f/8)*(Pr^(2/3)-1));
        else
            error("Re out of range for implemented turbulent correlation.");
        end
    case "liquid metal"
        if condition == ""
            error('Condition is required for liquid metal: use "isothermal" or "isoflux".');
        end
        if Re >= 1e4 && Re <= 1e6
            switch condition
                case "isothermal"
                    Nuss = 4.8 + 0.0156*Re^(0.85)*Pr^(0.93);
                    Corr = sprintf("Correlation Used: Nu = 4.8 + 0.0156Re^(0.85)Pr^(0.93)") ;
                case "isoflux"
                    Nuss = 6.3 + 0.0167*Re^(0.85)*Pr^(0.93);
                    Corr = sprintf("Correlation Used: Nu = 6.3 + 0.0167Re^(0.85)Pr^(0.93)") ;
                otherwise
                    error('Condition must be "isothermal" or "isoflux".');
            end
        else
            error("Liquid metal correlation valid only for 1e4 <= Re <= 1e6.");
        end
end
end