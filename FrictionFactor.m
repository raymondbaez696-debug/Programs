function f = FrictionFactor(Re, pipe, csection, material, D, units, parameter)

arguments
    Re (1,1) double {mustBePositive, mustBeFinite}
    pipe (1,1) string {mustBeMember(pipe, ["", "smooth","rough"])}
    csection (1,1) string {mustBeMember(csection, ["", "circle",...
        "rectangle","ellipse","triangle","annulus"])}
    material (1,1) string {mustBeMember(material, [ ...
    "", "glass","plastic","concrete","wood stave","smoothed rubber",...
    "copper","brass tubing","cast iron","galvanized iron","wrought iron", ...
    "stainless steel","commercial steel"])}
    D (1,1) double {mustBePositive, mustBeFinite}
    units (1,1) string = ""
    parameter (1,1) double {mustBeNonnegative, mustBeFinite} = 0
end
pipe = lower(pipe); csection = lower(csection); 
material = lower(material); units = lower(units);

if Re < 2300
    if csection == "circle"
        f = 64/Re;
        return
    end
    T = readtable("Nusselt Isothermal Internal Flow Laminar.xlsx",'Sheet',csection) ;
    x = T{:,1}; y = T{:,3};
    x = str2double(string(x)); 
    y = str2double(string(y));
    xf = x(isfinite(x) & isfinite(y));
    yf = y(isfinite(x) & isfinite(y));
    if isempty(xf)
        error("Table %s has no finite Parameter values.", csection);
    end
    if any(isinf(x))
        yInf = y(find(isinf(x),1,'first'));
    end
    if parameter < min(xf)
        error("Parameter %.4g outside %s table range [%.4g, %.4g].", parameter, csection, min(xf), max(xf));
    elseif parameter > max(xf)
        if any(isinf(x))
            a = yInf;
        else
            error("Parameter %.4g outside %s table range [%.4g, %.4g].", parameter, csection, min(xf), max(xf));
        end
    else
        a = interp1(xf, yf, parameter, 'linear');
    end
    f = a/Re ;
elseif Re >= 2300 && Re < 3000 || Re > 5e6 && Re < 10000
    error('No friction factor correlation for the Reynolds number: %d',Re) ;
elseif Re >= 10000
    if pipe == "smooth"
        f = (0.79*log(Re)-1.64)^(-2) ;
    else
        T = readtable("Roughness Internal Flow.xlsx",'Sheet',units) ;
        idx = strcmpi(strtrim(string(T.material)), strtrim(material));
        if ~any(idx)
            error("Material '%s' not found in sheet '%s' of Roughness Internal Flow.xlsx.", material, system);
        end
        if nnz(idx) > 1
            error("Material '%s' appears multiple times in sheet '%s'. Make it unique.", material, system);
        end
        epsilon = T.Roughness(idx);
        f = (-1.8*log10(6.9/Re+((epsilon/D)/3.7)^(1.11)))^(-2) ;
    end
end
end