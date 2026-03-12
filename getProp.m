function p = getProp(DB, phase, units, fluid, Tq, side)
% GETPROP  Returns interpolated properties at temperature Tq.
% Usage:
%   p = getProp(DB, phase, unitSys, fluid, Tq);
%   mu = p.mu; k = p.k; Pr = p.Pr; rho = p.rho;
%
% Notes:
% - Interpolation method is fixed inside (linear).
% - For saturation-style tables (water, R134a, propane, ammonia):
%   side = "l" (default) uses liquid columns (mul, kl, Prl, rhol, Cpl, etc.)
%   side = "v" uses vapor columns (muv, kv, Prv, rhov, Cpv, etc.)
% - For single-phase tables (rho, Cp, k, mu, Pr, ...), side is ignored.

arguments
    DB (1,1) struct
    phase (1,1) string
    units (1,1) string
    fluid (1,1) string
    Tq (1,1) double {mustBeFinite}
    side (1,1) string {mustBeMember(side,["","l","L","v","V"])} = ""
end

phase   = lower(strtrim(phase));
units   = upper(strtrim(units));
fluidField = matlab.lang.makeValidName(lower(strtrim(fluid)));
side    = lower(strtrim(side));

if ~isfield(DB, phase)
    error("Phase '%s' not found in DB.", phase);
end
if ~isfield(DB.(phase), units)
    error("Unit system '%s' not found in DB.%s.", units, phase);
end
if ~isfield(DB.(phase).(units), fluidField)
    error("Fluid '%s' not found in DB.%s.%s.", fluid, phase, units);
end

tbl = DB.(phase).(units).(fluidField);

% Fixed interpolation method (you can change here once)
method = "linear";

% Pull T vector
if ~any(strcmp(tbl.Properties.VariableNames,"T"))
    error("Table for '%s' does not contain column 'T'.", fluid);
end
T = double(tbl.T);
vars = string(tbl.Properties.VariableNames);
lowvars = lower(vars);

% Detect saturation-table style (has rhol/rhov or mul/muv etc.)
isSat = any(lowvars=="rhol") || any(lowvars=="rhov") || any(lowvars=="mul") || any(lowvars=="muv");

% If saturation table and user didn't provide side, default to liquid
if isSat && side == ""
    side = "l";
end

% Helper: interpolate a column if it exists
    function val = interpCol(colName)
        if ~any(strcmp(vars, colName))
            val = NaN;
            return
        end
        Y = double(tbl.(colName));
        val = interp1(T, Y, Tq, method, "extrap");
    end

% Build output struct (numeric properties at Tq)
p = struct();
p.phase = phase;
p.units = units;
p.fluid = fluidField;
p.T = Tq;
p.side = side;
p.isSat = isSat;

if isSat
    % Saturation tables: choose liquid or vapor columns
    if side == "l"
        p.rho = interpCol("rhol");
        p.Cp  = interpCol("Cpl");
        p.k   = interpCol("kl");
        p.mu  = interpCol("mul");
        p.Pr  = interpCol("Prl");
    elseif side == "v"
        p.rho = interpCol("rhov");
        p.Cp  = interpCol("Cpv");
        p.k   = interpCol("kv");
        p.mu  = interpCol("muv");
        p.Pr  = interpCol("Prv");
    else
        error("For saturation tables, side must be 'l' or 'v'.");
    end

    % Common saturation columns (if present)
    p.Psat  = interpCol("Psat");
    p.hfg   = interpCol("hfg");
    p.beta  = interpCol("beta");
    p.sigma = interpCol("sigma");   % only exists in some 15-col tables

else
    % Single-phase tables: typical columns
    % (If a column doesn't exist, it returns NaN)
    p.rho   = interpCol("rho");
    p.Cp    = interpCol("Cp");
    p.k     = interpCol("k");
    p.alpha = interpCol("alpha");
    p.mu    = interpCol("mu");
    p.nu    = interpCol("nu");
    p.Pr    = interpCol("Pr");
    p.beta  = interpCol("beta");
end

end