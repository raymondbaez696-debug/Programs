function DB = loadPropDB()
% LOADPROPDB  Loads all property workbooks (gas/liquid, SI/SE).
% Each sheet = one fluid. Sheets have NO header row.
% Column names are assigned based on number of columns and fluid name.

DB = struct();

% Put the Excel files in the same folder as this function
thisFolder = fileparts(mfilename("fullpath"));

% ---- Excel files (full paths) ----
files.gas.SI    = fullfile(thisFolder, "Gas Properties SI.xlsx");
files.gas.SE    = fullfile(thisFolder, "Gas Properties SE.xlsx");
files.liquid.SI = fullfile(thisFolder, "Liquid Properties SI.xlsx");
files.liquid.SE = fullfile(thisFolder, "Liquid Properties SE.xlsx");

phases = fieldnames(files);  % gas, liquid

for i = 1:numel(phases)
    phase = phases{i};
    unitSystems = fieldnames(files.(phase));  % SI, SE

    for j = 1:numel(unitSystems)
        unitSys  = unitSystems{j};
        filename = files.(phase).(unitSys);

        % ---- Verify file exists ----
        if ~isfile(filename)
            error("File not found: %s", filename);
        end

        % ---- OneDrive-safe sheet reader ----
        [~, sheets] = xlsfinfo(filename);
        if isempty(sheets)
            error("No sheets found in workbook: %s", filename);
        end

        for s = 1:numel(sheets)
            sheetNameRaw = sheets{s};
            fluidField   = matlab.lang.makeValidName(lower(sheetNameRaw));

            % Read with NO headers
            tbl = readtable(filename, ...
                "Sheet", sheetNameRaw, ...
                "ReadVariableNames", false);

            % Assign column names
            tbl.Properties.VariableNames = makePropNames(sheetNameRaw, width(tbl));

            DB.(phase).(unitSys).(fluidField) = tbl;
        end
    end
end
end

% ---------------- helper function ----------------
function names = makePropNames(sheetName, ncol)
% Returns the correct VariableNames cell array based on your rules.

s = lower(sheetName);
s = regexprep(s,'[^a-z0-9]','');   % normalize name

isWater   = contains(s,"water");
isR134a   = contains(s,"134a");
isPropane = contains(s,"propane");
isAmmonia = contains(s,"ammonia") || contains(s,"nh3");

% Saturation tables
if ncol == 14 && isWater
    names = {'T','Psat','rhol','rhov','hfg','Cpl','Cpv','kl','kv','mul','muv','Prl','Prv','beta'};
    return
end

if ncol == 15 && (isR134a || isPropane || isAmmonia)
    names = {'T','Psat','rhol','rhov','hfg','Cpl','Cpv','kl','kv','mul','muv','Prl','Prv','beta','sigma'};
    return
end

% Single-phase tables
if ncol == 9
    names = {'T','rho','Cp','k','alpha','mu','nu','Pr','beta'};
    return
elseif ncol == 8
    names = {'T','rho','Cp','k','alpha','mu','nu','Pr'};
    return
end

error("Sheet '%s' has %d columns (unexpected).", sheetName, ncol);
end