clear ; clc ;

DB = loadPropDB() ;
units = upper(strtrim(input('Unit System (SE/SI): ',"s"))) ;
phase = lower(strtrim(input('Phase of Analysis (Gas/Liquid/Liquid Metal): ',"s"))) ;
avail = fieldnames(DB.(phase).(units)); disp("Available fluids:"); disp(avail)
fluid = lower(strtrim(input('Fluid of Analysis (Gas/Liquid/Liquid Metal): ',"s"))) ;
switch phase
    case "liquid"
        p    = getProp(DB, phase, units, fluid, Tf, "l");
        mu = p.mu ; k = p.k ; Pr = p.Pr ; rho = p.rho ; nu = mu/rho ;
    case "liquid metal"
        p    = getProp(DB, phase, units, fluid, Tf, "l");
        mu = p.mu ; k = p.k ; Pr = p.Pr ; rho = p.rho ; nu = mu/rho ;
    case "gas"
        p    = getProp(DB, phase, units, fluid, Tf, "");
        mu = p.mu ; k = p.k ; Pr = p.Pr ; rho = p.rho ; nu = mu/rho ;
end
valid_main = false ;
while~valid_main
    type = lower(strtrim(input('Type of Convection (External/Internal/Natural): ',"s"))) ;
    switch type
        case "external"
            valid_main = true ;
            Tinf = input('Free Stream Temperature (C): ') ;
            Ts   = input('Surface Temperature (C): ') ;
            V    = input('Inlet Velocity (m/s): ') ;
            Tf   = (Tinf + Ts)/2 ;
            geometry = lower(strtrim(input('Geometry (flat plate/cylinder/sphere): ',"s"))) ;
            condition = lower(strtrim(input('Constant Condition (temperature/flux): ',"s"))) ;
            switch geometry
                case "flat plate"
                    Lc = input('Length of plate: ') ; 
                    csection = "" ;
                case "cylinder"
                    disp(sheetnames('Nusselt Constants Cylinder External Flow.xlsx')) ;
                    csection = lower(strtrim(input('Cross Section of Cylinder: \n',"s"))) ;
                    Lc = input('Characteristic length: ') ;
                case "sphere"
                    Lc = input('Sphere diameter: ') ;
                    csection = "" ;
            end
            Re = rho*V*Lc/mu ; 
            [Nu, Corr] = NuE(Re, Pr, condition, geometry, csection) ; 
            h  = Nu*k/Lc ;
        case "internal"
            valid_main = true ;
            Ti = input('Inlet Temperature (C): ') ;
            Ts   = input('Surface Temperature (C): ') ;
            V    = input('Inlet Velocity (m/s): ') ;
            state = lower(strtrim(input('Developing State of the Fluid (Developing/Developed): ',"s"))) ;
            csection = lower(strtrim(input('Cross Section of pipe: \n',"s"))) ;
            condition = lower(strtrim(input('Constant Condition (temperature/flux): ',"s"))) ;
            T = readtable("Roughness Internal Flow.xlsx",'ReadRowNames',true);
            T.Properties.RowNames
            material = lower(strtrim(input('Material of Pipe: '))) ;
            Tf = (Ts + Ti)/2 ;
            switch csection
                case ""
                case ""
            end
            Nu = NuI(Re, Pr, fluid, state, D, L, pipe, parameter,...
            condition, csection, surface, material, units) ;
            h  = Nu*k/Lc ;
        case "natural"
            Tf = (Te + Ti)/2 ;
            valid_main = true ;
        otherwise
            fprintf('\nInvalid type of convection. Input either External/Internal/Natural.\n\n') ;
    end
end

fprintf('\n') ; disp(Corr) ; fprintf('\n') ; 
fprintf('Nusselt Number: %.3f\n\n',Nu) ; 
fprintf('Convective Coefficient: %.3f W/(m^2*K)\n\n',h) ; 