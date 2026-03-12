clear ; clc ;

u = symunit ;
%% Input Calculation from Sprocket & Output Calculations

Pfin = 140 ; Pmin = 200 ; ts = 150 ; td = 15 ;
Pfin = Pfin * 0.737562 ; Pmin = Pmin * 0.737562 ;

wf = double(separateUnits(unitConvert(70*u.rpm,u.rad/u.s))) ;
wm = double(separateUnits(unitConvert(90*u.rpm,u.rad/u.s))) ;
Wt = 150 + 120 + 160 ; g = 32.174 ; Dw = 2 ; theta = 30 ; m = Wt/g ; f = 0.05 ;
th = 15 ; ws = double(separateUnits(unitConvert(75*u.rpm,u.rad/u.s))) ;
Tin = (Pfin + Pmin) / ws ; a = [0 0 0] ;
v = double(separateUnits(unitConvert([2.518  1.800  3.500]*u.m/u.s, u.ft/u.s))) ;

F = 1/2*[ ...
    f*Wt ; ...
    (m*a(2) + f*m*g*cosd(theta) + m*g*sind(theta)) ; ...
    (m*a(3) + m*g*sind(-theta)  - f*m*g*cosd(theta)) ] ;

Tout = 1/2*Dw*F ; Tout = 12*Tout ; Tin  = 12*Tin ;
eT = zeros(1,3) ; eTa = abs(Tout(3)/Tin) ; ew = zeros(1,3) ;
for i = 1:3
    eT(i) = Tout(i)/Tin ; ew(i) = 1/eT(i) ;
end
ew(3) = abs(ew(3)) ; ewg = ew/0.5 ;

%% Gears

ei = [ewg(1)^(1/2) ewg(2)^(1/2) ewg(3)] ; eta = 0.96 ;
Shaft = 3 ; Win = 75 ; Wshaft = zeros(1, Shaft); Wshaft(1) = Win;

psi  = [35 35 35];              % common helix angle (deg)
phin = [25 25 25];              % common normal pressure angle (deg)

phit = zeros(1,3);
for i = 1:3
    phit(i) = atand(tand(phin(i))/cosd(psi(i)));   % transverse pressure angle
end

k = 1;
m_req = 1./ei;                 % required ratio m = Ng/Np (since m = 1/ei in your definition)
Np_min = zeros(1,3);

fprintf('\n--- Minimum Pinion Teeth Required (Helical, COMMON psi/phin) ---\n');
fprintf('Stage   ei        m=1/ei     Np_min_required\n');

for i = 1:3
    Np_cont = (2*k*cosd(psi(i)))/((1 + 2*m_req(i)) * (sind(phit(i))^2)) * ...
              ( m_req(i) + sqrt( m_req(i)^2 + (1 + 2*m_req(i)) * (sind(phit(i))^2) ) );

    Np_min(i) = ceil(Np_cont);

    fprintf('%3d   %8.4f   %8.4f        %8d\n', ...
        i, ei(i), m_req(i), Np_min(i));
end
fprintf('\n');

GearG = cell(1,3); 
GearG{1} = cell(5,14); 
GearG{2} = cell(5,14); 
GearG{3} = cell(3,14);

headers = { ...
    'Gear','Shaft','Speed (rpm)','Torque (lbf*in)','Teeth','Virtual Teeth','Pitch Diameter (in)','Face Width (in)',...
    'Addendum (in)','Dedendum(in)', 'Base Diameter (in)','Root Diameter (in)',...
    'Helix Angle (deg)','Pressure Angle (deg)'} ;

GearG{1}(1,:) = headers; GearG{2}(1,:) = headers; GearG{3}(1,:) = headers;
for k = 1:3
    for r = 2:size(GearG{k},1)
        for c = 1:size(GearG{k},2)
            GearG{k}{r,c} = 0;
        end
    end
end

GearG{1}(2:5,1) = num2cell([1; 3; 4; 7]); GearG{1}(2:5,2) = num2cell([1; 2; 2; 3]);
GearG{2}(2:5,1) = num2cell([1; 3; 5; 8]); GearG{2}(2:5,2) = num2cell([1; 2; 2; 3]);
GearG{3}(2:3,1) = num2cell([2; 6]);       GearG{3}(2:3,2) = num2cell([1; 3]);

GearG{1}(2,4)   = {Tin};
GearG{1}(3:5,4) = num2cell([eta*Tin/ei(1); eta*Tin/ei(1); eta^2*Tin/ei(1)^2]);

GearG{2}(2,4)   = {Tin};
GearG{2}(3:5,4) = num2cell([eta*Tin/ei(2); eta*Tin/ei(2); eta^2*Tin/ei(2)^2]);

GearG{3}(2,4)   = {Tin};
GearG{3}(3,4)   = num2cell(eta*Tin/ei(3));

for k = 1:3
    Wshaft = zeros(1,3);
    Wshaft(1) = Win;
    if k == 1
        Wshaft(2) = Wshaft(1)*ei(1); Wshaft(3) = Wshaft(2)*ei(1);
    elseif k == 2
        Wshaft(2) = Wshaft(1)*ei(1); Wshaft(3) = Wshaft(2)*ei(2);
    else
        Wshaft(2) = Wshaft(1)*ei(1); Wshaft(3) = Wshaft(1)*ei(3);
    end
    for g = 2:size(GearG{k},1)
        GearG{k}{g,3} = Wshaft(GearG{k}{g,2});
    end
end

Pd = [8 10 8];

dp_min = 3.5;
dp_max = 11;

Pd1 = Pd(1); Pd2 = Pd(2); Pd3 = Pd(3);

N1_min = ceil(dp_min*Pd1);  N1_max = floor(dp_max*Pd1);
N5_min = ceil(dp_min*Pd2);  N5_max = floor(dp_max*Pd2);
N2_min = ceil(dp_min*Pd3);  N2_max = floor(dp_max*Pd3);

inRange = @(N, Pd_i) ((N/Pd_i) >= dp_min) && ((N/Pd_i) <= dp_max);

r13_target = ei(1);   % incline priority
r26_target = ei(3);

topK_stage1 = 60;   % keep best Stage1 candidates only (tight ratio)
topK_stage2 = 60;   % keep best Stage2 candidates per Stage1
topK_stage3 = 60;   % keep best Stage3 candidates per (S12,S23)

cand13 = [];  % rows: [G1 G3 Sum13 S12 r13 r13_err]
for G1_try = N1_min:N1_max
    for G3_try = N1_min:N1_max
        Sum13 = G1_try + G3_try;
        S12 = Sum13/(2*Pd1);
        r13 = G3_try/G1_try;
        r13_err = abs((r13 - r13_target)/r13_target);
        cand13 = [cand13; G1_try, G3_try, Sum13, S12, r13, r13_err];
    end
end
cand13 = sortrows(cand13, 6);                 % sort by r13_err
cand13 = cand13(1:min(topK_stage1,size(cand13,1)), :);

best = [];
bestScore = inf;

for ii = 1:size(cand13,1)

    G1_try = cand13(ii,1);
    G3_try = cand13(ii,2);
    S12    = cand13(ii,4);
    r13    = cand13(ii,5);
    r13_err= cand13(ii,6);
    G4_try = G1_try;
    G7_try = G3_try;

    r58_target = ewg(2)/r13;

    cand58 = []; % [G5 G8 Sum58 S23 r58 r58_err]
    for G5_try = N5_min:N5_max
        for G8_try = N5_min:N5_max
            Sum58 = G5_try + G8_try;
            S23 = Sum58/(2*Pd2);
            r58 = G8_try/G5_try;
            r58_err = abs((r58 - r58_target)/r58_target);
            cand58 = [cand58; G5_try, G8_try, Sum58, S23, r58, r58_err];
        end
    end
    cand58 = sortrows(cand58, 6);
    cand58 = cand58(1:min(topK_stage2,size(cand58,1)), :);

    for jj = 1:size(cand58,1)

        G5_try = cand58(jj,1);
        G8_try = cand58(jj,2);
        S23    = cand58(jj,4);
        r58    = cand58(jj,5);
        r58_err= cand58(jj,6);
        S13 = sqrt(S12^2 + S23^2);
        Sum26_cont = 2*Pd3*S13;
        Sum26 = round(Sum26_cont);

        if (Sum26 < 2*N2_min) || (Sum26 > 2*N2_max)
            continue
        end
        G2_cont = Sum26/(1 + r26_target);
        G2_seed = round(G2_cont);

        candG2 = (G2_seed-150):(G2_seed+150);
        candG2 = candG2(candG2 >= N2_min & candG2 <= N2_max);

        best_local = [];
        best_local_err = inf;

        for kk = 1:numel(candG2)
            G2_try = candG2(kk);
            G6_try = Sum26 - G2_try;

            if G6_try < N2_min || G6_try > N2_max
                continue
            end

            r26 = G6_try/G2_try;
            r26_err = abs((r26 - r26_target)/r26_target);

            if r26_err < best_local_err
                best_local_err = r26_err;
                best_local = [G2_try, G6_try, r26, r26_err];
            end
        end

        if isempty(best_local)
            continue
        end

        G2_try = best_local(1);
        G6_try = best_local(2);
        r26    = best_local(3);
        r26_err= best_local(4);

        S13_from_gears = (G2_try + G6_try)/(2*Pd3);
        tri_err = abs((S13_from_gears - S13)/S13);

        score = 1000*r13_err + 50*r26_err + 20*r58_err + 10*tri_err;

        if score < bestScore
            bestScore = score;
            best = [G1_try,G3_try,G4_try,G7_try,G5_try,G8_try,G2_try,G6_try, ...
                    S12,S23,S13_from_gears, r13,r58,r26, r58_target, r13_err,r58_err,r26_err, tri_err];
        end

    end
end

if isempty(best)
    fprintf('--- REDESIGN FAILED ---\n');
    fprintf('No feasible integer-tooth solution found inside dp bounds.\n\n');

    G1 = 28; G3 = 68; G4 = 28; G7 = 68;
    G5 = 54; G8 = 41;
    G2 = 48; G6 = 27;
else
    G1 = best(1); G3 = best(2); G4 = best(3); G7 = best(4);
    G5 = best(5); G8 = best(6);
    G2 = best(7); G6 = best(8);

    S12 = best(9);  S23 = best(10);  S13 = best(11);

    r13 = best(12); r58 = best(13); r26 = best(14);
    r58_target = best(15);

    r13_err = best(16); r58_err = best(17); r26_err = best(18);
    tri_err = best(19);

    fprintf('--- REDESIGN SUCCESS (Incline Priority) ---\n');
    fprintf('Stage1: G1=%d G3=%d | r13=%.6f target=%.6f err=%.3f %%\n', ...
        G1, G3, r13, r13_target, 100*(r13-r13_target)/r13_target);

    fprintf('Stage2: G5=%d G8=%d | r58=%.6f target=%.6f err=%.3f %%\n', ...
        G5, G8, r58, r58_target, 100*(r58-r58_target)/r58_target);

    fprintf('Stage3: G2=%d G6=%d | r26=%.6f target=%.6f err=%.3f %%\n\n', ...
        G2, G6, r26, r26_target, 100*(r26-r26_target)/r26_target);

    S12_check = (G1+G3)/(2*Pd1);
    S23_check = (G5+G8)/(2*Pd2);
    S13_check = (G2+G6)/(2*Pd3);
    S13_tri   = sqrt(S12_check^2 + S23_check^2);

    fprintf('Center distances (in):\n');
    fprintf('S12 = (G1+G3)/(2*Pd1) = %.4f\n', S12_check);
    fprintf('S23 = (G5+G8)/(2*Pd2) = %.4f\n', S23_check);
    fprintf('S13 = (G2+G6)/(2*Pd3) = %.4f\n', S13_check);
    fprintf('Triangle: sqrt(S12^2 + S23^2) = %.4f\n', S13_tri);
    fprintf('Triangle error = %.4f %%\n\n', 100*(S13_check - S13_tri)/S13_tri);

    fprintf('Pitch diameters (in): d1=%.3f d3=%.3f d5=%.3f d8=%.3f d2=%.3f d6=%.3f\n\n', ...
        G1/Pd1, G3/Pd1, G5/Pd2, G8/Pd2, G2/Pd3, G6/Pd3);
end

Np_act = zeros(1,3);

Np_act(1) = min(G1,G3);
Np_act(2) = min(G5,G8);
Np_act(3) = min(G2,G6);

OK = (Np_act >= Np_min);

fprintf('\n--- Interference Check (Helical, COMMON psi/phin) ---\n');
fprintf('Stage   ei        m=1/ei     Np_min(req)  Np_actual   Status\n');
for i = 1:3
    status = 'FAIL';
    if OK(i), status = 'PASS'; end
    fprintf('%3d   %8.4f   %8.4f     %8d     %8d     %s\n', ...
        i, ei(i), m_req(i), Np_min(i), Np_act(i), status);
end
fprintf('\n');


GearG{1}{2,5} = G1;   % Gear 1
GearG{1}{3,5} = G3;   % Gear 3
GearG{1}{4,5} = G4;   % Gear 4
GearG{1}{5,5} = G7;   % Gear 7

for r = 2:size(GearG{1},1)
    N_actual = GearG{1}{r,5};
    GearG{1}{r,6} = floor(N_actual/(cosd(psi(1))^3));   % Virtual Teeth
end

GearG{2}{2,5} = G1;   % Gear 1
GearG{2}{3,5} = G3;   % Gear 3
GearG{2}{4,5} = G5;   % Gear 5
GearG{2}{5,5} = G8;   % Gear 8

for r = 2:size(GearG{2},1)
    N_actual = GearG{2}{r,5};
    GearG{2}{r,6} = floor(N_actual/(cosd(psi(2))^3));   % Virtual Teeth
end

GearG{3}{2,5} = G2;   % Gear 2
GearG{3}{3,5} = G6;   % Gear 6

for r = 2:size(GearG{3},1)
    N_actual = GearG{3}{r,5};
    GearG{3}{r,6} = floor(N_actual/(cosd(psi(3))^3));   % Virtual Teeth
end

colPsi = 13; 
colPhi = 14;

for k = 1:3
    for r = 2:size(GearG{k},1)
        if (k == 2) && (r == 2 || r == 3)
            % Stage 2 Gear1/Gear3 are the SAME physical gears as Stage 1
            GearG{k}{r,colPsi} = psi(1);
            GearG{k}{r,colPhi} = phin(1);
        else
            GearG{k}{r,colPsi} = psi(k);
            GearG{k}{r,colPhi} = phin(k);
        end
    end
end


S13_stage1 = (G1 + G3)/(2*Pd(1));   % S12
S47_stage1 = (G4 + G7)/(2*Pd(1));   % S12 mirror
S58_stage2 = (G5 + G8)/(2*Pd(2));   % S23
S26_stage3 = (G2 + G6)/(2*Pd(3));   % S13
S12_from_stage1  = S13_stage1;
S12_from_stage1b = S47_stage1;
S23_from_stage2  = S58_stage2;
S13_from_stage3  = S26_stage3;

S13_from_triangle = sqrt(S12_from_stage1^2 + S23_from_stage2^2);

err_S12_pair_pct = (S12_from_stage1b - S12_from_stage1)/S12_from_stage1 * 100;
err_triangle_pct = (S13_from_stage3 - S13_from_triangle)/S13_from_triangle * 100;

fprintf('--- Shaft Center Distance Check (Triangle Layout) ---\n');
fprintf('Pair      From Mesh         Formula                          Value (in)\n');
fprintf('S12       Stage1 G1-G3      (G1+G3)/(2*Pd1)                   %8.4f\n', S12_from_stage1);
fprintf('S12       Stage1 G4-G7      (G4+G7)/(2*Pd1)                   %8.4f\n', S12_from_stage1b);
fprintf('S23       Stage2 G5-G8      (G5+G8)/(2*Pd2)                   %8.4f\n', S23_from_stage2);
fprintf('S13       Stage3 G2-G6      (G2+G6)/(2*Pd3)                   %8.4f\n', S13_from_stage3);
fprintf('S13       Triangle check    sqrt(S12^2 + S23^2)               %8.4f\n', S13_from_triangle);

fprintf('\nErrors:\n');
fprintf('S12 pair consistency (G1-G3 vs G4-G7): % .4f %%\n', err_S12_pair_pct);
fprintf('Triangle consistency (Stage3 vs sqrt): % .4f %%\n\n', err_triangle_pct);

if abs(err_S12_pair_pct) > 1e-6
    fprintf('WARNING: S12 mismatch between the two Shaft1-Shaft2 meshes.\n');
end
if abs(err_triangle_pct) > 0.5
    fprintf('WARNING: Triangle mismatch > 0.5%% (Stage3 center distance not matching triangle).\n');
end

for k = 1:3
    for r = 2:size(GearG{k},1)
        N_actual = GearG{k}{r,5};
        GearG{k}{r,7} = N_actual/Pd(k);     % Pitch Diameter (in)
    end
end

GearG{2}{2,6} = GearG{1}{2,6};   % Virtual Teeth (Gear 1)
GearG{2}{3,6} = GearG{1}{3,6};   % Virtual Teeth (Gear 3)
GearG{2}{2,7} = GearG{1}{2,7};   % Pitch Diameter (Gear 1)
GearG{2}{3,7} = GearG{1}{3,7};   % Pitch Diameter (Gear 3)

for k = 1:3
    Fw  = 12/Pd(k);
    Fw  = min(1.50, max(0.75, Fw));

    add = 1/Pd(k);
    ded = 1.25/Pd(k);

    for r = 2:size(GearG{k},1)
        dp = GearG{k}{r,7};      % Pitch diameter (in)

        GearG{k}{r,8}  = Fw;                 % Face Width (in)
        GearG{k}{r,9}  = add;                % Addendum (in)
        GearG{k}{r,10} = ded;                % Dedendum (in)
        GearG{k}{r,11} = dp * cosd(phit(k)); % Base Diameter (in)
        GearG{k}{r,12} = dp - 2*ded;         % Root Diameter (in)
    end
end

GearF = cell(1,3); GearF{1} = cell(5,7); GearF{2} = cell(5,7); GearF{3} = cell(3,7);
headers = { ...
    'Gear','Shaft','Speed (rpm)','Torque (lbf*in)','Tangential Force (lbf)',...
    'Radial Force (lbf)','Axial Force (lbf)'} ;
GearF{1}(1,:) = headers; GearF{2}(1,:) = headers; GearF{3}(1,:) = headers;

for k = 1:3
    for r = 2:size(GearF{k},1)
        for c = 1:size(GearF{k},2)
            GearF{k}{r,c} = 0;
        end
    end
end

GearF{1}(2:5,1) = num2cell([1; 3; 4; 7]); GearF{1}(2:5,2) = num2cell([1; 2; 2; 3]);
GearF{1}(2,4)   = {Tin};
GearF{1}(3:5,4) = num2cell([eta*Tin/ei(1); eta*Tin/ei(1); eta^2*Tin/ei(1)^2]);

GearF{2}(2:5,1) = num2cell([1; 3; 5; 8]); GearF{2}(2:5,2) = num2cell([1; 2; 2; 3]);
GearF{2}(2,4)   = {Tin};
GearF{2}(3:5,4) = num2cell([eta*Tin/ei(2); eta*Tin/ei(2); eta^2*Tin/ei(2)^2]);

GearF{3}(2:3,1) = num2cell([2; 6]); GearF{3}(2:3,2) = num2cell([1; 3]);
GearF{3}(2,4)   = {Tin}; GearF{3}(3,4) = num2cell(eta*Tin/ei(3));

for k = 1:3
    Wshaft = zeros(1,3);
    Wshaft(1) = Win;
    if k == 1
        Wshaft(2) = Wshaft(1)*ei(1); Wshaft(3) = Wshaft(2)*ei(1);
    elseif k == 2
        Wshaft(2) = Wshaft(1)*ei(1); Wshaft(3) = Wshaft(2)*ei(2);
    else
        Wshaft(2) = Wshaft(1)*ei(1); Wshaft(3) = Wshaft(1)*ei(3);
    end
    for g = 2:size(GearF{k},1)
        GearF{k}{g,3} = Wshaft(GearF{k}{g,2});
    end
end

for k = 1:3
    T = cell2mat(GearF{k}(2:end,4));
    d = cell2mat(GearG{k}(2:end,7));
    Ft = zeros(size(T));
    Ft(1) = 2*T(1)/d(1);
    Ft(2) = Ft(1);
    if numel(T) >= 4
        Ft(3) = 2*T(3)/d(3);
        Ft(4) = Ft(3);
    end
    GearF{k}(2:end,5) = num2cell(Ft);
    GearF{k}(2:end,6) = num2cell(Ft.*tand(phit(k)));
    GearF{k}(2:end,7) = num2cell(Ft.*tand(psi(k)));
end

for k = 1:3
    Fw  = 12/Pd(k); Fw = min(1.50, max(0.75, Fw));
    add = 1/Pd(k);  ded = 1.25/Pd(k);

    for r = 2:size(GearG{k},1)
        if (k == 2) && (r == 2 || r == 3)
            GearG{k}{r,8}  = GearG{1}{r,8};
            GearG{k}{r,9}  = GearG{1}{r,9};
            GearG{k}{r,10} = GearG{1}{r,10};
            GearG{k}{r,11} = GearG{1}{r,11};
            GearG{k}{r,12} = GearG{1}{r,12};
            continue
        end
        dp = GearG{k}{r,7};
        GearG{k}{r,8}  = Fw;
        GearG{k}{r,9}  = add;
        GearG{k}{r,10} = ded;
        GearG{k}{r,11} = dp * cosd(phit(k));
        GearG{k}{r,12} = dp - 2*ded;
    end
end

r58_actual = G8/G5;                 % actual stage-2 mesh ratio used in your optimizer
T_shaft2   = eta*Tin/ei(1);         % torque on shaft 2 after Stage 1 mesh (Gear 3)
GearG{2}{2,4} = Tin;                % Gear 1 torque (input)
GearG{2}{3,4} = T_shaft2;           % Gear 3 torque (shaft 2)
GearG{2}{4,4} = T_shaft2;           % Gear 5 torque (same shaft 2)
GearG{2}{5,4} = eta*T_shaft2/r58_actual;   % Gear 8 torque using actual ratio + mesh efficiency

GearF{2}{2,4} = Tin;
GearF{2}{3,4} = T_shaft2;
GearF{2}{4,4} = T_shaft2;
GearF{2}{5,4} = eta*T_shaft2/r58_actual;

for i = 1:3
    fprintf('Gear Geometry at Stage %d\n\n',i) ;
    TG = cell2table(GearG{i}(2:end,:), 'VariableNames', GearG{i}(1,:));
    disp(TG)
    fprintf('\n\n') ;
end

for i = 1:3
    fprintf('Gear Forces at Stage %d\n\n',i) ;
    TF = cell2table(GearF{i}(2:end,:), 'VariableNames', GearF{i}(1,:));
    disp(TF)
    fprintf('\n\n')
end

%% Failure Analysis

% Material Chosen: AISI 1050 Cold Drawn
% Strenghts in ksi
% Assumptions: a) dhole/D = 0.1 b) rfillet = 0.02ds
Ls1 = 0.3125 ; Ls2 = 0.188 ; Lh1 = 1.5 ; Lh2 = 1.5 ; Lb1 = 1.5 ; Lb2 = 1.5 ;
Sut = 100e3 ; Sy = 84e3 ; nd = 2 ; Set = Sut/2 ; Sut_ksi = Sut/1000 ;
ka = 2*(Sut_ksi)^(-0.217) ; kc = 1 ; kd = 1 ; ke = 0.702 ;
Kt = 5 ; Kts = 3 ;
sqrab = 0.246 - 3.08e-3*Sut_ksi + 1.51e-5*Sut_ksi^2 - 2.67e-8*Sut_ksi^3 ;
sqrat = 0.190 - 2.51e-3*Sut_ksi + 1.35e-5*Sut_ksi^2 - 2.67e-8*Sut_ksi^3 ;
d1 = zeros(1,5) ; d1(1) = 0.001 ; I1 = zeros(1,5) ;
valid = false ;

Ft = zeros(3,4,1); Fr = zeros(3,4,1); Fa = zeros(3,4,1);
for i = 1:3
    ft = cell2mat(GearF{i}(2:end,5)).'; Ft(i,:,1) = [ft zeros(1,4-numel(ft))];
    fr = cell2mat(GearF{i}(2:end,6)).'; Fr(i,:,1) = [fr zeros(1,4-numel(fr))];
    fa = cell2mat(GearF{i}(2:end,7)).'; Fa(i,:,1) = [fa zeros(1,4-numel(fa))];
end
Fc = [2 4] ; Tc = [225.38 157.77] ;
syms B12x B12y B12z B11x B11y B11z B22x B22y B22z B21x B21y B21z ...
     B32x B32y B32z B31x B31y B31z
Eq = sym(zeros(6,1,3));

while~valid
    r1 = 0.02*d1 ;
    I1(1) = pi*d1(1)^4/64 ;
    for i = 2:5
        d1(i) = 1.15*d1(i-1) ;
        r1 = 0.02*d1 ;
        I1(i) = pi*d1(i)^4/64 ;
    end

    x10 = 0 ; x11 = Lb1/2 + r1(1) ; x12 = x11 + 2 + Ls1/2 ;
    x13 = x12 + Ls1/2 + 3 + r1(2) ; x14 = x13 + 9 + Ls2/2 ; x15 = x14 + Ls2/2 + 3 + r1(3);
    x16 = x15 + 12 + Lh1/2 ; x17 = x16 + Lh1/2 + 3 + r1(4) ; x18 = x17 + 5.75 ; x19 = x18 + r1(2) + 3 + Lh2/2 ;
    x110 = x19 + Lh2/2 + 2 ; x111 = x110 + r1(1) + Lb2/2 ;

    Eq(1,1,1) = B11y + B12y - Fc(1) - Fc(2) - Ft(1,1,1)*cosd(45) - Fr(1,1,1)*cosd(45) == 0 ;
    Eq(2,1,1) = -Fc(1)*x12 - Fc(2)*x14 - (Ft(1,1,1)*cosd(45) + Fr(1,1,1)*cosd(45))*x16 + B12y*x111 == 0 ;
    Eq(3,1,1) = B11z + B12z - Ft(1,1,1)*sind(45) + Fr(1,1,1)*sind(45) == 0 ;
    Eq(4,1,1) = (Fr(1,1,1)*sind(45) - Ft(1,1,1)*sind(45))*x16 + B12z*x111 == 0 ;
    Eq(5,1,1) = B11x + B12x + Fa(1,1,1) == 0 ; Eq(6,1,1) = B11x == 0 ;

    C1 = solve(Eq(1:6,1,1) , [B11x B12x B11y B12y B11z B12z]) ;

    Eq(1,1,2) = B21y + B22y - Fc(1) - Fc(2) - Ft(2,1,1)*cosd(45) - Fr(2,1,1)*cosd(45) == 0 ;
    Eq(2,1,2) = -Fc(1)*x12 - Fc(2)*x14 - (Ft(2,1,1)*cosd(45) + Fr(2,1,1)*cosd(45))*x16 + B22y*x111 == 0 ;
    Eq(3,1,2) = B21z + B22z - Ft(2,1,1)*sind(45) + Fr(2,1,1)*sind(45) == 0 ;
    Eq(4,1,2) = (Fr(2,1,1)*sind(45) - Ft(2,1,1)*sind(45))*x16 + B22z*x111 == 0 ;
    Eq(5,1,2) = B21x + B22x + Fa(2,1,1) == 0 ; Eq(6,1,2) = B21x == 0 ;

    C2 = solve(Eq(1:6,1,2) , [B21x B22x B22y B22z B21y B21z]) ;

    Eq(1,1,3) = B31y + B32y - Fc(1) - Fc(2) - Ft(3,1,1)*cosd(45) - Fr(3,1,1)*cosd(45) - Fr(3,3,1) == 0 ;
    Eq(2,1,3) = -Fc(1)*x12 - Fc(2)*x14 - (Ft(3,1,1)*cosd(45) + Fr(3,1,1)*cosd(45))*x16 - Fr(3,3,1)*x19 + B32y*x111 == 0 ;
    Eq(3,1,3) = B31z + B32z - Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45) + Ft(3,3,1) == 0 ;
    Eq(4,1,3) = (Fr(3,1,1)*sind(45) - Ft(3,1,1)*sind(45))*x16 + Ft(3,3,1)*x19 + B32z*x111 == 0 ;
    Eq(5,1,3) = B31x + B32x + Fa(3,1,1) + Fa(3,3,1) == 0 ; Eq(6,1,3) = B31x == 0 ;

    C3 = solve(Eq(1:6,1,3) , [B31x B32x B32y B32z B31y B31z]) ;

    x = [x10 x12 x14 x16 x19 x111];
    My11 = C1.B11y.*x; My12 = C1.B11y.*x - Fc(1).*(x - x12);
    My13 = C1.B11y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14);
    My14 = C1.B11y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
            - (Ft(1,1,1)*cosd(45) + Fr(1,1,1)*cosd(45)).*(x - x16);
    Mz11 = C1.B11z.*x; Mz12 = C1.B11z.*x + (-Ft(1,1,1)*sind(45) + Fr(1,1,1)*sind(45)).*(x - x16);
    My21 = C2.B21y.*x; My22 = C2.B21y.*x - Fc(1).*(x - x12);
    My23 = C2.B21y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14);
    My24 = C2.B21y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
            - (Ft(2,1,1)*cosd(45) + Fr(2,1,1)*cosd(45)).*(x - x16);
    Mz21 = C2.B21z.*x;
    Mz22 = C2.B21z.*x + (-Ft(2,1,1)*sind(45) + Fr(2,1,1)*sind(45)).*(x - x16);
    My11 = My11.*(x>=x10 & x<x12) + My12.*(x>=x12 & x<x14) + My13.*(x>=x14 & x<x16) + My14.*(x>=x16 & x<=x111);
    Mz11 = Mz11.*(x>=x10 & x<x16) + Mz12.*(x>=x16 & x<=x111);
    My21 = My21.*(x>=x10 & x<x12) + My22.*(x>=x12 & x<x14) + My23.*(x>=x14 & x<x16) + My24.*(x>=x16 & x<=x111);
    Mz21 = Mz21.*(x>=x10 & x<x16) + Mz22.*(x>=x16 & x<=x111);
    My31 = C3.B31y.*x;  My32 = C3.B31y.*x - Fc(1).*(x - x12);
    My33 = C3.B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14);
    My34 = C3.B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
            - (Ft(3,1,1)*cosd(45) + Fr(3,1,1)*cosd(45)).*(x - x16);
    My35 = C3.B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
            - (Ft(3,1,1)*cosd(45) + Fr(3,1,1)*cosd(45)).*(x - x16) ...
            - Fr(3,3,1).*(x - x19);
    Mz31 = C3.B31z.*x; Mz32 = C3.B31z.*x + (-Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45)).*(x - x16);
    Mz33 = C3.B31z.*x + (-Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45)).*(x - x16) ...
            + Ft(3,3,1).*(x - x19);
    My31 = My31.*(x>=x10 & x<x12) + My32.*(x>=x12 & x<x14) + My33.*(x>=x14 & x<x16) ...
         + My34.*(x>=x16 & x<x19) + My35.*(x>=x19 & x<=x111);
    Mz31 = Mz31.*(x>=x10 & x<x16) + Mz32.*(x>=x16 & x<x19) + Mz33.*(x>=x19 & x<=x111);
    Mr1 = hypot(My11,Mz11); Mr2 = hypot(My21,Mz21); Mr3 = hypot(My31,Mz31);
    [Mmax1, idx1] = max(abs(Mr1)); [Mmax2, idx2] = max(abs(Mr2)); [Mmax3, idx3] = max(abs(Mr3));
    xMmax1 = x(idx1); xMmax2 = x(idx2); xMmax3 = x(idx3); Mmax = max([Mmax1, Mmax2, Mmax3]) ;

    Tg1 = [GearF{1}{2,4} GearF{2}{2,4}] ; Tg2 = GearF{3}{2,4} ; Tc = [255.38 157.77] ;
    Ti = Tc(1)+Tc(2) ; Tn = [Tc(1)+Tc(2)-Tg1(1) Tc(1)+Tc(2)-Tg1(2) Tc(1)+Tc(2)-Tg2] ;
    xx  = [0     x12     x12     x14     x14    x16     x16] ;
    T1  = [0     0       Tc(1)   Tc(1)   Ti    Tn(1)    0] ;
    T2  = [0     0       Tc(1)   Tc(1)   Ti    Tn(2)    0] ;
    xxx = [0     x12     x12     x14     x14    x19     x19] ;
    T3  = [0     0       Tc(1)   Tc(1)   Ti    Tn(3)    0] ;

    Mm = 0 ; Ma = Mmax ; Ta = 0 ; Tm = Ti ;

    if d1(1) < 0.3
        kb = 1 ;
    elseif d1(1) >= 0.3 && d1(1) <= 2
        kb = (d1(1)/0.3)^(-0.107) ;
    elseif d1(1) > 2 && d1(1) <= 10
        kb = 0.91*d1(1)^(-0.157) ;
    else
    end

    rhole = 0.025*d1(1) ;
    Se = Set*ka*kb*kc*kd*ke ;
    Kf = 1 + (Kt - 1)/(1+sqrab/sqrt(rhole)) ;
    Kfs = 1 + (Kts - 1)/(1+sqrat/sqrt(rhole)) ;
    A = sqrt(4*(Kf*Ma)^2+3*(Kfs*Ta)^2) ;
    B = sqrt(4*(Kf*Mm)^2+3*(Kfs*Tm)^2) ;
    n = (pi*d1(1)^3)/(16*(A/Se + B/Sut)) ;

    if n >= nd
        valid = true ;
    else
        d1(1) = d1(1) + .01 ;
    end

    if d1(1) > 10
        error('Diameter search exceeded 10 in; check units/loads/fits.') ;
    end
end


%% Final Equilibrium Analysis

% Critical Diameters: 2.4010 in 2.7611 in 3.1753 in 3.6516 in 4.1994 in
% For manufacturing Purposes: 2.5 2.625 2.75 3 3.5

d1 = [2.5 2.625 2.75 3 3.5 ] ; r1 = zeros(1,5) ;
    for i = 2:5
        r1(i) = 0.02*d1(i) ;
        I1(i) = pi*d1(i)^4/64 ;
    end

    x10 = 0 ; x11 = Lb1/2 + r1(1) ; x12 = x11 + 2 + Ls1/2 ;
    x13 = x12 + Ls1/2 + 3 + r1(2) ; x14 = x13 + 9 + Ls2/2 ; x15 = x14 + Ls2/2 + 3 + r1(3);
    x16 = x15 + 12 + Lh1/2 ; x17 = x16 + Lh1/2 + 3 + r1(4) ; x18 = x17 + 5.75 ; x19 = x18 + r1(2) + 3 + Lh2/2 ;
    x110 = x19 + Lh2/2 + 2 ; x111 = x110 + r1(1) + Lb2/2 ;

    Eq(1,1,1) = B11y + B12y - Fc(1) - Fc(2) - Ft(1,1,1)*cosd(45) - Fr(1,1,1)*cosd(45) == 0 ;
    Eq(2,1,1) = -Fc(1)*x12 - Fc(2)*x14 - (Ft(1,1,1)*cosd(45) + Fr(1,1,1)*cosd(45))*x16 + B12y*x111 == 0 ;
    Eq(3,1,1) = B11z + B12z - Ft(1,1,1)*sind(45) + Fr(1,1,1)*sind(45) == 0 ;
    Eq(4,1,1) = (Fr(1,1,1)*sind(45) - Ft(1,1,1)*sind(45))*x16 + B12z*x111 == 0 ;
    Eq(5,1,1) = B11x + B12x + Fa(1,1,1) == 0 ; Eq(6,1,1) = B11x == 0 ;

    C1 = solve(Eq(1:6,1,1) , [B11x B12x B11y B12y B11z B12z]) ;

    Eq(1,1,2) = B21y + B22y - Fc(1) - Fc(2) - Ft(2,1,1)*cosd(45) - Fr(2,1,1)*cosd(45) == 0 ;
    Eq(2,1,2) = -Fc(1)*x12 - Fc(2)*x14 - (Ft(2,1,1)*cosd(45) + Fr(2,1,1)*cosd(45))*x16 + B22y*x111 == 0 ;
    Eq(3,1,2) = B21z + B22z - Ft(2,1,1)*sind(45) + Fr(2,1,1)*sind(45) == 0 ;
    Eq(4,1,2) = (Fr(2,1,1)*sind(45) - Ft(2,1,1)*sind(45))*x16 + B22z*x111 == 0 ;
    Eq(5,1,2) = B21x + B22x + Fa(2,1,1) == 0 ; Eq(6,1,2) = B21x == 0 ;

    C2 = solve(Eq(1:6,1,2) , [B21x B22x B22y B22z B21y B21z]) ;

    Eq(1,1,3) = B31y + B32y - Fc(1) - Fc(2) - Ft(3,1,1)*cosd(45) - Fr(3,1,1)*cosd(45) - Fr(3,3,1) == 0 ;
    Eq(2,1,3) = -Fc(1)*x12 - Fc(2)*x14 - (Ft(3,1,1)*cosd(45) + Fr(3,1,1)*cosd(45))*x16 - Fr(3,3,1)*x19 + B32y*x111 == 0 ;
    Eq(3,1,3) = B31z + B32z - Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45) + Ft(3,3,1) == 0 ;
    Eq(4,1,3) = (Fr(3,1,1)*sind(45) - Ft(3,1,1)*sind(45))*x16 + Ft(3,3,1)*x19 + B32z*x111 == 0 ;
    Eq(5,1,3) = B31x + B32x + Fa(3,1,1) + Fa(3,3,1) == 0 ; Eq(6,1,3) = B31x == 0 ;

    C3 = solve(Eq(1:6,1,3) , [B31x B32x B32y B32z B31y B31z]) ;

x = linspace(0,x111,800);
Vy11 = C1.B11y; Vy12 = C1.B11y - Fc(1); Vy13 = C1.B11y - Fc(1) - Fc(2);
Vy14 = C1.B11y - Fc(1) - Fc(2) - Ft(1,1,1)*cosd(45) - Fr(1,1,1)*cosd(45);

My11 = C1.B11y.*x; My12 = C1.B11y.*x - Fc(1).*(x - x12);
My13 = C1.B11y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14);
My14 = C1.B11y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
        - (Ft(1,1,1)*cosd(45) + Fr(1,1,1)*cosd(45)).*(x - x16);

Vz11 = C1.B11z; Vz12 = C1.B11z - Ft(1,1,1)*sind(45) + Fr(1,1,1)*sind(45);
Mz11 = C1.B11z.*x; Mz12 = C1.B11z.*x + (-Ft(1,1,1)*sind(45) + Fr(1,1,1)*sind(45)).*(x - x16);

Vy21 = C2.B21y; Vy22 = C2.B21y - Fc(1); Vy23 = C2.B21y - Fc(1) - Fc(2);
Vy24 = C2.B21y - Fc(1) - Fc(2) - Ft(2,1,1)*cosd(45) - Fr(2,1,1)*cosd(45);

My21 = C2.B21y.*x; My22 = C2.B21y.*x - Fc(1).*(x - x12);
My23 = C2.B21y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14);
My24 = C2.B21y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
        - (Ft(2,1,1)*cosd(45) + Fr(2,1,1)*cosd(45)).*(x - x16);

Vz21 = C2.B21z; Vz22 = C2.B21z - Ft(2,1,1)*sind(45) + Fr(2,1,1)*sind(45);
Mz21 = C2.B21z.*x;
Mz22 = C2.B21z.*x + (-Ft(2,1,1)*sind(45) + Fr(2,1,1)*sind(45)).*(x - x16);

Vy11 = Vy11.*(x>=x10 & x<x12) + Vy12.*(x>=x12 & x<x14) + Vy13.*(x>=x14 & x<x16) + Vy14.*(x>=x16 & x<=x111);
My11 = My11.*(x>=x10 & x<x12) + My12.*(x>=x12 & x<x14) + My13.*(x>=x14 & x<x16) + My14.*(x>=x16 & x<=x111);

Vz11 = Vz11.*(x>=x10 & x<x16) + Vz12.*(x>=x16 & x<=x111);
Mz11 = Mz11.*(x>=x10 & x<x16) + Mz12.*(x>=x16 & x<=x111);

Vy21 = Vy21.*(x>=x10 & x<x12) + Vy22.*(x>=x12 & x<x14) + Vy23.*(x>=x14 & x<x16) + Vy24.*(x>=x16 & x<=x111);
My21 = My21.*(x>=x10 & x<x12) + My22.*(x>=x12 & x<x14) + My23.*(x>=x14 & x<x16) + My24.*(x>=x16 & x<=x111);

Vz21 = Vz21.*(x>=x10 & x<x16) + Vz22.*(x>=x16 & x<=x111);
Mz21 = Mz21.*(x>=x10 & x<x16) + Mz22.*(x>=x16 & x<=x111);

Vy31 = C3.B31y; Vy32 = C3.B31y - Fc(1); Vy33 = C3.B31y - Fc(1) - Fc(2);
Vy34 = C3.B31y - Fc(1) - Fc(2) - Ft(3,1,1)*cosd(45) - Fr(3,1,1)*cosd(45);
Vy35 = C3.B31y - Fc(1) - Fc(2) - Ft(3,1,1)*cosd(45) - Fr(3,1,1)*cosd(45) - Fr(3,3,1);

My31 = C3.B31y.*x;  My32 = C3.B31y.*x - Fc(1).*(x - x12);
My33 = C3.B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14);
My34 = C3.B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
        - (Ft(3,1,1)*cosd(45) + Fr(3,1,1)*cosd(45)).*(x - x16);
My35 = C3.B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) ...
        - (Ft(3,1,1)*cosd(45) + Fr(3,1,1)*cosd(45)).*(x - x16) ...
        - Fr(3,3,1).*(x - x19);

Vz31 = C3.B31z;  Vz32 = C3.B31z - Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45);
Vz33 = C3.B31z - Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45) + Ft(3,3,1);

Mz31 = C3.B31z.*x; Mz32 = C3.B31z.*x + (-Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45)).*(x - x16);
Mz33 = C3.B31z.*x + (-Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45)).*(x - x16) ...
        + Ft(3,3,1).*(x - x19);

Vy31 = Vy31.*(x>=x10 & x<x12) + Vy32.*(x>=x12 & x<x14) + Vy33.*(x>=x14 & x<x16) ...
     + Vy34.*(x>=x16 & x<x19) + Vy35.*(x>=x19 & x<=x111);

My31 = My31.*(x>=x10 & x<x12) + My32.*(x>=x12 & x<x14) + My33.*(x>=x14 & x<x16) ...
     + My34.*(x>=x16 & x<x19) + My35.*(x>=x19 & x<=x111);

Vz31 = Vz31.*(x>=x10 & x<x16) + Vz32.*(x>=x16 & x<x19) + Vz33.*(x>=x19 & x<=x111);
Mz31 = Mz31.*(x>=x10 & x<x16) + Mz32.*(x>=x16 & x<x19) + Mz33.*(x>=x19 & x<=x111);
Mr1 = hypot(My11,Mz11); Vr1 = hypot(Vy11,Vz11);
Mr2 = hypot(My21,Mz21); Vr2 = hypot(Vy21,Vz21);
Mr3 = hypot(My31,Mz31); Vr3 = hypot(Vy31,Vz31);
xS  = [0, x, x111];

Vy11S = [0, Vy11, 0];  Vz11S = [0, Vz11, 0];  Vr1S = [0, Vr1, 0];
Vy21S = [0, Vy21, 0];  Vz21S = [0, Vz21, 0];  Vr2S = [0, Vr2, 0];
Vy31S = [0, Vy31, 0];  Vz31S = [0, Vz31, 0];  Vr3S = [0, Vr3, 0];

figure('Name','1st Gear','NumberTitle','off')
tiledlayout(3,2,'TileSpacing','compact','Padding','compact')
nexttile; plot(x,My11,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M_y (lbf*in)'); title('M_y'); xlim([0 x111])
nexttile; stairs(xS,Vy11S,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V_y (lbf)');   title('V_y'); xlim([0 x111])
nexttile; plot(x,Mz11,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M_z (lbf*in)'); title('M_z'); xlim([0 x111])
nexttile; stairs(xS,Vz11S,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V_z (lbf)');   title('V_z'); xlim([0 x111])
nexttile; plot(x,Mr1 ,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M (lbf*in)'); title('M');   xlim([0 x111])
nexttile; stairs(xS,Vr1S ,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V (lbf)');   title('V'); xlim([0 x111])

figure('Name','2nd Gear','NumberTitle','off')
tiledlayout(3,2,'TileSpacing','compact','Padding','compact')
nexttile; plot(x,My21,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M_y (lbf*in)'); title('M_y'); xlim([0 x111])
nexttile; stairs(xS,Vy21S,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V_y (lbf)');   title('V_y'); xlim([0 x111])
nexttile; plot(x,Mz21,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M_z (lbf*in)'); title('M_z'); xlim([0 x111])
nexttile; stairs(xS,Vz21S,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V_z (lbf)');   title('V_z'); xlim([0 x111])
nexttile; plot(x,Mr2 ,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M (lbf*in)'); title('M');   xlim([0 x111])
nexttile; stairs(xS,Vr2S ,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V (lbf)');   title('V'); xlim([0 x111])

figure('Name','3rd Gear','NumberTitle','off')
tiledlayout(3,2,'TileSpacing','compact','Padding','compact')
nexttile; plot(x,My31,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M_y (lbf*in)'); title('M_y'); xlim([0 x111])
nexttile; stairs(xS,Vy31S,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V_y (lbf)');   title('V_y'); xlim([0 x111])
nexttile; plot(x,Mz31,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M_z (lbf*in)'); title('M_z'); xlim([0 x111])
nexttile; stairs(xS,Vz31S,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V_z (lbf)');   title('V_z'); xlim([0 x111])
nexttile; plot(x,Mr3 ,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('M (lbf*in)'); title('M');   xlim([0 x111])
nexttile; stairs(xS,Vr3S ,'b','LineWidth',2); grid on; xlabel('x (in)'); ylabel('V (lbf)');   title('V'); xlim([0 x111])

figure('Name','Torque Diagrams First Gear ','NumberTitle','off')
tiledlayout(3,1,'TileSpacing','compact','Padding','compact')
nexttile
stairs(xx,T1,'b','LineWidth',2); grid on ; xlabel('x (in)'); ylabel('T (lbf*in)')
title('First Gear'); xlim([0 x111]);
nexttile
stairs(xx,T2,'b','LineWidth',2); grid on
xlabel('x (in)'); ylabel('T (lbf*in)') ; title('Second Gear') ; xlim([0 x111]) ;
nexttile
stairs(xxx,T3,'b','LineWidth',2); grid on ; xlabel('x (in)'); ylabel('T (lbf*in)')
title('Third Gear') ; xlim([0 x111])

disp(d1) ;

%% Deflection Analysis (1st Gear)

syms x real
My = cell(1,10); Mz = cell(1,10); E = 30e6 ;

B11y = double(C1.B11y);
B11z = double(C1.B11z);

My{1}  = B11y.*x ; My{2}  = My{1} ;
My{3}  = B11y.*x - Fc(1).*(x - x12) ; My{4}  = My{3} ;
My{5}  = B11y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14); My{6}  = My{5} ;

G16y = (Ft(1,1,1)*cosd(45) + Fr(1,1,1)*cosd(45));
My{7}  = B11y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) - G16y.*(x - x16);
My{8}  = My{7}; My{9}  = My{7}; My{10} = My{7};

Mz{1}  = B11z.*x ; Mz{2}  = Mz{1} ;
Mz{3}  = B11z.*x ; Mz{4}  = Mz{3} ;
Mz{5}  = B11z.*x ; Mz{6}  = Mz{5} ;

G16z = (-Ft(1,1,1)*sind(45) + Fr(1,1,1)*sind(45));
Mz{7}  = B11z.*x + G16z.*(x - x16);
Mz{8}  = Mz{7}; Mz{9}  = Mz{7}; Mz{10} = Mz{7};

nSec = numel(My);
I1s = [I1(1) I1(2) I1(2) I1(3) I1(3) I1(4) I1(4) I1(5) I1(3) I1(1)];
xB = [x10 x11 x12 x13 x14 x15 x16 x17 x19 x110 x111];

Cy = sym('C', [1, 2*nSec], 'real'); thetay = cell(1, nSec); vy = cell(1, nSec);

k1 = My{1}./(E*I1s(1)); thetay{1} = int(k1, x) + Cy(1);
vy{1} = int(thetay{1}, x) + Cy(1+nSec);
k10 = My{10}./(E*I1s(10)); thetay{10} = int(k10, x) + Cy(10);
vy{10} = int(thetay{10}, x) + Cy(10+nSec);

for i = 2:9
    k = My{i}./(E*I1s(i));
    thetay{i} = int(k, x) + Cy(i);
    vy{i}     = int(thetay{i}, x) + Cy(i+nSec);
end

neq  = 2*(nSec-1) + 2;
eqs  = sym(zeros(neq,1));

for j = 1:nSec-1
    xint = xB(j+1);
    eqs(j)            = subs(vy{j}, x, xint) == subs(vy{j+1}, x, xint);
    eqs(j+(nSec-1))   = subs(thetay{j}, x, xint) == subs(thetay{j+1}, x, xint);
end

eqs(2*(nSec-1)+1) = subs(vy{1},    x, xB(1))   == 0;
eqs(2*(nSec-1)+2) = subs(vy{nSec}, x, xB(end)) == 0;
vars = Cy.'; [A,b] = equationsToMatrix(eqs,vars); Csol  = A\b;

for i = 1:nSec
    thetay{i} = simplify(subs(thetay{i}, vars, Csol));
    vy{i}     = simplify(subs(vy{i},     vars, Csol));
end

Cz = sym('Cz', [1, 2*nSec], 'real'); thetaz = cell(1, nSec); vz = cell(1, nSec);

k1 = Mz{1}./(E*I1s(1)); thetaz{1} = int(k1, x) + Cz(1);
vz{1} = int(thetaz{1}, x) + Cz(1+nSec);
k10 = Mz{10}./(E*I1s(10)); thetaz{10} = int(k10, x) + Cz(10);
vz{10} = int(thetaz{10}, x) + Cz(10+nSec);

for i = 2:9
    k = Mz{i}./(E*I1s(i));
    thetaz{i} = int(k, x) + Cz(i);
    vz{i}     = int(thetaz{i}, x) + Cz(i+nSec);
end

eqsz  = sym(zeros(neq,1));

for j = 1:nSec-1
    xint = xB(j+1);
    eqsz(j)            = subs(vz{j}, x, xint) == subs(vz{j+1}, x, xint);
    eqsz(j+(nSec-1))   = subs(thetaz{j}, x, xint) == subs(thetaz{j+1}, x, xint);
end

eqsz(2*(nSec-1)+1) = subs(vz{1},    x, xB(1))   == 0;
eqsz(2*(nSec-1)+2) = subs(vz{nSec}, x, xB(end)) == 0;
varsz = Cz.'; [Az,bz] = equationsToMatrix(eqsz,varsz); Csolz  = Az\bz;

for i = 1:nSec
    thetaz{i} = simplify(subs(thetaz{i}, varsz, Csolz));
    vz{i}     = simplify(subs(vz{i},     varsz, Csolz));
end

figure('Name','1st Gear Deflection & Slope','NumberTitle','off');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact')

nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    vvy = double(subs(vy{i}, x, xx));
    plot(xx, vvy,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('v_y (in)'); title('v_y');

nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    vvz = double(subs(vz{i}, x, xx));
    plot(xx, vvz,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('v_z (in)'); title('v_z');

nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    tty = double(subs(thetay{i}, x, xx));
    plot(xx, tty,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('\theta_y (rad)'); title('\theta_y');

nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    ttz = double(subs(thetaz{i}, x, xx));
    plot(xx, ttz,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('\theta_z (rad)'); title('\theta_z');

%% Deflection Analysis (2nd Gear)

syms x real
My = cell(1,10); Mz = cell(1,10); E = 30e6 ;

B21y = double(C2.B21y);
B21z = double(C2.B21z);

My{1}  = B21y.*x ; My{2}  = My{1} ;
My{3}  = B21y.*x - Fc(1).*(x - x12) ; My{4}  = My{3} ;
My{5}  = B21y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14); My{6}  = My{5} ;

G16y = (Ft(2,1,1)*cosd(45) + Fr(2,1,1)*cosd(45));
My{7}  = B21y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) - G16y.*(x - x16);
My{8}  = My{7}; My{9}  = My{7}; My{10} = My{7};

Mz{1}  = B21z.*x ; Mz{2}  = Mz{1} ;
Mz{3}  = B21z.*x ; Mz{4}  = Mz{3} ;
Mz{5}  = B21z.*x ; Mz{6}  = Mz{5} ;

G16z = (-Ft(2,1,1)*sind(45) + Fr(2,1,1)*sind(45));
Mz{7}  = B21z.*x + G16z.*(x - x16);
Mz{8}  = Mz{7}; Mz{9}  = Mz{7}; Mz{10} = Mz{7};

nSec = numel(My);
I1s = [I1(1) I1(2) I1(2) I1(3) I1(3) I1(4) I1(4) I1(5) I1(3) I1(1)];
xB = [x10 x11 x12 x13 x14 x15 x16 x17 x19 x110 x111];

Cy = sym('C', [1, 2*nSec], 'real'); thetay = cell(1, nSec); vy = cell(1, nSec);

k1 = My{1}./(E*I1s(1)); thetay{1} = int(k1, x) + Cy(1);
vy{1} = int(thetay{1}, x) + Cy(1+nSec);
k10 = My{10}./(E*I1s(10)); thetay{10} = int(k10, x) + Cy(10);
vy{10} = int(thetay{10}, x) + Cy(10+nSec);

for i = 2:9
    k = My{i}./(E*I1s(i));
    thetay{i} = int(k, x) + Cy(i);
    vy{i}     = int(thetay{i}, x) + Cy(i+nSec);
end

neq  = 2*(nSec-1) + 2;
eqs  = sym(zeros(neq,1));

for j = 1:nSec-1
    xint = xB(j+1);
    eqs(j)            = subs(vy{j}, x, xint) == subs(vy{j+1}, x, xint);
    eqs(j+(nSec-1))   = subs(thetay{j}, x, xint) == subs(thetay{j+1}, x, xint);
end

eqs(2*(nSec-1)+1) = subs(vy{1},    x, xB(1))   == 0;
eqs(2*(nSec-1)+2) = subs(vy{nSec}, x, xB(end)) == 0;
vars = Cy.'; [A,b] = equationsToMatrix(eqs,vars); Csol  = A\b;

for i = 1:nSec
    thetay{i} = simplify(subs(thetay{i}, vars, Csol));
    vy{i}     = simplify(subs(vy{i},     vars, Csol));
end

Cz = sym('Cz', [1, 2*nSec], 'real'); thetaz = cell(1, nSec); vz = cell(1, nSec);

k1 = Mz{1}./(E*I1s(1)); thetaz{1} = int(k1, x) + Cz(1);
vz{1} = int(thetaz{1}, x) + Cz(1+nSec);
k10 = Mz{10}./(E*I1s(10)); thetaz{10} = int(k10, x) + Cz(10);
vz{10} = int(thetaz{10}, x) + Cz(10+nSec);

for i = 2:9
    k = Mz{i}./(E*I1s(i));
    thetaz{i} = int(k, x) + Cz(i);
    vz{i}     = int(thetaz{i}, x) + Cz(i+nSec);
end

eqsz  = sym(zeros(neq,1));

for j = 1:nSec-1
    xint = xB(j+1);
    eqsz(j)            = subs(vz{j}, x, xint) == subs(vz{j+1}, x, xint);
    eqsz(j+(nSec-1))   = subs(thetaz{j}, x, xint) == subs(thetaz{j+1}, x, xint);
end

eqsz(2*(nSec-1)+1) = subs(vz{1},    x, xB(1))   == 0;
eqsz(2*(nSec-1)+2) = subs(vz{nSec}, x, xB(end)) == 0;
varsz = Cz.'; [Az,bz] = equationsToMatrix(eqsz,varsz); Csolz  = Az\bz;

for i = 1:nSec
    thetaz{i} = simplify(subs(thetaz{i}, varsz, Csolz));
    vz{i}     = simplify(subs(vz{i},     varsz, Csolz));
end

figure('Name','2nd Gear Deflection & Slope','NumberTitle','off');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact')
nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    vvy = double(subs(vy{i}, x, xx));
    plot(xx, vvy,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('v_y (in)'); title('v_y');

nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    vvz = double(subs(vz{i}, x, xx));
    plot(xx, vvz,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('v_z (in)'); title('v_z');
nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    tty = double(subs(thetay{i}, x, xx));
    plot(xx, tty,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('\theta_y (rad)'); title('\theta_y');
nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    ttz = double(subs(thetaz{i}, x, xx));
    plot(xx, ttz,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('\theta_z (rad)'); title('\theta_z');

%% Deflection Analysis (3rd Gear)

syms x real
My = cell(1,10); Mz = cell(1,10); E = 30e6 ;

B31y = double(C3.B31y);
B31z = double(C3.B31z);

My{1}  = B31y.*x ; My{2}  = My{1} ;
My{3}  = B31y.*x - Fc(1).*(x - x12) ; My{4}  = My{3} ;
My{5}  = B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14); My{6}  = My{5} ;

G16y = (Ft(3,1,1)*cosd(45) + Fr(3,1,1)*cosd(45));
My{7}  = B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) - G16y.*(x - x16);
My{8}  = My{7};

G19y = Fr(3,3,1);
My{9}  = B31y.*x - Fc(1).*(x - x12) - Fc(2).*(x - x14) - G16y.*(x - x16) - G19y.*(x - x19);
My{10} = My{9};

Mz{1}  = B31z.*x ; Mz{2}  = Mz{1} ;
Mz{3}  = B31z.*x ; Mz{4}  = Mz{3} ;
Mz{5}  = B31z.*x ; Mz{6}  = Mz{5} ;

G16z = (-Ft(3,1,1)*sind(45) + Fr(3,1,1)*sind(45));
Mz{7}  = B31z.*x + G16z.*(x - x16);
Mz{8}  = Mz{7};

G19z = Ft(3,3,1);
Mz{9}  = B31z.*x + G16z.*(x - x16) + G19z.*(x - x19);
Mz{10} = Mz{9};

nSec = numel(My);
I1s = [I1(1) I1(2) I1(2) I1(3) I1(3) I1(4) I1(4) I1(5) I1(3) I1(1)];
xB = [x10 x11 x12 x13 x14 x15 x16 x17 x19 x110 x111];

Cy = sym('C', [1, 2*nSec], 'real'); thetay = cell(1, nSec); vy = cell(1, nSec);

k1 = My{1}./(E*I1s(1)); thetay{1} = int(k1, x) + Cy(1);
vy{1} = int(thetay{1}, x) + Cy(1+nSec);
k10 = My{10}./(E*I1s(10)); thetay{10} = int(k10, x) + Cy(10);
vy{10} = int(thetay{10}, x) + Cy(10+nSec);

for i = 2:9
    k = My{i}./(E*I1s(i));
    thetay{i} = int(k, x) + Cy(i);
    vy{i}     = int(thetay{i}, x) + Cy(i+nSec);
end

neq  = 2*(nSec-1) + 2;
eqs  = sym(zeros(neq,1));

for j = 1:nSec-1
    xint = xB(j+1);
    eqs(j)            = subs(vy{j}, x, xint) == subs(vy{j+1}, x, xint);
    eqs(j+(nSec-1))   = subs(thetay{j}, x, xint) == subs(thetay{j+1}, x, xint);
end

eqs(2*(nSec-1)+1) = subs(vy{1},    x, xB(1))   == 0;
eqs(2*(nSec-1)+2) = subs(vy{nSec}, x, xB(end)) == 0;
vars = Cy.'; [A,b] = equationsToMatrix(eqs,vars); Csol  = A\b;

for i = 1:nSec
    thetay{i} = simplify(subs(thetay{i}, vars, Csol));
    vy{i}     = simplify(subs(vy{i},     vars, Csol));
end

Cz = sym('Cz', [1, 2*nSec], 'real'); thetaz = cell(1, nSec); vz = cell(1, nSec);

k1 = Mz{1}./(E*I1s(1)); thetaz{1} = int(k1, x) + Cz(1);
vz{1} = int(thetaz{1}, x) + Cz(1+nSec);
k10 = Mz{10}./(E*I1s(10)); thetaz{10} = int(k10, x) + Cz(10);
vz{10} = int(thetaz{10}, x) + Cz(10+nSec);

for i = 2:9
    k = Mz{i}./(E*I1s(i));
    thetaz{i} = int(k, x) + Cz(i);
    vz{i}     = int(thetaz{i}, x) + Cz(i+nSec);
end

eqsz  = sym(zeros(neq,1));

for j = 1:nSec-1
    xint = xB(j+1);
    eqsz(j)            = subs(vz{j}, x, xint) == subs(vz{j+1}, x, xint);
    eqsz(j+(nSec-1))   = subs(thetaz{j}, x, xint) == subs(thetaz{j+1}, x, xint);
end

eqsz(2*(nSec-1)+1) = subs(vz{1},    x, xB(1))   == 0;
eqsz(2*(nSec-1)+2) = subs(vz{nSec}, x, xB(end)) == 0;
varsz = Cz.'; [Az,bz] = equationsToMatrix(eqsz,varsz); Csolz  = Az\bz;

for i = 1:nSec
    thetaz{i} = simplify(subs(thetaz{i}, varsz, Csolz));
    vz{i}     = simplify(subs(vz{i},     varsz, Csolz));
end

figure('Name','3rd Gear Deflection & Slope','NumberTitle','off');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact')
nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    vvy = double(subs(vy{i}, x, xx));
    plot(xx, vvy,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('v_y (in)'); title('v_y');

nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    vvz = double(subs(vz{i}, x, xx));
    plot(xx, vvz,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('v_z (in)'); title('v_z');
nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    tty = double(subs(thetay{i}, x, xx));
    plot(xx, tty,'b','LineWidth',2);
end
xlabel('x (in)'); ylabel('\theta_y (rad)'); title('\theta_y');
nexttile; hold on; grid on
for i = 1:nSec
    xx = linspace(xB(i), xB(i+1), 250);
    ttz = double(subs(thetaz{i}, x, xx));
    plot(xx, ttz,'b','LineWidth',2);
end
ax = gca; ax.XGrid = 'on'; ax.YGrid = 'on'; ax.Layer = 'top';
xlabel('x (in)'); ylabel('\theta_z (rad)'); title('\theta_z');

%% Display Reactions (Rounded)

R = zeros(3,6);

R(1,1) = double(C1.B11x); R(1,2) = double(C1.B11y); R(1,3) = double(C1.B11z);
R(1,4) = double(C1.B12x); R(1,5) = double(C1.B12y); R(1,6) = double(C1.B12z);
R(2,1) = double(C2.B21x); R(2,2) = double(C2.B21y); R(2,3) = double(C2.B21z);
R(2,4) = double(C2.B22x); R(2,5) = double(C2.B22y); R(2,6) = double(C2.B22z);
R(3,1) = double(C3.B31x); R(3,2) = double(C3.B31y); R(3,3) = double(C3.B31z);
R(3,4) = double(C3.B32x); R(3,5) = double(C3.B32y); R(3,6) = double(C3.B32z);

R = round(R,3);

ReactionTable = array2table(R,...
    'VariableNames',{'B1x (lbf)','B1y (lbf)','B1z (lbf)','B2x (lbf)','B2y (lbf)','B2z (lbf)'},...
    'RowNames',{'Shaft1','Shaft2','Shaft3'});

disp(ReactionTable)

%% Critical Points Table (Deflection + Slope) - Shaft 1 (All Stages)

RowNames = ["Bearing 1","Sprocket 1","Sprocket 2","Gear 1","Gear 2","Bearing 2"];
xP = [x10, x12, x14, x16, x18, x111];

for k = 1:3
    vy_k = vy;  vz_k = vz;  thetay_k = thetay;  thetaz_k = thetaz;
    vX = zeros(1,numel(xP)); vZ = zeros(1,numel(xP)); v = zeros(1,numel(xP));
    tX = zeros(1,numel(xP)); tZ = zeros(1,numel(xP)); t = zeros(1,numel(xP));
    for i = 1:numel(xP)
        xi = xP(i);
        sec = find(xi >= xB(1:end-1) & xi <= xB(2:end), 1, 'first');
        if isempty(sec)
            sec = numel(vy_k);
        end
        vX(i) = double(subs(vy_k{sec}, x, xi));
        vZ(i) = double(subs(vz_k{sec}, x, xi));
        v(i)  = hypot(vX(i), vZ(i));
        tX(i) = double(subs(thetay_k{sec}, x, xi));
        tZ(i) = double(subs(thetaz_k{sec}, x, xi));
        t(i)  = hypot(tX(i), tZ(i));
    end
    fprintf('\n==============================\n');
    fprintf('STAGE %d - SHAFT 1 RESULTS\n',k);
    fprintf('==============================\n');
    fprintf('\nCritical Points of Deflection (in):\n\n');
    fprintf('%-16s %12s %12s %12s\n\n','Point','vy (in)','vz (in)','v (in)');
    for i = 1:numel(RowNames)
        fprintf('%-16s %12.4e %12.4e %12.4e\n', ...
            RowNames(i), vX(i), vZ(i), v(i));
    end
    fprintf('\nCritical Points of Slope (rad):\n\n');
    fprintf('%-16s %12s %12s %12s\n\n','Point','θy (rad)','θz (rad)','θ (rad)');

    for i = 1:numel(RowNames)
        fprintf('%-16s %12.4e %12.4e %12.4e\n', ...
            RowNames(i), tX(i), tZ(i), t(i));
    end
end

%% Angle of Twist

phi = [] ; J1 = 2*I1s ; G = 11.5e6 ;
phi(1) = 2*pi/(60*G)*(Tc(1)*(x13/J1(2) + x14/J1(3)) + (Tc(1)+Tc(2))*(x15/J1(3)...
    + x16/J1(4))) ; phi(2) = phi(1) ;
phi(3) = 2*pi/(60*G)*(Tc(1)*(x13/J1(2) + x14/J1(3)) + (Tc(1)+Tc(2))*(x15/J1(3)...
    + (x16+x17)/J1(4) + x18/J1(5) + x19/J1(3))) ; phi = rad2deg(phi) ;
disp(phi)

%% 1st Lateral Critical Speed (Rayleigh) - Shaft 1 (All Stages)

gamma = 0.282 ;     % lbf/in^3
g = 386.4 ;         % in/s^2
PdRay = 6 ;         % diametral pitch (same as force section)

% Gear is located at x16 on Shaft 1
xg = x16 ;

% Find deflection magnitude at x16
sec = find(xg >= xB(1:end-1) & xg <= xB(2:end), 1, 'first') ;

vyg = double(subs(vy{sec}, x, xg)) ;
vzg = double(subs(vz{sec}, x, xg)) ;

ydef = hypot(vyg,vzg) ;

ncr = zeros(1,3) ;
nb  = zeros(1,3) ;

for k = 1:3

    N = GearG{k}{2,5} ;        % teeth
    nb(k) = GearG{k}{2,3} ;    % shaft speed (rpm)

    % Geometry
    b = 1.5*cosd(45);          % bevel face width projection
    dBore = d1(4);             % shaft diameter near x16

    OD = (N/PdRay) + 2/PdRay ;       % outside diameter
    V  = (pi/4)*(OD^2 - dBore^2)*b ;
    W  = gamma*V ;
    omega_cr = sqrt( g*(W*ydef) / (W*(ydef^2)) ) ;
    ncr(k)   = (60/(2*pi))*omega_cr ;

end

fprintf('\n1st Lateral Critical Speed - Shaft 1:\n\n');
fprintf('Stage 1: n_cr = %.2f rpm   |  Required ≥ %.2f rpm\n', ncr(1), 2*nb(1));
fprintf('Stage 2: n_cr = %.2f rpm   |  Required ≥ %.2f rpm\n', ncr(2), 2*nb(2));
fprintf('Stage 3: n_cr = %.2f rpm   |  Required ≥ %.2f rpm\n\n', ncr(3), 2*nb(3));

for k = 1:3
    if ncr(k) >= 2*nb(k)
        fprintf('Stage %d: Adequate separation\n',k);
    else
        fprintf('Stage %d: NOT enough separation\n',k);
    end
end
