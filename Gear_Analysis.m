clear; clc

MIN_BENDING_TEETH = 17;

valid_main = false;

try

while ~valid_main

    restartMain = false;

    Shafts = inputNum('How many shafts present in the geartrain? ');
    Pin    = inputNum('Power to the geartrain (hp)? ');
    Win    = inputNum('Angular velocity to the geartrain (rpm)? ');
    Wout   = inputNum('Angular velocity from the geartrain (rpm)? ');
    Phi    = inputNum('Pressure angle (deg) [14.5, 20, 25]: ');

    TGears = 2*Shafts - 2;
    nPairs = TGears/2;

    Meshes = Shafts - 1;
    e_tot  = Wout / Win;

    etaSpur    = 0.98;
    etaHelical = 0.96;

    e_ith = (e_tot)^(1/(Shafts-1));

    if ~isfinite(e_ith) || e_ith <= 0
        fprintf('\nERROR: invalid e_ith computed. Check Win/Wout/Shafts.\n\n');
        continue;
    end

    % ============== %
    % --- Speeds --- %
    % ============== %
    Wshaft = zeros(1, Shafts);
    Wshaft(1) = Win;
    for j = 2:Shafts
        Wshaft(j) = Wshaft(j-1) * e_ith;
    end

    % ============================ %
    % --- Which gear on what shaft %
    % ============================ %
    GearShaft = zeros(1, TGears);
    idx = 1;
    GearShaft(idx) = 1; idx = idx + 1;
    for s = 2:Shafts-1
        GearShaft(idx) = s; idx = idx + 1;
        GearShaft(idx) = s; idx = idx + 1;
    end
    GearShaft(idx) = Shafts;

    % ================================================ %
    % --- Preview performance (assume all spur losses) %
    % ================================================ %
    Pshaft_preview = zeros(1, Shafts);
    Tshaft_preview = zeros(1, Shafts);
    Pshaft_preview(1) = Pin;
    for j = 2:Shafts
        Pshaft_preview(j) = Pshaft_preview(j-1) * etaSpur;
    end
    for j = 1:Shafts
        Tshaft_preview(j) = 5252 * Pshaft_preview(j) / Wshaft(j);
    end

    Wgear_preview = Wshaft(GearShaft);
    Tgear_preview = Tshaft_preview(GearShaft);
    Pgear_preview = Pshaft_preview(GearShaft);

    fprintf('\n==================== PREVIEW PERFORMANCE (assumes all SPUR, eta = 0.98 per mesh) ====================\n');
    fprintf('\nShaft   Speed (rpm)       Torque (ft*lbf)      Power (hp)\n');
    for i = 1:Shafts
        fprintf(' %2d     %10.3f        %12.3f        %10.4f\n', i, Wshaft(i), Tshaft_preview(i), Pshaft_preview(i));
    end

    fprintf('\nGear    Speed (rpm)       Torque (ft*lbf)      Power (hp)\n');
    for i = 1:TGears
        fprintf(' %2d     %10.3f        %12.3f        %10.4f\n', i, Wgear_preview(i), Tgear_preview(i), Pgear_preview(i));
    end

    [~, pHighSpeed]  = max(Wshaft(1:end-1));
    [~, pHighTorque] = max(Tshaft_preview(1:end-1));
    fprintf('\nDesign note (rule-of-thumb):\n');
    fprintf('  Highest-speed mesh is mesh %d (shafts %d->%d). Helical is often beneficial here for noise.\n', pHighSpeed, pHighSpeed, pHighSpeed+1);
    fprintf('  Highest-torque mesh is mesh %d (shafts %d->%d). Helical may help load sharing but adds axial thrust.\n\n', pHighTorque, pHighTorque, pHighTorque+1);

    % ===================================== %
    % --- Spur vs Helical mesh selection --- %
    % ===================================== %
    valid_type = false;
    while ~valid_type
        MainType = lower(strtrim(inputStr('Main transmission gear type? (spur/helical): ')));
        if any(strcmp(MainType, {'spur','helical'}))
            valid_type = true;
        else
            fprintf('Invalid entry. Type "spur" or "helical".\n');
        end
    end

    isHelicalMesh = false(1, nPairs);
    psiMesh       = zeros(1, nPairs);
    phi_tMesh     = zeros(1, nPairs);
    mNMesh        = ones(1, nPairs);

    if strcmp(MainType,'helical')
        isHelicalMesh(:) = true;
    else
        isHelicalMesh(:) = false;
    end

    if strcmp(MainType,'spur')
        mixAns = lower(strtrim(inputStr('Any helical meshes in the transmission? (yes/no): ')));
        if strcmp(mixAns,'yes')
            fprintf('\nMeshes are between shaft p and p+1. Example: mesh 2 is shaft 2->3.\n');
            helicalList = inputNumVec('Enter mesh indices that are HELICAL (e.g., [2 3]): ');
            helicalList = helicalList(:).';
            helicalList = helicalList(helicalList>=1 & helicalList<=nPairs);
            isHelicalMesh(:) = false;
            isHelicalMesh(helicalList) = true;
        end
    else
        mixAns = lower(strtrim(inputStr('Any spur meshes in the transmission? (yes/no): ')));
        if strcmp(mixAns,'yes')
            fprintf('\nMeshes are between shaft p and p+1. Example: mesh 2 is shaft 2->3.\n');
            spurList = inputNumVec('Enter mesh indices that are SPUR (e.g., [1 4]): ');
            spurList = spurList(:).';
            spurList = spurList(spurList>=1 & spurList<=nPairs);
            isHelicalMesh(:) = true;
            isHelicalMesh(spurList) = false;
        end
    end

    for p = 1:nPairs
        if isHelicalMesh(p)
            ok = false;
            while ~ok
                psiMesh(p) = inputNum(sprintf('Helix angle psi for mesh %d (deg): ', p));
                if isfinite(psiMesh(p)) && psiMesh(p) > 0 && psiMesh(p) < 90
                    ok = true;
                else
                    fprintf('psi must be between 0 and 90 deg.\n');
                end
            end
            phi_tMesh(p) = atand( tand(Phi) / cosd(psiMesh(p)) );
            mNMesh(p)    = cosd(psiMesh(p));
        else
            psiMesh(p)   = 0;
            phi_tMesh(p) = Phi;
            mNMesh(p)    = 1.0;
        end
    end

    etaMesh = zeros(1, nPairs);
    for p = 1:nPairs
        etaMesh(p) = etaSpur;
        if isHelicalMesh(p)
            etaMesh(p) = etaHelical;
        end
    end

    gearMesh = zeros(1, TGears);
    p = 1;
    for g = 1:2:TGears
        gearMesh(g)   = p;
        gearMesh(g+1) = p;
        p = p + 1;
    end

    if any(isHelicalMesh)
        fprintf('\nNOTE: You entered Phi=%.2f deg.\n', Phi);
        fprintf('  - For HELICAL meshes: Phi is treated as phi_n (normal).\n');
        fprintf('  - For SPUR meshes:    Phi is treated as phi_t (transverse).\n');
        fprintf('  - The program computes phi_t per helical mesh using tan(phi_t)=tan(phi_n)/cos(psi).\n\n');
    end

    % ========================= %
    % --- Final power/torque --- %
    % ========================= %
    Pshaft = zeros(1, Shafts);
    Tshaft = zeros(1, Shafts);

    Pshaft(1) = Pin;
    for j = 2:Shafts
        Pshaft(j) = Pshaft(j-1) * etaMesh(j-1);
    end
    for j = 1:Shafts
        Tshaft(j) = 5252 * Pshaft(j) / Wshaft(j);
    end

    Wgear = Wshaft(GearShaft);
    Tgear = Tshaft(GearShaft);
    Pgear = Pshaft(GearShaft);

    fprintf('\n==================== FINAL PERFORMANCE (with selected spur/helical losses) ====================\n');
    fprintf('Performance table (e_tot = %.4f & e_ith = %.4f)\n', e_tot, e_ith);

    fprintf('\nMesh efficiencies (per mesh):\n');
    for p = 1:nPairs
        if isHelicalMesh(p)
            fprintf('  Mesh %d (shaft %d->%d): helical -> eta = %.3f (4%% loss)\n', p, p, p+1, etaMesh(p));
        else
            fprintf('  Mesh %d (shaft %d->%d): spur   -> eta = %.3f (2%% loss)\n', p, p, p+1, etaMesh(p));
        end
    end

    fprintf('\nShaft   Speed (rpm)       Torque (ft*lbf)      Power (hp)\n');
    for i = 1:Shafts
        fprintf(' %2d     %10.3f        %12.3f        %10.4f\n', i, Wshaft(i), Tshaft(i), Pshaft(i));
    end

    fprintf('\nGear    Speed (rpm)       Torque (ft*lbf)      Power (hp)       Mesh     Type\n');
    for i = 1:TGears
        pm = gearMesh(i);
        if isHelicalMesh(pm), tstr = 'helical'; else, tstr = 'spur'; end
        fprintf(' %2d     %10.3f        %12.3f        %10.4f       %4d      %s\n', ...
            i, Wgear(i), Tgear(i), Pgear(i), pm, tstr);
    end

    % ============================== %
    % --- Tooth geometry (k value) --- %
    % ============================== %
    valid_geom = false;
    while ~valid_geom
        Geometry = lower(strtrim(inputStr('\nAre the gears full depth or stub teeth? (full/stub): ')));
        switch Geometry
            case {'full depth','full'}
                k = 1.0;
                valid_geom = true;
            case {'stub teeth','stub'}
                k = 0.8;
                valid_geom = true;
            otherwise
                fprintf('Invalid geometry. Please input "full" or "stub".\n');
        end
    end

    approxSmallTeeth = false;

    optAvail = true(1,5);
    done_conflict = false;

    while ~done_conflict

        m = 1/e_ith;

        NpMin = zeros(1, nPairs);
        NgMax = zeros(1, nPairs);

        for p = 1:nPairs

            if isHelicalMesh(p)
                phit = phi_tMesh(p);
                psi  = psiMesh(p);

                Np_cont = (2*k*cosd(psi))/((1 + 2*m) * (sind(phit)^2)) * ...
                          ( m + sqrt( m^2 + (1 + 2*m) * (sind(phit)^2) ) );

                Ng_cont = (Np_cont^2*(sind(phit))^2 - 4*k^2*(cosd(psi))^2) / ...
                          (4*k*cosd(psi) - 2*Np_cont*(sind(phit)^2));
            else
                phit = Phi;

                Np_cont = (2*k)/((1 + 2*m) * (sind(phit)^2)) * ...
                          ( m + sqrt( m^2 + (1 + 2*m) * (sind(phit)^2) ) );

                Ng_cont = (Np_cont^2*(sind(phit)^2)) / (4*k - 2*Np_cont*(sind(phit)^2));
            end

            NpMin(p) = floor(Np_cont);
            NgMax(p) = floor(Ng_cont);

            if NpMin(p) < 1 || NgMax(p) < 1 || ~isfinite(NpMin(p)) || ~isfinite(NgMax(p))
                error('Interference calculation blew up in mesh %d. Check Phi/psi/ratios.', p);
            end
        end

        fprintf('\n==================== INTERFERENCE LIMITS (per mesh) ====================\n');
        for p = 1:nPairs
            if isHelicalMesh(p)
                fprintf('Mesh %d (HELICAL): Np(min)=%d | Ng(max)=%d | psi=%.2f deg | phi_t=%.2f deg\n', ...
                    p, NpMin(p), NgMax(p), psiMesh(p), phi_tMesh(p));
            else
                fprintf('Mesh %d (SPUR)  : Np(min)=%d | Ng(max)=%d | Phi=%.2f deg\n', ...
                    p, NpMin(p), NgMax(p), Phi);
            end
        end
        fprintf('\n');

        mG_mesh = NgMax ./ NpMin;
        hasInterference = any((1/e_ith) > mG_mesh);

        feasibleInterval = true;
        for p = 1:nPairs
            zP_low  = NpMin(p);
            zP_high = floor(NgMax(p) / m);
            if ~approxSmallTeeth
                zP_low = max(zP_low, ceil(MIN_BENDING_TEETH / m));
            end
            if zP_high < zP_low
                feasibleInterval = false;
            end
        end

        if ~hasInterference && feasibleInterval
            fprintf('Transmission will NOT have interference for the required input/output (all meshes).\n\n');
            done_conflict = true;
            break;
        end

        if ~any(optAvail)
            fprintf('\nNo options left.\n');
            fprintf('The desired performance parameters cannot be done without interference and/or without valid table tooth ranges.\n\n');
            error('NoFeasibleDesign');
        end

        fprintf('\n==================== CONFLICT RESOLUTION MENU ====================\n');
        fprintf('A conflict exists:\n');
        if hasInterference
            fprintf('  - Interference risk in at least one mesh\n');
        end
        if ~feasibleInterval
            fprintf('  - No valid pinion interval exists under current tooth-table constraints\n');
        end
        fprintf('\nChoose what to try next:\n');

        validChoices = [];

        if optAvail(1)
            fprintf('  (1) Redesign ratio distribution (change Shafts / Win / Wout)\n');
            validChoices(end+1) = 1;
        end
        if optAvail(2)
            fprintf('  (2) Change pressure angle (updates helical phi_t too)\n');
            validChoices(end+1) = 2;
        end
        if optAvail(3)
            fprintf('  (3) Switch tooth system full <-> stub (changes k only)\n');
            validChoices(end+1) = 3;
        end
        if optAvail(4)
            fprintf('  (4) Change spur/helical mesh selections\n');
            validChoices(end+1) = 4;
        end
        if optAvail(5)
            fprintf('  (5) Continue anyway using %d-tooth clamp for factor lookups (rough check)\n', MIN_BENDING_TEETH);
            validChoices(end+1) = 5;
        end
        fprintf('\n');

        pick = inputNum('Option: ');
        if ~any(pick == validChoices)
            fprintf('Invalid option.\n');
            continue;
        end

        if pick == 1
            optAvail(1) = false;
            fprintf('\nRestarting to change ratio inputs...\n\n');
            restartMain = true;
            break;
        end

        if pick == 2
            optAvail(2) = false;
            Phi = inputNum('Enter the new pressure angle (deg): ');
            for p = 1:nPairs
                if isHelicalMesh(p)
                    phi_tMesh(p) = atand( tand(Phi) / cosd(psiMesh(p)) );
                else
                    phi_tMesh(p) = Phi;
                end
            end
        end

        if pick == 3
            optAvail(3) = false;
            if abs(k - 1.0) < 1e-9
                k = 0.8;
                Geometry = 'stub';
            else
                k = 1.0;
                Geometry = 'full';
            end
        end

        if pick == 4
            optAvail(4) = false;

            if strcmp(MainType,'spur')
                mixAns = lower(strtrim(inputStr('Any helical meshes in the transmission? (yes/no): ')));
                if strcmp(mixAns,'yes')
                    helicalList = inputNumVec('Enter mesh indices that are HELICAL (e.g., [2 3]): ');
                    helicalList = helicalList(:).';
                    helicalList = helicalList(helicalList>=1 & helicalList<=nPairs);
                    isHelicalMesh(:) = false;
                    isHelicalMesh(helicalList) = true;
                else
                    isHelicalMesh(:) = false;
                end
            else
                mixAns = lower(strtrim(inputStr('Any spur meshes in the transmission? (yes/no): ')));
                if strcmp(mixAns,'yes')
                    spurList = inputNumVec('Enter mesh indices that are SPUR (e.g., [1 4]): ');
                    spurList = spurList(:).';
                    spurList = spurList(spurList>=1 & spurList<=nPairs);
                    isHelicalMesh(:) = true;
                    isHelicalMesh(spurList) = false;
                else
                    isHelicalMesh(:) = true;
                end
            end

            for p = 1:nPairs
                if isHelicalMesh(p)
                    psiMesh(p) = inputNum(sprintf('Helix angle psi for mesh %d (deg): ', p));
                    phi_tMesh(p) = atand( tand(Phi) / cosd(psiMesh(p)) );
                    mNMesh(p)    = cosd(psiMesh(p));
                else
                    psiMesh(p)   = 0;
                    phi_tMesh(p) = Phi;
                    mNMesh(p)    = 1.0;
                end
            end

            for p = 1:nPairs
                if isHelicalMesh(p)
                    etaMesh(p) = etaHelical;
                else
                    etaMesh(p) = etaSpur;
                end
            end

            Pshaft(1) = Pin;
            for j = 2:Shafts
                Pshaft(j) = Pshaft(j-1) * etaMesh(j-1);
            end
            for j = 1:Shafts
                Tshaft(j) = 5252 * Pshaft(j) / Wshaft(j);
            end

            Wgear = Wshaft(GearShaft);
            Tgear = Tshaft(GearShaft);
            Pgear = Pshaft(GearShaft);
        end

        if pick == 5
            optAvail(5) = false;
            approxSmallTeeth = true;
            fprintf('\nAPPROX MODE ENABLED: Factor-table lookups below %d teeth will be clamped to %d.\n\n', MIN_BENDING_TEETH, MIN_BENDING_TEETH);
        end

    end

    if restartMain
        continue;
    end

    % ===================================== %
    % --- Teeth selection with interval --- %
    % ===================================== %
    fprintf('\n==================== TEETH SELECTION (interval per mesh) ====================\n');
    Systeeth = zeros(1, TGears);

    for i = 1:2:(TGears-1)

        pMesh = (i+1)/2;
        m = 1/e_ith;

        zP_low  = NpMin(pMesh);
        zP_high = floor(NgMax(pMesh) / m);

        if ~approxSmallTeeth
            zP_low = max(zP_low, ceil(MIN_BENDING_TEETH / m));
        end

        if zP_high < zP_low
            fprintf('Mesh %d: no valid pinion interval available.\n', pMesh);
            error('No pinion interval');
        end

        fprintf('Mesh %d allowed interval: %d <= pinion teeth <= %d\n', pMesh, zP_low, zP_high);

        optAvail_T = true(1,5);
        valid_pair = false;

        while ~valid_pair

            Systeeth(i) = inputNum(sprintf('Enter the teeth count for gear %d (pinion) within [%d..%d]: ', i, zP_low, zP_high));

            if Systeeth(i) < zP_low || Systeeth(i) > zP_high

                fprintf('Pinion teeth must be within the allowed interval.\n');

                if ~any(optAvail_T)
                    fprintf('\nNo options left.\n');
                    fprintf('The desired performance parameters cannot be done without interference and/or without valid table tooth ranges.\n\n');
                    error('NoFeasibleDesign');
                end

                fprintf('\n==================== TEETH SELECTION MENU ====================\n');
                fprintf('You entered a value outside the allowed interval.\n');
                fprintf('Choose what to try next:\n');

                validChoices = [];

                if optAvail_T(1)
                    fprintf('  (1) Redesign ratio distribution (change Shafts / Win / Wout)\n');
                    validChoices(end+1) = 1;
                end
                if optAvail_T(2)
                    fprintf('  (2) Change pressure angle (updates helical phi_t too)\n');
                    validChoices(end+1) = 2;
                end
                if optAvail_T(3)
                    fprintf('  (3) Switch tooth system full <-> stub (changes k only)\n');
                    validChoices(end+1) = 3;
                end
                if optAvail_T(4)
                    fprintf('  (4) Change spur/helical mesh selections\n');
                    validChoices(end+1) = 4;
                end
                if optAvail_T(5)
                    fprintf('  (5) Continue anyway using %d-tooth clamp for factor lookups (rough check)\n', MIN_BENDING_TEETH);
                    validChoices(end+1) = 5;
                end
                fprintf('\n');

                pick = inputNum('Option: ');
                if ~any(pick == validChoices)
                    fprintf('Invalid option.\n\n');
                    continue;
                end

                if pick == 1
                    optAvail_T(1) = false;
                    fprintf('\nRestarting to change ratio inputs...\n\n');
                    restartMain = true;
                    valid_pair = true;
                    break;
                end

                if pick == 2
                    optAvail_T(2) = false;
                    Phi = inputNum('Enter the new pressure angle (deg): ');
                    for pp = 1:nPairs
                        if isHelicalMesh(pp)
                            phi_tMesh(pp) = atand( tand(Phi) / cosd(psiMesh(pp)) );
                        else
                            phi_tMesh(pp) = Phi;
                        end
                    end
                end

                if pick == 3
                    optAvail_T(3) = false;
                    if abs(k - 1.0) < 1e-9
                        k = 0.8;
                        Geometry = 'stub';
                    else
                        k = 1.0;
                        Geometry = 'full';
                    end
                end

                if pick == 4
                    optAvail_T(4) = false;

                    if strcmp(MainType,'spur')
                        mixAns = lower(strtrim(inputStr('Any helical meshes in the transmission? (yes/no): ')));
                        if strcmp(mixAns,'yes')
                            helicalList = inputNumVec('Enter mesh indices that are HELICAL (e.g., [2 3]): ');
                            helicalList = helicalList(:).';
                            helicalList = helicalList(helicalList>=1 & helicalList<=nPairs);
                            isHelicalMesh(:) = false;
                            isHelicalMesh(helicalList) = true;
                        else
                            isHelicalMesh(:) = false;
                        end
                    else
                        mixAns = lower(strtrim(inputStr('Any spur meshes in the transmission? (yes/no): ')));
                        if strcmp(mixAns,'yes')
                            spurList = inputNumVec('Enter mesh indices that are SPUR (e.g., [1 4]): ');
                            spurList = spurList(:).';
                            spurList = spurList(spurList>=1 & spurList<=nPairs);
                            isHelicalMesh(:) = true;
                            isHelicalMesh(spurList) = false;
                        else
                            isHelicalMesh(:) = true;
                        end
                    end

                    for pp = 1:nPairs
                        if isHelicalMesh(pp)
                            psiMesh(pp) = inputNum(sprintf('Helix angle psi for mesh %d (deg): ', pp));
                            phi_tMesh(pp) = atand( tand(Phi) / cosd(psiMesh(pp)) );
                            mNMesh(pp)    = cosd(psiMesh(pp));
                        else
                            psiMesh(pp)   = 0;
                            phi_tMesh(pp) = Phi;
                            mNMesh(pp)    = 1.0;
                        end
                    end

                    for pp = 1:nPairs
                        if isHelicalMesh(pp)
                            etaMesh(pp) = etaHelical;
                        else
                            etaMesh(pp) = etaSpur;
                        end
                    end

                    Pshaft(1) = Pin;
                    for jj = 2:Shafts
                        Pshaft(jj) = Pshaft(jj-1) * etaMesh(jj-1);
                    end
                    for jj = 1:Shafts
                        Tshaft(jj) = 5252 * Pshaft(jj) / Wshaft(jj);
                    end

                    Wgear = Wshaft(GearShaft);
                    Tgear = Tshaft(GearShaft);
                    Pgear = Pshaft(GearShaft);
                end

                if pick == 5
                    optAvail_T(5) = false;
                    approxSmallTeeth = true;
                    fprintf('\nAPPROX MODE ENABLED: Factor-table lookups below %d teeth will be clamped to %d.\n\n', MIN_BENDING_TEETH, MIN_BENDING_TEETH);
                end

                fprintf('\nTry again.\n\n');
                continue;
            end

            Systeeth(i+1) = floor(m * Systeeth(i));

            if Systeeth(i) < NpMin(pMesh)
                fprintf('WARNING: Pinion has %d teeth, below minimum %d for no interference (mesh %d).\n\n', Systeeth(i), NpMin(pMesh), pMesh);
                continue;
            end

            if Systeeth(i+1) > NgMax(pMesh)
                fprintf('Mating gear exceeds maximum tooth count (%d) for mesh %d.\n\n', NgMax(pMesh), pMesh);
                continue;
            end

            if ~approxSmallTeeth
                if Systeeth(i+1) < MIN_BENDING_TEETH
                    fprintf('WARNING: Mating gear is %d (<%d). Choose another pinion or enable approx mode earlier.\n\n', Systeeth(i+1), MIN_BENDING_TEETH);
                    continue;
                end
            end

            valid_pair = true;
        end

        if restartMain
            break;
        end

        if ~valid_pair
            break;
        end

    end

    if restartMain
        continue;
    end

    if any(Systeeth == 0)
        continue;
    end

    % =================================== %
    % --- Virtual number of teeth (Nv) --- %
    % =================================== %
    NvGear = zeros(1, TGears);
    for g = 1:TGears
        pm = gearMesh(g);
        if isHelicalMesh(pm)
            NvGear(g) = floor( Systeeth(g) / (cosd(psiMesh(pm))^3) );
        else
            NvGear(g) = Systeeth(g);
        end
    end

    if any(isHelicalMesh)
        fprintf('\nDiametral pitch meaning:\n');
        fprintf('  Spur gears: Pd = transverse diametral pitch\n');
        fprintf('  Helical gears: Pn = normal diametral pitch\n\n');
    end

    % ====================================== %
    % --- Diametral pitch / geometry setup --- %
    % ====================================== %
    Decision = lower(strtrim(inputStr('Do all the diameters have the same diametral pitch? (yes/no): ')));

    PdGear = zeros(1, TGears);
    d      = zeros(1, TGears);
    a      = zeros(1, TGears);
    b      = zeros(1, TGears);
    rb     = zeros(1, TGears);

    switch Decision
        case 'yes'
            Pd_all = inputNum('Diametral Pitch (Pd for spur, Pn for helical) [Typical Values: 2,2.25,2.5,3,4,6,8,10,12,16]: ');
            PdGear(:) = Pd_all;

            switch Phi
                case 20
                    switch Geometry
                        case {'full depth','full'}
                            a(:) = 1/Pd_all;
                            valid = false;
                            while ~valid
                                Dedendum = lower(strtrim(inputStr('Do you want the smaller or the bigger Dedendum: ')));
                                switch Dedendum
                                    case 'smaller'
                                        b(:) = 1.25/Pd_all; valid = true;
                                    case 'bigger'
                                        b(:) = 1.35/Pd_all; valid = true;
                                    otherwise
                                        fprintf('Invalid decision. Choose between smaller or bigger Dedendum.\n');
                                end
                            end
                        case {'stub teeth','stub'}
                            a(:) = 0.8/Pd_all;
                            b(:) = 1.0/Pd_all;
                    end
                case 25
                    a(:) = 1/Pd_all;
                    valid = false;
                    while ~valid
                        Dedendum = lower(strtrim(inputStr('Do you want the smaller or the bigger Dedendum: ')));
                        switch Dedendum
                            case 'smaller'
                                b(:) = 1.25/Pd_all; valid = true;
                            case 'bigger'
                                b(:) = 1.35/Pd_all; valid = true;
                            otherwise
                                fprintf('Invalid decision. Choose between smaller or bigger Dedendum.\n');
                        end
                    end
                case 14.5
                    a(:) = 1/Pd_all;
                    b(:) = 1.157/Pd_all;
                otherwise
                    error('Invalid pressure angle. Choose 14.5, 20, or 25.');
            end

            for i = 1:TGears
                pm = gearMesh(i);
                if isHelicalMesh(pm)
                    d(i)  = Systeeth(i) / (Pd_all * cosd(psiMesh(pm)));
                    rb(i) = (d(i)/2) * cosd(phi_tMesh(pm));
                else
                    d(i)  = Systeeth(i) / Pd_all;
                    rb(i) = (d(i)/2) * cosd(Phi);
                end
            end

        case 'no'
            Pairs = lower(strtrim(inputStr('Do the gear pairs have their own diametral pitch? (yes/no): ')));

            switch Pairs
                case 'yes'
                    Pd = zeros(1, nPairs);
                    for p = 1:nPairs
                        Pd(p) = inputNum(sprintf('Diametral pitch of pair %d (Pd for spur, Pn for helical) [Typical Values: 2,2.25,2.5,3,4,6,8,10,12,16]: ', p));
                    end

                    p = 1;
                    for i = 1:2:TGears
                        PdGear(i)   = Pd(p);
                        PdGear(i+1) = Pd(p);

                        switch Phi
                            case 20
                                switch Geometry
                                    case {'full depth','full'}
                                        a(i) = 1/Pd(p);   a(i+1) = 1/Pd(p);
                                        valid = false;
                                        while ~valid
                                            Dedendum = lower(strtrim(inputStr('Do you want the smaller or the bigger Dedendum: ')));
                                            switch Dedendum
                                                case 'smaller'
                                                    b(i) = 1.25/Pd(p); b(i+1) = 1.25/Pd(p); valid = true;
                                                case 'bigger'
                                                    b(i) = 1.35/Pd(p); b(i+1) = 1.35/Pd(p); valid = true;
                                                otherwise
                                                    fprintf('Invalid decision. Choose between smaller or bigger Dedendum.\n');
                                            end
                                        end
                                    case {'stub teeth','stub'}
                                        a(i) = 0.8/Pd(p); a(i+1) = 0.8/Pd(p);
                                        b(i) = 1.0/Pd(p); b(i+1) = 1.0/Pd(p);
                                end
                            case 25
                                a(i) = 1/Pd(p);   a(i+1) = 1/Pd(p);
                                valid = false;
                                while ~valid
                                    Dedendum = lower(strtrim(inputStr('Do you want the smaller or the bigger Dedendum: ')));
                                    switch Dedendum
                                        case 'smaller'
                                            b(i) = 1.25/Pd(p); b(i+1) = 1.25/Pd(p); valid = true;
                                        case 'bigger'
                                            b(i) = 1.35/Pd(p); b(i+1) = 1.35/Pd(p); valid = true;
                                        otherwise
                                            fprintf('Invalid decision. Choose between smaller or bigger Dedendum.\n');
                                    end
                                end
                            case 14.5
                                a(i) = 1/Pd(p);     a(i+1) = 1/Pd(p);
                                b(i) = 1.157/Pd(p); b(i+1) = 1.157/Pd(p);
                            otherwise
                                error('Invalid pressure angle. Choose 14.5, 20, or 25.');
                        end

                        if isHelicalMesh(p)
                            d(i)    = Systeeth(i)   / (Pd(p) * cosd(psiMesh(p)));
                            d(i+1)  = Systeeth(i+1) / (Pd(p) * cosd(psiMesh(p)));
                            rb(i)   = (d(i)/2)   * cosd(phi_tMesh(p));
                            rb(i+1) = (d(i+1)/2) * cosd(phi_tMesh(p));
                        else
                            d(i)    = Systeeth(i)   / Pd(p);
                            d(i+1)  = Systeeth(i+1) / Pd(p);
                            rb(i)   = (d(i)/2)   * cosd(Phi);
                            rb(i+1) = (d(i+1)/2) * cosd(Phi);
                        end

                        p = p + 1;
                    end

                case 'no'
                    for i = 1:TGears
                        PdGear(i) = inputNum(sprintf('Diametral pitch of gear %d (Pd for spur, Pn for helical) [Typical Values: 2,2.25,2.5,3,4,6,8,10,12,16]: ', i));

                        switch Phi
                            case 20
                                switch Geometry
                                    case {'full depth','full'}
                                        a(i) = 1/PdGear(i);
                                        valid = false;
                                        while ~valid
                                            Dedendum = lower(strtrim(inputStr('Do you want the smaller or the bigger Dedendum: ')));
                                            switch Dedendum
                                                case 'smaller'
                                                    b(i) = 1.25/PdGear(i); valid = true;
                                                case 'bigger'
                                                    b(i) = 1.35/PdGear(i); valid = true;
                                                otherwise
                                                    fprintf('Invalid decision. Choose between smaller or bigger Dedendum.\n');
                                            end
                                        end
                                    case {'stub teeth','stub'}
                                        a(i) = 0.8/PdGear(i);
                                        b(i) = 1.0/PdGear(i);
                                end
                            case 25
                                a(i) = 1/PdGear(i);
                                valid = false;
                                while ~valid
                                    Dedendum = lower(strtrim(inputStr('Do you want the smaller or the bigger Dedendum: ')));
                                    switch Dedendum
                                        case 'smaller'
                                            b(i) = 1.25/PdGear(i); valid = true;
                                        case 'bigger'
                                            b(i) = 1.35/PdGear(i); valid = true;
                                        otherwise
                                            fprintf('Invalid decision. Choose between smaller or bigger Dedendum.\n');
                                    end
                                end
                            case 14.5
                                a(i) = 1/PdGear(i);
                                b(i) = 1.157/PdGear(i);
                            otherwise
                                error('Invalid pressure angle. Choose 14.5, 20, or 25.');
                        end

                        pm = gearMesh(i);
                        if isHelicalMesh(pm)
                            d(i)  = Systeeth(i) / (PdGear(i) * cosd(psiMesh(pm)));
                            rb(i) = (d(i)/2) * cosd(phi_tMesh(pm));
                        else
                            d(i)  = Systeeth(i) / PdGear(i);
                            rb(i) = (d(i)/2) * cosd(Phi);
                        end
                    end
                otherwise
                    error('Invalid input for pair choice.');
            end
        otherwise
            error('Invalid input. Please enter yes or no.');
    end

    % ============================ %
    % --- Radii & tooth height --- %
    % ============================ %
    rf = zeros(1, TGears);
    ra = zeros(1, TGears);
    h  = zeros(1, TGears);

    for i = 1:TGears
        rf(i) = d(i)/2 - b(i);
        ra(i) = d(i)/2 + a(i);
        h(i)  = a(i) + b(i);
    end

    % ================= %
    % --- Face width --- %
    % ================= %
    valid = false;
    while ~valid
        DecisionFW = lower(strtrim(inputStr('Does every gear in the transmission have the same face width? (yes/no): ')));

        switch DecisionFW
            case 'yes'
                Flow  = 3*pi/PdGear(1);
                Fhigh = 5*pi/PdGear(1);

                Fall = inputNum(sprintf('Face width of the gears (in) [Typical values: %.2f < F < %.2f]: ', Flow, Fhigh));

                if Fall <= 0
                    fprintf('Face width must be positive. Try again.\n\n');
                    continue;
                end

                F = Fall * ones(1, TGears);
                valid = true;

            case 'no'
                PdPair = zeros(1, nPairs);
                p = 1;
                for i = 1:2:TGears
                    PdPair(p) = PdGear(i);
                    p = p + 1;
                end

                Fpair = zeros(1, nPairs);

                for p = 1:nPairs
                    ok = false;
                    while ~ok
                        Flow  = 3*pi/PdPair(p);
                        Fhigh = 5*pi/PdPair(p);

                        Fpair(p) = inputNum(sprintf('Face width of pair %d (in) [Typical values: %.2f < F < %.2f]: ', p, Flow, Fhigh));

                        if Fpair(p) > 0
                            ok = true;
                        else
                            fprintf('Face width must be positive. Try again.\n\n');
                        end
                    end
                end

                F = zeros(1, TGears);
                p = 1;
                for i = 1:2:TGears
                    F(i)   = Fpair(p);
                    F(i+1) = Fpair(p);
                    p = p + 1;
                end

                valid = true;

            otherwise
                fprintf('Decision not available. Enter either yes or no.\n\n');
        end
    end

    % =========================== %
    % --- Geometry output table --- %
    % =========================== %
    fprintf('\n');
    fprintf('%-6s %-6s %-8s %-10s %-16s %-16s %-16s %-16s %-18s %-18s %-18s %-10s\n', ...
        'Gear','Teeth','Nv','Mesh','Pitch Diameter (in)','Face Width (in)', ...
        'Addendum (in)','Dedendum (in)','Base Radius (in)', ...
        'Addendum Radius (in)','Root Radius (in)','Type');

    fprintf('%s\n', repmat('-',1,170));

    for i = 1:TGears
        pm = gearMesh(i);
        if isHelicalMesh(pm), tstr = 'helical'; else, tstr = 'spur'; end

        fprintf('%s %s %s %s %s %s %s %s %s %s %s %s\n', ...
            centerFmt(i,6,'%d'), ...
            centerFmt(Systeeth(i),6,'%d'), ...
            centerFmt(NvGear(i),8,'%d'), ...
            centerFmt(pm,10,'%d'), ...
            centerFmt(d(i),16,'%.3f'), ...
            centerFmt(F(i),16,'%.3f'), ...
            centerFmt(a(i),16,'%.3f'), ...
            centerFmt(b(i),16,'%.3f'), ...
            centerFmt(rb(i),18,'%.3f'), ...
            centerFmt(ra(i),18,'%.3f'), ...
            centerFmt(rf(i),18,'%.3f'), ...
            centerFmt(tstr,10,'%s'));
    end

    fprintf('\n\n');

    % ========================= %
    % --- Ko (Overload factor) --- %
    % ========================= %
    TKo = table( ...
        [1.00; 1.25; 1.50], ...
        [1.25; 1.50; 1.75], ...
        [1.75; 2.00; 2.25], ...
        'VariableNames', {'(1) Uniform','(2) Moderate Shock','(3) Heavy Shock'}, ...
        'RowNames', {'(1) Uniform','(2) Light Shock','(3) Medium Shock'});

    disp('Table of Overload Factors')
    disp(TKo)
    PSource  = inputNum('Power Source Service Factor (row 1-3): ');
    DMachine = inputNum('Driven Machine Service Factor (col 1-3): ');
    Ko = TKo{PSource, DMachine};

    % ========================== %
    % --- Kv (Dynamic factor) --- %
    % ========================== %
    Qv = inputNum('Quality Number (5 - 11): ');
    Bq = 0.25*(12-Qv)^(2/3);
    Aq = 50 + 56*(1-Bq);

    V  = zeros(1, TGears);
    Kv = zeros(1, TGears);
    for i = 1:TGears
        V(i)  = pi*d(i)*Wgear(i)/12;
        Kv(i) = ((Aq + sqrt(V(i)))/Aq)^Bq;
    end

    % ======================= %
    % --- Lewis factor table --- %
    % ======================= %
    if abs(Phi - 14.5) < 1e-6
        TLewis = readtable("Lewis Factor.xlsx","Sheet","14.5");
    elseif abs(Phi - 20) < 1e-6
        TLewis = readtable("Lewis Factor.xlsx","Sheet","20");
    elseif abs(Phi - 25) < 1e-6
        TLewis = readtable("Lewis Factor.xlsx","Sheet","25");
    else
        error("Pressure angle not found. Use 14.5, 20, or 25.");
    end

    % ===================== %
    % --- Crown/adjustment --- %
    % ===================== %
    valid = false;
    while ~valid
        Cmc_in = lower(strtrim(inputStr('Are the gear teeth crowned or uncrowned? ')));
        switch Cmc_in
            case {'uncrowned','not crowned','flat'}
                Cmc = 1.0; valid = true;
            case {'crowned','yes','crown'}
                Cmc = 0.8; valid = true;
            otherwise
                fprintf('Invalid entry. Please type "crowned" or "uncrowned".\n\n');
        end
    end

    valid = false;
    while ~valid
        Ce_in = lower(strtrim(inputStr('Is the gear adjusted/lapped? (yes/no): ')));
        switch Ce_in
            case 'yes'
                Ce = 0.8; valid = true;
            case 'no'
                Ce = 1.0; valid = true;
            otherwise
                fprintf('Invalid entry. Please type "yes" or "no".\n\n');
        end
    end

    % ========================= %
    % --- Shaft alignment input --- %
    % ========================= %
    Slen     = zeros(1, Shafts);
    Dshaft   = zeros(1, Shafts);
    ScGear   = zeros(1, TGears);
    CpmGear  = zeros(1, TGears);

    fprintf('\n==================== SHAFT ALIGNMENT INPUTS ====================\n');

    for s = 1:Shafts

        ok = false;
        while ~ok
            Slen(s) = inputNum(sprintf('Shaft Length of shaft %d (in): ', s));
            if Slen(s) > 0
                ok = true;
            else
                fprintf('Invalid entry. Shaft length must be positive.\n\n');
            end
        end

        gears_on_s = find(GearShaft == s);
        rf_allowed = min(rf(gears_on_s));
        Dshaft_max = 2*rf_allowed;

        ok = false;
        while ~ok
            Dshaft(s) = inputNum(sprintf('Shaft Diameter of shaft %d (in) [must be < %.3f]: ', s, Dshaft_max));
            if Dshaft(s) > 0 && Dshaft(s) < Dshaft_max
                ok = true;
            else
                fprintf('Invalid entry. Shaft diameter must be positive and < %.3f in.\n\n', Dshaft_max);
            end
        end

        for g = gears_on_s
            ok = false;
            while ~ok
                prompt = sprintf('Offset from the middle of gear %d from shaft %d (in): ', g, s);
                Sc_try = inputNum(prompt);

                if Sc_try < 0
                    fprintf('Invalid entry. Offset must be >= 0.\n\n');
                    continue;
                end

                if d(g) <= 0
                    error('Invalid gear diameter detected at gear %d.', g);
                end

                if (Sc_try / d(g)) > 1
                    fprintf('Invalid entry. Offset/diameter must be <= 1. Retype.\n\n');
                    continue;
                end

                ScGear(g) = Sc_try;
                ok = true;
            end

            Ratio = ScGear(g) / Slen(s);
            if Ratio < 0.175
                CpmGear(g) = 1.0;
            else
                CpmGear(g) = 1.1;
            end
        end
    end

    % ===================== %
    % --- Cma coefficients --- %
    % ===================== %
    TCma = table( ...
        [0.247; 0.127; 0.0675; 0.00360], ...
        [0.0167; 0.0158; 0.0128; 0.0102], ...
        [-0.765e-4; -0.930e-4; -0.926e-4; -0.822e-4], ...
        'VariableNames', {'A','B','C'}, ...
        'RowNames', { ...
            '(1) Open gearing', ...
            '(2) Commercial, enclosed units', ...
            '(3) Precision, enclosed units', ...
            '(4) Extraprecision enclosed gear units'} );

    disp('Cma Coefficients Table')
    disp(TCma)

    valid = false;
    while ~valid
        Condition = inputNum('Condition (1, 2, 3, 4): ');
        if isscalar(Condition) && Condition >= 1 && Condition <= 4
            A_cma = TCma{Condition,'A'};
            B_cma = TCma{Condition,'B'};
            C_cma = TCma{Condition,'C'};
            valid = true;
        else
            fprintf('Invalid entry. Enter a value from 1 to 4.\n\n');
        end
    end

    % =========================== %
    % --- KB (Rim thickness factor) --- %
    % =========================== %
    MbGear = zeros(1, TGears);
    KBGear = zeros(1, TGears);

    for g = 1:TGears
        s = GearShaft(g);

        tRim = rf(g) - Dshaft(s)/2;

        if tRim <= 0
            error('Invalid rim thickness for gear %d: rf <= shaft radius. Increase gear size or reduce shaft diameter.', g);
        end

        MbGear(g) = h(g) / tRim;

        if MbGear(g) < 1.2
            KBGear(g) = 1.6*log(2.242/MbGear(g));
        else
            KBGear(g) = 1.0;
        end
    end

    % ============================ %
    % --- Internal gear detection --- %
    % ============================ %
    internalGear = false(1, TGears);
    valid = false;
    while ~valid
        Config = lower(strtrim(inputStr('Is there any internal gears? (yes/no): ')));
        switch Config
            case 'yes'
                Count = inputNum('How many? ');
                for kint = 1:Count
                    Index = inputNum('Which gear is internal? Look at the previous table for location: ');
                    if Index >= 1 && Index <= TGears
                        internalGear(Index) = true;
                    else
                        fprintf('The gear does not exist in this transmission. Look at table for correct gear index.\n');
                    end
                end
                valid = true;
            case 'no'
                valid = true;
            otherwise
                fprintf('Invalid entry. Please type "yes" or "no".\n\n');
        end
    end

    % ============================= %
    % --- Geometry factors sources --- %
    % ============================= %
    fileJ_spur = "Bending Factor.xlsx";
    getJ_spur  = @(sheetStr, TeethVal) localGetJ(fileJ_spur, sheetStr, TeethVal);
    zAvail     = [17 25 35 50 85 170 1000];

    fileJ_hel  = "Bending Factor Helical Gears.xlsx";
    fileJ_helM = "Bending Factor Multiplier Helical Gears.xlsx";
    zHelMulSheets = [20 30 150 500];

    % ============================= %
    % --- Material / Cp (elastic) --- %
    % ============================= %
    Egear = zeros(1, TGears);
    vgear = zeros(1, TGears);

    DecisionMat = lower(strtrim(inputStr('Do all gears have the same material? (yes/no): ')));

    MatPairs = "";
    switch DecisionMat
        case 'yes'
            Eall = inputNum('Modulus of Elasticity (Kpsi): ');
            vall = inputNum('Poisson Ratio: ');
            Egear(:) = Eall;
            vgear(:) = vall;

        case 'no'
            MatPairs = lower(strtrim(inputStr('Do the gear pairs have their own material? (yes/no): ')));
            switch MatPairs
                case 'yes'
                    Epair = zeros(1, nPairs);
                    vpair = zeros(1, nPairs);
                    for p = 1:nPairs
                        Epair(p) = inputNum(sprintf('Modulus of Elasticity of pair %d (Kpsi): ', p));
                        vpair(p) = inputNum(sprintf('Poisson Ratio of pair %d: ', p));
                    end
                    p = 1;
                    for i = 1:2:TGears
                        Egear(i)   = Epair(p);
                        Egear(i+1) = Epair(p);
                        vgear(i)   = vpair(p);
                        vgear(i+1) = vpair(p);
                        p = p + 1;
                    end

                case 'no'
                    for g = 1:TGears
                        Egear(g) = inputNum(sprintf('Modulus of Elasticity of gear %d (Kpsi): ', g));
                        vgear(g) = inputNum(sprintf('Poisson Ratio of gear %d: ', g));
                    end

                otherwise
                    error('Invalid input for material pair choice.');
            end

        otherwise
            error('Invalid input. Please enter yes or no.');
    end

    % ====================== %
    % --- Surface Factor --- %
    % ====================== %
    Cf = 1;

    % =================================================================================== %
    % --- Allowable Bending Stress Number (St) & Allowable Contact Stress Number (Sc) --- %
    % =================================================================================== %

    Hb = zeros(1, TGears) ;
    St = zeros(1, TGears) ;
    Sc = zeros(1, TGears) ;

    switch DecisionMat
        case 'yes'
            valid = false;
            while ~valid
                Hb_local = inputNum('Brinell Hardness Number of Gears: ');
                if Hb_local >= 160 && Hb_local <= 400
                    Hb(:) = Hb_local;
                    valid = true;
                end
            end

        case 'no'
            switch MatPairs
                case 'yes'
                    valid = false;
                    while ~valid
                        for j = 2:2:TGears
                            Hb(j) = inputNum(sprintf('Brinell Hardness number for Pair %2d: ', j));
                            if Hb(j) >= 160 && Hb(j) <= 400
                                Hb(j-1) = Hb(j);
                                valid = true;
                            end
                        end
                    end

                case 'no'
                    for j = 1:TGears
                        valid = false;
                        while ~valid
                            Hb(j) = inputNum(sprintf('Brinell Hardness number for Gear %2d: ', j));
                            if Hb(j) >= 160 && Hb(j) <= 400
                                valid = true;
                            end
                        end
                    end
            end
    end

    fprintf('Gear Strength Grades\n\n');
    fprintf('Grade 1: Standard Commercial Gears\n');
    fprintf('Grade 2: Higher Strength Gears Requiring Better Metallurgy and Quality Control\n');

    Grade = lower(strtrim(inputStr('Gear Grade: ')));

    switch Grade
        case {'standard','commercial','standard gears','standard commercial', ...
              'standard commercial gears','grade 1','1'}
            for j = 1:TGears
                St(j) = 77.3*Hb(j) + 12800 ;
                Sc(j) = 322*Hb(j) + 29100 ;
            end

        case {'higher','higher strength','higher strength gears', ...
              'strength gears','grade 2','2'}
            for j = 1:TGears
                St(j) = 102*Hb(j) + 16400 ;
                Sc(j) = 349*Hb(j) + 34300 ;
            end
    end

    % =================================================== %
    % --- Stress Cycle Factor for Bending Stress (Yn) --- %
    % =================================================== %

    Yn = zeros(1, TGears);
    N  = inputNum('For how many cycles do you want this performance? ');

    switch DecisionMat

        case 'yes'

            Hb_i = Hb(1);

            if N <= 1e3
                if Hb_i <= 160
                    Yn(:) = 1.59;
                elseif Hb_i < 250
                    Yn(:) = 1.59 + (2.37-1.59)/(250-160)*(Hb_i-160);
                elseif Hb_i == 250
                    Yn(:) = 2.37;
                elseif Hb_i < 400
                    Yn(:) = 2.37 + (3.4-2.37)/(400-250)*(Hb_i-250);
                else
                    Yn(:) = 3.4;
                end

            elseif N <= 2e6
                Yn160 = 2.3194*N^(-0.0538);
                Yn250 = 4.9404*N^(-0.1045);
                Yn400 = 9.4518*N^(-0.148);

                if Hb_i <= 160
                    Yn(:) = Yn160;
                elseif Hb_i < 250
                    Yn(:) = Yn160 + (Yn250-Yn160)/(250-160)*(Hb_i-160);
                elseif Hb_i == 250
                    Yn(:) = Yn250;
                elseif Hb_i < 400
                    Yn(:) = Yn250 + (Yn400-Yn250)/(400-250)*(Hb_i-250);
                else
                    Yn(:) = Yn400;
                end

            else
                valid = false;
                while ~valid
                    Choice = lower(strtrim(inputStr('Would you like the conservative or optimistic (non-conservative) approach? ')));
                    switch Choice
                        case 'conservative'
                            Yn(:) = 1.6381*N^(-0.0323);
                            valid = true;
                        case {'optimistic','non-conservative'}
                            Yn(:) = 1.3558*N^(-0.0178);
                            valid = true;
                    end
                end
            end

        case 'no'
            switch MatPairs

                case 'yes'
                    for i = 2:2:TGears
                        Hb_i = Hb(i);

                        if N <= 1e3
                            if Hb_i <= 160
                                Yn(i-1:i) = 1.59;
                            elseif Hb_i < 250
                                Yn(i-1:i) = 1.59 + (2.37-1.59)/(250-160)*(Hb_i-160);
                            elseif Hb_i == 250
                                Yn(i-1:i) = 2.37;
                            elseif Hb_i < 400
                                Yn(i-1:i) = 2.37 + (3.4-2.37)/(400-250)*(Hb_i-250);
                            else
                                Yn(i-1:i) = 3.4;
                            end

                        elseif N <= 2e6
                            Yn160 = 2.3194*N^(-0.0538);
                            Yn250 = 4.9404*N^(-0.1045);
                            Yn400 = 9.4518*N^(-0.148);

                            if Hb_i <= 160
                                Yn(i-1:i) = Yn160;
                            elseif Hb_i < 250
                                Yn(i-1:i) = Yn160 + (Yn250-Yn160)/(250-160)*(Hb_i-160);
                            elseif Hb_i == 250
                                Yn(i-1:i) = Yn250;
                            elseif Hb_i < 400
                                Yn(i-1:i) = Yn250 + (Yn400-Yn250)/(400-250)*(Hb_i-250);
                            else
                                Yn(i-1:i) = Yn400;
                            end

                        else
                            valid = false;
                            while ~valid
                                Choice = lower(strtrim(inputStr('Would you like the conservative or optimistic (non-conservative) approach? ')));
                                switch Choice
                                    case 'conservative'
                                        Yn(i-1:i) = 1.6381*N^(-0.0323);
                                        valid = true;
                                    case {'optimistic','non-conservative'}
                                        Yn(i-1:i) = 1.3558*N^(-0.0178);
                                        valid = true;
                                end
                            end
                        end
                    end

                case 'no'
                    for g = 1:TGears
                        Hb_i = Hb(g);

                        if N <= 1e3
                            if Hb_i <= 160
                                Yn(g) = 1.59;
                            elseif Hb_i < 250
                                Yn(g) = 1.59 + (2.37-1.59)/(250-160)*(Hb_i-160);
                            elseif Hb_i == 250
                                Yn(g) = 2.37;
                            elseif Hb_i < 400
                                Yn(g) = 2.37 + (3.4-2.37)/(400-250)*(Hb_i-250);
                            else
                                Yn(g) = 3.4;
                            end

                        elseif N <= 2e6
                            Yn160 = 2.3194*N^(-0.0538);
                            Yn250 = 4.9404*N^(-0.1045);
                            Yn400 = 9.4518*N^(-0.148);

                            if Hb_i <= 160
                                Yn(g) = Yn160;
                            elseif Hb_i < 250
                                Yn(g) = Yn160 + (Yn250-Yn160)/(250-160)*(Hb_i-160);
                            elseif Hb_i == 250
                                Yn(g) = Yn250;
                            elseif Hb_i < 400
                                Yn(g) = Yn250 + (Yn400-Yn250)/(400-250)*(Hb_i-250);
                            else
                                Yn(g) = Yn400;
                            end

                        else
                            valid = false;
                            while ~valid
                                Choice = lower(strtrim(inputStr('Would you like the conservative or optimistic (non-conservative) approach? ')));
                                switch Choice
                                    case 'conservative'
                                        Yn(g) = 1.6381*N^(-0.0323);
                                        valid = true;
                                    case {'optimistic','non-conservative'}
                                        Yn(g) = 1.3558*N^(-0.0178);
                                        valid = true;
                                end
                            end
                        end
                    end
            end
    end

    % =============================== %
    % --- Temperature Factor (Kt) --- %
    % =============================== %

    Kt = 1 ;

    % =============================== %
    % --- Reliability Factor (Kr) --- %
    % =============================== %

    valid = false ;
    while~valid
        Kr = zeros (1, TGears) ;
        R = input('Gears Reliability (.5 < R <= .9999): ','s') ;
        Rn = str2double(strtrim(R));
        if strcmp(strtrim(string(R)),"-1")
            error('GEARTRAIN:USER_EXIT','User requested exit.');
        end
        if isnan(Rn) || ~isfinite(Rn)
            fprintf('Invalid input.\n\n');
            continue;
        end
        R = Rn;
        if R > 0.5 && R < 0.99
            Kr(:) = 0.658 - 0.0759*log(1-R) ;
            valid = true ;
        elseif R >= 0.99 && R <= 0.9999
            Kr(:) = 0.50 - 0.109*log(1-R) ;
            valid = true ;
        else
            fprintf('Invalid range.\n\n');
        end
    end

    % ===================================== %
    % --- Stress Cycle Life Factor (Zn) --- %
    % ===================================== %

    Zn = zeros(1, TGears) ;

    switch DecisionMat

        case 'yes'

            if N < 1e4
                Zn(:) = 1.46 ;
            elseif N > 1e4 && N < 7e6
                Zn(:) = 2.466*N^(-0.056) ;
            elseif N > 7e6
                valid = false ;
                while ~valid
                    Choice = lower(strtrim(inputStr('Would you like the conservative or optimistic (non-conservative) approach? ')));
                    switch Choice
                        case 'conservative'
                            Zn(:) = 2.466*N^(-0.056) ;
                            valid = true ;
                        case {'optimistic','non-conservative'}
                            Zn(:) = 1.4488*N^(-0.023) ;
                            valid = true ;
                        otherwise
                            fprintf('Invalid entry. Please choose between conservative or optimistic (non-conservative) approach\n');
                    end
                end
            end

        case 'no'
            switch MatPairs

                case 'yes'
                    if N < 1e4
                        Zn(:) = 1.46 ;
                    elseif N > 1e4 && N < 7e6
                        Zn(:) = 2.466*N^(-0.056) ;
                    elseif N > 7e6
                        valid = false ;
                        while ~valid
                            Choice = lower(strtrim(inputStr('Would you like the conservative or optimistic (non-conservative) approach? ')));
                            switch Choice
                                case 'conservative'
                                    Zn(:) = 2.466*N^(-0.056) ;
                                    valid = true ;
                                case {'optimistic','non-conservative'}
                                    Zn(:) = 1.4488*N^(-0.023) ;
                                    valid = true ;
                                otherwise
                                    fprintf('Invalid entry. Please choose between conservative or optimistic (non-conservative) approach\n');
                            end
                        end
                    end

                case 'no'
                    if N < 1e4
                        Zn(:) = 1.46 ;
                    elseif N > 1e4 && N < 7e6
                        Zn(:) = 2.466*N^(-0.056) ;
                    elseif N > 7e6
                        valid = false ;
                        while ~valid
                            Choice = lower(strtrim(inputStr('Would you like the conservative or optimistic (non-conservative) approach? ')));
                            switch Choice
                                case 'conservative'
                                    Zn(:) = 2.466*N^(-0.056) ;
                                    valid = true ;
                                case {'optimistic','non-conservative'}
                                    Zn(:) = 1.4488*N^(-0.023) ;
                                    valid = true ;
                                otherwise
                                    fprintf('Invalid entry. Please choose between conservative or optimistic (non-conservative) approach\n');
                            end
                        end
                    end
            end
    end

    % ========================================================== %
    % --- Hardness Ratio Factors for Pitting Resistance (CH) --- %
    % ========================================================== %

    CH = ones(1, TGears);

    for i = 2:2:TGears

        Hb1 = Hb(i-1);
        Hb2 = Hb(i);

        HR = max(Hb1,Hb2) / min(Hb1,Hb2);

        if HR < 1.2
            Ach = 0;
        elseif HR >= 1.2 && HR <= 1.7
            Ach = 8.98e-3*HR - 8.29e-3;
        elseif HR > 1.7
            Ach = 0.00698;
        else
            Ach = 0;
        end

        mg = Systeeth(i) / Systeeth(i-1);

        CH(i-1) = 1 + Ach*(mg - 1);
        CH(i)   = CH(i-1);

    end

    % ============================ %
    % --- Stress table storage --- %
    % ============================ %
    nRowsMax = 2*(nPairs);
    MeshCol = zeros(nRowsMax,1);
    TypeCol = strings(nRowsMax,1);
    GearACol = zeros(nRowsMax,1);
    GearMCol = zeros(nRowsMax,1);
    TeethACol = zeros(nRowsMax,1);
    TeethMCol = zeros(nRowsMax,1);

    KoCol = zeros(nRowsMax,1);
    KvCol = zeros(nRowsMax,1);
    KsCol = zeros(nRowsMax,1);
    KmCol = zeros(nRowsMax,1);
    KBCol = zeros(nRowsMax,1);
    CpCol = zeros(nRowsMax,1);
    CfCol = zeros(nRowsMax,1);
    ICol  = zeros(nRowsMax,1);
    JCol  = zeros(nRowsMax,1);

    WtCol = zeros(nRowsMax,1);
    FtCol = zeros(nRowsMax,1);
    FrCol = zeros(nRowsMax,1);
    FaCol = zeros(nRowsMax,1);

    BStressCol = zeros(nRowsMax,1);
    CStressCol = zeros(nRowsMax,1);

    SFCol = zeros(nRowsMax,1);
    SHCol = zeros(nRowsMax,1);

    StCol = zeros(nRowsMax,1);
    ScCol = zeros(nRowsMax,1);
    YnCol = zeros(nRowsMax,1);
    ZnCol = zeros(nRowsMax,1);
    CHCol = zeros(nRowsMax,1);

    row = 0;

    fprintf('\n==================== STRESS RESULTS (table) ====================\n');

    for i = 1:2:TGears-1

        gA = i;
        gB = i+1;

        pMesh = (i+1)/2;

        Phit_local = phi_tMesh(pMesh);
        mN_local   = mNMesh(pMesh);

        if Systeeth(gA) <= Systeeth(gB)
            gP = gA; gG = gB;
        else
            gP = gB; gG = gA;
        end

        Wt = (24 * Tgear(gP)) / d(gP);

        for pass = 1:2

            if pass == 1
                GA = gA; GM = gB;
            else
                GA = gB; GM = gA;
            end

            TeethReal  = Systeeth(GA);
            MatingReal = Systeeth(GM);

            TeethEff_spur  = NvGear(GA);
            MatingEff_spur = NvGear(GM);

            Pitch  = PdGear(GA);
            F_i    = F(GA);
            d_i    = d(GA);
            Kv_i   = Kv(GA);
            Cpm    = CpmGear(GA);
            KB     = KBGear(GA);

            TeethLewis = TeethEff_spur;
            if approxSmallTeeth && TeethLewis < MIN_BENDING_TEETH
                TeethLewis = MIN_BENDING_TEETH;
            end
            Y = localGetLewisY(TLewis, TeethLewis);
            Ks = 1.192*(F_i*sqrt(Y)/Pitch)^(0.0535);

            if F_i <= 1
                Cpf = F_i/(10*d_i) - 0.025;
            elseif F_i <= 17
                Cpf = F_i/(10*d_i) - 0.0375 + 0.0125*F_i;
            elseif F_i <= 40
                Cpf = F_i/(10*d_i) - 0.1109 + 0.0207*F_i - 0.000228*F_i^2;
            else
                error("Face Width (F) must be <= 40 in for this Cpf correlation.");
            end

            Cma = A_cma + B_cma*F_i + C_cma*F_i^2;
            Km  = 1 + Cmc*(Cpf*Cpm + Cma*Ce);

            if isHelicalMesh(pMesh)

                analysisTeeth_forHelJ = TeethReal;
                if approxSmallTeeth && analysisTeeth_forHelJ < MIN_BENDING_TEETH
                    analysisTeeth_forHelJ = MIN_BENDING_TEETH;
                end

                J75 = localGetJ_auto(fileJ_hel, "75", analysisTeeth_forHelJ);

                if MatingReal == 75
                    J = J75;
                else
                    Mmult = localGetHelicalMultiplier(fileJ_helM, zHelMulSheets, MatingReal, analysisTeeth_forHelJ, approxSmallTeeth, MIN_BENDING_TEETH);
                    J = J75 * Mmult;
                end

            else

                Teeth_forSpurJ  = TeethEff_spur;
                Mating_forSpurJ = MatingEff_spur;

                if approxSmallTeeth
                    Teeth_forSpurJ  = max(Teeth_forSpurJ,  MIN_BENDING_TEETH);
                    Mating_forSpurJ = max(Mating_forSpurJ, MIN_BENDING_TEETH);
                end

                if Mating_forSpurJ == 1
                    J = getJ_spur("Load at tip", Teeth_forSpurJ);

                elseif any(Mating_forSpurJ == zAvail)
                    J = getJ_spur(string(Mating_forSpurJ), Teeth_forSpurJ);

                elseif Mating_forSpurJ < zAvail(1)
                    J = getJ_spur("17", Teeth_forSpurJ);

                elseif Mating_forSpurJ > zAvail(end)
                    J = getJ_spur(string(zAvail(end)), Teeth_forSpurJ);

                else
                    i2 = find(zAvail >= Mating_forSpurJ, 1, 'first');
                    i1 = i2 - 1;
                    z1 = zAvail(i1); z2 = zAvail(i2);
                    J1 = getJ_spur(string(z1), Teeth_forSpurJ);
                    J2 = getJ_spur(string(z2), Teeth_forSpurJ);
                    J  = J1 + (J2 - J1) * (Mating_forSpurJ - z1) / (z2 - z1);
                end
            end

            dp = d(gP);
            mp = Systeeth(gG)/Systeeth(gP);

            if isHelicalMesh(pMesh)
                rP  = d(gP)/2;
                rG  = d(gG)/2;
                a_local  = a(gP);
                rbP = rb(gP);
                rbG = rb(gG);

                Z = sqrt((rP + a_local)^2 - rbP^2) + sqrt((rG + a_local)^2 - rbG^2) - (rP + rG)*sind(Phit_local);

                pn = pi / PdGear(gP);
                pN = pn * cosd(Phi);
                mN_local = pN/(0.95*Z);
            else
                mN_local = 1.0;
            end

            isInternal = internalGear(gP) || internalGear(gG);

            if isInternal
                I_mesh = (cosd(Phit_local)*sind(Phit_local)/(2*mN_local))*(mp/(mp-1));
            else
                I_mesh = (cosd(Phit_local)*sind(Phit_local)/(2*mN_local))*(mp/(mp+1));
            end

            va = vgear(GA); vm = vgear(GM);
            Ea = Egear(GA); Em = Egear(GM);
            Cp = sqrt(1/(pi*((1-va^2)/Ea + (1-vm^2)/Em)));

            BStress = Wt * Ko * Kv_i * Ks * Pitch * Km * KB / (F_i * J);
            CStress = Cp * sqrt( (Wt*Ko*Kv_i*Ks) * (Km/(dp*F_i)) * (Cf/I_mesh) );

            SF = (St(GA) * Yn(GA)) / (Kt * Kr(GA) * BStress);
            SH = (Sc(GA) * Zn(GA) * CH(GA)) / (Kt * Kr(GA) * CStress);

            if isHelicalMesh(pMesh)
                Ft = Wt;
                Fr = Ft * tand(Phit_local);
                Fa = Ft * tand(psiMesh(pMesh));
            else
                Ft = Wt;
                Fr = Ft * tand(Phi);
                Fa = 0;
            end

            row = row + 1;

            MeshCol(row)   = pMesh;
            if isHelicalMesh(pMesh)
                TypeCol(row) = "helical";
            else
                TypeCol(row) = "spur";
            end

            GearACol(row)  = GA;
            GearMCol(row)  = GM;
            TeethACol(row) = TeethReal;
            TeethMCol(row) = MatingReal;

            KoCol(row) = Ko;
            KvCol(row) = Kv_i;
            KsCol(row) = Ks;
            KmCol(row) = Km;
            KBCol(row) = KB;
            CpCol(row) = Cp;
            CfCol(row) = Cf;
            ICol(row)  = I_mesh;
            JCol(row)  = J;

            WtCol(row) = Wt;
            FtCol(row) = Ft;
            FrCol(row) = Fr;
            FaCol(row) = Fa;

            BStressCol(row) = BStress;
            CStressCol(row) = CStress;

            SFCol(row) = SF;
            SHCol(row) = SH;

            StCol(row) = St(GA);
            ScCol(row) = Sc(GA);
            YnCol(row) = Yn(GA);
            ZnCol(row) = Zn(GA);
            CHCol(row) = CH(GA);

        end
    end

    % ========================= %
    % --- Build & display table --- %
    % ========================= %
    MeshCol   = MeshCol(1:row);
    TypeCol   = TypeCol(1:row);
    GearACol  = GearACol(1:row);
    GearMCol  = GearMCol(1:row);
    TeethACol = TeethACol(1:row);
    TeethMCol = TeethMCol(1:row);

    KoCol = KoCol(1:row);
    KvCol = KvCol(1:row);
    KsCol = KsCol(1:row);
    KmCol = KmCol(1:row);
    KBCol = KBCol(1:row);
    CpCol = CpCol(1:row);
    CfCol = CfCol(1:row);
    ICol  = ICol(1:row);
    JCol  = JCol(1:row);

    WtCol = WtCol(1:row);
    FtCol = FtCol(1:row);
    FrCol = FrCol(1:row);
    FaCol = FaCol(1:row);

    BStressCol = BStressCol(1:row);
    CStressCol = CStressCol(1:row);

    SFCol = SFCol(1:row);
    SHCol = SHCol(1:row);

    StCol = StCol(1:row);
    ScCol = ScCol(1:row);
    YnCol = YnCol(1:row);
    ZnCol = ZnCol(1:row);
    CHCol = CHCol(1:row);

    StressTable = table( ...
        MeshCol, TypeCol, GearACol, GearMCol, TeethACol, TeethMCol, ...
        KoCol, KvCol, KsCol, KmCol, KBCol, CpCol, CfCol, ICol, JCol, ...
        WtCol, FtCol, FrCol, FaCol, ...
        StCol, YnCol, ScCol, ZnCol, CHCol, ...
        BStressCol, CStressCol, ...
        SFCol, SHCol );

    StressPrint_Transposed(StressTable);

    valid_main = true;

end

catch ME
    if strcmp(ME.identifier,'GEARTRAIN:USER_EXIT')
        fprintf('\nProgram exited by user (-1).\n');
    else
        rethrow(ME);
    end
end

% ======================= %
% --- Helper Functions --- %
% ======================= %

function x = inputNum(prompt)
    xraw = input(prompt,'s');
    s = strtrim(string(xraw));

    if s == "-1"
        error('GEARTRAIN:USER_EXIT','User requested exit.');
    end

    xn = str2double(s);
    if isnan(xn) || ~isfinite(xn)
        error('GEARTRAIN:BAD_NUM','Invalid numeric input.');
    end
    x = xn;
end

function v = inputNumVec(prompt)
    vraw = input(prompt,'s');
    s = strtrim(string(vraw));

    if s == "-1"
        error('GEARTRAIN:USER_EXIT','User requested exit.');
    end

    v = str2num(char(s)); %#ok<ST2NM>
    if isempty(v) || ~isnumeric(v)
        error('GEARTRAIN:BAD_VEC','Invalid numeric vector input.');
    end
    if any(v(:) == -1)
        error('GEARTRAIN:USER_EXIT','User requested exit.');
    end
end

function s = inputStr(prompt)
    sraw = input(prompt,'s');
    t = strtrim(string(sraw));

    if t == "-1"
        error('GEARTRAIN:USER_EXIT','User requested exit.');
    end

    s = char(t);
end

function J = localGetJ(file, sheetStr, TeethVal)
    T = readtable(file, "Sheet", sheetStr);
    nums = T{:,1};
    facs = T{:,2};

    idx = (nums == TeethVal);
    if ~any(idx)
        error('GEARTRAIN:J_NOT_FOUND', "Teeth value %d not found in sheet: %s", TeethVal, sheetStr);
    end
    J = facs(find(idx, 1, 'first'));
end

function J = localGetJ_auto(file, preferredSheet, TeethVal)
    try
        T = readtable(file, "Sheet", preferredSheet);
    catch
        T = readtable(file);
    end

    nums = T{:,1};
    facs = T{:,2};

    idx = (nums == TeethVal);
    if ~any(idx)
        error('GEARTRAIN:J_NOT_FOUND', ...
              "Teeth value %d not found in file %s (sheet %s or default).", TeethVal, file, preferredSheet);
    end
    J = facs(find(idx, 1, 'first'));
end

function M = localGetHelicalMultiplier(fileMul, zSheets, matingTeethReal, analysisTeethReal, approxSmallTeeth, MIN_BENDING_TEETH)

    z = matingTeethReal;
    if approxSmallTeeth && z < MIN_BENDING_TEETH
        z = MIN_BENDING_TEETH;
    end

    if z <= zSheets(1)
        M = localGetJ_auto(fileMul, string(zSheets(1)), analysisTeethReal);
        return;
    end
    if z >= zSheets(end)
        M = localGetJ_auto(fileMul, string(zSheets(end)), analysisTeethReal);
        return;
    end

    if any(z == zSheets)
        M = localGetJ_auto(fileMul, string(z), analysisTeethReal);
        return;
    end

    i2 = find(zSheets >= z, 1, 'first');
    i1 = i2 - 1;

    z1 = zSheets(i1);
    z2 = zSheets(i2);

    M1 = localGetJ_auto(fileMul, string(z1), analysisTeethReal);
    M2 = localGetJ_auto(fileMul, string(z2), analysisTeethReal);

    M = M1 + (M2 - M1) * (z - z1) / (z2 - z1);
end

function Y = localGetLewisY(TLewis, TeethVal)
    nums = TLewis{:,1};
    facs = TLewis{:,2};

    idx = (nums == TeethVal);
    if ~any(idx)
        error('GEARTRAIN:LEWIS_NOT_FOUND', "Lewis Factor: Teeth value %d not found in that sheet.", TeethVal);
    end
    Y = facs(find(idx, 1, 'first'));
end

function s = centerFmt(val, width, fmt)
    txt = sprintf(fmt, val);
    n = length(txt);

    if n >= width
        s = txt(1:width);
    else
        padL = floor((width - n)/2);
        padR = width - n - padL;
        s = [repmat(' ',1,padL) txt repmat(' ',1,padR)];
    end
end

function StressPrint_Transposed(T)
    fprintf('\n');
    n = height(T);

    colW = 14;
    rowW = 10;

    fprintf('%-*s', rowW, '');
    for j = 1:n
        fprintf('%-*s', colW, sprintf('Gear%d', j));
    end
    fprintf('\n');

    fprintf('%-*s', rowW, '');
    for j = 1:n
        fprintf('%-*s', colW, repmat('-',1,min(colW-2,8)));
    end
    fprintf('\n');

    intNames = {'Mesh','GearA','GearM','TeethA','TeethM'};
    rows = { ...
        'Mesh',   T.MeshCol; ...
        'Type',   string(T.TypeCol); ...
        'GearA',  T.GearACol; ...
        'GearM',  T.GearMCol; ...
        'TeethA', T.TeethACol; ...
        'TeethM', T.TeethMCol; ...
        'Ko',     T.KoCol; ...
        'Kv',     T.KvCol; ...
        'Ks',     T.KsCol; ...
        'Km',     T.KmCol; ...
        'KB',     T.KBCol; ...
        'Cp',     T.CpCol; ...
        'Cf',     T.CfCol; ...
        'I',      T.ICol; ...
        'J',      T.JCol; ...
        'Wt',     T.WtCol; ...
        'Ft',     T.FtCol; ...
        'Fr',     T.FrCol; ...
        'Fa',     T.FaCol; ...
        'St',     T.StCol; ...
        'Yn',     T.YnCol; ...
        'Sc',     T.ScCol; ...
        'Zn',     T.ZnCol; ...
        'CH',     T.CHCol; ...
        'BStress',T.BStressCol; ...
        'CStress',T.CStressCol; ...
        'SF',     T.SFCol; ...
        'SH',     T.SHCol; ...
    };

    for r = 1:size(rows,1)
        name = rows{r,1};
        vec  = rows{r,2};

        fprintf('%-*s', rowW, name);

        if strcmp(name,'Type')
            for j = 1:n
                fprintf('%-*s', colW, char(vec(j)));
            end
            fprintf('\n');
            continue;
        end

        if any(strcmp(name, intNames))
            for j = 1:n
                fprintf('%-*s', colW, sprintf('%d', vec(j)));
            end
            fprintf('\n');
            continue;
        end

        for j = 1:n
            fprintf('%-*s', colW, sprintf('%.6g', vec(j)));
        end
        fprintf('\n');
    end

    fprintf('\n');
end
