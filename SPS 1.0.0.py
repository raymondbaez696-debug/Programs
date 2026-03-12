import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy.sparse as sp_sparse
import scipy.sparse.linalg as spla

# ============================================================
# GENERIC INPUT HELPERS (used by both Structural + Heat FDM)
# ============================================================

def ask_choice(prompt, valid_options):
    valid_upper = [v.upper() for v in valid_options]
    while True:
        ans = input(prompt).strip().upper()
        if ans in valid_upper:
            return ans
        print(f"Invalid input. Valid options are: {', '.join(valid_options)}\n")

def ask_float(prompt, positive_only=False):
    while True:
        try:
            val = float(input(prompt))
            if positive_only and val <= 0:
                print("Value must be positive.\n")
                continue
            return val
        except ValueError:
            print("Invalid number. Try again.\n")

def ask_int(prompt, min_val=None, max_val=None):
    while True:
        try:
            val = int(input(prompt))
            if min_val is not None and val < min_val:
                print(f"Value must be ≥ {min_val}\n")
                continue
            if max_val is not None and val > max_val:
                print(f"Value must be ≤ {max_val}\n")
                continue
            return val
        except ValueError:
            print("Invalid integer. Try again.\n")

def ask_yes_no(prompt):
    while True:
        a = input(prompt + " [1 = Yes, 0 = No]: ").strip()
        if a in ("1", "0"):
            return a == "1"
        print("Please enter 1 or 0.\n")

# NEW: enforce 0 ≤ x ≤ L
def ask_position(prompt, L):
    while True:
        x = ask_float(prompt)
        if 0.0 <= x <= L:
            return x
        print(f"Distance must be between 0 and {L} m. Please re-enter.\n")


# ============================================================
# STRUCTURAL SPS PART
# ============================================================

SUPPORT_LEGEND = [
    ("C",  "Cable"),
    ("SS", "Smooth Support"),
    ("JB", "Journal Bearing"),
    ("R",  "Roller"),
    ("EP", "External Pin"),
    ("IP", "Internal Pin"),
    ("TB", "Thrust Bearing"),
    ("F",  "Fixed"),
]

SUPPORT_DOF = {
    "C":  {"H": False, "V": True,  "M": False},
    "SS": {"H": False, "V": True,  "M": False},
    "JB": {"H": False, "V": True,  "M": False},
    "R":  {"H": False, "V": True,  "M": False},
    "EP": {"H": True,  "V": True,  "M": False},
    "IP": {"H": True,  "V": True,  "M": False},
    "TB": {"H": True,  "V": True,  "M": False},
    "F":  {"H": True,  "V": True,  "M": True },
}

SUPPORT_BC = {
    "C":  {"v": 0, "theta": None},
    "SS": {"v": 0, "theta": None},
    "JB": {"v": 0, "theta": None},
    "R":  {"v": 0, "theta": None},
    "EP": {"v": 0, "theta": None},
    "IP": {"v": 0, "theta": None},
    "TB": {"v": 0, "theta": None},
    "F":  {"v": 0, "theta": 0},
}

def show_supports_matplotlib():
    fig = plt.figure(figsize=(6, 6))
    fig.patch.set_facecolor("white")
    plt.subplots_adjust(left=0.08, right=0.92, top=0.95, bottom=0.05)
    ax = fig.add_subplot(111)
    ax.axis("off")

    lines = []
    lines.append("Legend (Supports)")
    lines.append("Abbrev   Description")
    lines.append("---------------------------")
    for abbr, desc in SUPPORT_LEGEND:
        lines.append(f"{abbr:<7} {desc}")
    text = "\n".join(lines)

    ax.text(
        0.02, 0.98, text,
        transform=ax.transAxes,
        va="top", ha="left",
        family="monospace",
        fontsize=10,
    )

    plt.show(block=False)
    plt.pause(0.1)


def animate_diagrams(xV_all, V_all, xM_all, Mstep_all, x_all, th_all, v_all):
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    ax1, ax2, ax3, ax4 = axes

    line_V,  = ax1.plot(xV_all, np.zeros_like(V_all))
    line_M,  = ax2.plot(xM_all, np.zeros_like(Mstep_all))
    line_th, = ax3.plot(x_all, np.zeros_like(th_all))
    line_v,  = ax4.plot(x_all, np.zeros_like(v_all))

    ax1.set_ylabel("V(x) [N]")
    ax1.set_title("Shear diagram")
    ax1.grid(True)

    ax2.set_ylabel("M(x) [N·m]")
    ax2.set_title("Moment diagram")
    ax2.grid(True)

    ax3.set_ylabel("θ(x) [rad]")
    ax3.set_title("Slope diagram")
    ax3.grid(True)

    ax4.set_xlabel("x [m]")
    ax4.set_ylabel("v(x) [m]")
    ax4.set_title("Deflection diagram")
    ax4.grid(True)

    ax1.set_xlim(xV_all.min(), xV_all.max())
    ax2.set_xlim(xM_all.min(), xM_all.max())
    ax3.set_xlim(x_all.min(),  x_all.max())
    ax4.set_xlim(x_all.min(),  x_all.max())

    def set_ylim_from_data(ax, ydata):
        y_min = float(np.min(ydata))
        y_max = float(np.max(ydata))
        if abs(y_max - y_min) < 1e-9:
            margin = 1.0 if abs(y_max) < 1e-9 else 0.1 * abs(y_max)
            ax.set_ylim(y_min - margin, y_max + margin)
        else:
            margin = 0.1 * (y_max - y_min)
            ax.set_ylim(y_min - margin, y_max + margin)

    set_ylim_from_data(ax1, V_all)
    set_ylim_from_data(ax2, Mstep_all)
    set_ylim_from_data(ax3, th_all)
    set_ylim_from_data(ax4, v_all)

    n_up = 60
    n_frames = 2 * n_up - 2

    def lam_from_frame(frame_idx):
        if frame_idx < n_up:
            return frame_idx / (n_up - 1)
        else:
            return (n_frames - frame_idx) / (n_up - 1)

    def update(frame):
        lam = lam_from_frame(frame)
        line_V.set_ydata(lam * V_all)
        line_M.set_ydata(lam * Mstep_all)
        line_th.set_ydata(lam * th_all)
        line_v.set_ydata(lam * v_all)
        return line_V, line_M, line_th, line_v

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=n_frames,
        interval=80,
        blit=False
    )

    plt.tight_layout()
    return fig, anim


# ============================================================
# STRUCTURAL QUERY MODE (SHEAR / MOMENT / SLOPE / DEFLECTION / THERMAL)
# ============================================================

def structural_query_mode(L,
                          xV_all, V_all,
                          xM_all, Mstep_all,
                          x_all, th_all, v_all,
                          thermal_data=None):
    """
    Text-menu query mode for structural results.
    thermal_data = (T_C, x_T, y_T, dx_T, dy_T, k_T) if available.
    """

    # ------------ helpers for structural quantities -------------
    def ask_x():
        while True:
            try:
                xq = float(input(f"Enter x (meters, 0 <= x <= {L:.6f}): "))
                if 0.0 <= xq <= L:
                    return xq
                print("x outside domain.\n")
            except ValueError:
                print("Invalid number.\n")

    def interp_value(x_arr, y_arr, xq):
        return float(np.interp(xq, x_arr, y_arr))

    def submenu(title, x_arr, y_arr, units):
        while True:
            print(f"\n--- {title} query ---")
            print("  1 = Value at a point x")
            print("  2 = Show max and min over the beam")
            print("  3 = Plot diagram again")
            print("  0 = Back")
            c = input("Choose option: ").strip()

            if c == "0":
                break
            elif c == "1":
                xq = ask_x()
                val = interp_value(x_arr, y_arr, xq)
                print(f"{title} at x = {xq:.6f} m: {val:.6g} {units}")
            elif c == "2":
                idx_max = int(np.argmax(y_arr))
                idx_min = int(np.argmin(y_arr))
                print(f"Max {title}: {y_arr[idx_max]:.6g} {units} at x = {x_arr[idx_max]:.6f} m")
                print(f"Min {title}: {y_arr[idx_min]:.6g} {units} at x = {x_arr[idx_min]:.6f} m")
            elif c == "3":
                plt.figure()
                plt.plot(x_arr, y_arr)
                plt.xlabel("x [m]")
                plt.ylabel(f"{title} [{units}]")
                plt.title(f"{title} diagram")
                plt.grid(True)
                plt.tight_layout()
                plt.show(block=False)
                plt.pause(0.001)
            else:
                print("Invalid choice.\n")

    # ---------------------- Thermal submenu ----------------------
    def submenu_thermal():
        if thermal_data is None:
            print("\nNo thermal data (thermal coupling not used).")
            return

        T_C, x_T, y_T, dx_T, dy_T, k_T = thermal_data
        Ny, Nx = T_C.shape

        while True:
            print("\n--- Thermal query ---")
            print("  1 = Temperature at (x,y)")
            print("  2 = Horizontal section T(x) at fixed y")
            print("  3 = Vertical section T(y) at fixed x")
            print("  4 = Show peak temperatures")
            print("  5 = Heat-flux vector field q(x,y)")
            print("  0 = Back")
            c = input("Choose: ").strip()

            if c == "0":
                break

            elif c == "1":
                try:
                    raw = input("Point (x,y) in meters (e.g. 0.25,0.01): ")
                    if "(" in raw:
                        raw = raw.strip("()")
                    xs, ys = raw.replace(",", " ").split()
                    xq = float(xs)
                    yq = float(ys)
                except Exception:
                    print("Invalid input.\n")
                    continue

                if not (0 <= xq <= x_T[-1] and 0 <= yq <= y_T[-1]):
                    print("Point outside domain.\n")
                    continue

                i = np.argmin(abs(x_T - xq))
                j = np.argmin(abs(y_T - yq))
                print(f"T({xq:.6f},{yq:.6f}) ≈ {T_C[j, i]:.4f} °C")

            elif c == "2":
                try:
                    yq = float(input("Enter y (meters): "))
                except Exception:
                    print("Invalid input.\n")
                    continue
                if not (0 <= yq <= y_T[-1]):
                    print("Outside domain.\n")
                    continue
                j = np.argmin(abs(y_T - yq))
                plt.figure()
                plt.plot(x_T, T_C[j, :])
                plt.xlabel("x [m]")
                plt.ylabel("T [°C]")
                plt.title(f"T(x) at y = {y_T[j]:.4f} m")
                plt.grid(True)
                plt.tight_layout()
                plt.show(block=False)
                plt.pause(0.001)

            elif c == "3":
                try:
                    xq = float(input("Enter x (meters): "))
                except Exception:
                    print("Invalid input.\n")
                    continue
                if not (0 <= xq <= x_T[-1]):
                    print("Outside domain.\n")
                    continue
                i = np.argmin(abs(x_T - xq))
                plt.figure()
                plt.plot(y_T, T_C[:, i])
                plt.xlabel("y [m]")
                plt.ylabel("T [°C]")
                plt.title(f"T(y) at x = {x_T[i]:.4f} m")
                plt.grid(True)
                plt.tight_layout()
                plt.show(block=False)
                plt.pause(0.001)

            elif c == "4":
                print(f"Max T = {float(np.max(T_C)):.3f} °C")
                print(f"Min T = {float(np.min(T_C)):.3f} °C")

            elif c == "5":
                # Heat-flux vector field (no scaling, physical coords)
                dTdx = np.zeros_like(T_C)
                dTdy = np.zeros_like(T_C)

                dTdx[:, 1:-1] = (T_C[:, 2:] - T_C[:, :-2]) / (2*dx_T)
                dTdy[1:-1, :] = (T_C[2:, :] - T_C[:-2, :]) / (2*dy_T)
                dTdx[:, 0]  = (T_C[:, 1] - T_C[:, 0]) / dx_T
                dTdx[:, -1] = (T_C[:, -1] - T_C[:, -2]) / dx_T
                dTdy[0, :]  = (T_C[1, :] - T_C[0, :]) / dy_T
                dTdy[-1, :] = (T_C[-1, :] - T_C[-2, :]) / dy_T

                qx_field = -k_T * dTdx
                qy_field = -k_T * dTdy

                Xg, Yg = np.meshgrid(x_T, y_T)

                total_nodes = Nx * Ny
                target_arrows = 150
                stride = max(1, int(np.sqrt(total_nodes / target_arrows)))

                Xs = Xg[::stride, ::stride]
                Ys = Yg[::stride, ::stride]
                qxs = qx_field[::stride, ::stride]
                qys = qy_field[::stride, ::stride]

                mag = np.sqrt(qxs**2 + qys**2)
                eps = 1e-12
                qxs_n = qxs / (mag + eps)
                qys_n = qys / (mag + eps)

                plt.figure(figsize=(10, 3.5))
                levels = np.linspace(float(T_C.min()), float(T_C.max()), 80)
                plt.contourf(Xg, Yg, T_C, levels=levels, cmap="turbo")
                cbar = plt.colorbar()
                cbar.set_label("T [°C]")
                plt.quiver(Xs, Ys, qxs_n, qys_n, pivot="mid", scale=20)
                plt.xlabel("x [m]")
                plt.ylabel("y [m]")
                plt.title("Heat-flux direction field q = -k ∇T")
                plt.gca().set_aspect("equal")
                plt.tight_layout()
                plt.show(block=False)
                plt.pause(0.001)

            else:
                print("Invalid option.\n")

    # ------------------------- Main loop -------------------------
    while True:
        print("\n=== Structural query mode ===")
        print("  1 = Shear V(x)")
        print("  2 = Moment M(x)")
        print("  3 = Slope θ(x)")
        print("  4 = Deflection v(x)")
        print("  5 = Thermal results")
        print("  0 = Exit structural query mode")
        choice = input("Choose option: ").strip()

        if choice == "0":
            break
        elif choice == "1":
            submenu("Shear V(x)", xV_all, V_all, "N")
        elif choice == "2":
            submenu("Moment M(x)", xM_all, Mstep_all, "N·m")
        elif choice == "3":
            submenu("Slope θ(x)", x_all, th_all, "rad")
        elif choice == "4":
            submenu("Deflection v(x)", x_all, v_all, "m")
        elif choice == "5":
            submenu_thermal()
        else:
            print("Invalid choice.\n")


# ============================================================
# THERMAL-STRUCTURAL COUPLING HELPERS
# ============================================================

def compute_thermal_section_data(T_C, x_arr, y_arr, Ly_section, alpha, T_ref_C, I, b_width):
    """
    Compute:
      - T_avg(x): area-average temperature across depth at each x
      - kappa_T(x): thermal curvature due to non-uniform T(y)
    """
    T_C = np.asarray(T_C)
    x_arr = np.asarray(x_arr)
    y_arr = np.asarray(y_arr)

    Ny, Nx = T_C.shape
    if x_arr.shape[0] != Nx:
        raise ValueError("x_arr length must match T_C.shape[1]")
    if y_arr.shape[0] != Ny:
        raise ValueError("y_arr length must match T_C.shape[0]")

    if Ny < 2:
        dy = 0.0
    else:
        dy = float(y_arr[1] - y_arr[0])

    # y measured from centroid (symmetric cross-section)
    y_nodes = y_arr - Ly_section / 2.0

    T_avg = np.zeros(Nx)
    kappa_T = np.zeros(Nx)

    for i in range(Nx):
        col = T_C[:, i]

        # Area-average temperature across depth
        if Ny > 1:
            T_avg[i] = np.sum(col) * dy / Ly_section
        else:
            T_avg[i] = col[0]

        # Thermal curvature term:
        # kappa_T = alpha/I * ∫ (T - T_ref) y dA,  dA = b_width * dy
        if Ny > 1 and I > 0:
            integrand = (col - T_ref_C) * y_nodes
            integral_y = np.sum(integrand) * dy * b_width
            kappa_T[i] = alpha * integral_y / I
        else:
            kappa_T[i] = 0.0

    return T_avg, kappa_T


def compute_thermal_bc_shifts(x_arr, kappa_T):
    """
    From discrete kappa_T(x), compute:
      theta_T(x) = ∫ kappa_T dx
      v_T(x)     = ∫ theta_T dx

    Return:
      thetaT0, thetaTL, vT0, vTL, theta_T(x), v_T(x)
    """
    x_arr = np.asarray(x_arr)
    kappa_T = np.asarray(kappa_T)

    N = len(x_arr)
    if N < 2:
        theta_T = np.zeros_like(x_arr)
        v_T = np.zeros_like(x_arr)
        return 0.0, 0.0, 0.0, 0.0, theta_T, v_T

    theta_T = np.zeros(N)
    v_T = np.zeros(N)

    for i in range(1, N):
        dx = x_arr[i] - x_arr[i-1]
        theta_T[i] = theta_T[i-1] + 0.5 * (kappa_T[i] + kappa_T[i-1]) * dx
        v_T[i]     = v_T[i-1]     + 0.5 * (theta_T[i] + theta_T[i-1]) * dx

    thetaT0 = theta_T[0]
    thetaTL = theta_T[-1]
    vT0     = v_T[0]
    vTL     = v_T[-1]

    return thetaT0, thetaTL, vT0, vTL, theta_T, v_T


# ============================================================
# STRUCTURAL SOLVER (with optional thermal coupling)
# ============================================================

def run_structural():
    show_supports_matplotlib()

    print("Available supports:")
    for abbr, desc in SUPPORT_LEGEND:
        print(f"  {abbr} = {desc}")

    while True:
        config_str = input(
            "\nEnter support configuration (e.g. C-EP, R-F, F-F): "
        ).strip().upper()
        parts = config_str.split("-")
        if len(parts) == 2 and parts[0] in SUPPORT_DOF and parts[1] in SUPPORT_DOF:
            s1, s2 = parts[0], parts[1]
            break
        print("Invalid configuration. Example: C-EP or F-R.\n")

    dofL = SUPPORT_DOF[s1]
    dofR = SUPPORT_DOF[s2]
    bcL = SUPPORT_BC[s1]
    bcR = SUPPORT_BC[s2]

    angle_left = None
    angle_right = None

    if s1 == "C":
        angle_left = ask_float("Enter cable angle at LEFT support (deg, from +y-axis CCW): ")
    elif s1 == "SS":
        angle_left = ask_float("Enter normal angle of LEFT smooth support (deg, from +y-axis CCW): ")

    if s2 == "C":
        angle_right = ask_float("Enter cable angle at RIGHT support (deg, from +y-axis CCW): ")
    elif s2 == "SS":
        angle_right = ask_float("Enter normal angle of RIGHT smooth support (deg, from +y-axis CCW): ")

    L  = ask_float("Total beam length L (m): ", positive_only=True)

    Fv = ask_int("How many transverse forces? ", min_val=0)
    Fa = ask_int("How many eccentric axial forces? ", min_val=0)
    Fd = ask_int("How many diagonal forces? ", min_val=0)
    Fm = ask_int("How many concentrated moments? ", min_val=0)

    E  = ask_float("Modulus of Elasticity (GPa): ", positive_only=True) * 1e9

    geo_choice = ask_choice("Enter Cross-Sectional Geometry (R = Rectangle, C = Circle): ", ["R", "C"])
    if geo_choice == "C":
        d = ask_float("Circle diameter (m): ", positive_only=True)
        I = np.pi * d**4 / 64.0
        Ly_section = d
        b_width = d      # approx
        A_section = 0.25 * np.pi * d**2
    else:
        w = ask_float("Rectangle width (m): ", positive_only=True)
        h = ask_float("Rectangle height (m): ", positive_only=True)
        I = w * h**3 / 12.0
        Ly_section = h
        b_width = w
        A_section = w * h

    # ---- THERMAL COUPLING: solve HEAT FIRST, then structural ----

    include_thermal = ask_yes_no("\nInclude coupled thermal analysis for this beam (2D FDM)?")

    T_avg = None
    kappa_T = None
    N_T = None
    M_T = None
    alpha = 0.0
    T_ref_C = 0.0
    thetaT0 = thetaTL = vT0 = vTL = 0.0
    T_C = None
    x_T = None
    y_T = None
    theta_T_arr = None
    v_T_arr = None
    dx_T = dy_T = 0.0
    k_heat = 0.0

    if include_thermal:
        alpha = ask_float("Thermal expansion coefficient alpha [1/K]: ", positive_only=True)
        T_ref_C = ask_float("Reference temperature for zero thermal strain [°C]: ", positive_only=False)

        print("\nNow running 2D heat solver using:")
        print(f"  Lx = beam length = {L:.6f} m")
        print(f"  Ly = section height/diameter = {Ly_section:.6f} m")

        # Run heat in "solver-only" mode, get the temperature field
        T_C, x_T, y_T, dx_T, dy_T, Nx_T, Ny_T, k_heat = run_heat(
            optional_Lx=L,
            optional_Ly=Ly_section,
            return_field=True
        )

        # From T(x,y), compute T_avg(x) and kappa_T(x)
        T_avg, kappa_T = compute_thermal_section_data(
            T_C, x_T, y_T, Ly_section, alpha, T_ref_C, I, b_width
        )

        # Free thermal axial force and "thermal moment" distributions
        N_T = E * A_section * alpha * (T_avg - T_ref_C)
        M_T = E * I * kappa_T
        # Integrals of thermal axial force with the correct weights
        if len(x_T) > 1:
            INT_NT_Lminusx = float(np.trapz(N_T * (L - x_T), x_T))
            INT_NT_x       = float(np.trapz(N_T * x_T,          x_T))
        else:
            INT_NT_Lminusx = 0.0
            INT_NT_x       = 0.0

        # Thermal slope and deflection fields + shifts
        (thetaT0, thetaTL,
         vT0, vTL,
         theta_T_arr, v_T_arr) = compute_thermal_bc_shifts(x_T, kappa_T)

        N_T_avg = float(np.mean(N_T))
        sigma_T_avg = N_T_avg / A_section if A_section > 0 else 0.0

        print("\n=== THERMAL COUPLING SUMMARY (approximate) ===")
        print(f"Average T̄(x) = {np.mean(T_avg):.3f} °C")
        print(f"Average thermal axial force (free) N_T ≈ {N_T_avg:.6g} N")
        print(f"Average thermal axial stress (free) σ_T ≈ {sigma_T_avg:.6g} Pa")
        print(f"Thermal slope shift at x=L: θ_T(L) ≈ {thetaTL:.6g} rad")
        print(f"Thermal deflection shift at x=L: v_T(L) ≈ {vTL:.6g} m")

        # Plot the free thermal force/moment distributions
        plt.figure(figsize=(10, 4))
        plt.subplot(1, 2, 1)
        plt.plot(x_T, N_T)
        plt.xlabel("x [m]")
        plt.ylabel("N_T(x) [N]")
        plt.title("Free thermal axial force distribution")
        plt.grid(True)

        plt.subplot(1, 2, 2)
        plt.plot(x_T, M_T)
        plt.xlabel("x [m]")
        plt.ylabel("M_T(x) [N·m]")
        plt.title("Free thermal bending moment (E I κ_T)")
        plt.grid(True)

        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)

    # ----------------------------------------------------------------------
    # From here on: original SPS bending solver (mechanical M(x)),
    # but BCs will be shifted by thermal thetaT, vT if include_thermal=True.
    # ----------------------------------------------------------------------

    Trav = []
    Xtrav = []
    for i in range(Fv):
        print(f"\n--- Transverse force {i+1:02d} ---")
        Tval = ask_float("  Magnitude (N, +up / -down): ")
        Xval = ask_position("  Distance from left wall (m): ", L)
        Trav.append(Tval)
        Xtrav.append(Xval)

    Faxial = []
    Xaxial = []
    eAxial = []
    for i in range(Fa):
        print(f"\n--- Eccentric axial force {i+1:02d} ---")
        Nval = ask_float("  Axial magnitude (N, +right / -left): ")
        Xval = ask_position("  Distance from left wall (m): ", L)
        eval_ = ask_float("  Vertical offset e (m, +up / -down): ")
        Faxial.append(Nval)
        Xaxial.append(Xval)
        eAxial.append(eval_)

    Fdiag = []
    AngDeg = []
    Xdiag = []
    eDiag = []
    for i in range(Fd):
        print(f"\n--- Diagonal force {i+1:02d} ---")
        Fval = ask_float("  Magnitude (N): ")
        Aval = ask_float("  Angle (deg, from +x toward +y): ")
        Xval = ask_position("  Distance from left wall (m): ", L)
        eval_ = ask_float("  Vertical offset e (m, +up / -down): ")
        Fdiag.append(Fval)
        AngDeg.append(Aval)
        Xdiag.append(Xval)
        eDiag.append(eval_)

    AngRad = np.deg2rad(AngDeg) if Fd > 0 else np.array([])
    Fdiag = np.array(Fdiag, dtype=float) if Fd > 0 else np.array([])
    FdiagV = Fdiag * np.sin(AngRad) if Fd > 0 else np.array([])
    FdiagH = Fdiag * np.cos(AngRad) if Fd > 0 else np.array([])

    Trav = np.array(Trav, dtype=float) if Fv > 0 else np.array([])
    Xtrav = np.array(Xtrav, dtype=float) if Fv > 0 else np.array([])

    Faxial = np.array(Faxial, dtype=float) if Fa > 0 else np.array([])
    Xaxial = np.array(Xaxial, dtype=float) if Fa > 0 else np.array([])
    eAxial = np.array(eAxial, dtype=float) if Fa > 0 else np.array([])

    Xdiag = np.array(Xdiag, dtype=float) if Fd > 0 else np.array([])
    eDiag = np.array(eDiag, dtype=float) if Fd > 0 else np.array([])

    Mconc = []
    Xmom = []
    for k in range(Fm):
        print(f"\n--- Concentrated moment {k+1:02d} ---")
        Mval = ask_float("  Moment magnitude (N*m, +CCW / -CW): ")
        Xval = ask_position("  Distance from left wall (m): ", L)
        Mconc.append(Mval)
        Xmom.append(Xval)

    Mconc = np.array(Mconc, dtype=float) if Fm > 0 else np.array([])
    Xmom = np.array(Xmom, dtype=float) if Fm > 0 else np.array([])

    if Fv + Fa + Fd + Fm > 0:
        allPos = []
        if Fv > 0:
            allPos.extend(Xtrav.tolist())
        if Fa > 0:
            allPos.extend(Xaxial.tolist())
        if Fd > 0:
            allPos.extend(Xdiag.tolist())
        if Fm > 0:
            allPos.extend(Xmom.tolist())
        allPos = sorted(set(allPos))
        boundaries = [0.0] + allPos + [L]
    else:
        boundaries = [0.0, L]

    nSections = len(boundaries) - 1

    x = sp.Symbol('x', real=True)
    Ry_left, Ry_right, MR_left, MR_right = sp.symbols(
        'Ry_left Ry_right MR_left MR_right', real=True
    )

    C1 = [sp.Symbol(f"C1_{i+1}", real=True) for i in range(nSections)]
    C2 = [sp.Symbol(f"C2_{i+1}", real=True) for i in range(nSections)]

    sections = []
    Vsym = [0] * nSections
    Msym = [0] * nSections
    thetaEI = [0] * nSections
    vEI = [0] * nSections

    for s in range(nSections):
        xL = boundaries[s]
        xR = boundaries[s + 1]

        idxPrevTrav  = [i for i in range(Fv) if Xtrav[i] <= xL] if Fv > 0 else []
        idxPrevAxial = [i for i in range(Fa) if Xaxial[i] <= xL] if Fa > 0 else []
        idxPrevDiag  = [i for i in range(Fd) if Xdiag[i]  <= xL] if Fd > 0 else []
        idxPrevMom   = [i for i in range(Fm) if Xmom[i]   <= xL] if Fm > 0 else []

        sections.append({
            "xL": xL, "xR": xR,
            "TravIdx": idxPrevTrav,
            "AxialIdx": idxPrevAxial,
            "DiagIdx": idxPrevDiag,
            "MomIdx": idxPrevMom
        })

        sumTravLeft = float(np.sum(Trav[idxPrevTrav])) if idxPrevTrav else 0.0
        sumDiagVLeft = float(np.sum(FdiagV[idxPrevDiag])) if idxPrevDiag else 0.0

        Vx_expr = -(Ry_left + sumTravLeft + sumDiagVLeft)
        Vsym[s] = Vx_expr

        Mx = Ry_left * x
        for iF in idxPrevTrav:
            Fi = Trav[iF]
            Xi = Xtrav[iF]
            Mx += Fi * (x - Xi)
        for iD in idxPrevDiag:
            Fv_d = FdiagV[iD]
            Xd = Xdiag[iD]
            Mx += Fv_d * (x - Xd)

        for iA in idxPrevAxial:
            Na = Faxial[iA]
            ea = eAxial[iA]
            Mx += Na * ea
        for iD in idxPrevDiag:
            Nd = FdiagH[iD]
            ed = eDiag[iD]
            Mx += Nd * ed

        for iM in idxPrevMom:
            Mx += Mconc[iM]

        Mx = Mx - MR_left
        Mx = sp.expand(Mx)
        Msym[s] = Mx

        tEI = sp.integrate(Mx, (x,)) + C1[s]
        tEI = sp.expand(tEI)
        thetaEI[s] = tEI

        vEI_s = sp.integrate(tEI, (x,)) + C2[s]
        vEI_s = sp.expand(vEI_s)
        vEI[s] = vEI_s

    C1val = [None] * nSections
    C2val = [None] * nSections
    C1val[0] = C1[0]
    C2val[0] = C2[0]

    for s in range(nSections - 1):
        xInt = boundaries[s + 1]
        knownCs = C1[:s+1] + C2[:s+1]
        knownVal = C1val[:s+1] + C2val[:s+1]

        thetaEI_L = thetaEI[s].subs(dict(zip(knownCs, knownVal)))
        vEI_L_    = vEI[s].subs(dict(zip(knownCs, knownVal)))
        thetaEI_R = thetaEI[s+1].subs(dict(zip(knownCs, knownVal)))
        vEI_R_    = vEI[s+1].subs(dict(zip(knownCs, knownVal)))

        eqTheta = sp.Eq(thetaEI_L.subs(x, xInt), thetaEI_R.subs(x, xInt))
        eqV     = sp.Eq(vEI_L_.subs(x, xInt), vEI_R_.subs(x, xInt))

        sol = sp.solve((eqTheta, eqV), (C1[s+1], C2[s+1]), dict=True)
        if not sol:
            raise RuntimeError("Failed to solve continuity constants.")
        sol = sol[0]

        C1val[s+1] = sp.simplify(sol[C1[s+1]])
        C2val[s+1] = sp.simplify(sol[C2[s+1]])

    thetaEI_final = []
    vEI_final = []
    for s in range(nSections):
        expr_theta = thetaEI[s].subs(dict(zip(C1 + C2, C1val + C2val)))
        expr_v = vEI[s].subs(dict(zip(C1 + C2, C1val + C2val)))
        thetaEI_final.append(expr_theta)
        vEI_final.append(expr_v)

    ThetaFinal = [sp.simplify(expr / (E * I)) for expr in thetaEI_final]
    VdefFinal  = [sp.simplify(expr / (E * I)) for expr in vEI_final]

    # ---------------- HORIZONTAL REACTIONS ----------------

    sumHorizLoads_mech = 0.0
    if Fa > 0:
        sumHorizLoads_mech += float(np.sum(Faxial))
    if Fd > 0:
        sumHorizLoads_mech += float(np.sum(FdiagH))

    sumFa_L_minus_a_mech = 0.0
    sumFa_a_mech = 0.0
    if Fa > 0:
        sumFa_L_minus_a_mech += float(np.sum(Faxial * (L - Xaxial)))
        sumFa_a_mech         += float(np.sum(Faxial * Xaxial))
    if Fd > 0:
        sumFa_L_minus_a_mech += float(np.sum(FdiagH * (L - Xdiag)))
        sumFa_a_mech         += float(np.sum(FdiagH * Xdiag))

    left_has_H  = dofL["H"]
    right_has_H = dofR["H"]

    Rx_left_mech = 0.0
    Rx_right_mech = 0.0
    Rx_left = 0.0
    Rx_right = 0.0
    Rx_left_thermal_delta = 0.0
    Rx_right_thermal_delta = 0.0

    # If no axial DOF at any end, or no axial loads + no thermal coupling with two axial supports,
    # then horizontal reactions are zero.
    if not (left_has_H or right_has_H) or (Fa + Fd == 0 and not (include_thermal and left_has_H and right_has_H)):
        Rx_left = 0.0
        Rx_right = 0.0
    else:
        # CASE 1: both ends carry axial reaction (two unknowns, compatibility)
        if left_has_H and right_has_H:
            # mechanical-only (Raymond's original formulas)
            if abs(L) > 0.0:
                Rx_left_mech  = - sumFa_L_minus_a_mech / L
                Rx_right_mech = - sumFa_a_mech / L
            else:
                Rx_left_mech = 0.0
                Rx_right_mech = 0.0

            Rx_left = Rx_left_mech
            Rx_right = Rx_right_mech

            # Add thermal contribution using ∫ N_T(x)(L-x) dx and ∫ N_T(x)x dx
            if include_thermal and (abs(INT_NT_Lminusx) + abs(INT_NT_x) > 0.0) and abs(L) > 0.0:
                Rx_left_total  = - (sumFa_L_minus_a_mech + INT_NT_Lminusx) / L
                Rx_right_total = - (sumFa_a_mech         + INT_NT_x)        / L

                Rx_left_thermal_delta  = Rx_left_total  - Rx_left_mech
                Rx_right_thermal_delta = Rx_right_total - Rx_right_mech

                Rx_left  = Rx_left_total
                Rx_right = Rx_right_total

        # CASE 2: only left has axial reaction (bar free at right in axial)
        elif left_has_H and not right_has_H:
            Rx_left_mech  = - sumHorizLoads_mech
            Rx_right_mech = 0.0
            # For a single axial restraint, free thermal expansion ⇒ no net thermal force.
            Rx_left  = Rx_left_mech
            Rx_right = Rx_right_mech

        # CASE 3: only right has axial reaction
        elif not left_has_H and right_has_H:
            Rx_left_mech  = 0.0
            Rx_right_mech = - sumHorizLoads_mech
            Rx_left  = Rx_left_mech
            Rx_right = Rx_right_mech

    # ---------------- GLOBAL EQUILIBRIUM (MECHANICAL) ----------------

    sumTravAll = float(np.sum(Trav)) + (float(np.sum(FdiagV)) if Fd > 0 else 0.0)

    sumFTx = 0.0
    if Fv > 0:
        sumFTx += float(np.sum(Trav * Xtrav))
    if Fd > 0:
        sumFTx += float(np.sum(FdiagV * Xdiag))

    sumNe = 0.0
    if Fa > 0:
        sumNe += float(np.sum(Faxial * eAxial))
    if Fd > 0:
        sumNe += float(np.sum(FdiagH * eDiag))

    sumMc = float(np.sum(Mconc)) if Fm > 0 else 0.0

    eqFy_full = sp.Eq(Ry_left + Ry_right + sumTravAll, 0)
    eqM_full  = sp.Eq(MR_left + MR_right + Ry_right * L + sumFTx - sumNe - sumMc, 0)

    subs_dofs = {}
    if not dofL["V"]:
        subs_dofs[Ry_left] = 0
    if not dofR["V"]:
        subs_dofs[Ry_right] = 0
    if not dofL["M"]:
        subs_dofs[MR_left] = 0
    if not dofR["M"]:
        subs_dofs[MR_right] = 0

    eqFy_sub = sp.Eq(eqFy_full.lhs.subs(subs_dofs), 0)
    eqM_sub  = sp.Eq(eqM_full.lhs.subs(subs_dofs),  0)

    # ---------------- SLOPE & DEFLECTION AT ENDS (MECH + THERMAL) ----------------

    Theta0 = ThetaFinal[0].subs(x, 0)
    V0     = VdefFinal[0].subs(x, 0)
    ThetaL = ThetaFinal[-1].subs(x, L)
    VL     = VdefFinal[-1].subs(x, L)

    Theta0_sub = Theta0.subs(subs_dofs)
    V0_sub     = V0.subs(subs_dofs)
    ThetaL_sub = ThetaL.subs(subs_dofs)
    VL_sub     = VL.subs(subs_dofs)

    # ---------------- UNKNOWN VECTOR ----------------

    bending_unknowns = []
    if dofL["V"]:
        bending_unknowns.append(Ry_left)
    if dofL["M"]:
        bending_unknowns.append(MR_left)
    if dofR["V"]:
        bending_unknowns.append(Ry_right)
    if dofR["M"]:
        bending_unknowns.append(MR_right)

    base_C1 = C1[0]
    base_C2 = C2[0]
    unknowns_total = bending_unknowns + [base_C1, base_C2]

    # ------ helper: solve for generic support shifts (for mech vs thermal) ------
    def solve_for_shifts(v_shift0, theta_shift0, v_shiftL, theta_shiftL):
        bc_eqs_loc = []
        if bcL["v"] == 0:
            bc_eqs_loc.append(sp.Eq(V0_sub, -v_shift0))
        if bcL["theta"] == 0:
            bc_eqs_loc.append(sp.Eq(Theta0_sub, -theta_shift0))
        if bcR["v"] == 0:
            bc_eqs_loc.append(sp.Eq(VL_sub, -v_shiftL))
        if bcR["theta"] == 0:
            bc_eqs_loc.append(sp.Eq(ThetaL_sub, -theta_shiftL))

        eqs_loc = [eqFy_sub, eqM_sub] + bc_eqs_loc
        A_mat_loc, b_vec_loc = sp.linear_eq_to_matrix(eqs_loc, unknowns_total)
        sol_vec = A_mat_loc.LUsolve(b_vec_loc)
        return dict(zip(unknowns_total, sol_vec))

    # total solution (with thermal shifts)
    if include_thermal:
        sol_dict = solve_for_shifts(vT0, thetaT0, vTL, thetaTL)
    else:
        sol_dict = solve_for_shifts(0.0, 0.0, 0.0, 0.0)

    Ry_left_num   = float(sol_dict.get(Ry_left,   0.0))
    MR_left_num   = float(sol_dict.get(MR_left,   0.0))
    Ry_right_num  = float(sol_dict.get(Ry_right,  0.0))
    MR_right_num  = float(sol_dict.get(MR_right,  0.0))
    C1_1_num      = float(sol_dict.get(base_C1,   0.0))
    C2_1_num      = float(sol_dict.get(base_C2,   0.0))

    print("\n=====================================")
    print("REACTIONS (INCLUDING THERMAL EFFECTS)")
    print("=====================================")
    print("Vertical:")
    print(f"  Ry_left   = {Ry_left_num:.6g} N")
    print(f"  Ry_right  = {Ry_right_num:.6g} N")
    print("\nMoments:")
    print(f"  M_left    = {MR_left_num:.6g} N*m")
    print(f"  M_right   = {MR_right_num:.6g} N*m")
    print("\nHorizontal:")
    print(f"  Rx_left   = {Rx_left:.6g} N")
    print(f"  Rx_right  = {Rx_right:.6g} N")

    # ----- Reaction dissection: mechanical-only vs thermal contribution -----
    if include_thermal:
        sol_mech = solve_for_shifts(0.0, 0.0, 0.0, 0.0)
        Ry_left_mech  = float(sol_mech.get(Ry_left,  0.0))
        MR_left_mech  = float(sol_mech.get(MR_left,  0.0))
        Ry_right_mech = float(sol_mech.get(Ry_right, 0.0))
        MR_right_mech = float(sol_mech.get(MR_right,0.0))

        print("\n--- Reaction breakdown (mechanical vs thermal) ---")
        if dofL["V"]:
            dRyL = Ry_left_num - Ry_left_mech
            print(f"  Ry_left:  mech-only = {Ry_left_mech:.6g} N,  thermal Δ = {dRyL:.6g} N")
        if dofR["V"]:
            dRyR = Ry_right_num - Ry_right_mech
            print(f"  Ry_right: mech-only = {Ry_right_mech:.6g} N,  thermal Δ = {dRyR:.6g} N")
        if dofL["M"]:
            dML = MR_left_num - MR_left_mech
            print(f"  M_left:   mech-only = {MR_left_mech:.6g} N*m, thermal Δ = {dML:.6g} N*m")
        if dofR["M"]:
            dMR = MR_right_num - MR_right_mech
            print(f"  M_right:  mech-only = {MR_right_mech:.6g} N*m, thermal Δ = {dMR:.6g} N*m")

    subs_dict = {
        Ry_left:   Ry_left_num,
        Ry_right:  Ry_right_num,
        MR_left:   MR_left_num,
        MR_right:  MR_right_num,
        base_C1:   C1_1_num,
        base_C2:   C2_1_num,
    }

    V_plot_expr = []
    M_plot_expr = []
    theta_plot_expr = []
    v_plot_expr = []

    for s in range(nSections):
        V_plot_expr.append(sp.simplify(Vsym[s].subs(subs_dict)))
        M_plot_expr.append(sp.simplify(Msym[s].subs(subs_dict)))
        theta_plot_expr.append(sp.simplify(ThetaFinal[s].subs(subs_dict)))
        v_plot_expr.append(sp.simplify(VdefFinal[s].subs(subs_dict)))

    nPtsSection = 200

    xV_all = []
    V_all = []
    for s in range(nSections):
        xL = sections[s]["xL"]
        xR = sections[s]["xR"]
        xs = np.linspace(xL, xR, nPtsSection)
        Vs = [float(V_plot_expr[s].subs(x, xv)) for xv in xs]
        if s == 0:
            xV_all.extend([xL])
            V_all.extend([0.0])
            xV_all.extend(xs.tolist())
            V_all.extend(Vs)
        else:
            xV_all.extend([xL, xL])
            V_all.extend([V_all[-1], Vs[0]])
            xV_all.extend(xs[1:].tolist())
            V_all.extend(Vs[1:])
    xV_all.extend([L, L])
    V_all.extend([V_all[-1], 0.0])
    xV_all = np.array(xV_all)
    V_all = np.array(V_all)

    x_all = []
    M_all = []
    th_all = []
    v_all = []
    for s in range(nSections):
        xL = sections[s]["xL"]
        xR = sections[s]["xR"]
        xs = np.linspace(xL, xR, nPtsSection)
        Ms = [float(M_plot_expr[s].subs(x, xv)) for xv in xs]
        th = [float(theta_plot_expr[s].subs(x, xv)) for xv in xs]
        vv = [float(v_plot_expr[s].subs(x, xv)) for xv in xs]
        if s > 0:
            xs = xs[1:]
            Ms = Ms[1:]
            th = th[1:]
            vv = vv[1:]
        x_all.extend(xs.tolist())
        M_all.extend(Ms)
        th_all.extend(th)
        v_all.extend(vv)
    x_all = np.array(x_all)
    M_all = np.array(M_all)
    th_all = np.array(th_all)
    v_all = np.array(v_all)

    # ----- add thermal contribution to slope/deflection fields -----
    if include_thermal and (theta_T_arr is not None):
        theta_T_interp = np.interp(x_all, x_T, theta_T_arr)
        v_T_interp     = np.interp(x_all, x_T, v_T_arr)
        th_total = th_all + theta_T_interp
        v_total  = v_all  + v_T_interp
    else:
        th_total = th_all
        v_total  = v_all

    # quick BC check (total)
    print("\nCheck (total fields, including thermal):")
    print(f"  θ_tot(0) ≈ {th_total[0]:.6e} rad")
    print(f"  v_tot(0) ≈ {v_total[0]:.6e} m")
    print(f"  θ_tot(L) ≈ {th_total[-1]:.6e} rad")
    print(f"  v_tot(L) ≈ {v_total[-1]:.6e} m")

    xM_all = []
    Mstep_all = []
    tol = 1e-9
    for s in range(nSections):
        xL = sections[s]["xL"]
        xR = sections[s]["xR"]
        xs = np.linspace(xL, xR, nPtsSection)
        Ms = [float(M_plot_expr[s].subs(x, xv)) for xv in xs]
        if s == 0:
            xM_all.extend([xL])
            Mstep_all.extend([0.0])
            xM_all.extend(xs.tolist())
            Mstep_all.extend(Ms)
        else:
            hasCoupleHere = False
            if Fa > 0 and np.any(np.abs(Xaxial - xL) < tol):
                hasCoupleHere = True
            if Fd > 0 and np.any(np.abs(Xdiag - xL) < tol):
                hasCoupleHere = True
            if Fm > 0 and np.any(np.abs(Xmom - xL) < tol):
                hasCoupleHere = True
            if hasCoupleHere:
                M_new_at_L = float(M_plot_expr[s].subs(x, xL))
                xM_all.extend([xL, xL])
                Mstep_all.extend([Mstep_all[-1], M_new_at_L])
                xM_all.extend(xs[1:].tolist())
                Mstep_all.extend(Ms[1:])
            else:
                xM_all.extend(xs[1:].tolist())
                Mstep_all.extend(Ms[1:])
    xM_all.extend([L, L])
    Mstep_all.extend([Mstep_all[-1], 0.0])
    xM_all = np.array(xM_all)
    Mstep_all = np.array(Mstep_all)

    fig_static, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    ax1, ax2, ax3, ax4 = axes

    ax1.plot(xV_all, V_all)
    ax1.set_ylabel("V(x) [N]")
    ax1.set_title("Shear diagram")
    ax1.grid(True)

    for k in range(Fv):
        ax1.axvline(Xtrav[k], linestyle='--')
    for k in range(Fa):
        ax1.axvline(Xaxial[k], linestyle='--')
    for k in range(Fd):
        ax1.axvline(Xdiag[k], linestyle='--')
    for k in range(Fm):
        ax1.axvline(Xmom[k], linestyle='--')

    ax2.plot(xM_all, Mstep_all)
    ax2.set_ylabel("M(x) [N·m]")
    ax2.set_title("Moment diagram")
    ax2.grid(True)

    for k in range(Fv):
        ax2.axvline(Xtrav[k], linestyle='--')
    for k in range(Fa):
        ax2.axvline(Xaxial[k], linestyle='--')
    for k in range(Fd):
        ax2.axvline(Xdiag[k], linestyle='--')
    for k in range(Fm):
        ax2.axvline(Xmom[k], linestyle='--')

    ax3.plot(x_all, th_total)
    ax3.set_ylabel("θ(x) [rad]")
    ax3.set_title("Slope diagram")
    ax3.grid(True)

    for k in range(Fv):
        ax3.axvline(Xtrav[k], linestyle='--')
    for k in range(Fa):
        ax3.axvline(Xaxial[k], linestyle='--')
    for k in range(Fd):
        ax3.axvline(Xdiag[k], linestyle='--')
    for k in range(Fm):
        ax3.axvline(Xmom[k], linestyle='--')

    ax4.plot(x_all, v_total)
    ax4.set_xlabel("x [m]")
    ax4.set_ylabel("v(x) [m]")
    ax4.set_title("Deflection diagram")
    ax4.grid(True)

    for k in range(Fv):
        ax4.axvline(Xtrav[k], linestyle='--')
    for k in range(Fa):
        ax4.axvline(Xaxial[k], linestyle='--')
    for k in range(Fd):
        ax4.axvline(Xdiag[k], linestyle='--')
    for k in range(Fm):
        ax4.axvline(Xmom[k], linestyle='--')

    plt.tight_layout()

    fig_anim, anim = animate_diagrams(xV_all, V_all, xM_all, Mstep_all, x_all, th_total, v_total)

    plt.show(block=False)
    plt.pause(0.1)

    # ---- structural + thermal query menu ----
    if include_thermal and (T_C is not None):
        thermal_data = (T_C, x_T, y_T, dx_T, dy_T, k_heat)
    else:
        thermal_data = None

    structural_query_mode(
        L,
        xV_all, V_all,
        xM_all, Mstep_all,
        x_all, th_total, v_total,
        thermal_data
    )

    return L, Ly_section


# ============================================================
# HEAT FDM – LETTER BCs, dx/dy, SETUP CHECK, SCALING + FLUX
# ============================================================

sigma = 5.670374419e-8  # Stefan–Boltzmann [W/m^2·K^4]


def heat_input_geometry_and_mesh(optional_Lx=None, optional_Ly=None):
    print("\n=== 2D Steady FDM Heat Conduction Solver ===")

    if optional_Lx is None:
        Lx = float(input("Plate length in x, Lx [m] (default 0.04): ") or 0.04)
    else:
        Lx = optional_Lx
        print(f"Plate length in x, Lx [m]: {Lx:.6f} (from structural solver)")

    if optional_Ly is None:
        Ly = float(input("Plate height in y, Ly [m] (default 0.02): ") or 0.02)
    else:
        Ly = optional_Ly
        print(f"Plate height in y, Ly [m]: {Ly:.6f} (from structural solver)")

    print("\nMesh definition:")
    print("  1 = Specify element size dx and dy (independent)")
    print("  2 = Specify number of nodes Nx, Ny")
    mesh_mode = int(input("Choose mesh mode (1 or 2): ") or 1)

    if mesh_mode == 1:
        dx = float(input("Enter dx [m] (default 2e-4): ") or 2e-4)
        dy = float(input("Enter dy [m] (default 2e-4): ") or 2e-4)

        Nx = int(round(Lx / dx)) + 1
        Ny = int(round(Ly / dy)) + 1

        Lx = dx * (Nx - 1)
        Ly = dy * (Ny - 1)
    else:
        Nx = int(input("Enter number of nodes in x, Nx (default 201): ") or 201)
        Ny = int(input("Enter number of nodes in y, Ny (default 101): ") or 101)
        dx = Lx / (Nx - 1)
        dy = Ly / (Ny - 1)

    print(f"\nMesh: Nx = {Nx}, Ny = {Ny}, dx = {dx:.4e}, dy = {dy:.4e}")
    return Lx, Ly, dx, dy, Nx, Ny


def heat_input_material():
    print("\n=== Material and heat generation ===")
    k = float(input("Thermal conductivity k [W/m-K] (default 3): ") or 3.0)
    q_vol = float(input("Volumetric heat generation q_vol [W/m^3] (default 0): ") or 0.0)
    return k, q_vol


def heat_get_bc(side_name):
    print(f"\n--- {side_name.upper()} boundary ---")
    print("Choose type:")
    print("  T  = Temperature")
    print("  CV = Convection")
    print("  R  = Radiation (linearized)")
    print("  C  = Convection + Radiation")
    print("  Q  = Specified heat flux q''")
    print("  I  = Insulation (q'' = 0)")
    code = input("Type (T/CV/R/C/Q/I): ").strip().upper()

    bc = {"side": side_name, "code": code}

    if code == "T":
        T_C = float(input("Wall temperature [°C]: "))
        bc["T"] = T_C + 273.15

    elif code == "Q":
        q = float(input("Heat flux q'' [W/m^2], positive INTO domain: "))
        bc["q"] = q

    elif code == "CV":
        h = float(input("Convection h [W/m^2-K]: "))
        Tinf_C = float(input("Ambient T_inf [°C]: "))
        bc["h"] = h
        bc["Tinf"] = Tinf_C + 273.15

    elif code == "R":
        eps = float(input("Emissivity ε (0–1): "))
        Tsur_C = float(input("Surrounding temperature T_sur [°C]: "))
        Ts_C = float(input("Estimated surface temperature Ts [°C] (for linearization): "))
        Tsur = Tsur_C + 273.15
        Ts = Ts_C + 273.15
        h_rad = eps * sigma * ((Ts**2 + Tsur**2) * (Ts + Tsur))
        bc["eps"] = eps
        bc["Tsur"] = Tsur
        bc["Ts"] = Ts
        bc["hrad"] = h_rad

    elif code == "C":
        h = float(input("Convection h [W/m^2-K]: "))
        Tinf_C = float(input("Ambient T_inf [°C]: "))
        eps = float(input("Emissivity ε (0–1): "))
        Tsur_C = float(input("Surrounding temperature T_sur [°C]: "))
        Ts_C = float(input("Estimated surface temperature Ts [°C] (for linearization): "))
        Tinf = Tinf_C + 273.15
        Tsur = Tsur_C + 273.15
        Ts = Ts_C + 273.15
        h_rad = eps * sigma * ((Ts**2 + Tsur**2) * (Ts + Tsur))
        bc["h"] = h
        bc["Tinf"] = Tinf
        bc["eps"] = eps
        bc["Tsur"] = Tsur
        bc["Ts"] = Ts
        bc["hrad"] = h_rad

    elif code == "I":
        bc["code"] = "I"

    else:
        print("Invalid type, defaulting to insulation (I).")
        bc["code"] = "I"

    return bc


def heat_describe_bc(b):
    c = b["code"]
    if c == "T":
        return f"T = {b['T'] - 273.15:.2f} °C"
    if c == "CV":
        return f"Convection: h = {b['h']} W/m²K, T_inf = {b['Tinf'] - 273.15:.2f} °C"
    if c == "R":
        return (f"Radiation: eps = {b['eps']}, T_sur = {b['Tsur'] - 273.15:.2f} °C, "
                f"Ts ≈ {b['Ts'] - 273.15:.2f} °C")
    if c == "C":
        return (f"Conv + Rad: h = {b['h']} W/m²K, T_inf = {b['Tinf'] - 273.15:.2f} °C, "
                f"eps = {b['eps']}, T_sur = {b['Tsur'] - 273.15:.2f} °C, "
                f"Ts ≈ {b['Ts'] - 273.15:.2f} °C")
    if c == "Q":
        return f"Heat flux: q'' = {b['q']} W/m²"
    if c == "I":
        return "Insulated (q'' = 0)"
    return "Unknown"


def heat_parse_point_input(s):
    s = s.strip()
    if s.startswith("(") and s.endswith(")"):
        s = s[1:-1]
    if "," in s:
        parts = s.split(",")
    else:
        parts = s.split()
    if len(parts) != 2:
        raise ValueError("Enter two numbers like 0.02,0.01 or (0.02,0.01)")
    xq = float(parts[0])
    yq = float(parts[1])
    return xq, yq


def heat_assemble_system(Lx, Ly, Nx, Ny, dx, dy, k, q_vol, bc):
    N = Nx * Ny
    A = sp_sparse.lil_matrix((N, N))
    b_vec = np.zeros(N)

    def idx(i, j):
        return j * Nx + i

    for j in range(Ny):
        for i in range(Nx):
            p = idx(i, j)

            if i == 0 and bc["left"]["code"] == "T":
                A[p, p] = 1.0
                b_vec[p] = bc["left"]["T"]
                continue
            if i == Nx - 1 and bc["right"]["code"] == "T":
                A[p, p] = 1.0
                b_vec[p] = bc["right"]["T"]
                continue
            if j == 0 and bc["bottom"]["code"] == "T":
                A[p, p] = 1.0
                b_vec[p] = bc["bottom"]["T"]
                continue
            if j == Ny - 1 and bc["top"]["code"] == "T":
                A[p, p] = 1.0
                b_vec[p] = bc["top"]["T"]
                continue

            gE = gW = gN = gS = 0.0
            Q = 0.0

            # EAST
            if i < Nx - 1:
                gE = k * dy / dx
            else:
                side = bc["right"]
                code = side["code"]
                area = dy
                if code == "Q":
                    Q += side["q"] * area
                elif code == "CV":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    gB = h * area
                    gE += gB
                    Q += gB * Tinf
                elif code == "R":
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = h_rad * area
                    gE += gB
                    Q += gB * Tsur
                elif code == "C":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = (h + h_rad) * area
                    gE += gB
                    Q += h * area * Tinf + h_rad * area * Tsur

            # WEST
            if i > 0:
                gW = k * dy / dx
            else:
                side = bc["left"]
                code = side["code"]
                area = dy
                if code == "Q":
                    Q += side["q"] * area
                elif code == "CV":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    gB = h * area
                    gW += gB
                    Q += gB * Tinf
                elif code == "R":
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = h_rad * area
                    gW += gB
                    Q += gB * Tsur
                elif code == "C":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = (h + h_rad) * area
                    gW += gB
                    Q += h * area * Tinf + h_rad * area * Tsur

            # NORTH
            if j < Ny - 1:
                gN = k * dx / dy
            else:
                side = bc["top"]
                code = side["code"]
                area = dx
                if code == "Q":
                    Q += side["q"] * area
                elif code == "CV":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    gB = h * area
                    gN += gB
                    Q += gB * Tinf
                elif code == "R":
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = h_rad * area
                    gN += gB
                    Q += gB * Tsur
                elif code == "C":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = (h + h_rad) * area
                    gN += gB
                    Q += h * area * Tinf + h_rad * area * Tsur

            # SOUTH
            if j > 0:
                gS = k * dx / dy
            else:
                side = bc["bottom"]
                code = side["code"]
                area = dx
                if code == "Q":
                    Q += side["q"] * area
                elif code == "CV":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    gB = h * area
                    gS += gB
                    Q += gB * Tinf
                elif code == "R":
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = h_rad * area
                    gS += gB
                    Q += gB * Tsur
                elif code == "C":
                    h = side["h"]
                    Tinf = side["Tinf"]
                    h_rad = side["hrad"]
                    Tsur = side["Tsur"]
                    gB = (h + h_rad) * area
                    gS += gB
                    Q += h * area * Tinf + h_rad * area * Tsur

            if q_vol != 0.0:
                Q += q_vol * dx * dy

            aP = gE + gW + gN + gS
            A[p, p] = aP
            b_vec[p] = Q

            if i < Nx - 1:
                A[p, idx(i+1, j)] = -gE
            if i > 0:
                A[p, idx(i-1, j)] = -gW
            if j < Ny - 1:
                A[p, idx(i, j+1)] = -gN
            if j > 0:
                A[p, idx(i, j-1)] = -gS

    return A.tocsr(), b_vec


def heat_compute_scaled_coords(T_C, x_arr, y_arr, dx, dy,
                               threshold_frac=0.10, small_weight=0.02):
    Ny, Nx = T_C.shape

    if Nx > 1:
        dTdx_local = (T_C[:, 1:] - T_C[:, :-1]) / dx
        g_x = np.mean(np.abs(dTdx_local), axis=0)
        g_x_max = np.max(g_x)
        if g_x_max < 1e-14:
            x_star = x_arr.copy()
        else:
            g_x_norm = g_x / g_x_max
            w_x = np.where(g_x_norm < threshold_frac, small_weight, 1.0)
            dx_star = w_x * dx
            x_star = np.concatenate(([0.0], np.cumsum(dx_star)))
    else:
        x_star = x_arr.copy()

    if Ny > 1:
        dTdy_local = (T_C[1:, :] - T_C[:-1, :]) / dy
        g_y = np.mean(np.abs(dTdy_local), axis=1)
        g_y_max = np.max(g_y)
        if g_y_max < 1e-14:
            y_star = y_arr.copy()
        else:
            g_y_norm = g_y / g_y_max
            w_y = np.where(g_y_norm < threshold_frac, small_weight, 1.0)
            dy_star = w_y * dy
            y_star = np.concatenate(([0.0], np.cumsum(dy_star)))
    else:
        y_star = y_arr.copy()

    return x_star, y_star


def heat_query_temp_flux(T_C, x, y, dx, dy, k, Lx, Ly, Nx, Ny, xq, yq):
    if not (0 <= xq <= Lx and 0 <= yq <= Ly):
        raise ValueError("Point outside domain")

    i = int(xq / dx)
    j = int(yq / dy)
    if i == Nx - 1:
        i -= 1
    if j == Ny - 1:
        j -= 1

    tx = (xq - i * dx) / dx
    ty = (yq - j * dy) / dy

    T11 = T_C[j,   i]
    T21 = T_C[j,   i+1]
    T12 = T_C[j+1, i]
    T22 = T_C[j+1, i+1]

    Tq = (T11 * (1-tx)*(1-ty) +
          T21 * tx*(1-ty) +
          T12 * (1-tx)*ty +
          T22 * tx*ty)

    ii = int(round(xq / dx))
    jj = int(round(yq / dy))
    ii = max(1, min(Nx-2, ii))
    jj = max(1, min(Ny-2, jj))

    dTdx = (T_C[jj, ii+1] - T_C[jj, ii-1]) / (2*dx)
    dTdy = (T_C[jj+1, ii] - T_C[jj-1, ii]) / (2*dy)

    qx = -k * dTdx
    qy = -k * dTdy
    qmag = np.sqrt(qx**2 + qy**2)

    return Tq, qx, qy, qmag


def run_heat(optional_Lx=None, optional_Ly=None, return_field=False):
    Lx, Ly, dx, dy, Nx, Ny = heat_input_geometry_and_mesh(optional_Lx, optional_Ly)
    k, q_vol = heat_input_material()

    print("\n=== Define boundary conditions for each side ===")
    bc = {
        "left":   heat_get_bc("left"),
        "right":  heat_get_bc("right"),
        "bottom": heat_get_bc("bottom"),
        "top":    heat_get_bc("top"),
    }

    while True:
        print("\n=== SETUP SUMMARY ===")
        print("Geometry:")
        print(f"  Lx = {Lx:.6f} m, Ly = {Ly:.6f} m")
        print("Mesh:")
        print(f"  Nx = {Nx}, Ny = {Ny}, dx = {dx:.4e} m, dy = {dy:.4e} m")
        print("Material:")
        print(f"  k = {k} W/m-K, q_vol = {q_vol} W/m³")
        print("Boundaries:")
        for side_name in ["left", "right", "bottom", "top"]:
            print(f"  {side_name.capitalize():6s}: {heat_describe_bc(bc[side_name])}")

        ans = input("\nIs this setup correct? (y/n): ").strip().lower()
        if ans.startswith("y"):
            break

        print("\nWhat do you want to change?")
        print("  1 = Geometry and mesh")
        print("  2 = Material and heat generation")
        print("  3 = Left boundary")
        print("  4 = Right boundary")
        print("  5 = Bottom boundary")
        print("  6 = Top boundary")
        choice = input("Choose option (1-6): ").strip()

        if choice == "1":
            Lx, Ly, dx, dy, Nx, Ny = heat_input_geometry_and_mesh(optional_Lx, optional_Ly)
        elif choice == "2":
            k, q_vol = heat_input_material()
        elif choice == "3":
            bc["left"] = heat_get_bc("left")
        elif choice == "4":
            bc["right"] = heat_get_bc("right")
        elif choice == "5":
            bc["bottom"] = heat_get_bc("bottom")
        elif choice == "6":
            bc["top"] = heat_get_bc("top")
        else:
            print("Invalid option, try again.")

    A_csr, b_vec = heat_assemble_system(Lx, Ly, Nx, Ny, dx, dy, k, q_vol, bc)
    print("\nSolving heat equation (direct sparse LU)...")

    # Only sparse direct solve (no AMG)
    T_vec = spla.spsolve(A_csr, b_vec)

    T = T_vec.reshape(Ny, Nx)
    T_K = T
    T_C = T_K - 273.15

    x = np.linspace(0, Lx, Nx)
    y = np.linspace(0, Ly, Ny)

    T_max = float(T_C.max())
    T_min = float(T_C.min())

    ixC = np.argmin(np.abs(x - Lx/2))
    iyC = np.argmin(np.abs(y - Ly/2))
    T_centroid = float(T_C[iyC, ixC])

    print("\n=== HEAT RESULTS ===")
    print(f"Max T = {T_max:.3f} °C")
    print(f"Min T = {T_min:.3f} °C")
    print(f"T at centroid = {T_centroid:.3f} °C")

    threshold_frac = 0.10
    small_weight   = 0.02

    aspect = max(Lx / Ly, Ly / Lx)
    if aspect > 3.0:
        x_star, y_star = heat_compute_scaled_coords(
            T_C, x, y, dx, dy,
            threshold_frac=threshold_frac,
            small_weight=small_weight
        )
        scaled_plot = True
    else:
        x_star = x
        y_star = y
        scaled_plot = False

    X_star, Y_star = np.meshgrid(x_star, y_star)

    def plot_temperature_field():
        plt.figure(figsize=(12, 4))

        vmin = T_min
        vmax = T_max
        if abs(vmax - vmin) < 1e-12:
            vmax = vmin + 1e-6

        levels = np.linspace(vmin, vmax, 200)
        cnt = plt.contourf(X_star, Y_star, T_C, levels=levels,
                           cmap="turbo", vmin=vmin, vmax=vmax)

        n_ticks = 7
        ticks = np.linspace(vmin, vmax, n_ticks)
        labels = []
        for k_tick, t in enumerate(ticks):
            if k_tick == 0:
                labels.append(f"Min = {vmin:.3f} °C")
            elif k_tick == n_ticks - 1:
                labels.append(f"Max = {vmax:.3f} °C")
            else:
                labels.append(f"{t:.1f}")
        cbar = plt.colorbar(cnt)
        cbar.set_ticks(ticks)
        cbar.set_ticklabels(labels)
        cbar.set_label("Temperature [°C]")

        if scaled_plot:
            plt.xlabel("x* [scaled]")
            plt.ylabel("y* [scaled]")
            plt.title("Steady-State Temperature (FDM, gradient-scaled view)")
        else:
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.title("Steady-State Temperature (FDM, physical coordinates)")

        plt.gca().set_aspect("equal")
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.001)

    plot_temperature_field()

    if return_field:
        # For structural coupling: return the raw field and data, skip query loop.
        return T_C, x, y, dx, dy, Nx, Ny, k

    if scaled_plot:
        print("\nNOTE: Plot uses scaled coordinates x*, y*.")
        print(f"Real domain: x in [0, {Lx:.4f}] m, y in [0, {Ly:.4f}] m.")
        print(f"Regions where |∇T| is < {threshold_frac*100:.1f}% of max are "
              f"compressed by factor {small_weight:.3f} in the plot.")
        print("Query mode ALWAYS expects real coordinates (x, y in meters).")

        print("\nSample mapping x* → x (scaled → real) at a few nodes:")
        for i_s in np.linspace(0, Nx-1, min(5, Nx), dtype=int):
            print(f"  x* = {x_star[i_s]:.4f}  ->  x = {x[i_s]:.4f} m")

        print("\nSample mapping y* → y (scaled → real) at a few nodes:")
        for j_s in np.linspace(0, Ny-1, min(3, Ny), dtype=int):
            print(f"  y* = {y_star[j_s]:.4f}  ->  y = {y[j_s]:.4f} m")

    while True:
        print("\n--- Query mode ---")
        print("1 = Temperature & heat flux at a point")
        print("2 = Temperature cross-section (horizontal/vertical)")
        print("3 = Plot heat-flux vector field")
        print("4 = Plot temperature distribution again")
        print("0 = Exit query mode")
        choice = input("Choose option: ").strip()

        if choice == "0":
            break

        if choice == "1":
            user = input("Enter point (x, y) in meters (e.g. 0.02,0.01 or (0.02,0.01)): ").strip()
            try:
                xq, yq = heat_parse_point_input(user)
            except Exception as e:
                print(f"Invalid input: {e}")
                continue

            try:
                Tq, qx, qy, qmag = heat_query_temp_flux(
                    T_C, x, y, dx, dy, k, Lx, Ly, Nx, Ny, xq, yq
                )
            except ValueError as e:
                print(e)
                continue

            print(f"T({xq:.6f}, {yq:.6f}) = {Tq:.4f} °C")
            print(f"q_x = {qx:.4f} W/m²   (positive +x)")
            print(f"q_y = {qy:.4f} W/m²   (positive +y)")
            print(f"|q|  = {qmag:.4f} W/m²")

        elif choice == "2":
            print("Cross-section type:")
            print("  h = horizontal (T vs x at fixed y)")
            print("  v = vertical   (T vs y at fixed x)")
            ctype = input("Type h or v: ").strip().lower()

            if ctype == "h":
                yq = float(input("Enter y (meters): "))
                if not (0 <= yq <= Ly):
                    print("y outside domain.")
                    continue
                jrow = int(round(yq / dy))
                jrow = max(0, min(Ny-1, jrow))
                plt.figure()
                plt.plot(x, T_C[jrow, :])
                plt.xlabel("x [m]")
                plt.ylabel("T [°C]")
                plt.title(f"Horizontal section at y = {y[jrow]:.4f} m")
                plt.grid(True)
                plt.tight_layout()
                plt.show(block=False)
                plt.pause(0.001)

            elif ctype == "v":
                xq = float(input("Enter x (meters): "))
                if not (0 <= xq <= Lx):
                    print("x outside domain.")
                    continue
                icol = int(round(xq / dx))
                icol = max(0, min(Nx-1, icol))
                plt.figure()
                plt.plot(y, T_C[:, icol])
                plt.xlabel("y [m]")
                plt.ylabel("T [°C]")
                plt.title(f"Vertical section at x = {x[icol]:.4f} m")
                plt.grid(True)
                plt.tight_layout()
                plt.show(block=False)
                plt.pause(0.001)
            else:
                print("Invalid type.")

        elif choice == "3":
            dTdx = np.zeros_like(T_C)
            dTdy = np.zeros_like(T_C)

            dTdx[:, 1:-1] = (T_C[:, 2:] - T_C[:, :-2]) / (2*dx)
            dTdy[1:-1, :] = (T_C[2:, :] - T_C[:-2, :]) / (2*dy)
            dTdx[:, 0]  = (T_C[:, 1] - T_C[:, 0]) / dx
            dTdx[:, -1] = (T_C[:, -1] - T_C[:, -2]) / dx
            dTdy[0, :]  = (T_C[1, :] - T_C[0, :]) / dy
            dTdy[-1, :] = (T_C[-1, :] - T_C[-2, :]) / dy

            qx_field = -k * dTdx
            qy_field = -k * dTdy

            total_nodes = Nx * Ny
            target_arrows = 150
            stride = max(1, int(np.sqrt(total_nodes / target_arrows)))

            Xs = X_star[::stride, ::stride]
            Ys = Y_star[::stride, ::stride]
            qxs = qx_field[::stride, ::stride]
            qys = qy_field[::stride, ::stride]

            mag = np.sqrt(qxs**2 + qys**2)
            eps = 1e-12
            qxs_n = qxs / (mag + eps)
            qys_n = qys / (mag + eps)

            plt.figure(figsize=(10, 3.5))
            levels = np.linspace(T_min, T_max, 80)
            plt.contourf(X_star, Y_star, T_C, levels=levels, cmap="turbo")
            cbar = plt.colorbar()
            cbar.set_label("T [°C]")
            plt.quiver(Xs, Ys, qxs_n, qys_n, pivot="mid", scale=20)
            plt.xlabel("x* [scaled]" if scaled_plot else "x [m]")
            plt.ylabel("y* [scaled]" if scaled_plot else "y [m]")
            plt.title("Heat-flux direction field q = -k ∇T (gradient-scaled view)")
            plt.gca().set_aspect("equal")
            plt.tight_layout()
            plt.show(block=False)
            plt.pause(0.001)

        elif choice == "4":
            plot_temperature_field()
        else:
            print("Invalid choice.")


# ============================================================
# MAIN MENU
# ============================================================

def main():
    print("====================================")
    print("        SPS STRUCTURAL + HEAT       ")
    print("====================================")

    while True:
        print("\nSelect analysis:")
        print("  1) Structural (beam SPS, with optional thermal coupling)")
        print("  2) Heat (2D FDM only)")
        print("  0) Exit")
        choice = ask_int("Choice: ", 0, 2)

        if choice == 0:
            break
        elif choice == 1:
            run_structural()
        elif choice == 2:
            run_heat()

    print("\nDone. Close the figure windows when finished.")
    plt.show()


if __name__ == "__main__":
    main()
