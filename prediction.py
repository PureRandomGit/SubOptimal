import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import Ridge
from sklearn.pipeline import make_pipeline
from scipy.optimize import curve_fit

# ─── Config ───────────────────────────────────────────────────────────────────
SHEET_ID     = "1cS6S8ULz608Zomaz527y1Dg4Eam2BbXB60R-wgyfbHs"
THRUST_GOAL  = 650    # g
AMP_LIMIT    = 14     # A
EXTRAP_PITCH = 100    # how far to extrapolate pitch beyond tested data
MIN_POINTS   = 1      # minimum data points needed to attempt fitting

# ─── Load ─────────────────────────────────────────────────────────────────────
url = f"https://docs.google.com/spreadsheets/d/{SHEET_ID}/gviz/tq?tqx=out:csv"
df = pd.read_csv(url)
df = df.loc[:, ~df.columns.str.startswith("Unnamed")]
df.columns = df.columns.str.strip()
df = df.rename(columns={
    "blades":"Blades","Diamter":"Diameter","Pitch":"Pitch",
    "average thrust":"Thrust","average amps":"Amps",
})
df = df.dropna(subset=["Blades","Diameter","Pitch","Thrust","Amps"])
df["Blades"] = df["Blades"].astype(int)

# ─── Model functions ──────────────────────────────────────────────────────────
def thrust_phys(X, k, a, b):
    D, P = X
    return k * (D**a) * (P**b)

def amps_phys(X, k, c, d):
    D, P = X
    return k * (D**c) * (P**d)

def r2(actual, predicted):
    ss_res = np.sum((actual - predicted)**2)
    ss_tot = np.sum((actual - actual.mean())**2)
    return 1 - ss_res / ss_tot

# ─── Colors per diameter ──────────────────────────────────────────────────────
diam_colors = {
    28:"#e07b39", 32:"#7b9de0", 33:"#9de07b",
    37:"#de7be0", 42:"#e0c97b", 40:"#7be0d8",
}

# ─── Process each blade count ─────────────────────────────────────────────────
all_blade_counts = sorted(df["Blades"].unique())

for blades in all_blade_counts:
    sub = df[df.Blades == blades].copy()
    n   = len(sub)

    print(f"\n{'═'*65}")
    print(f"  {blades}-BLADE  ({n} data points)")
    print(f"{'═'*65}")

    # ── Not enough data ────────────────────────────────────────────────────────
    if n < MIN_POINTS:
        print(f"  ✗ Skipping: only {n} data point(s) — need at least {MIN_POINTS} to fit a model.")
        print(f"    Collect more {blades}-blade test data to enable predictions.")
        continue

    # ── Outlier detection: flag points where amps deviate >2.5 std from a quick fit
    # Fit a rough physics model, find residuals, exclude gross outliers from fitting only
    def _amps_phys(X, k, c, d): D, P = X; return k * (D**c) * (P**d)
    try:
        _pa, _ = curve_fit(_amps_phys,
                           (sub.Diameter.values.astype(float), sub.Pitch.values.astype(float)),
                           sub.Amps.values, p0=[0.001,1.0,0.5], maxfev=5000)
        _resid = sub.Amps.values - _amps_phys(
                     (sub.Diameter.values.astype(float), sub.Pitch.values.astype(float)), *_pa)
        outlier_mask = np.abs(_resid) > 2.5 * _resid.std()
        if outlier_mask.any():
            print(f"  ⚠ Suspected amp outlier(s) excluded from model fitting (shown as hollow points on chart):")
            for _, r in sub[outlier_mask].iterrows():
                print(f"    D={int(r.Diameter)} P={int(r.Pitch)}: measured {r.Amps:.1f}A, model expected {_amps_phys((r.Diameter, r.Pitch), *_pa):.1f}A")
        sub_fit = sub[~outlier_mask].copy()
        sub_outliers = sub[outlier_mask].copy()
    except Exception:
        sub_fit = sub.copy()
        sub_outliers = sub.iloc[0:0].copy()
        outlier_mask = np.zeros(len(sub), dtype=bool)

    X     = sub_fit[["Diameter","Pitch"]].values
    y_t   = sub_fit["Thrust"].values
    y_a   = sub_fit["Amps"].values
    D_arr = sub_fit["Diameter"].values.astype(float)
    P_arr = sub_fit["Pitch"].values.astype(float)
    data_max_pitch = sub["Pitch"].max()
    diameters      = sorted(sub["Diameter"].unique())

    # ── Polynomial model (degree 2) ───────────────────────────────────────────
    pipe_t = make_pipeline(PolynomialFeatures(2), Ridge(alpha=1.0))
    pipe_a = make_pipeline(PolynomialFeatures(2), Ridge(alpha=1.0))
    pipe_t.fit(X, y_t)
    pipe_a.fit(X, y_a)

    poly_t_std = (y_t - pipe_t.predict(X)).std()
    poly_a_std = (y_a - pipe_a.predict(X)).std()
    poly_r2_t  = r2(y_t, pipe_t.predict(X))
    poly_r2_a  = r2(y_a, pipe_a.predict(X))

    # ── Physics model ─────────────────────────────────────────────────────────
    try:
        pt, _ = curve_fit(thrust_phys, (D_arr, P_arr), y_t, p0=[0.001, 2.0, 1.0], maxfev=10000)
        pa, _ = curve_fit(amps_phys,   (D_arr, P_arr), y_a, p0=[0.001, 1.0, 0.5], maxfev=10000)
        phys_ok = True
    except RuntimeError:
        print("  ⚠ Physics model failed to converge — showing polynomial only.")
        phys_ok = False

    if phys_ok:
        phys_t_std = (y_t - thrust_phys((D_arr, P_arr), *pt)).std()
        phys_a_std = (y_a - amps_phys((D_arr, P_arr), *pa)).std()
        phys_r2_t  = r2(y_t, thrust_phys((D_arr, P_arr), *pt))
        phys_r2_a  = r2(y_a, amps_phys((D_arr, P_arr), *pa))
    else:
        phys_t_std = phys_a_std = phys_r2_t = phys_r2_a = None

    print(f"  Polynomial  — Thrust R²: {poly_r2_t:.3f}  Amps R²: {poly_r2_a:.3f}")
    if phys_ok:
        print(f"  Physics     — Thrust R²: {phys_r2_t:.3f}  Amps R²: {phys_r2_a:.3f}")

    # ── Chart ─────────────────────────────────────────────────────────────────
    pitch_range = np.linspace(sub["Pitch"].min(), EXTRAP_PITCH, 300)
    tested_mask = pitch_range <= data_max_pitch
    extrap_mask = ~tested_mask

    subtitle = (
        f"Physics R²: T={phys_r2_t:.3f} A={phys_r2_a:.3f}  |  "
        f"Poly R²: T={poly_r2_t:.3f} A={poly_r2_a:.3f}"
        if phys_ok else
        f"Poly R²: T={poly_r2_t:.3f} A={poly_r2_a:.3f}  |  Physics model unavailable"
    )

    fig = make_subplots(
        rows=2, cols=2,
        subplot_titles=[
            "Polynomial — Thrust vs Pitch",
            "Physics model — Thrust vs Pitch",
            "Polynomial — Amps vs Pitch",
            "Physics model — Amps vs Pitch",
        ],
        vertical_spacing=0.12,
        horizontal_spacing=0.08,
    )

    for diam in diameters:
        col      = diam_colors.get(diam, "#aaaaaa")
        label    = f"{int(diam)}\""
        pts_test = sub[sub.Diameter == diam]

        poly_t_pred = pipe_t.predict([[diam, p] for p in pitch_range])
        poly_a_pred = pipe_a.predict([[diam, p] for p in pitch_range])

        if phys_ok:
            phys_t_pred = thrust_phys((np.full_like(pitch_range, diam), pitch_range), *pt)
            phys_a_pred = amps_phys((np.full_like(pitch_range, diam), pitch_range), *pa)
        else:
            phys_t_pred = phys_a_pred = None

        model_variants = [(poly_t_pred, poly_a_pred, poly_t_std, poly_a_std, 1)]
        if phys_ok:
            model_variants.append((phys_t_pred, phys_a_pred, phys_t_std, phys_a_std, 2))

        for t_pred, a_pred, t_std, a_std, col_idx in model_variants:
            first = (col_idx == 1)

            # Thrust — solid in tested range
            fig.add_trace(go.Scatter(
                x=pitch_range[tested_mask], y=t_pred[tested_mask],
                mode="lines", line=dict(color=col, width=2),
                name=label, legendgroup=label, showlegend=first,
            ), row=1, col=col_idx)

            # Thrust — dotted extrapolation + confidence band
            if extrap_mask.any():
                fig.add_trace(go.Scatter(
                    x=np.concatenate([pitch_range[extrap_mask], pitch_range[extrap_mask][::-1]]),
                    y=np.concatenate([t_pred[extrap_mask]+t_std, (t_pred[extrap_mask]-t_std)[::-1]]),
                    fill="toself", fillcolor=col, line=dict(width=0), opacity=0.10,
                    showlegend=False, legendgroup=label,
                ), row=1, col=col_idx)
                fig.add_trace(go.Scatter(
                    x=pitch_range[extrap_mask], y=t_pred[extrap_mask],
                    mode="lines", line=dict(color=col, width=2, dash="dot"),
                    showlegend=False, legendgroup=label,
                ), row=1, col=col_idx)

            # Thrust — real data points
            fig.add_trace(go.Scatter(
                x=pts_test["Pitch"], y=pts_test["Thrust"],
                mode="markers", marker=dict(color=col, size=8, line=dict(color="white", width=1)),
                showlegend=False, legendgroup=label,
                hovertemplate=f"{int(diam)}\" diam<br>Pitch: %{{x}}<br>Thrust: %{{y:.0f}}g<extra></extra>",
            ), row=1, col=col_idx)

            # Amps — solid
            fig.add_trace(go.Scatter(
                x=pitch_range[tested_mask], y=a_pred[tested_mask],
                mode="lines", line=dict(color=col, width=2),
                showlegend=False, legendgroup=label,
            ), row=2, col=col_idx)

            # Amps — dotted extrapolation
            if extrap_mask.any():
                fig.add_trace(go.Scatter(
                    x=pitch_range[extrap_mask], y=a_pred[extrap_mask],
                    mode="lines", line=dict(color=col, width=2, dash="dot"),
                    showlegend=False, legendgroup=label,
                ), row=2, col=col_idx)

            # Amps — real data points
            fig.add_trace(go.Scatter(
                x=pts_test["Pitch"], y=pts_test["Amps"],
                mode="markers", marker=dict(color=col, size=8, line=dict(color="white", width=1)),
                showlegend=False, legendgroup=label,
                hovertemplate=f"{int(diam)}\" diam<br>Pitch: %{{x}}<br>Amps: %{{y:.1f}}A<extra></extra>",
            ), row=2, col=col_idx)

        # Outlier points — hollow markers with warning in hover
        if len(sub_outliers) > 0:
            pts_out = sub_outliers[sub_outliers.Diameter == diam]
            if len(pts_out):
                fig.add_trace(go.Scatter(
                    x=pts_out["Pitch"], y=pts_out["Amps"],
                    mode="markers",
                    marker=dict(color="rgba(0,0,0,0)", size=10,
                                line=dict(color=col, width=2)),
                    showlegend=False, legendgroup=label,
                    hovertemplate=f"{int(diam)}\" diam<br>Pitch: %{{x}}<br>Amps: %{{y:.1f}}A<br><b>⚠ suspected outlier — excluded from model</b><extra></extra>",
                ), row=2, col=col_idx)

        # If physics unavailable, fill col 2 with a "no data" note
        if not phys_ok:
            for row_idx in [1, 2]:
                fig.add_trace(go.Scatter(
                    x=[None], y=[None], mode="lines",
                    showlegend=False,
                ), row=row_idx, col=2)

    # Goal / limit lines
    for col_idx in [1, 2]:
        fig.add_hline(y=THRUST_GOAL, line=dict(color="rgba(60,180,60,0.7)", width=1.5, dash="dash"),
                      annotation_text=f"{THRUST_GOAL}g goal", annotation_position="bottom right",
                      row=1, col=col_idx)
        fig.add_hline(y=AMP_LIMIT, line=dict(color="rgba(220,60,60,0.7)", width=1.5, dash="dash"),
                      annotation_text=f"{AMP_LIMIT}A limit", annotation_position="top right",
                      row=2, col=col_idx)

    # Extrapolation boundary
    for row_idx in [1, 2]:
        for col_idx in [1, 2]:
            fig.add_vline(x=data_max_pitch,
                          line=dict(color="rgba(150,150,150,0.5)", width=1, dash="dash"),
                          annotation_text="data ends", annotation_position="top left",
                          row=row_idx, col=col_idx)

    for col_idx in [1, 2]:
        fig.update_xaxes(title_text="Pitch (in)", row=2, col=col_idx)
        fig.update_yaxes(title_text="Thrust (g)", row=1, col=col_idx)
        fig.update_yaxes(title_text="Amps (A)",   row=2, col=col_idx)

    fig.update_layout(
        title=dict(
            text=(f"{blades}-Blade Propeller — Predictive Models "
                  f"(solid = tested range, dotted = extrapolated)<br>"
                  f"<sup>{subtitle}</sup>"),
            font=dict(size=15), x=0.5, xanchor="center",
        ),
        legend=dict(title="Diameter", x=1.02, y=1),
        height=750,
        paper_bgcolor="white",
        plot_bgcolor="rgb(248,248,252)",
    )
    fig.update_xaxes(gridcolor="white", gridwidth=1)
    fig.update_yaxes(gridcolor="white", gridwidth=1)
    fig.show()

    # ── 2D optimum search ─────────────────────────────────────────────────────
    if not phys_ok:
        print(f"\n  ✗ Cannot run 2D optimum search — physics model unavailable.")
        continue

    diam_range_grid  = np.linspace(sub.Diameter.min(), sub.Diameter.max(), 400)
    pitch_range_grid = np.linspace(sub.Pitch.min(), EXTRAP_PITCH, 400)
    DG, PG   = np.meshgrid(diam_range_grid, pitch_range_grid)
    D_flat   = DG.ravel()
    P_flat   = PG.ravel()
    T_grid   = thrust_phys((D_flat, P_flat), *pt)
    A_grid   = amps_phys((D_flat, P_flat), *pa)

    valid_mask = (T_grid >= THRUST_GOAL) & (A_grid < AMP_LIMIT)
    score_grid = D_flat - P_flat
    score_grid[~valid_mask] = np.inf

    if np.isinf(score_grid).all():
        print(f"\n  ✗ No combination meets both goals within the modeled range.")
        continue

    best_i = np.argmin(score_grid)
    best_d = D_flat[best_i]; best_p = P_flat[best_i]
    best_t = T_grid[best_i]; best_a = A_grid[best_i]
    extrap = " [EXTRAPOLATED]" if best_p > data_max_pitch else ""

    print(f"\n  2D Optimum — Physics model (smallest D, highest P, goals met)")
    print(f"  {'─'*45}")
    print(f"  Diameter : {best_d:.1f} in")
    print(f"  Pitch    : {best_p:.1f} in{extrap}")
    print(f"  Thrust   : {best_t:.0f} g  (±{phys_t_std:.0f}g)  goal ≥{THRUST_GOAL}g")
    print(f"  Amps     : {best_a:.2f} A  (±{phys_a_std:.2f}A)  limit <{AMP_LIMIT}A")

    print(f"\n  Top 15 predicted configs (goals met, best score first):")
    print(f"  {'Diam':>6}  {'Pitch':>6}  {'Thrust':>9}  {'Amps':>7}  Note")
    print(f"  {'─'*58}")

    valid_idx = np.where(valid_mask)[0]
    order     = np.argsort(score_grid[valid_idx])
    seen = []; count = 0
    for i in order:
        idx  = valid_idx[i]
        d, p = round(D_flat[idx], 1), round(P_flat[idx], 1)
        if any(abs(d-sd) < 0.8 and abs(p-sp) < 2.0 for sd, sp in seen):
            continue
        seen.append((d, p))
        t, a = T_grid[idx], A_grid[idx]
        note = "[extrapolated]" if p > data_max_pitch else "within data range"
        print(f"  {d:>6.1f}  {p:>6.1f}  {t:>7.0f}g  {a:>5.2f}A  {note}")
        count += 1
        if count >= 15:
            break

    print(f"  {'─'*58}")
    print(f"  Uncertainty on all predictions: ±{phys_t_std:.0f}g / ±{phys_a_std:.2f}A")

print(f"\n{'═'*65}\n  Done.\n{'═'*65}\n")