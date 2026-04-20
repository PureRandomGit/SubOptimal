import pandas as pd
import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import griddata

# ─── Config ───────────────────────────────────────────────────────────────────
SHEET_ID      = "1cS6S8ULz608Zomaz527y1Dg4Eam2BbXB60R-wgyfbHs"
THRUST_GOAL   = 650    # grams
AMP_LIMIT     = 14     # amps
BLADE_CHOICE  = None   # set to 2, 3, or 4 to skip the prompt

# ─── Load ─────────────────────────────────────────────────────────────────────
url = f"https://docs.google.com/spreadsheets/d/{SHEET_ID}/gviz/tq?tqx=out:csv"
df = pd.read_csv(url)
df = df.loc[:, ~df.columns.str.startswith("Unnamed")]
df.columns = df.columns.str.strip()
df = df.rename(columns={
    "blades":         "Blades",
    "Diamter":        "Diameter",
    "Pitch":          "Pitch",
    "average thrust": "Thrust",
    "average amps":   "Amps",
    "max thrust":     "Max Thrust",
    "Chord":          "Chord",
})
df = df.dropna(subset=["Blades", "Diameter", "Pitch", "Thrust", "Amps"])
df["Blades"] = df["Blades"].astype(int)

# ─── Blade selection ──────────────────────────────────────────────────────────
available = sorted(df["Blades"].unique())
if BLADE_CHOICE is None:
    print(f"Available blade counts: {available}")
    BLADE_CHOICE = int(input(f"Enter blade count to plot {available}: "))

sub = df[df["Blades"] == BLADE_CHOICE].copy()

# ─── Interpolated surface ─────────────────────────────────────────────────────
pts      = sub[["Diameter", "Pitch"]].values
diams    = np.linspace(sub.Diameter.min(), sub.Diameter.max(), 250)
pitches  = np.linspace(sub.Pitch.min(),   sub.Pitch.max(),    250)
D, P     = np.meshgrid(diams, pitches)
grid_pts = np.column_stack([D.ravel(), P.ravel()])

T_surf = griddata(pts, sub["Thrust"].values, grid_pts, method="linear").reshape(D.shape)
A_surf = griddata(pts, sub["Amps"].values,   grid_pts, method="linear").reshape(D.shape)

# ─── Goal masks ───────────────────────────────────────────────────────────────
valid = (T_surf >= THRUST_GOAL) & (A_surf < AMP_LIMIT) & ~np.isnan(T_surf)

# ─── Theoretical optimum ──────────────────────────────────────────────────────
score = D - P
score[~valid] = np.inf
best_idx = np.unravel_index(np.argmin(score), score.shape)
best_d   = D[best_idx]
best_p   = P[best_idx]
best_t   = T_surf[best_idx]
best_a   = A_surf[best_idx]

print(f"\n{'─'*50}")
print(f"  Theoretical optimum ({BLADE_CHOICE}-blade)")
print(f"{'─'*50}")
print(f"  Diameter : {best_d:.1f} in")
print(f"  Pitch    : {best_p:.1f} in")
print(f"  Thrust   : {best_t:.1f} g  (goal: ≥{THRUST_GOAL} g)")
print(f"  Amps     : {best_a:.2f} A  (limit: <{AMP_LIMIT} A)")
print(f"{'─'*50}\n")

# ─── Real candidates ──────────────────────────────────────────────────────────
real_ok   = sub[(sub.Thrust >= THRUST_GOAL) & (sub.Amps < AMP_LIMIT)].copy()
real_fail = sub[~((sub.Thrust >= THRUST_GOAL) & (sub.Amps < AMP_LIMIT))].copy()

# ─── Plot ─────────────────────────────────────────────────────────────────────
fig = go.Figure()

# 1. Main surface — Z=thrust, color=amps, customdata carries amps for hover
fig.add_trace(go.Surface(
    x=diams, y=pitches, z=T_surf,
    surfacecolor=A_surf,
    customdata=A_surf,
    colorscale="RdYlGn_r",  # green=low amps (good), red=high amps (bad)
    cmin=sub["Amps"].min(),
    cmax=sub["Amps"].max(),
    opacity=0.75,
    showscale=True,
    colorbar=dict(
        title=dict(text="Amps (A)", side="right"),
        thickness=15, len=0.5,
        tickvals=[sub["Amps"].min(), AMP_LIMIT, sub["Amps"].max()],
        ticktext=[f"{sub['Amps'].min():.0f}A", f"{AMP_LIMIT}A limit", f"{sub['Amps'].max():.0f}A"],
    ),
    name="Thrust surface",
    hovertemplate=(
        "Diameter: %{x:.1f} in<br>"
        "Pitch: %{y:.1f} in<br>"
        "Thrust: %{z:.0f} g<br>"
        "Amps: %{customdata:.1f} A"
        "<extra></extra>"
    ),
))

# 2. Thrust goal reference plane
fig.add_trace(go.Surface(
    x=diams,
    y=pitches,
    z=np.full_like(T_surf, THRUST_GOAL),
    colorscale=[[0, "rgba(255,80,80,0.18)"], [1, "rgba(255,80,80,0.18)"]],
    showscale=False,
    opacity=0.5,
    name=f"Thrust goal ({THRUST_GOAL} g)",
    hoverinfo="skip",
))

# 3. Passing data points
if len(real_ok):
    fig.add_trace(go.Scatter3d(
        x=real_ok["Diameter"], y=real_ok["Pitch"], z=real_ok["Thrust"],
        mode="markers+text",
        marker=dict(size=7, color="limegreen", line=dict(color="white", width=1)),
        text=[f"{int(r.Diameter)}×{int(r.Pitch)}" for _, r in real_ok.iterrows()],
        textposition="top center",
        textfont=dict(size=9, color="limegreen"),
        customdata=real_ok[["Amps"]].values,
        name="✓ Meets goals",
        hovertemplate=(
            "<b>%{text}</b><br>"
            "Thrust: %{z:.0f} g<br>"
            "Amps: %{customdata[0]:.1f} A"
            "<extra></extra>"
        ),
    ))

# 4. Failing data points
if len(real_fail):
    fig.add_trace(go.Scatter3d(
        x=real_fail["Diameter"], y=real_fail["Pitch"], z=real_fail["Thrust"],
        mode="markers",
        marker=dict(size=5, color="tomato", opacity=0.7, line=dict(color="white", width=0.5)),
        customdata=real_fail[["Amps"]].values,
        name="✗ Fails goals",
        hovertemplate=(
            "Diameter: %{x} in<br>"
            "Pitch: %{y} in<br>"
            "Thrust: %{z:.0f} g<br>"
            "Amps: %{customdata[0]:.1f} A"
            "<extra></extra>"
        ),
    ))

# 5. Theoretical optimum marker
fig.add_trace(go.Scatter3d(
    x=[best_d], y=[best_p], z=[best_t],
    mode="markers+text",
    marker=dict(size=12, color="gold", symbol="diamond", line=dict(color="white", width=2)),
    text=[f"Optimum<br>{best_d:.1f}×{best_p:.1f}<br>{best_t:.0f}g / {best_a:.1f}A"],
    textposition="top center",
    textfont=dict(size=10, color="gold"),
    name=f"★ Optimum ({best_d:.1f}\"Ø × {best_p:.1f}\" pitch)",
))

# ─── Layout ───────────────────────────────────────────────────────────────────
fig.update_layout(
    title=dict(
        text=(f"{BLADE_CHOICE}-Blade Propeller — Thrust Surface<br>"
              f"<sup>Goal: ≥{THRUST_GOAL} g thrust  |  <{AMP_LIMIT} A  |  "
              f"smallest diameter, highest pitch</sup>"),
        font=dict(size=17),
        x=0.5, xanchor="center",
    ),
    legend=dict(
        x=0.01, y=0.99,
        bgcolor="rgba(0,0,0,0.4)",
        font=dict(color="white", size=11),
        bordercolor="rgba(255,255,255,0.2)",
        borderwidth=1,
    ),
    scene=dict(
        xaxis=dict(title="Diameter (in)", backgroundcolor="rgb(210,215,235)", gridcolor="white", showbackground=True),
        yaxis=dict(title="Pitch (in)",    backgroundcolor="rgb(225,210,235)", gridcolor="white", showbackground=True),
        zaxis=dict(title="Thrust (g)",    backgroundcolor="rgb(235,230,210)", gridcolor="white", showbackground=True),
        camera=dict(eye=dict(x=1.6, y=-1.6, z=0.9)),
        aspectmode="manual",
        aspectratio=dict(x=1, y=1, z=0.85),
    ),
    margin=dict(l=10, r=10, b=10, t=80),
    paper_bgcolor="white",
)

fig.show()