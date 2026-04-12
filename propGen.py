import pandas as pd
import numpy as np
from scipy.interpolate import griddata

# ─── Config ───────────────────────────────────────────────────────────────────
SHEET_ID    = "1cS6S8ULz608Zomaz527y1Dg4Eam2BbXB60R-wgyfbHs"
THRUST_GOAL = 650   # g
AMP_LIMIT   = 14    # A
GRID_RES    = 400   # interpolation resolution (higher = more precise)

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

print("=" * 60)
print(f"  Propeller Optimizer")
print(f"  Goal: ≥{THRUST_GOAL} g thrust  |  <{AMP_LIMIT} A  |  min diameter, max pitch")
print("=" * 60)

results = []

for blades in sorted(df["Blades"].unique()):
    sub = df[df["Blades"] == blades]
    if len(sub) < 4:
        continue

    pts     = sub[["Diameter","Pitch"]].values
    diams   = np.linspace(sub.Diameter.min(), sub.Diameter.max(), GRID_RES)
    pitches = np.linspace(sub.Pitch.min(),    sub.Pitch.max(),    GRID_RES)
    D, P    = np.meshgrid(diams, pitches)
    gp      = np.column_stack([D.ravel(), P.ravel()])

    T = griddata(pts, sub["Thrust"].values, gp, method="linear").reshape(D.shape)
    A = griddata(pts, sub["Amps"].values,   gp, method="linear").reshape(D.shape)

    valid = (T >= THRUST_GOAL) & (A < AMP_LIMIT) & ~np.isnan(T) & ~np.isnan(A)

    if not valid.any():
        print(f"\n  {blades}-blade:  No region meets both goals within tested data range.")
        continue

    # Score: minimize diameter, maximize pitch → minimize (D - P)
    score = D - P
    score[~valid] = np.inf
    idx = np.unravel_index(np.argmin(score), score.shape)

    bd, bp, bt, ba = D[idx], P[idx], T[idx], A[idx]
    results.append({"Blades": blades, "Diameter": bd, "Pitch": bp, "Thrust": bt, "Amps": ba})

    # Real measured props that also pass
    real_pass = sub[(sub.Thrust >= THRUST_GOAL) & (sub.Amps < AMP_LIMIT)]\
                    .sort_values(["Diameter","Pitch"], ascending=[True,False])

    print(f"\n  ── {blades}-Blade ──────────────────────────────────────")
    print(f"  Theoretical optimum (interpolated):")
    print(f"    Diameter : {bd:.1f} in")
    print(f"    Pitch    : {bp:.1f} in")
    print(f"    Thrust   : {bt:.1f} g")
    print(f"    Amps     : {ba:.2f} A")

    if len(real_pass):
        print(f"\n  Measured props that meet goals (best first):")
        for _, r in real_pass.iterrows():
            flag = "  ← smallest diam / highest pitch" if (r.Diameter == real_pass.Diameter.min() and r.Pitch == real_pass[real_pass.Diameter == real_pass.Diameter.min()].Pitch.max()) else ""
            print(f"    {int(r.Diameter)}\" diam × {int(r.Pitch)}\" pitch  →  {r.Thrust:.0f} g  /  {r.Amps:.1f} A{flag}")
    else:
        print(f"  No measured props meet both goals.")

print("\n" + "=" * 60)
if results:
    best_overall = min(results, key=lambda r: r["Diameter"] - r["Pitch"])
    print(f"  Best across all blade counts:")
    print(f"    {int(best_overall['Blades'])} blades  |  {best_overall['Diameter']:.1f}\" diam  |  {best_overall['Pitch']:.1f}\" pitch")
    print(f"    {best_overall['Thrust']:.0f} g thrust  |  {best_overall['Amps']:.2f} A")
print("=" * 60)