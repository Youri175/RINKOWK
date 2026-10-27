"""Voorbeeldplaatjes van een 4DVis-model (orthografisch, met eenvoudige belichting).

    python3 tools/render.py lever/<CODE> [--stand Giek=-77 --stand Bak=95] [--uit voorbeeld.png]

Zonder --stand: ruststand in vier aanzichten (links, voor, boven, schuin).
Met --stand: dezelfde aanzichten in die joint-stand (cilinders en bakkoppeling volgen).
"""
import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controleer import joint_matrix, lees_cilinders, lees_joints, wereld_met_cilinders  # noqa: E402
from gltf_io import lees_glb, transformeer  # noqa: E402

LICHT = np.array([0.4, 0.8, 0.45])
LICHT /= np.linalg.norm(LICHT)


def kleuren(sc):
    uit = []
    for m in sc.gltf.materials or []:
        c = m.pbrMetallicRoughness.baseColorFactor if m.pbrMetallicRoughness else [0.8] * 4
        uit.append(np.array(c, float))
    return uit


def driehoeken(sc, wm):
    mats = kleuren(sc)
    tri, kl = [], []
    for n in sc.nodes:
        if n.mesh is None:
            continue
        for p in sc.meshes[n.mesh]:
            v = transformeer(p.posities, wm[n.index])[p.indices]
            tri.append(v)
            c = mats[p.materiaal] if p.materiaal is not None else np.array([0.8, 0.8, 0.8, 1])
            kl.append(np.repeat(c[None], len(v), 0))
    return np.vstack(tri), np.vstack(kl)


def teken(ax, tri, kl, kijk, omhoog, titel):
    kijk = np.asarray(kijk, float) / np.linalg.norm(kijk)
    rechts = np.cross(kijk, omhoog)
    rechts /= np.linalg.norm(rechts)
    op = np.cross(rechts, kijk)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nl = np.linalg.norm(n, axis=1)
    ok = nl > 1e-12
    tri, kl, n = tri[ok], kl[ok], n[ok] / nl[ok, None]
    zicht = n @ -kijk
    voor = zicht > 0                                  # achterkanten weglaten, behalve glas
    voor |= kl[:, 3] < 1
    tri, kl, n = tri[voor], kl[voor], n[voor]
    diepte = (tri.mean(1) @ kijk)
    volg = np.argsort(-diepte)
    p2 = np.stack([tri @ rechts, tri @ op], -1)[volg]
    licht = 0.35 + 0.65 * np.clip(np.abs(n @ LICHT), 0, 1)
    c = kl[volg].copy()
    c[:, :3] *= licht[volg, None]
    ax.add_collection(PolyCollection(p2, facecolors=np.clip(c, 0, 1), edgecolors="none", antialiased=False))
    ax.autoscale()
    ax.set_aspect("equal")
    ax.set_title(titel, fontsize=9)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25)


def render(map_, standen, uit, titel=None):
    sc = lees_glb(os.path.join(map_, "model.glb"))
    with open(os.path.join(map_, "manifest.json"), encoding="utf-8") as f:
        man = json.load(f)
    joints = {j["naam"]: j for j in lees_joints(sc, man)}
    extra = {joints[k]["index"]: joint_matrix(joints[k], v) for k, v in standen.items()}
    kop = man.get("koppeling") if man.get("koppeling") and sc.node(man["koppeling"].get("tuimelaar", "")) else None
    wm, _ = wereld_met_cilinders(sc, lees_cilinders(sc), extra, kop)
    tri, kl = driehoeken(sc, wm)
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    teken(ax[0, 0], tri, kl, [1, 0, 0], [0, 1, 0], "Linkerzijde (kijkrichting +X; voorkant links)")
    teken(ax[0, 1], tri, kl, [0, 0, 1], [0, 1, 0], "Vooraanzicht (kijkrichting +Z)")
    teken(ax[1, 0], tri, kl, [0, -1, 0], [0, 0, -1], "Bovenaanzicht (voorkant boven)")
    teken(ax[1, 1], tri, kl, [0.75, -0.45, 0.6], [0, 1, 0], "Schuin linksvoor")
    for a in ax.flat:
        a.axhline(0, color="#7a6a50", lw=0.6) if a is not ax[1, 0] else None
    stand = ", ".join(f"{k} {v:+g}°" for k, v in standen.items()) or "ruststand"
    fig.suptitle(titel or f"{man.get('merk', '')} {man.get('typeaanduiding', '')} — {stand}", fontsize=12)
    fig.tight_layout()
    fig.savefig(uit, dpi=110)
    plt.close(fig)
    return uit


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map")
    ap.add_argument("--stand", action="append", default=[], help="Joint=hoek, bv. Giek=-77")
    ap.add_argument("--uit")
    a = ap.parse_args()
    standen = {s.split("=")[0]: float(s.split("=")[1]) for s in a.stand}
    print(render(a.map, standen, a.uit or os.path.join(a.map, "voorbeeld.png")))


if __name__ == "__main__":
    main()
