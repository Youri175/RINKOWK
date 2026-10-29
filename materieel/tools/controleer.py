"""Stap 7: controle van een 4DVis-materieelmodel.

Gebruik:
    python3 tools/controleer.py <map met model.glb + manifest.json> [--referentie a.glb b.glb]
                                [--doel-polygonen 50000] [--geen-animatie]

Schrijft in de map: rapport.md, controle.json en test_animatie.glb
(per joint 0 -> min -> max -> 0, cilinders opnieuw gericht). Exitcode 1 bij een FOUT.
Referentie-GLB's in materieel/referentie/ worden automatisch meegenomen.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gltf_io import (lees_glb, matrix_naar_quat, rotatie_matrix,  # noqa: E402
                     rotatie_van_naar, voeg_animaties_toe)

HIER = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HIER, "conventies.json"), encoding="utf-8") as _f:
    CONV = json.load(_f)
SL = CONV["extras_sleutels"]
RICHTING = np.array(CONV["cilinders"]["richting_lokaal"], float)
ASSEN_VOOR = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}
TOL_PIVOT = 0.01          # m: afwijking draaipunt t.o.v. manifest
TOL_GROND = 0.01          # m
TOL_DOORSNIJDING = 0.01   # m: nieuw contact tussen meshdelen
TOL_BESTAAND = 0.05       # m: extra indringing van een contact dat in rust al bestaat (pen in oog e.d.)
TOL_PEN = 0.01            # m: penafstand
TOL_WERKBEREIK = 0.05     # relatief


class Resultaat:
    def __init__(self):
        self.items = []

    def __call__(self, groep, status, tekst):
        self.items.append({"groep": groep, "status": status, "tekst": tekst})

    def ok(self, g, t):
        self(g, "OK", t)

    def warn(self, g, t):
        self(g, "WAARSCHUWING", t)

    def fout(self, g, t):
        self(g, "FOUT", t)

    @property
    def fouten(self):
        return [i for i in self.items if i["status"] == "FOUT"]

    def heeft_fout(self, groep):
        return any(i["groep"] == groep and i["status"] == "FOUT" for i in self.items)


def bbox(punten):
    return punten.min(0), punten.max(0)


def buiten_bbox(p, lo, hi):
    return float(np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi))))


def scene_punten(sc, wm, indices=None):
    tri = sc.wereld_driehoeken(indices if indices is not None else range(len(sc.nodes)), wm)
    return np.vstack([v for v, _ in tri.values()]) if tri else np.zeros((0, 3))


def zonder_voorvoegsel(naam):
    return re.sub(r"^[A-Z]{2}_[A-Za-z0-9\-]+_", "", naam)


def voorkant_schatting(sc, wm, namen=("Giek", "Bak", "Stick", "Voorwagen")):
    """Horizontale richting (as + teken) van voorkant-onderdelen t.o.v. het nulpunt."""
    for n in namen:
        node = sc.node(n) or next((x for x in sc.nodes if zonder_voorvoegsel(x.naam) == n), None)
        if node is None:
            continue
        pts = scene_punten(sc, wm, sc.afstammelingen(node.index))
        if len(pts) == 0:
            continue
        h = np.array([pts.mean(0)[0], 0, pts.mean(0)[2]])
        if np.linalg.norm(h) < 0.05:
            continue
        i = int(np.argmax(np.abs(h)))
        return ("+" if h[i] > 0 else "-") + "XYZ"[i], n
    return None, None


# --------------------------------------------------------------------------
# Joints en cilinders uit het model
# --------------------------------------------------------------------------
def lees_joints(sc, man):
    """[{index, naam, as, as_gltf, min, max, type}] uit extras.draaias (+ manifest)."""
    man_j = {j["naam"]: j for j in man.get("joints", [])}
    uit = []
    for n in sc.nodes:
        tekst = n.extras.get(SL["as_tekst"])
        if not isinstance(tekst, str):
            continue
        m = re.search(r"lokaal\s+([XYZ])", tekst)
        if not m:
            continue
        mj = man_j.get(n.naam, {})
        lo = n.extras.get(SL["min"], mj.get("min"))
        hi = n.extras.get(SL["max"])
        if hi is None:                          # referentie kent ook max_knik_graden e.d.
            hi = next((v for k, v in n.extras.items() if re.match(r"^max_.*_graden$", k)), mj.get("max"))
        if hi is None:                          # alleen beschrijvende draaias (bv. hendels)
            continue
        uit.append({"index": n.index, "naam": n.naam, "as": m.group(1),
                    "as_gltf": mj.get("as_gltf", CONV["as_naar_gltf"][m.group(1)]),
                    "min": lo if lo is not None else 0, "max": hi,
                    "type": mj.get("type", "rotatie"), "positief": mj.get("positief", "")})
    return uit


def lees_cilinders(sc):
    paren = []
    for n in sc.nodes:
        if n.naam.endswith("_Huis"):
            s = sc.node(n.naam[:-5] + "_Stang")
            if s is not None:
                paren.append((n.index, s.index))
    return paren


def joint_matrix(j, hoek):
    if j["type"] == "translatie":
        m = np.eye(4)
        m[:3, 3] = np.asarray(j["as_gltf"], float) * hoek
        return m
    return rotatie_matrix(j["as_gltf"], hoek)


def rot_x(hoek_rad):
    m = np.eye(4)
    c, s = np.cos(hoek_rad), np.sin(hoek_rad)
    m[1:3, 1:3] = [[c, -s], [s, c]]
    return m


def los_koppeling(sc, kop, extra):
    """Vierstangen Stick-Tuimelaar-Bakstang-Bak: zet Tuimelaar en Bakstang bij de bakstand in `extra`.
    Rekent in het lokale vlak (Y, Z) van de Stick; alle scharnieren draaien om lokale X."""
    if not kop or not sc.node(kop.get("tuimelaar", "")):
        return extra
    st, bak = sc.node(kop["stick"]), sc.node(kop["bak"])
    tui, bst = sc.node(kop["tuimelaar"]), sc.node(kop["bakstang"])
    wm0 = sc.wereld_matrices()
    inv = np.linalg.inv(wm0[st.index])

    def lok(p):
        return (inv @ np.append(p, 1))[1:3]

    K, B, C = lok(wm0[bak.index][:3, 3]), lok(wm0[tui.index][:3, 3]), lok(wm0[bst.index][:3, 3])
    D = lok(np.array(kop["pen_bakstang_bak_m"], float))
    rb = extra.get(bak.index, np.eye(4))[1:3, 1:3]
    D2 = K + rb @ (D - K)
    r, l_ = np.linalg.norm(C - B), np.linalg.norm(D - C)
    d = np.linalg.norm(D2 - B)
    a = (r ** 2 - l_ ** 2 + d ** 2) / (2 * d)
    h = np.sqrt(max(r ** 2 - a ** 2, 0.0))
    e = (D2 - B) / d
    m = B + a * e
    opl = [m + h * np.array([-e[1], e[0]]), m - h * np.array([-e[1], e[0]])]

    def kr(u, v):
        return u[0] * v[1] - u[1] * v[0]

    kant = np.sign(kr(D - B, C - B))
    C2 = max(opl, key=lambda q: kant * kr(D2 - B, q - B))

    def hoek(v):
        return np.arctan2(v[1], v[0])            # vanaf lokale +Y naar +Z = positief om +X

    ft = hoek(C2 - B) - hoek(C - B)
    fb = hoek(D2 - C2) - hoek(D - C)
    extra = dict(extra)
    extra[tui.index] = rot_x(ft)
    extra[bst.index] = rot_x(fb - ft)
    return extra


def wereld_met_cilinders(sc, cilinders, extra, kop=None):
    """Wereldmatrices met joint-standen `extra`: eerst de bakkoppeling oplossen, dan elke
    cilinderhelft opnieuw naar de pen van de andere helft richten (zoals de 4DVis-rig doet)."""
    extra = los_koppeling(sc, kop, dict(extra))
    wm = sc.wereld_matrices(extra)
    for h, s in cilinders:
        for eigen, ander in ((h, s), (s, h)):
            n = sc.nodes[eigen]
            pw = wm[n.parent][:3, :3] if n.parent is not None else np.eye(3)
            d = wm[ander][:3, 3] - wm[eigen][:3, 3]
            r0 = n.matrix[:3, :3]
            dl = pw.T @ d
            rn = rotatie_van_naar(r0 @ RICHTING, dl) @ r0
            m = np.eye(4)
            m[:3, :3] = r0.T @ rn
            extra[eigen] = m
    return sc.wereld_matrices(extra), extra


# --------------------------------------------------------------------------
def controleer_referenties(paden, sc, man, model_info, R):
    rij = []
    for p in paden:
        try:
            ref = lees_glb(p)
        except Exception as e:
            R.warn("referentie", f"{os.path.basename(p)} niet leesbaar: {e}")
            continue
        wm = ref.wereld_matrices()
        pts = scene_punten(ref, wm)
        lo, hi = bbox(pts)
        vk, via = voorkant_schatting(ref, wm)
        wortels = [ref.nodes[w].naam for w in ref.wortels]
        sleutels = sorted({k for n in ref.nodes for k in n.extras})
        rij.append({"bestand": os.path.basename(p), "afm": (hi - lo).round(3).tolist(),
                    "min": lo.round(3).tolist(), "max": hi.round(3).tolist(),
                    "voorkant": vk, "via": via, "wortels": wortels, "extras_sleutels": sleutels})
        if vk and model_info["voorkant"] and vk != model_info["voorkant"]:
            R.fout("referentie", f"voorkant model {model_info['voorkant']} wijkt af van "
                                 f"{os.path.basename(p)} ({vk}); corrigeer het model")
        elif vk and model_info["voorkant"]:
            R.ok("referentie", f"voorkant {vk} gelijk aan {os.path.basename(p)}")
        # opbouw: dezelfde rignamen moeten onder dezelfde parent hangen
        ref_par = {n.naam: (ref.nodes[n.parent].naam if n.parent is not None else None) for n in ref.nodes}
        if len(ref.wortels) == 1:
            ref_par = {k: (None if v == ref.nodes[ref.wortels[0]].naam else v) for k, v in ref_par.items()}
        model_par = {}
        for n in sc.nodes:
            par = sc.nodes[n.parent].naam if n.parent is not None else None
            if n.parent is not None and n.parent in sc.wortels:
                par = None
            model_par[zonder_voorvoegsel(n.naam)] = zonder_voorvoegsel(par) if par else None
        rigdef = CONV["rignamen"].get(man.get("rig", ""), {})
        rig_pat = [re.compile(x) for x in rigdef.get("optioneel_patronen", [])]
        if not rigdef or not all(ref.node(v) for v in rigdef.get("verplicht", [])):
            gedeeld = []                                # andere machinesoort: opbouw niet vergelijken
        else:
            gedeeld = [k for k in model_par if k in ref_par and (
                k in rigdef["verplicht"] or k in rigdef.get("optioneel", []) or any(x.match(k) for x in rig_pat))]
        afwijkend = [f"{k}: model onder {model_par[k] or 'wortel'}, referentie onder {ref_par[k] or 'wortel'}"
                     for k in gedeeld if model_par[k] != ref_par[k]]
        if gedeeld:
            (R.warn if afwijkend else R.ok)(
                "referentie", f"opbouw t.o.v. {os.path.basename(p)}: " +
                ("; ".join(afwijkend) if afwijkend else f"{len(gedeeld)} gedeelde nodes hangen onder dezelfde parent"))
            # extras-sleutels per gedeelde node
            mist = []
            for k in gedeeld:
                rn = ref.node(k)
                mn = next((n for n in sc.nodes if zonder_voorvoegsel(n.naam) == k), None)
                ontbr = [s for s in rn.extras if s not in mn.extras and s not in ("ao_groep",)]
                if ontbr:
                    mist.append(f"{k}: {ontbr}")
            if mist:
                R.warn("referentie", f"extras die de referentie wel heeft: {'; '.join(mist)}")
    return rij


def werkbereik(sc, joints, man, R):
    ds = man.get("datasheet", {}).get("werkbereik")
    namen = {j["naam"]: j for j in joints}
    if not ds or not all(k in namen for k in ("Giek", "Stick", "Bak")) or sc.node("Bovenwagen") is None:
        return None
    bak = sc.node("Bak").index
    zw = sc.wereld_matrices()[sc.node("Bovenwagen").index][:3, 3]
    hoeken = {k: np.linspace(namen[k]["min"], namen[k]["max"], n) for k, n in (("Giek", 25), ("Stick", 25), ("Bak", 9))}
    mesh = sc.meshes[sc.nodes[bak].mesh]
    lok = np.vstack([p.posities for p in mesh])
    straal = np.linalg.norm(lok, axis=1)                 # oorsprong = bakpen
    lok = lok[straal >= straal.max() - 0.03]             # tandpunt: datasheetmaten gaan tot de tanden
    diepte, hoogte, reik = 0.0, 0.0, 0.0
    for g in hoeken["Giek"]:
        for s in hoeken["Stick"]:
            for b in hoeken["Bak"]:
                wm = sc.wereld_matrices({namen["Giek"]["index"]: joint_matrix(namen["Giek"], g),
                                         namen["Stick"]["index"]: joint_matrix(namen["Stick"], s),
                                         namen["Bak"]["index"]: joint_matrix(namen["Bak"], b)})
                p = lok @ wm[bak][:3, :3].T + wm[bak][:3, 3]
                diepte = max(diepte, -p[:, 1].min())
                hoogte = max(hoogte, p[:, 1].max())
                grond = np.abs(p[:, 1]) < 0.05
                if grond.any():
                    reik = max(reik, float(np.hypot(p[grond, 0] - zw[0], p[grond, 2] - zw[2]).max()))
    model = {"max_graafdiepte_m": diepte, "max_graafhoogte_m": hoogte, "max_reikwijdte_grond_m": reik}
    rijen = []
    for k, v in model.items():
        if k not in ds:
            continue
        rel = (v - ds[k]) / ds[k]
        rijen.append({"grootheid": k, "model": round(v, 3), "datasheet": ds[k], "afwijking": round(rel, 3)})
        (R.ok if abs(rel) <= TOL_WERKBEREIK else R.warn)(
            "werkbereik", f"{k}: model {v:.2f} m, datasheet {ds[k]:.2f} m ({rel:+.1%})")
    return rijen


# --------------------------------------------------------------------------
def controleer(map_, referenties=(), doel_poly=50000, animatie=True):
    R = Resultaat()
    glb = os.path.join(map_, "model.glb")
    with open(os.path.join(map_, "manifest.json"), encoding="utf-8") as f:
        man = json.load(f)
    sc = lees_glb(glb)
    wm = sc.wereld_matrices()
    namen = [n.naam for n in sc.nodes]
    code = man.get("code", os.path.basename(os.path.normpath(map_)))
    rig = man.get("rig", "")
    rigdef = CONV["rignamen"].get(rig, {})
    joints = lees_joints(sc, man)
    cilinders = lees_cilinders(sc)
    cil_nodes = {i for p in cilinders for i in p}
    kop = man.get("koppeling")
    if kop and not all(sc.node(kop.get(k, "")) for k in ("stick", "bak", "tuimelaar", "bakstang")):
        kop = None
    kop_nodes = {sc.node(kop[k]).index for k in ("tuimelaar", "bakstang")} if kop else set()
    # gekoppelde joints, bv. {"KR_x_Takel": {"joint": "KR_x_Giek", "factor": -1}}: last hangt verticaal
    per_naam = {j["naam"]: j for j in joints}
    volgt = {k: v for k, v in man.get("volgt", {}).items() if k in per_naam and v.get("joint") in per_naam}

    def stand(j, hoek):
        """Joint-matrices voor `j` op `hoek`, inclusief joints die hem volgen."""
        uit = {j["index"]: joint_matrix(j, hoek)}
        for naam, v in volgt.items():
            if v["joint"] == j["naam"]:
                uit[per_naam[naam]["index"]] = joint_matrix(per_naam[naam], hoek * v.get("factor", 1))
        return uit

    # ---------------- manifest ----------------
    sjabloon = ["merk", "typeaanduiding", "machinetype", "bron_url", "licentie", "bronformaat",
                "eindformaat", "eenheid", "yaw_correctie_deg", "schaal", "afmetingen_m", "polygonen",
                "detailniveau", "rig", "categorie", "bibliotheek_csv", "joints",
                "afwijkingen_van_conventie", "aannames", "openstaande_punten"]
    mis = [k for k in sjabloon if k not in man]
    (R.fout if mis else R.ok)("manifest", f"ontbrekende sleutels: {mis}" if mis else "alle sjabloonsleutels aanwezig")
    if man.get("categorie") not in CONV["categorie_lijst"]:
        R.fout("manifest", f"categorie '{man.get('categorie')}' is geen 4DVis-groep")
    csv = man.get("bibliotheek_csv", "").split(";")
    if len(csv) != 4 or csv[3] not in ("", "Graafmachine", "Knikdumper"):
        R.fout("manifest", f"bibliotheek_csv '{man.get('bibliotheek_csv')}' heeft niet de vorm Model;ModelDraaiing;ModelSchaal;Rig")
    else:
        R.ok("manifest", f"bibliotheek_csv: {man['bibliotheek_csv']}")
    if man.get("eindformaat") != "GLB" or man.get("eenheid") != "m":
        R.fout("manifest", "eindformaat moet GLB en eenheid m zijn")

    # ---------------- hiërarchie ----------------
    if len(sc.wortels) != 1:
        R.warn("hiërarchie", f"{len(sc.wortels)} wortelnodes; verwacht 1 (zoals de referentie)")
    elif sc.nodes[sc.wortels[0]].mesh is not None:
        R.warn("hiërarchie", "wortelnode heeft geometrie; referentie gebruikt een lege wortel")
    verwacht = man.get("hierarchie", {})
    for n, p in verwacht.items():
        node = sc.node(n)
        if node is None:
            R.fout("hiërarchie", f"node {n} ontbreekt in GLB")
            continue
        echte = sc.nodes[node.parent].naam if node.parent is not None else None
        if echte != p:
            R.fout("hiërarchie", f"{n}: parent {echte}, manifest zegt {p}")
    extra = [n for n in namen if n not in verwacht]
    if extra:
        R.warn("hiërarchie", f"nodes niet in manifest: {extra}")
    if not R.heeft_fout("hiërarchie"):
        R.ok("hiërarchie", f"{len(namen)} nodes, parent-child gelijk aan manifest")
    for ouder, kind in rigdef.get("keten", []):
        a, b = sc.node(ouder), sc.node(kind)
        if a is None or b is None:
            continue
        if a.index not in sc.voorouders(b.index):
            R.fout("hiërarchie", f"rigketen: {kind} hangt niet onder {ouder}")
        elif b.parent != a.index:
            R.warn("hiërarchie", f"rigketen: {kind} hangt indirect onder {ouder}")
        else:
            R.ok("hiërarchie", f"rigketen {ouder} → {kind}")

    # ---------------- bevroren transformaties ----------------
    niet, schaal = [], []
    for n in sc.nodes:
        m3 = np.asarray(n.matrix[:3, :3])
        if not np.allclose(np.linalg.norm(m3, axis=0), 1, atol=1e-6) or np.linalg.det(m3) < 0:
            schaal.append(n.naam)
        elif not np.allclose(m3, np.eye(3), atol=1e-6) and n.index not in cil_nodes:
            niet.append(n.naam)
    (R.fout if niet or schaal else R.ok)(
        "bevroren", f"rotatie niet toegepast: {niet}; schaal/spiegeling: {schaal}" if niet or schaal
        else "alle nodes schaal 1; rotatie alleen op gerichte cilinderhelften")

    # ---------------- namen ----------------
    dubbel = sorted({n for n in namen if namen.count(n) > 1})
    (R.fout if dubbel else R.ok)("namen", f"dubbele namen: {dubbel}" if dubbel else "alle namen uniek")
    tcode = CONV["typecodes"].get(man.get("machinetype", ""), "")
    prefix_re = re.compile(rf"^{tcode}_[A-Za-z0-9\-]+_[A-Za-z0-9_\-]+$")
    rig_exact = set(rigdef.get("verplicht", [])) | set(rigdef.get("optioneel", []))
    rig_pat = [re.compile(p) for p in rigdef.get("optioneel_patronen", [])]
    alle_rig = set().union(*[set(d.get("verplicht", [])) | set(d.get("optioneel", []))
                             for d in CONV["rignamen"].values()])
    for n in rigdef.get("verplicht", []):
        (R.ok if n in namen else R.fout)("namen", f"rignaam {n} {'aanwezig' if n in namen else 'ONTBREEKT'}")
    for n in namen:
        if n in rig_exact or any(p.match(n) for p in rig_pat) or n == man.get("wortel"):
            continue
        staart = n.split("_")[-1]
        if staart in alle_rig and rig and n != staart:
            R.fout("namen", f"{n}: rignaam met voorvoegsel; 4DVis matcht letterlijk '{staart}'")
        elif not prefix_re.match(n):
            R.warn("namen", f"{n}: volgt [TYPE]_[MODEL]_[ONDERDEEL] niet (type {tcode})")

    # ---------------- bounding box, nulpunt, voorkant ----------------
    pts = scene_punten(sc, wm)
    lo, hi = bbox(pts)
    afm = hi - lo
    R.ok("afmetingen", f"bbox breedte(X) {afm[0]:.3f} × hoogte(Y) {afm[1]:.3f} × lengte(Z) {afm[2]:.3f} m")
    if afm.max() > 60 or afm.max() < 0.5:
        R.fout("afmetingen", f"grootste maat {afm.max():.2f} m: waarschijnlijk schaalfout (mm/m)")
    ma = man.get("afmetingen_m", {})
    if ma:
        d = np.abs(np.array([ma.get("breedte", 0), ma.get("hoogte", 0), ma.get("lengte", 0)]) - afm)
        (R.ok if d.max() < 0.01 else R.warn)("afmetingen", f"manifest-afmetingen wijken max {d.max():.3f} m af")
    for k, v in man.get("datasheet", {}).get("afmetingen", {}).items():
        as_i = {"breedte_m": 0, "hoogte_m": 1, "lengte_m": 2}.get(k)
        if as_i is not None and v:
            rel = (afm[as_i] - v) / v
            (R.ok if abs(rel) <= TOL_WERKBEREIK else R.warn)(
                "afmetingen", f"{k} model {afm[as_i]:.3f} m, datasheet {v:.3f} m ({rel:+.1%})")
    nul = sc.node(man.get("nulpunt_onderdeel", "")) or sc.nodes[sc.wortels[0]]
    joint_idx = {j["index"] for j in joints}
    vast = [i for i in sc.afstammelingen(nul.index)
            if not ({i} | set(sc.voorouders(i))) & joint_idx]          # nulpunt-onderdeel + vaste kinderen
    np_ = scene_punten(sc, wm, vast)
    if len(np_):
        nl, nh = bbox(np_)
        (R.ok if abs(nl[1]) <= TOL_GROND else R.fout)("nulpunt", f"grond van {nul.naam}: Y = {nl[1]:.4f} m")
        mxz = np.array([(nl[0] + nh[0]) / 2, (nl[2] + nh[2]) / 2])
        (R.ok if np.abs(mxz).max() <= 0.10 else R.fout)(          # referentie Cat336E: 7,5 cm
            "nulpunt", f"midden {nul.naam} op X,Z = {mxz.round(3).tolist()}")
    if lo[1] < -TOL_GROND:
        R.warn("nulpunt", f"in ruststand ligt geometrie onder maaiveld (Y = {lo[1]:.3f})")
    vk, via = voorkant_schatting(sc, wm)
    if vk:
        (R.ok if vk == CONV["doel_voorkant"] else R.fout)(
            "voorkant", f"voorkant {vk} (afgeleid van {via}); conventie {CONV['doel_voorkant']}")
    else:
        R.warn("voorkant", "voorkant niet automatisch te bepalen (geen Giek/Bak/Voorwagen)")

    ref_paden = list(referenties) or sorted(glob.glob(os.path.join(os.path.dirname(HIER), "referentie", "*.glb")))
    refs = controleer_referenties(ref_paden, sc, man, {"voorkant": vk}, R) if ref_paden else []
    if not ref_paden:
        R.warn("referentie", "geen referentie-GLB's gevonden (materieel/referentie/)")

    # ---------------- polygonen, normalen, open randen ----------------
    tri_n = {n.naam: sum(len(p.indices) for p in sc.meshes[n.mesh]) for n in sc.nodes if n.mesh is not None}
    totaal = sum(tri_n.values())
    (R.ok if totaal <= doel_poly else R.fout)("polygonen", f"{totaal} driehoeken (doel ≤ {doel_poly})")
    if man.get("polygonen") != totaal:
        R.warn("polygonen", f"manifest zegt {man.get('polygonen')}, GLB heeft {totaal}")
    norm_fout, open_randen = [], {}
    for n in sc.nodes:
        if n.mesh is None:
            continue
        for k, p in enumerate(sc.meshes[n.mesh]):
            if p.normalen is None:
                norm_fout.append(f"{n.naam}[{k}]: geen normalen")
                continue
            if np.any(np.abs(np.linalg.norm(p.normalen, axis=1) - 1) > 1e-3):
                norm_fout.append(f"{n.naam}[{k}]: normalen niet genormaliseerd")
            v = p.posities[p.indices]
            fn = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
            ok = np.linalg.norm(fn, axis=1) > 1e-12
            vn = p.normalen[p.indices].sum(1)
            omgekeerd = np.mean(np.einsum("ij,ij->i", fn[ok], vn[ok]) < 0) if ok.any() else 0
            if omgekeerd > 0.01:
                norm_fout.append(f"{n.naam}[{k}]: {omgekeerd:.1%} faces met omgekeerde normaal")
        v_all = np.vstack([p.posities for p in sc.meshes[n.mesh]])
        f_all, off = [], 0
        for p in sc.meshes[n.mesh]:
            f_all.append(p.indices + off)
            off += len(p.posities)
        m = trimesh.Trimesh(v_all, np.vstack(f_all), process=True)
        if not m.is_watertight:
            randen = int(len(trimesh.grouping.group_rows(m.edges_sorted, require_count=1)))
            if randen:
                open_randen[n.naam] = randen          # 0 = gesloten, maar overlappende delen
        elif m.volume < 0:
            norm_fout.append(f"{n.naam}: gesloten mesh met negatief volume (binnenstebuiten)")
    (R.fout if norm_fout else R.ok)("normalen", "; ".join(norm_fout) if norm_fout else
                                    "alle primitieven hebben eenheidsnormalen, consistent met de winding")
    if open_randen:
        R.warn("faces", f"open randen (mogelijk ontbrekende faces): {open_randen}")
    else:
        R.ok("faces", "alle meshes gesloten (geen open randen)")

    # ---------------- texturen ----------------
    if not sc.afbeeldingen:
        R.ok("texturen", "geen texturen gebruikt")
    for a in sc.afbeeldingen:
        if a["ingebed"]:
            R.warn("texturen", f"{a['naam']}: ingebed; eis is een bestand naast het GLB")
        elif a["uri"] and not os.path.exists(os.path.join(map_, a["uri"])):
            R.fout("texturen", f"ontbrekende texture {a['uri']}")
        else:
            R.ok("texturen", f"{a['uri']} aanwezig")

    # ---------------- pivots ----------------
    dp_man = man.get("draaipunten_m", {})
    for j in joints:
        n = sc.nodes[j["index"]]
        p = wm[n.index][:3, 3]
        eigen = scene_punten(sc, wm, [n.index])
        tekst = f"{n.naam}: draaipunt {p.round(3).tolist()}"
        if n.naam in dp_man and np.linalg.norm(p - np.array(dp_man[n.naam])) > TOL_PIVOT:
            R.fout("pivot", f"{tekst} wijkt af van manifest {dp_man[n.naam]}")
            continue
        if len(eigen):
            elo, ehi = bbox(eigen)
            diag = float(np.linalg.norm(ehi - elo))
            b = buiten_bbox(p, elo, ehi)
            if j["as"] == "Z":
                b = buiten_bbox(p * [1, 0, 1], elo * [1, 0, 1], ehi * [1, 0, 1])  # verticale as: alleen X,Z
            if b > max(0.05, 0.05 * diag):
                (R.fout if b > 0.25 else R.warn)("pivot", f"{tekst} ligt {b:.3f} m buiten de eigen geometrie")
                continue
            is_wiel = "straal_band" in n.extras or "straal_steekcirkel" in n.extras
            if j["as"] != "Z" and not is_wiel and np.linalg.norm(p - (elo + ehi) / 2) < 0.02 * diag:
                R.warn("pivot", f"{tekst} ligt in het midden van de eigen bbox: scharnier niet gezet?")
                continue
        R.ok("pivot", f"{tekst} (= manifest, op de eigen geometrie)")

    # ---------------- extras ----------------
    man_j = {j["naam"]: j for j in man.get("joints", [])}
    for j in joints:
        n = sc.nodes[j["index"]]
        mj = man_j.get(n.naam)
        if mj is None:
            R.fout("extras", f"{n.naam}: joint niet in manifest")
        elif (mj["min"], mj["max"], mj["as"]) != (j["min"], j["max"], j["as"]):
            R.fout("extras", f"{n.naam}: extras en manifest verschillen")
        verw = rigdef.get("assen", {}).get(n.naam)
        if verw and verw != j["as"]:
            R.fout("extras", f"{n.naam}: as {j['as']}, rig {rig} verwacht {verw}")
    jn = {j["naam"] for j in joints}
    for naam in man_j:
        if naam not in jn:
            R.fout("extras", f"manifest-joint {naam} zonder {SL['as_tekst']}/{SL['max']} in GLB")
    for n in sc.nodes:
        if re.match(r"^(Wiel|Kettingwiel|Leiwiel)_", n.naam):
            mis = [k for k in ("straal_band", "straal_steekcirkel") if k not in n.extras]
            (R.warn if mis else R.ok)("extras", f"{n.naam}: {'mist ' + str(mis) if mis else 'stralen aanwezig'}")
    if joints and not R.heeft_fout("extras"):
        R.ok("extras", f"{len(joints)} joints met {SL['as_tekst']}/{SL['min']}/{SL['max']} in node-extras = manifest")

    # ---------------- cilinders ----------------
    for h, s in cilinders:
        nh, ns = sc.nodes[h], sc.nodes[s]
        ph, ps = wm[h][:3, 3], wm[s][:3, 3]
        d = float(np.linalg.norm(ps - ph))
        fouten = []
        for eigen, ander in ((h, s), (s, h)):
            r = wm[eigen][:3, :3] @ RICHTING
            doel = wm[ander][:3, 3] - wm[eigen][:3, 3]
            hoek = np.degrees(np.arccos(np.clip(r @ doel / np.linalg.norm(doel), -1, 1)))
            if hoek > 0.5:
                fouten.append(f"{sc.nodes[eigen].naam} wijst {hoek:.1f}° naast de andere pen")
        for k in ("penafstand_rust", "penafstand_min", "penafstand_max"):
            if k not in nh.extras or k not in ns.extras:
                fouten.append(f"{k} ontbreekt")
        if "penafstand_rust" in nh.extras and abs(nh.extras["penafstand_rust"] - d) > TOL_PEN:
            fouten.append(f"penafstand_rust {nh.extras['penafstand_rust']} ≠ gemeten {d:.4f}")
        # slag binnen min..max over het hele bereik van de bedienende joint (manifest)
        mc = next((x for x in man.get("cilinders", []) if x.get("huis") == nh.naam), None)
        stuur = next((jj for jj in joints if mc and jj["naam"] == mc["joint"]), None)
        if stuur and "penafstand_min" in nh.extras:
            afst = []
            for a in np.linspace(stuur["min"], stuur["max"], 25):
                wmx, _ = wereld_met_cilinders(sc, [], stand(stuur, a), kop)
                afst.append(float(np.linalg.norm(wmx[s][:3, 3] - wmx[h][:3, 3])))
            if min(afst) < nh.extras["penafstand_min"] - TOL_PEN or max(afst) > nh.extras["penafstand_max"] + TOL_PEN:
                fouten.append(f"penafstand {min(afst):.3f}..{max(afst):.3f} over {stuur['naam']} buiten "
                              f"{nh.extras['penafstand_min']}..{nh.extras['penafstand_max']}")
        naam = nh.naam[:-5]
        (R.fout if fouten else R.ok)("cilinders", f"{naam}: " + ("; ".join(fouten) if fouten else
                                     f"richt op elkaars pen, penafstand {d:.3f} m"
                                     + (f", slag via {stuur['naam']} binnen min..max" if stuur else "")))

    # ---------------- joint-test ----------------
    joint_rijen = []
    mesh_nodes = [n.index for n in sc.nodes if n.mesh is not None]
    zelfde_cil = {tuple(sorted(p)) for p in cilinders}

    # losse meshdelen (lokaal), zodat de indringdiepte per (bijna) convex deel wordt bepaald;
    # FCL geeft voor één grote niet-convexe mesh onbetrouwbare dieptes
    delen_lokaal = {}
    for i in mesh_nodes:
        v = np.vstack([p.posities for p in sc.meshes[sc.nodes[i].mesh]])
        f_, off = [], 0
        for p in sc.meshes[sc.nodes[i].mesh]:
            f_.append(p.indices + off)
            off += len(p.posities)
        delen_lokaal[i] = trimesh.Trimesh(v, np.vstack(f_)).split(only_watertight=False)

    def botsingen(wmx):
        """{((a, ka), (b, kb)): indringdiepte} per paar losse meshdelen van verschillende nodes."""
        cm = trimesh.collision.CollisionManager()
        for i, stukken in delen_lokaal.items():
            for k, m in enumerate(stukken):
                cm.add_object(f"{i}:{k}", m, transform=wmx[i])
        _, _, data = cm.in_collision_internal(return_names=True, return_data=True)
        uit = {}
        for d in data:
            a, b = sorted(tuple(int(v) for v in x.split(":")) for x in d.names)
            if a[0] != b[0]:
                uit[(a, b)] = max(uit.get((a, b), 0.0), float(d.depth))
        return uit

    try:
        rust = botsingen(wm)
        fcl_ok = True
    except Exception as e:  # python-fcl ontbreekt
        R.warn("joint-test", f"botsingstest niet beschikbaar ({e}); alleen standen berekend")
        rust, fcl_ok = {}, False
    if rust:
        paren = {(a[0], b[0]) for a, b in rust}
        R.ok("joint-test", f"{len(paren)} node-paren raken elkaar in ruststand (pennen, bokken, cilinders); "
                           f"doorsnijding = nieuw contact > {TOL_DOORSNIJDING * 100:.0f} cm of bestaand contact "
                           f"> {TOL_BESTAAND * 100:.0f} cm dieper")

    links = np.cross([0, 1, 0], np.array(ASSEN_VOOR[CONV["doel_voorkant"]]))  # links = up × voor
    for j in joints:
        n = sc.nodes[j["index"]]
        if n.naam in volgt:
            R.ok("joint-test", f"{n.naam}: volgt {volgt[n.naam]['joint']} (factor {volgt[n.naam].get('factor', 1)}); "
                               "getest samen met die joint")
            continue
        bewegend = set(sc.afstammelingen(n.index))
        rust_pts = scene_punten(sc, wm, list(bewegend))
        rust_c = rust_pts.mean(0) if len(rust_pts) else wm[n.index][:3, 3]
        standen = {}
        for label in ("min", "max"):
            wmx, extra = wereld_met_cilinders(sc, cilinders, stand(j, j[label]), kop)
            veranderd = {i for i in range(len(sc.nodes)) if not np.allclose(wmx[i], wm[i], atol=1e-6)}
            bp = scene_punten(sc, wmx, list(bewegend))
            c = bp.mean(0) if len(bp) else wmx[n.index][:3, 3]
            nieuw = []
            if fcl_ok:
                per_paar = {}
                for ((a, ka), (b, kb)), diepte in botsingen(wmx).items():
                    drempel = (rust[((a, ka), (b, kb))] + TOL_BESTAAND if ((a, ka), (b, kb)) in rust
                               else TOL_DOORSNIJDING)
                    if diepte <= drempel:
                        continue
                    if tuple(sorted((a, b))) in zelfde_cil or (a in bewegend and b in bewegend):
                        continue
                    if a not in veranderd and b not in veranderd:
                        continue
                    per_paar[(a, b)] = max(per_paar.get((a, b), 0.0), diepte)
                nieuw += [f"{sc.nodes[a].naam}/{sc.nodes[b].naam} ({d_:.3f} m)" for (a, b), d_ in sorted(per_paar.items())]
            if len(bp) and bp[:, 1].min() < -TOL_GROND and zonder_voorvoegsel(j["naam"]) not in ("Giek", "Stick", "Bak"):
                nieuw.append(f"onder maaiveld (Y={bp[:, 1].min():.2f})")
            standen[label] = {"hoek": j[label], "zwaartepunt": c.round(3).tolist(), "doorsnijding": nieuw}
        kp = scene_punten(sc, sc.wereld_matrices({n.index: joint_matrix(j, 5.0)}), list(bewegend))
        dc = (kp.mean(0) if len(kp) else rust_c) - rust_c
        if n.naam == "Bak":
            teken = j["positief"] or "n.v.t."            # openen is geen hoogte-effect van het zwaartepunt
        elif "straal_band" in n.extras or "straal_steekcirkel" in n.extras:
            teken = "n.v.t. (wiel)"
        elif j["as"] == "X":
            teken = "heffen" if dc[1] > 1e-4 else ("zakken" if dc[1] < -1e-4 else "n.v.t. (wiel)")
        elif j["as"] == "Z":
            teken = "links" if dc @ links > 1e-4 else ("rechts" if dc @ links < -1e-4 else "n.v.t.")
        else:
            teken = "n.v.t."
        joint_rijen.append({"naam": n.naam, "as": j["as"], "as_gltf": j["as_gltf"], "positief": teken,
                            "betekenis": j["positief"], **standen})
        doorsn = standen["min"]["doorsnijding"] + standen["max"]["doorsnijding"]
        (R.warn if doorsn else R.ok)(
            "joint-test", f"{n.naam}: {'doorsnijding ' + str(doorsn) if doorsn else 'geen nieuwe doorsnijding'} "
                          f"op min {j['min']} / max {j['max']}; positief = {teken}")
        if teken in ("zakken", "rechts") and n.naam not in ("Bak",):
            R.warn("joint-test", f"{n.naam}: positieve hoek = {teken}; afspraak is heffen/uit/links")
    if joints:
        R.ok("joint-test", "geen vervorming mogelijk: alle nodes rigide (schaal 1, geen skinning)"
             if not schaal and not sc.gltf.skins else "controleer schaal/skinning")

    wb = werkbereik(sc, joints, man, R)

    # ---------------- testanimatie ----------------
    if animatie and joints:
        stappen, t0 = [], 0.0
        for j in joints:
            if j["type"] != "rotatie" or j["naam"] in volgt:
                continue
            for ta, ha, tb, hb in ((0, 0, 1, j["min"]), (1, j["min"], 3, j["max"]), (3, j["max"], 4, 0)):
                for k in range(7):
                    f = k / 6
                    stappen.append((t0 + ta + f * (tb - ta), j, ha + f * (hb - ha)))
            t0 += 4.0
        sporen = {}
        for t, j, h in stappen:
            st = stand(j, h)
            _, extra = wereld_met_cilinders(sc, cilinders, st, kop)
            for i in sorted(set(st) | cil_nodes | kop_nodes):
                r = sc.nodes[i].matrix[:3, :3] @ extra.get(i, np.eye(4))[:3, :3]
                sporen.setdefault(i, []).append((t, matrix_naar_quat(r)))
        kanalen = []
        for i, sp in sporen.items():
            tijden, quats = [], []
            for t, q in sp:
                if tijden and abs(t - tijden[-1]) < 1e-9:
                    quats[-1] = q
                else:
                    tijden.append(t)
                    quats.append(q)
            kanalen.append({"node": i, "tijden": tijden, "quats": quats})
        voeg_animaties_toe(sc, [{"naam": "Jointtest", "kanalen": kanalen}],
                           os.path.join(map_, "test_animatie.glb"))
        R.ok("animatie", f"test_animatie.glb: per joint 0 → min → max → 0, cilinders mee gericht"
                         f"{', bakkoppeling opgelost' if kop else ''} ({t0:.0f} s)")

    schrijf_rapport(map_, man, sc, wm, R, joint_rijen, tri_n, refs, afm, lo, hi, wb)
    with open(os.path.join(map_, "controle.json"), "w", encoding="utf-8") as f:
        json.dump({"code": code, "resultaten": R.items, "joints": joint_rijen, "werkbereik": wb,
                   "referenties": refs}, f, indent=2, ensure_ascii=False)
    print(f"[{code}] controle: {len(R.items)} punten, {len(R.fouten)} FOUT, "
          f"{sum(i['status'] == 'WAARSCHUWING' for i in R.items)} waarschuwing")
    for i in R.items:
        if i["status"] != "OK":
            print(f"  {i['status']:12s} {i['groep']}: {i['tekst']}")
    return 1 if R.fouten else 0


def boom(sc, i, wm, diepte=0):
    n = sc.nodes[i]
    p = wm[i][:3, 3].round(3).tolist()
    j = ""
    if SL["as_tekst"] in n.extras and SL["max"] in n.extras:
        j = f"  [{n.extras.get(SL['min'], 0)}..{n.extras[SL['max']]}°] {n.extras[SL['as_tekst']]}"
    elif "penafstand_rust" in n.extras:
        j = f"  [pen {n.extras['penafstand_min']}..{n.extras['penafstand_max']} m]"
    regels = [f"{'  ' * diepte}- {n.naam}  (oorsprong {p}){j}"]
    for k in n.kinderen:
        regels += boom(sc, k, wm, diepte + 1)
    return regels


def schrijf_rapport(map_, man, sc, wm, R, joints, tri_n, refs, afm, lo, hi, wb):
    r = [f"# Rapport {man.get('code', '')}: {man.get('merk', '')} {man.get('typeaanduiding', '')}", "",
         f"- Machinetype: {man.get('machinetype')}  |  detailniveau: {man.get('detailniveau')}  |  "
         f"rig: {man.get('rig') or '—'}  |  categorie: {man.get('categorie')}",
         f"- Bron: {man.get('bron_url') or '—'}  |  licentie: {man.get('licentie')}  |  formaat: {man.get('bronformaat')}",
         f"- Afmetingen (m): lengte {afm[2]:.3f}, breedte {afm[0]:.3f}, hoogte {afm[1]:.3f}  "
         f"(bbox {lo.round(3).tolist()} … {hi.round(3).tolist()}, ruststand)",
         f"- Polygonen: {sum(tri_n.values())}",
         f"- Bibliotheek-CSV: `{man.get('bibliotheek_csv')}`", "",
         ""]
    plaatjes = sorted(f for f in os.listdir(map_) if f.startswith("voorbeeld_") and f.endswith(".png"))
    if plaatjes:
        r += ["## Voorbeeld", ""] + [f"![{p[10:-4]}]({p})" for p in plaatjes] + [""]
    r += ["## Controle", "", "| Groep | Status | Bevinding |", "|---|---|---|"]
    r += [f"| {i['groep']} | {i['status']} | {i['tekst'].replace('|', '/')} |" for i in R.items]
    if wb:
        r += ["", "## Werkbereik t.o.v. datasheet", "", "| Grootheid | Model (m) | Datasheet (m) | Afwijking |",
              "|---|---|---|---|"]
        r += [f"| {w['grootheid']} | {w['model']} | {w['datasheet']} | {w['afwijking']:+.1%} |" for w in wb]
    r += ["", "## Hiërarchie", "", "```"]
    for w in sc.wortels:
        r += boom(sc, w, wm)
    r += ["```", "", "## Joints: begin- en eindstand", ""]
    if joints:
        r += ["| Joint | As | As glTF | Positief (gemeten) | Min | Zwaartepunt bij min | Max | Zwaartepunt bij max | Doorsnijding |",
              "|---|---|---|---|---|---|---|---|---|"]
        for j in joints:
            d = (j["min"]["doorsnijding"] + j["max"]["doorsnijding"]) or ["geen"]
            r.append(f"| {j['naam']} | {j['as']} | {j['as_gltf']} | {j['positief']} | {j['min']['hoek']}° | "
                     f"{j['min']['zwaartepunt']} | {j['max']['hoek']}° | {j['max']['zwaartepunt']} | {', '.join(d)} |")
        r += ["", "Testanimatie: `test_animatie.glb` (animatie `Jointtest`, cilinders mee gericht)."]
    else:
        r.append("Geen joints (detailniveau eenvoudig).")
    ds = man.get("datasheet")
    if ds:
        r += ["", "## Datasheetwaarden", "", "| Waarde | | Bron |", "|---|---|---|"]
        for groep in ("afmetingen", "werkbereik", "overig"):
            for k, v in ds.get(groep, {}).items():
                r.append(f"| {k} | {v} | {ds.get('bronnen', {}).get(k, '')} |")
    r += ["", "## Polygonen per node", "", "| Node | Driehoeken |", "|---|---|"]
    r += [f"| {k} | {v} |" for k, v in tri_n.items()]
    if refs:
        r += ["", "## Vergelijking met referentie", "", "| Bestand | Afm X×Y×Z (m) | Min | Max | Voorkant | Wortel |",
              "|---|---|---|---|---|---|"]
        for x in refs:
            r.append(f"| {x['bestand']} | {x['afm']} | {x['min']} | {x['max']} | {x['voorkant']} ({x['via']}) | {x['wortels']} |")
    for kop, sleutel in (("Afwijkingen van de conventie", "afwijkingen_van_conventie"),
                         ("Aannames", "aannames"), ("Openstaande punten", "openstaande_punten")):
        r += ["", f"## {kop}", ""]
        r += [f"- {x}" for x in man.get(sleutel, [])] or ["- geen"]
    with open(os.path.join(map_, "rapport.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(r) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map")
    ap.add_argument("--referentie", nargs="*", default=[])
    ap.add_argument("--doel-polygonen", type=int, default=50000)
    ap.add_argument("--geen-animatie", action="store_true")
    a = ap.parse_args()
    return controleer(a.map, a.referentie, a.doel_polygonen, not a.geen_animatie)


if __name__ == "__main__":
    sys.exit(main())
