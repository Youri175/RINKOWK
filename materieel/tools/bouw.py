"""Stap 2-5 en 8: bronmodel inspecteren, converteren, opschonen, benoemen en
als 4DVis-GLB met manifest wegschrijven.

Gebruik:
    python3 tools/bouw.py machines/<code>.json [--uit lever]

De machineconfig beschrijft bron, eenheid, oriëntatie, onderdelen, draaipunten
en joints; zie machines/README.md. Na het bouwen draait controleer.py
automatisch (uit te zetten met --geen-controle).
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gltf_io import (Deel, Materiaal, matrix_naar_quat, rotatie_matrix,  # noqa: E402
                     rotatie_van_naar, schrijf_glb, transformeer)

HIER = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HIER, "conventies.json"), encoding="utf-8") as _f:
    CONV = json.load(_f)

EENHEDEN = {"m": 1.0, "dm": 0.1, "cm": 0.01, "mm": 0.001, "inch": 0.0254, "ft": 0.3048}
ASSEN = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0), "-Y": (0, -1, 0),
         "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


def asrotatie(bron_up, bron_voor, doel_up="+Y", doel_voor="+Z"):
    """Rotatiematrix die bron-up/voorkant op doel-up/voorkant legt. `bron_voor` mag ook een
    vector zijn (bv. de giekrichting van een gezwenkt model); die wordt loodrecht op up gemaakt."""
    bu = np.array(ASSEN[bron_up], float)
    bv = np.array(ASSEN[bron_voor] if isinstance(bron_voor, str) else bron_voor, float)
    bv = bv - (bv @ bu) * bu
    bv /= np.linalg.norm(bv)
    du, dv = np.array(ASSEN[doel_up], float), np.array(ASSEN[doel_voor], float)
    if abs(bu @ bv) > 1e-9 or abs(du @ dv) > 1e-9:
        raise ValueError("up en voorkant moeten loodrecht op elkaar staan")
    b = np.stack([bu, bv, np.cross(bu, bv)], axis=1)
    d = np.stack([du, dv, np.cross(du, dv)], axis=1)
    return d @ b.T


class Log:
    def __init__(self):
        self.regels, self.afwijkingen, self.aannames, self.open = [], [], [], []

    def info(self, s):
        self.regels.append(s)
        print("  " + s)


# --------------------------------------------------------------------------
# Bron laden en inspecteren (stap 2)
# --------------------------------------------------------------------------
def laad_bron(pad, log):
    ext = os.path.splitext(pad)[1].lower()
    if ext in (".fbx", ".dae", ".3ds"):
        if not shutil.which("assimp"):
            raise RuntimeError(
                f"{ext} vraagt Assimp (apt install assimp-utils) of Blender: "
                "`blender -b -P tools/blender_naar_glb.py -- bron.fbx bron.glb`")
        tmp = os.path.join(tempfile.mkdtemp(prefix="bron_"), "bron.glb")
        r = subprocess.run(["assimp", "export", pad, tmp, "-fglb2"], capture_output=True, text=True)
        if r.returncode != 0 or not os.path.exists(tmp):
            raise RuntimeError(f"Assimp kon {os.path.basename(pad)} niet omzetten: {r.stderr[-300:]}")
        log.info(f"{ext} via Assimp omgezet naar glTF (Y-up; FBX-eenheid blijft in de nodeschaal)")
        log.afwijkingen.append(f"bron {ext.upper()[1:]} via Assimp naar glTF omgezet; punten en lijnen vervallen")
        pad = tmp
    elif ext == ".blend":
        raise RuntimeError("`blender -b -P tools/blender_naar_glb.py -- bron.blend bron.glb` en gebruik het GLB")
    sc = trimesh.load(pad, force="scene", process=False)
    parents = sc.graph.transforms.parents

    def pad_van(node):
        delen = [node]
        while node in parents:
            node = parents[node]
            delen.append(node)
        return "/".join(reversed(delen))

    items = []
    for node in sc.graph.nodes_geometry:
        T, gnaam = sc.graph[node]
        g = sc.geometry[gnaam]
        if not isinstance(g, trimesh.Trimesh) or len(g.faces) == 0:
            continue
        items.append({"node": node, "pad": pad_van(node) + "/" + str(gnaam),
                      "T": np.array(T, float), "mesh": g})
    log.info(f"bron: {len(items)} mesh-instanties, "
             f"{sum(len(i['mesh'].faces) for i in items)} driehoeken")
    return items


def bepaal_eenheid(cfg, items, log):
    gegeven = cfg.get("bron_eenheid", "auto")
    pts = np.vstack([trimesh.transform_points(i["mesh"].bounds, i["T"]) for i in items])
    afm = pts.max(0) - pts.min(0)
    grootste = float(afm.max())
    if gegeven != "auto":
        f = EENHEDEN[gegeven]
    else:
        # bouwmachines: 2-40 m. Groter dan 200 eenheden => vrijwel zeker mm.
        f = 0.001 if grootste > 200 else (0.01 if grootste > 60 else 1.0)
        log.aannames.append(f"bron-eenheid automatisch bepaald: grootste maat {grootste:.1f} "
                            f"eenheden -> factor {f}")
    if f != 1.0:
        log.afwijkingen.append(f"bron in {gegeven if gegeven != 'auto' else 'auto-' + str(f)}: "
                               f"geschaald met factor {f} naar meter")
    log.info(f"bron-afmetingen {np.round(afm, 3).tolist()} eenheden -> "
             f"{np.round(afm * f, 3).tolist()} m")
    if grootste * f > 60 or grootste * f < 1:
        log.open.append(f"grootste maat {grootste * f:.2f} m valt buiten 1-60 m: schaal controleren")
    return f


# --------------------------------------------------------------------------
# Materialen
# --------------------------------------------------------------------------
class Materialen:
    def __init__(self, code, texdir):
        self.lijst, self.sleutels, self.code, self.texdir = [], {}, code, texdir
        self.texture_bestanden = []

    def _voeg(self, sleutel, maker):
        if sleutel not in self.sleutels:
            self.sleutels[sleutel] = len(self.lijst)
            self.lijst.append(maker())
        return self.sleutels[sleutel]

    def kleur(self, rgba, metallic=0.0, roughness=0.8, naam=None):
        rgba = tuple(round(float(c), 3) for c in rgba)
        k = ("kleur", rgba, round(metallic, 2), round(roughness, 2))
        return self._voeg(k, lambda: Materiaal(naam or f"{self.code}_Mat{len(self.lijst):02d}",
                                               rgba, metallic, roughness))

    def texture(self, beeld, rgba, metallic, roughness, naam=None):
        h = hashlib.sha1(beeld.tobytes()).hexdigest()[:10]
        k = ("tex", h, tuple(round(float(c), 3) for c in rgba))

        def maak():
            bestand = f"textures/{self.code}_{h}.png"
            os.makedirs(self.texdir, exist_ok=True)
            beeld.save(os.path.join(self.texdir, f"{self.code}_{h}.png"))
            self.texture_bestanden.append(bestand)
            return Materiaal(naam or f"{self.code}_Mat{len(self.lijst):02d}", tuple(rgba),
                             metallic, roughness, bestand)
        return self._voeg(k, maker=maak)


def splits_op_materiaal(mesh, mats, vervang=()):
    """Geeft [(trimesh, materiaal_idx, heeft_uv)]. `vervang`: [(regex, spec)] op de bronmateriaalnaam."""
    vis = mesh.visual
    if isinstance(vis, trimesh.visual.TextureVisuals):
        m = vis.material
        for pat, spec in vervang:
            if m is not None and pat.search(str(getattr(m, "name", "") or "")):
                return [(mesh, mats.kleur(spec["kleur"], spec.get("metallic", 0.0), spec.get("roughness", 0.8),
                                          spec.get("naam")), False)]
        rgba, met, rou, beeld = (0.8, 0.8, 0.8, 1.0), 0.0, 0.8, None
        if isinstance(m, trimesh.visual.material.PBRMaterial):
            if m.baseColorFactor is not None:
                rgba = np.asarray(m.baseColorFactor, float) / (255.0 if np.max(m.baseColorFactor) > 1 else 1)
            met = float(m.metallicFactor if m.metallicFactor is not None else 0.0)
            rou = float(m.roughnessFactor if m.roughnessFactor is not None else 0.8)
            beeld = m.baseColorTexture
        elif m is not None:
            d = getattr(m, "diffuse", None)
            if d is not None:
                rgba = np.asarray(d, float) / 255.0
            beeld = getattr(m, "image", None)
        naam = getattr(m, "name", None)
        if beeld is not None and vis.uv is not None:
            return [(mesh, mats.texture(beeld.convert("RGBA"), rgba, met, rou, naam), True)]
        return [(mesh, mats.kleur(rgba, met, rou, naam), False)]
    # kleurvisuals: per unieke facekleur een deel
    try:
        fc = np.asarray(vis.face_colors)
    except Exception:
        fc = None
    if fc is None or len(fc) != len(mesh.faces):
        return [(mesh, mats.kleur((0.8, 0.8, 0.8, 1.0)), False)]
    uitk = []
    uniek, inv = np.unique(fc, axis=0, return_inverse=True)
    for i, c in enumerate(uniek):
        sub = mesh.submesh([np.where(inv.reshape(-1) == i)[0]], append=True)
        uitk.append((sub, mats.kleur(np.asarray(c) / 255.0), False))
    return uitk


# --------------------------------------------------------------------------
# Opschonen (stap 4)
# --------------------------------------------------------------------------
def schoon_mesh(m, heeft_uv, cijfers=None):
    """Ontaarde en dubbele vlakken weg (ook de achterkant van dubbelzijdige SketchUp-vlakken),
    hoekpunten samenvoegen (optioneel binnen 10^-cijfers m) en de winding consistent maken."""
    m = m.copy()
    m.merge_vertices(merge_tex=heeft_uv, merge_norm=True, digits_vertex=cijfers)
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    if not heeft_uv:
        if m.is_watertight:
            trimesh.repair.fix_normals(m)   # consistente winding, naar buiten
        elif cijfers is not None:
            trimesh.repair.fix_winding(m)   # open schalen: in elk geval onderling consistent
    return m


def splits_hoekpunten(m, cijfers):
    """Losse stukken op gedeelde hoekpunten (binnen 10^-cijfers m). Robuuster dan trimesh.split
    bij dubbelzijdige vlakken, waar randen door vier vlakken gedeeld worden."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    _, inv = np.unique(np.round(m.vertices, cijfers), axis=0, return_inverse=True)
    f = inv.reshape(-1)[m.faces]
    e = np.vstack([f[:, [0, 1]], f[:, [1, 2]]])
    n = int(inv.max()) + 1
    k, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
    if k == 1:
        return [m]
    flab = lab[f[:, 0]]
    volg = np.argsort(flab, kind="stable")
    grenzen = np.flatnonzero(np.diff(flab[volg])) + 1
    return [m.submesh([idx], append=True) for idx in np.split(volg, grenzen)]


def is_intern(klein, groot):
    """Ligt `klein` volledig binnen het gesloten `groot`?"""
    if not groot.is_watertight:
        return False
    if np.any(klein.bounds[0] < groot.bounds[0]) or np.any(klein.bounds[1] > groot.bounds[1]):
        return False
    pts = klein.vertices
    if len(pts) > 300:
        pts = pts[np.random.default_rng(0).choice(len(pts), 300, replace=False)]
    return bool(np.all(groot.contains(pts)))


def cluster(pos, faces, cel):
    """Vertex-clustering: hoekpunten per rastercel samenvoegen, ontaarde/dubbele vlakken weg."""
    sleutel = np.floor(pos / cel).astype(np.int64)
    _, inv = np.unique(sleutel, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    n = inv.max() + 1
    nieuw = np.zeros((n, 3))
    np.add.at(nieuw, inv, pos)
    nieuw /= np.bincount(inv, minlength=n)[:, None]
    f = inv[faces]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    f = f[np.unique(np.sort(f, axis=1), axis=0, return_index=True)[1]]
    gebruikt = np.unique(f)
    remap = np.full(n, -1)
    remap[gebruikt] = np.arange(len(gebruikt))
    return nieuw[gebruikt], remap[f]


def decimeer(pos, faces, doel):
    """Quadric-decimatie; schiet die tekort (veel kleine losse stukjes, zoals rupsschakels),
    dan vertex-clustering met de kleinste celgrootte die het doel haalt."""
    if len(faces) <= doel or doel < 4:
        return pos, faces
    import fast_simplification
    red = 1.0 - doel / len(faces)
    p, f = fast_simplification.simplify(pos.astype(np.float32), faces.astype(np.int64), red)
    p, f = np.asarray(p, float), np.asarray(f, np.int64)
    if len(f) <= doel * 1.15:
        return p, f
    lo, hi = 1e-3, float(np.ptp(pos, axis=0).max()) / 4
    beste = None
    for _ in range(18):                      # bisectie op de celgrootte
        cel = np.sqrt(lo * hi)
        pc, fc = cluster(p, f, cel)
        if len(fc) > doel:
            lo = cel
        else:
            hi, beste = cel, (pc, fc)
    return beste if beste is not None else cluster(p, f, hi)


def vertex_normalen(pos, faces, hoek_deg=30.0):
    """Gladde normalen met harde randen boven `hoek_deg` (vertices worden daar gesplitst)."""
    m = trimesh.Trimesh(pos, faces, process=False)
    try:
        gl = trimesh.graph.smooth_shade(m, angle=np.radians(hoek_deg))
        v, f, n = np.asarray(gl.vertices), np.asarray(gl.faces), np.asarray(gl.vertex_normals)
    except Exception:
        v, f, n = pos, faces, np.asarray(m.vertex_normals)
    # bijna samenvallende tegengestelde vlakken (dubbelzijdige bron) middelen tot nul of tot een
    # omgekeerde normaal: die hoeken krijgen de eigen vlaknormaal
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    lengte = np.linalg.norm(fn, axis=1)
    fn = np.where(lengte[:, None] > 1e-12, fn / np.maximum(lengte, 1e-12)[:, None], [0, 1, 0])
    nl = np.linalg.norm(n, axis=1)
    slecht = (nl[f] < 1e-6) | (np.einsum("fkj,fj->fk", n[f], fn) < 0.2)
    if slecht.any():
        rijen, kol = np.nonzero(slecht)
        nieuw = len(v) + np.arange(len(rijen))
        v = np.vstack([v, v[f[rijen, kol]]])
        n = np.vstack([n, fn[rijen]])
        f = f.copy()
        f[rijen, kol] = nieuw
    nl = np.linalg.norm(n, axis=1)
    n = np.where(nl[:, None] > 1e-9, n / np.maximum(nl, 1e-12)[:, None], [0.0, 1.0, 0.0])  # losse hoekpunten
    return v, f, n


# --------------------------------------------------------------------------
# Draaipunten
# --------------------------------------------------------------------------
def draaipunt(spec, deel, bboxen, bron_naar_doel):
    if spec is None:
        return None
    if isinstance(spec, list):
        return np.array(spec, float)
    if "bron" in spec:
        return bron_naar_doel(np.array([spec["bron"]], float))[0]
    if "bbox" in spec:
        ref = spec.get("van", deel)
        lo, hi = bboxen[ref]
        u = np.array(spec["bbox"], float)
        p = (lo + hi) / 2 + u * (hi - lo) / 2
        if "plus_m" in spec:
            p = p + np.array(spec["plus_m"], float)
        return p
    raise ValueError(f"onbekende draaipuntspecificatie voor {deel}: {spec}")


# --------------------------------------------------------------------------
# Hoofdroutine
# --------------------------------------------------------------------------
def bouw(cfg_pad, uitmap, controle=True):
    with open(cfg_pad, encoding="utf-8") as f:
        cfg = json.load(f)
    cfg_dir = os.path.dirname(os.path.abspath(cfg_pad))
    log = Log()

    mtype = cfg["machinetype"]
    tcode = CONV["typecodes"][mtype]
    modelcode = cfg["modelcode"]
    code = cfg.get("code", f"{tcode}_{modelcode}")
    detail = cfg.get("detailniveau", "eenvoudig")
    rig = cfg.get("rig", CONV["standaard_rig"].get(mtype, "") if detail == "volledig" else "")
    prefix = f"{tcode}_{modelcode}_"
    doelmap = os.path.join(uitmap, code)
    os.makedirs(doelmap, exist_ok=True)
    print(f"[{code}] {cfg.get('merk', '')} {cfg.get('typeaanduiding', '')} ({mtype}, {detail})")

    lic = cfg.get("bron", {}).get("licentie", "")
    if not lic or lic.lower() in ("onbekend", "onduidelijk"):
        raise RuntimeError("licentie onduidelijk: machine overgeslagen (stap 1)")

    bronpad = os.path.join(cfg_dir, cfg["bron"]["bestand"])
    items = laad_bron(bronpad, log)
    f_eenh = bepaal_eenheid(cfg, items, log)
    R = asrotatie(cfg.get("bron_up", "+Y"), cfg.get("bron_voorkant", "+Z"),
                  "+Y", CONV["doel_voorkant"])
    if cfg.get("bron_up", "+Y") != "+Y" or cfg.get("bron_voorkant", "+Z") != CONV["doel_voorkant"]:
        log.afwijkingen.append(f"bron-oriëntatie up {cfg.get('bron_up', '+Y')}, voorkant "
                               f"{cfg.get('bron_voorkant', '+Z')}: gedraaid naar Y-up, voorkant "
                               f"{CONV['doel_voorkant']}")

    # onderdelen
    onderdelen = cfg.get("onderdelen") or [{"naam": prefix + "Machine", "bron": [".*"]}]
    if detail == "eenvoudig":
        for o in onderdelen:
            o.pop("joint", None)
    namen = [o["naam"] for o in onderdelen]
    if len(set(namen)) != len(namen):
        raise RuntimeError("onderdeelnamen niet uniek")
    standaard = cfg.get("standaard_onderdeel", namen[0])
    verwijder = [re.compile(p, re.I) for p in cfg.get("verwijder", [])]
    patronen = [(o["naam"], [re.compile(p, re.I) for p in o.get("bron", [])]) for o in onderdelen]

    # bakken: transformatie naar meter / Y-up (bevriezen)
    def naar_doel(p):
        return (np.asarray(p, float) * f_eenh) @ R.T

    def naar_bron(p):
        return (np.asarray(p, float) @ R) / f_eenh

    def regels(o):
        """Ruimtelijke toewijzing in bron-coördinaten: kaders (regio) en zones langs een lijn."""
        uit = []
        for k in o.get("regio", []):
            uit.append(("kader", np.array(k[0], float), np.array(k[1], float)))
        lijnen = o.get("lijn", [])
        for k in [lijnen] if isinstance(lijnen, dict) else lijnen:
            uit.append(("lijn", np.array(k["a"], float), np.array(k["b"], float), float(k["straal"])))
        return uit

    def binnen(c, rgl):
        for r in rgl:
            if r[0] == "kader" and np.all(c >= np.minimum(r[1], r[2])) and np.all(c <= np.maximum(r[1], r[2])):
                return True
            if r[0] == "lijn":
                ab = r[2] - r[1]
                t = np.clip((c - r[1]) @ ab / (ab @ ab), 0, 1)
                if np.linalg.norm(c - (r[1] + t * ab)) <= r[3]:
                    return True
        return False

    ruimte = {o["naam"]: regels(o) for o in onderdelen}
    behoud_regio = regels({"regio": cfg["behoud_regio"]}) if cfg.get("behoud_regio") else None

    tol = cfg.get("hoekpunt_tolerantie_m")          # bv. 1e-4 bij afrondingsruis in de bron
    cijfers = int(round(-np.log10(tol))) if tol else 6
    toewijzing = {n: [] for n in namen}
    weg, dubbel, hashes, buiten = [], 0, set(), 0
    for it in items:
        if any(p.search(it["pad"]) for p in verwijder):
            weg.append(it["pad"])
            continue
        m = it["mesh"].copy()
        m.apply_transform(it["T"])
        m.vertices = naar_doel(m.vertices)
        if np.linalg.det(it["T"][:3, :3]) < 0:
            m.invert()                       # gespiegelde instantie: winding herstellen
        stukken = [m]
        if cfg.get("componenten"):           # bv. SketchUp: objecten per materiaal, niet per onderdeel
            stukken = splits_hoekpunten(m, cijfers)
        for st in stukken:
            h = hashlib.sha1(np.round(np.sort(st.vertices, axis=0), 3).tobytes()).hexdigest()
            if h in hashes:
                dubbel += 1
                continue
            hashes.add(h)
            c_bron = naar_bron(st.bounds.mean(0))
            if behoud_regio is not None and not binnen(c_bron, behoud_regio):
                buiten += 1
                continue
            doel = standaard
            for naam, pats in patronen:
                rgl = ruimte[naam]
                if not pats and not rgl:
                    continue
                if (not pats or any(p.search(it["pad"]) for p in pats)) and (not rgl or binnen(c_bron, rgl)):
                    doel = naam
                    break
            toewijzing[doel].append(st)
    log.info(f"verwijderd via patroon: {len(weg)}, dubbele objecten: {dubbel}, buiten behoud_regio: {buiten}")
    for n in namen:
        log.info(f"  {n}: {len(toewijzing[n])} stukken, {sum(len(x.faces) for x in toewijzing[n])} driehoeken")

    # voor-draaiing: bv. een gezwenkte bovenwagen terug in de ruststand (om een as door een bron-punt)
    for vd in cfg.get("voor_draaiing", []):
        if "verschuiving_m" in vd:           # bv. delen die onder het maaiveld van de rupsen liggen
            T = np.eye(4)
            T[:3, 3] = vd["verschuiving_m"]
            for n in vd["onderdelen"]:
                for st in toewijzing[n]:
                    st.apply_transform(T)
            log.afwijkingen.append(f"{', '.join(vd['onderdelen'])} {vd['verschuiving_m']} m verschoven: "
                                   f"{vd.get('reden', '')}".rstrip(": "))
            continue
        punt = naar_doel(np.array([vd["punt_bron"]], float))[0]
        as_d = np.array(ASSEN[vd["as_bron"]], float) @ R.T
        T = rotatie_matrix(as_d, vd["hoek_deg"], punt)
        for n in vd["onderdelen"]:
            for st in toewijzing[n]:
                st.apply_transform(T)
        log.afwijkingen.append(f"{', '.join(vd['onderdelen'])} {vd['hoek_deg']}° gedraaid om bron-as "
                               f"{vd['as_bron']} door {vd['punt_bron']}: {vd.get('reden', '')}".rstrip(": "))

    if cfg.get("verwijder_intern", False):
        alle = [(n, i, m) for n, ms in toewijzing.items() for i, m in enumerate(ms)]
        intern = set()
        for n1, i1, m1 in alle:
            for n2, i2, m2 in alle:
                if (n1, i1) != (n2, i2) and (n2, i2) not in intern and is_intern(m1, m2):
                    intern.add((n1, i1))
                    break
        for n, i in sorted(intern, reverse=True):
            del toewijzing[n][i]
        log.info(f"interne geometrie verwijderd: {len(intern)}")

    # opschonen en per materiaal groeperen
    mats = Materialen(code, os.path.join(doelmap, "textures"))
    vervang = [(re.compile(v["patroon"], re.I), v) for v in cfg.get("materialen", [])]
    prims = {n: [] for n in namen}
    for n, ms in toewijzing.items():
        groepen = {}                         # per (materiaal, uv) samenvoegen: minder primitieven
        for m in ms:
            for sub, mi, uv in splits_op_materiaal(m, mats, vervang):
                groepen.setdefault((mi, uv), []).append(sub)
        for (mi, uv), subs in groepen.items():
            sub = schoon_mesh(trimesh.util.concatenate(subs) if len(subs) > 1 else subs[0], uv,
                              cijfers if tol else None)
            if len(sub.faces):
                prims[n].append([np.asarray(sub.vertices, float), np.asarray(sub.faces),
                                 np.asarray(sub.visual.uv, float) if uv else None, mi, sub])
    leeg = [n for n in namen if not prims[n]]
    if leeg:
        log.open.append(f"onderdelen zonder geometrie: {leeg}")

    # nulpunt: midden onderwagen op de grond
    nul_deel = cfg.get("nulpunt_onderdeel", namen[0])
    alle_pts = np.vstack([p[0] for n in namen for p in prims[n]])
    nul_pts = np.vstack([p[0] for p in prims[nul_deel]]) if prims[nul_deel] else alle_pts
    lo, hi = nul_pts.min(0), nul_pts.max(0)
    nulpunt = np.array([(lo[0] + hi[0]) / 2, lo[1], (lo[2] + hi[2]) / 2])
    if "nulpunt_m" in cfg:                  # bv. knikas van een knikdumper
        nulpunt = np.array(cfg["nulpunt_m"], float)
    if "nulpunt_bron" in cfg:               # bv. hart draaikrans op de grond, in bron-coördinaten
        nulpunt = naar_doel(np.array([cfg["nulpunt_bron"]], float))[0]
    if np.linalg.norm(nulpunt) > 0.01:
        log.afwijkingen.append(f"midden onderwagen op de grond lag in de bron op "
                               f"{np.round(nulpunt, 3).tolist()} m (na eenheid/as-conversie): naar 0 verschoven")
    for n in namen:
        for p in prims[n]:
            p[0] = p[0] - nulpunt

    def bron_naar_doel(p):
        return naar_doel(p) - nulpunt

    bboxen = {n: (np.vstack([p[0] for p in prims[n]]).min(0), np.vstack([p[0] for p in prims[n]]).max(0))
              for n in namen if prims[n]}

    # decimeren (stap 4)
    doel_poly = int(cfg.get("doel_polygonen", 50000))
    behoud = set(cfg.get("behoud", []))
    begin = sum(len(p[1]) for n in namen for p in prims[n])
    for n, budget in cfg.get("polygonen_per_onderdeel", {}).items():
        tot_n = sum(len(p[1]) for p in prims[n])
        if tot_n > budget:
            r = budget / tot_n
            for p in prims[n]:
                if p[2] is None:
                    p[0], p[1] = decimeer(p[0], p[1], max(12, int(len(p[1]) * r)))
        behoud.add(n)                        # niet nog eens verlagen
    if cfg.get("polygonen_per_onderdeel"):
        log.aannames.append("polygoonbudget per onderdeel: " + ", ".join(
            f"{k} {v}" for k, v in cfg["polygonen_per_onderdeel"].items()))
    totaal = sum(len(p[1]) for n in namen for p in prims[n])
    if totaal > doel_poly:
        b_tot = sum(len(p[1]) for n in namen if n in behoud for p in prims[n])
        r_tot = totaal - b_tot
        rest = doel_poly - b_tot
        if rest < 0.1 * doel_poly or r_tot == 0:
            ratio_b, ratio_r = doel_poly / totaal, doel_poly / totaal
            log.open.append("behouden onderdelen passen niet in het polygoonbudget: alles verlaagd")
        else:
            ratio_b, ratio_r = 1.0, rest / r_tot
        overgeslagen_uv = 0
        for n in namen:
            r = ratio_b if n in behoud else ratio_r
            for p in prims[n]:
                if r >= 1.0:
                    continue
                if p[2] is not None:
                    overgeslagen_uv += 1
                    continue
                p[0], p[1] = decimeer(p[0], p[1], max(12, int(len(p[1]) * r)))
        if overgeslagen_uv:
            log.open.append(f"{overgeslagen_uv} getextureerde primitieven niet gedecimeerd (UV's)")
        log.aannames.append("polygonen verlaagd met uniforme factor per niet-behouden onderdeel; "
                            "geen zichtbaarheidsanalyse")
    log.info(f"polygonen: {begin} -> {sum(len(p[1]) for n in namen for p in prims[n])} (doel {doel_poly})")

    # draaipunten en hiërarchie
    wortel = cfg.get("wortel", code)
    parent_van = {o["naam"]: o.get("parent") for o in onderdelen}
    dp = {}
    for o in onderdelen:
        p = draaipunt(o.get("draaipunt"), o["naam"], bboxen, bron_naar_doel)
        if p is None:
            p = np.zeros(3) if parent_van[o["naam"]] is None else None
        dp[o["naam"]] = p
    for n in namen:     # statische onderdelen zonder draaipunt: draaipunt van de parent
        if dp[n] is None:
            q = parent_van[n]
            while dp.get(q) is None and q is not None:
                q = parent_van[q]
            dp[n] = np.zeros(3) if q is None else dp[q]

    volgorde, gezien = [], set()

    def plaats(n):
        if n in gezien:
            return
        if parent_van[n] is not None:
            if parent_van[n] not in parent_van:
                raise RuntimeError(f"parent {parent_van[n]} van {n} bestaat niet")
            plaats(parent_van[n])
        gezien.add(n)
        volgorde.append(n)
    for n in namen:
        plaats(n)

    # wereldrotatie per onderdeel: identiteit (bevroren), behalve cilinderhelften die met
    # hun lokale richting (-Z) naar de pen van de andere helft wijzen (zoals de referentie)
    richting = np.array(CONV["cilinders"]["richting_lokaal"], float)
    Rw = {}
    for o in onderdelen:
        Rw[o["naam"]] = np.eye(3)
        if o.get("richt_naar") is not None:
            doelpunt = draaipunt(o["richt_naar"], o["naam"], bboxen, bron_naar_doel)
            Rw[o["naam"]] = rotatie_van_naar(richting, doelpunt - dp[o["naam"]])

    as_map = CONV["as_naar_gltf"]
    sl = CONV["extras_sleutels"]
    godot = {"X": "X", "Y": "-Z", "Z": "Y"}
    standaard_betekenis = {"X": "heffen", "Y": "naar links kantelen", "Z": "naar links"}
    cfg_joint = {o["naam"]: o["joint"] for o in onderdelen if o.get("joint") and detail == "volledig"}

    def voorouders(n):
        uit = []
        while parent_van.get(n) is not None:
            n = parent_van[n]
            uit.append(n)
        return uit

    def penafstand(huis, stang, joint, hoek):
        """Pen-tot-penafstand van een cilinder als `joint` op `hoek` staat."""
        j = cfg_joint[joint]
        as_g = np.array(j.get("as_gltf", as_map[j["as"]]), float)
        rot = rotatie_matrix(as_g, hoek, dp[joint])
        a, b = dp[huis], dp[stang]
        if joint in voorouders(stang) or joint == stang:
            b = transformeer([b], rot)[0]
        else:
            a = transformeer([a], rot)[0]
        return float(np.linalg.norm(b - a))

    delen = [Deel(wortel, None, np.zeros(3))]
    joints = []
    for n in volgorde:
        o = next(x for x in onderdelen if x["naam"] == n)
        par = parent_van[n]
        Rp = Rw[par] if par else np.eye(3)
        t = Rp.T @ (dp[n] - (dp[par] if par else np.zeros(3)))
        d = Deel(n, par or wortel, t)
        r_lok = Rp.T @ Rw[n]
        if not np.allclose(r_lok, np.eye(3), atol=1e-9):
            d.rotatie = matrix_naar_quat(r_lok)
        for pos, faces, uv, mi, _ in prims[n]:
            v_ = pos[faces]                     # vlakken zonder oppervlak (na decimeren) weg
            opp = np.linalg.norm(np.cross(v_[:, 1] - v_[:, 0], v_[:, 2] - v_[:, 0]), axis=1)
            faces = faces[opp > 1e-10]
            if not len(faces):
                continue
            lp = (pos - dp[n]) @ Rw[n]          # = Rw^T (p - draaipunt)
            if uv is None:
                lp, faces, nrm = vertex_normalen(lp, faces)
            else:
                nrm = np.asarray(trimesh.Trimesh(lp, faces, process=False).vertex_normals)
            d.primitieven.append((lp, nrm, uv, faces, mi))
        d.extras.update(o.get("extras", {}))
        if n in cfg_joint:
            j = cfg_joint[n]
            betekenis = j.get("positief", standaard_betekenis[j["as"]])
            d.extras[sl["as_tekst"]] = f"lokaal {j['as']} (Godot: {godot[j['as']]}); positieve hoek = {betekenis}"
            if j.get("sleutel_min", sl["min"]):           # bv. knikdumper: alleen max_knik_graden
                d.extras[j.get("sleutel_min", sl["min"])] = j["min"]
            d.extras[j.get("sleutel_max", sl["max"])] = j["max"]
            joints.append({"naam": n, "type": j.get("type", "rotatie"), "as": j["as"],
                           "min": j["min"], "max": j["max"], "bron": j.get("bron", "geschat"),
                           "positief": betekenis,
                           "as_gltf": j.get("as_gltf", as_map[j["as"]])})
        if o.get("wiel"):
            w = o["wiel"]
            lo_w, hi_w = bboxen[n]
            band = w.get("straal_band", "auto")
            if band == "auto":
                dims = hi_w - lo_w
                band = round(float(max(dims[1], dims[2]) / 2), 4)
                log.aannames.append(f"{n}: straal_band uit bounding box ({band} m)")
            d.extras["straal_band"] = band
            if w.get("straal_steekcirkel") is not None:
                d.extras["straal_steekcirkel"] = w["straal_steekcirkel"]
            else:
                log.open.append(f"{n}: straal_steekcirkel onbekend")
        delen.append(d)

    # cilinders: penafstand in rust en bij de grenzen van de joint die ze bedient
    cilinders = []
    for o in onderdelen:
        c = o.get("cilinder")
        if not c or not o["naam"].endswith("_Huis") or detail != "volledig":
            continue
        huis, stang = o["naam"], c["paar"] + "_Stang"
        joint = c["joint"]
        rust = float(np.linalg.norm(dp[stang] - dp[huis]))
        if "penafstand_min" in c:            # bv. via een koppeling: door de generator berekend
            grens = [c["penafstand_min"], c["penafstand_max"]]
        else:
            grens = [penafstand(huis, stang, joint, a)
                     for a in np.linspace(cfg_joint[joint]["min"], cfg_joint[joint]["max"], 41)]
        info = {"penafstand_rust": round(rust, 4), "penafstand_min": round(min(grens), 4),
                "penafstand_max": round(max(grens), 4)}
        for deel in delen:
            if deel.naam == huis:
                deel.extras.update({"functie": f"Cilinderhuis; lokale Y (Godot: -Z) wijst naar de pen van {stang}", **info})
            if deel.naam == stang:
                deel.extras.update({"functie": f"Zuigerstang; lokale Y (Godot: -Z) wijst naar de pen van {huis}", **info})
        cilinders.append({"naam": c["paar"], "huis": huis, "stang": stang, "joint": joint, **info})

    na = sum(len(p[3]) for d in delen for p in d.primitieven)
    delen[0].extras["driehoeken"] = int(na)
    glb = os.path.join(doelmap, "model.glb")
    if cfg.get("dubbelzijdig"):             # open schalen uit SketchUp/Revit: van twee kanten zichtbaar
        for m_ in mats.lijst:
            m_.dubbelzijdig = True
        log.aannames.append("materialen dubbelzijdig: de bron bestaat uit open, dubbelzijdige vlakken")
    schrijf_glb(glb, delen, mats.lijst)
    log.info(f"geschreven: {glb}")

    # manifest
    afm = alle_pts.max(0) - alle_pts.min(0)
    rignamen = set(CONV["rignamen"].get(rig, {}).get("verplicht", []))
    if rig:
        log.afwijkingen.append("rig-onderdelen zonder voorvoegsel [TYPE]_[MODEL]_ (exacte "
                               f"4DVis-namen voor rig {rig}): " +
                               ", ".join(n for n in namen if not n.startswith(prefix)))
    if detail == "volledig" and not rig:
        log.open.append(f"rig ontbreekt in 4DVis voor machinetype '{mtype}': keten en joints "
                        "zitten in het model en dit manifest")
    if joints:
        log.afwijkingen.append("spiegeling: glTF is rechtshandig, Unreal linkshandig; de importer "
                               "spiegelt één as. Niet in het model gecorrigeerd")
        log.aannames.append("joint-assen in Blender-notatie zoals extras.draaias in de referentie "
                            "(X = dwars, Z = verticaal); positief = heffen/uit/open (X) en naar links (Z)")
        if sl.get("min_nieuw"):
            log.open.append(f"sleutel '{sl['min']}' komt niet in de referentie voor (die kent alleen "
                            f"max_hoek_graden / max_knik_graden en cilinder-penafstanden); "
                            "naar hetzelfde patroon toegevoegd, bevestigen in 4DVis")
    if CONV.get("referentie_voorlopig"):
        log.open.append("referentiemodellen (Cat336E_LongReach.glb, A40G.glb) niet beschikbaar: "
                        f"voorkant {CONV['doel_voorkant']} en nulpunt niet geverifieerd")
    for j in joints:
        if j["bron"] == "geschat":
            log.aannames.append(f"joint {j['naam']}: limieten geschat ({j['min']}..{j['max']})")
    log.afwijkingen += cfg.get("afwijkingen", [])
    log.aannames += cfg.get("aannames", [])
    log.open += cfg.get("openstaande_punten", [])

    yaw = cfg.get("yaw_correctie_deg", 0)
    schaal = cfg.get("schaal", 1)
    categorie = cfg.get("categorie", CONV["categorieen"][mtype])
    manifest = {
        "code": code,
        "merk": cfg.get("merk", ""), "typeaanduiding": cfg.get("typeaanduiding", ""),
        "machinetype": mtype,
        "bron_url": cfg["bron"].get("url", ""), "licentie": lic,
        "bronformaat": cfg["bron"].get("formaat", os.path.splitext(bronpad)[1][1:].upper()),
        "eindformaat": "GLB",
        "eenheid": "m", "yaw_correctie_deg": yaw, "schaal": schaal,
        "afmetingen_m": {"lengte": round(float(afm[2]), 3), "breedte": round(float(afm[0]), 3),
                         "hoogte": round(float(afm[1]), 3)},
        "polygonen": int(na),
        "detailniveau": detail,
        "rig": rig, "categorie": categorie,
        "bibliotheek_csv": f"{code}/model.glb;{yaw};{schaal};{rig}",
        "joints": joints,
        "afwijkingen_van_conventie": log.afwijkingen,
        "aannames": log.aannames,
        "openstaande_punten": log.open,
        # aanvullend (niet in het basissjabloon), gebruikt door controleer.py
        "wortel": wortel,
        "nulpunt_onderdeel": nul_deel,
        "hierarchie": {wortel: None, **{n: parent_van[n] or wortel for n in volgorde}},
        "draaipunten_m": {n: [round(float(x), 4) for x in dp[n]] for n in volgorde},
        "cilinders": cilinders,
        "textures": mats.texture_bestanden,
        "gebouwd": datetime.date.today().isoformat(),
        **({"volgt": cfg["volgt"]} if cfg.get("volgt") else {}),
        **({"botsing_toegestaan": cfg["botsing_toegestaan"]} if cfg.get("botsing_toegestaan") else {}),
        **cfg.get("manifest_extra", {}),
    }
    if rignamen - set(namen) and rig:
        manifest["openstaande_punten"].append(
            f"verplichte rignamen ontbreken: {sorted(rignamen - set(namen))}")
    with open(os.path.join(doelmap, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    with open(os.path.join(doelmap, "bouwlog.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(log.regels) + "\n")

    # voorbeeldplaatjes (rust + standen uit de config); daarna de controle, die ze in het rapport opneemt
    standen = {"rust": {}, **cfg.get("voorbeeldstanden", {})} if detail == "volledig" else {"rust": {}}
    for naam, stand in standen.items():
        args = [sys.executable, os.path.join(HIER, "render.py"), doelmap,
                "--uit", os.path.join(doelmap, f"voorbeeld_{naam}.png")]
        args += [a for k, v in stand.items() for a in ("--stand", f"{k}={v}")]
        if subprocess.run(args, capture_output=True).returncode != 0:
            log.info(f"voorbeeld {naam} niet gemaakt (matplotlib?)")
    if controle:
        subprocess.run([sys.executable, os.path.join(HIER, "controleer.py"), doelmap], check=False)
    return doelmap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("configs", nargs="+", help="machineconfig(s); meerdere = batch")
    ap.add_argument("--uit", default=os.path.join(os.path.dirname(HIER), "lever"))
    ap.add_argument("--geen-controle", action="store_true")
    a = ap.parse_args()
    overgeslagen = []
    for c in a.configs:
        try:
            bouw(c, a.uit, controle=not a.geen_controle)
        except Exception as e:               # één fout stopt de batch niet
            print(f"OVERGESLAGEN {c}: {e}")
            overgeslagen.append({"config": c, "reden": str(e)})
    if overgeslagen:
        os.makedirs(a.uit, exist_ok=True)
        with open(os.path.join(a.uit, "overgeslagen.json"), "w", encoding="utf-8") as f:
            json.dump(overgeslagen, f, indent=2, ensure_ascii=False)
    return 1 if overgeslagen and len(overgeslagen) == len(a.configs) else 0


if __name__ == "__main__":
    sys.exit(main())
