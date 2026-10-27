"""Stap 7: controle van een 4DVis-materieelmodel.

Gebruik:
    python3 tools/controleer.py <map met model.glb + manifest.json> [--referentie a.glb b.glb]
                                [--doel-polygonen 50000] [--geen-animatie]

Schrijft in de map: rapport.md, controle.json en test_animatie.glb
(per joint 0 -> min -> max -> 0). Exitcode 1 bij een FOUT.
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
from gltf_io import as_hoek_naar_quat, lees_glb, rotatie_matrix, voeg_animaties_toe  # noqa: E402

HIER = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HIER, "conventies.json"), encoding="utf-8") as _f:
    CONV = json.load(_f)
SL = CONV["extras_sleutels"]
TOL_PIVOT = 0.01          # m: afwijking draaipunt t.o.v. manifest
TOL_GROND = 0.01          # m
TOL_DOORSNIJDING = 0.01   # m: extra indringing t.o.v. ruststand


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


def bbox(punten):
    return punten.min(0), punten.max(0)


def scene_punten(sc, wm, indices=None):
    tri = sc.wereld_driehoeken(indices if indices is not None else range(len(sc.nodes)), wm)
    return np.vstack([v for v, _ in tri.values()]) if tri else np.zeros((0, 3))


def voorkant_schatting(sc, wm, namen=("Giek", "Bak", "Stick", "Voorwagen")):
    """Horizontale richting (as + teken) van voorkant-onderdelen t.o.v. het nulpunt."""
    for n in namen:
        node = sc.node(n)
        if node is None:
            continue
        pts = scene_punten(sc, wm, sc.afstammelingen(node.index))
        if len(pts) == 0:
            continue
        c = pts.mean(0)
        h = np.array([c[0], 0, c[2]])
        if np.linalg.norm(h) < 0.05:
            continue
        i = int(np.argmax(np.abs(h)))
        return ("+" if h[i] > 0 else "-") + "XYZ"[i], n
    return None, None


# --------------------------------------------------------------------------
def controleer_referenties(paden, model_info, R):
    rij = []
    for p in paden:
        try:
            sc = lees_glb(p)
        except Exception as e:
            R.warn("referentie", f"{os.path.basename(p)} niet leesbaar: {e}")
            continue
        wm = sc.wereld_matrices()
        pts = scene_punten(sc, wm)
        lo, hi = bbox(pts)
        vk, via = voorkant_schatting(sc, wm)
        wortels = [sc.nodes[w].naam for w in sc.wortels]
        extras_sleutels = sorted({k for n in sc.nodes for k in n.extras})
        rij.append({"bestand": os.path.basename(p), "afm": (hi - lo).round(3).tolist(),
                    "min": lo.round(3).tolist(), "max": hi.round(3).tolist(),
                    "midden_xz": [round(float((lo[0] + hi[0]) / 2), 3), round(float((lo[2] + hi[2]) / 2), 3)],
                    "voorkant": vk, "via": via, "wortels": wortels, "extras_sleutels": extras_sleutels})
        if abs(lo[1]) > TOL_GROND:
            R.warn("referentie", f"{os.path.basename(p)}: grond op Y={lo[1]:.3f}, niet 0")
        if vk and vk != model_info["voorkant"] and model_info["voorkant"]:
            R.fout("referentie", f"voorkant model {model_info['voorkant']} wijkt af van "
                                 f"{os.path.basename(p)} ({vk}); corrigeer het model (doel_voorkant)")
        nieuw = [k for k in extras_sleutels if k.startswith("joint") or "min" in k.lower() or "max" in k.lower()]
        if nieuw:
            R.warn("referentie", f"{os.path.basename(p)} bevat extras-sleutels {nieuw}: neem ze over in "
                                 "tools/conventies.json (extras_sleutels)")
    return rij


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
        R.warn("hiërarchie", f"{len(sc.wortels)} wortelnodes; verwacht 1")
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
    if not [i for i in R.items if i["groep"] == "hiërarchie" and i["status"] == "FOUT"]:
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
    niet = []
    for n in sc.nodes:
        rot = np.asarray(n.matrix[:3, :3])
        if not np.allclose(rot, np.eye(3), atol=1e-6):
            niet.append(n.naam)
    (R.fout if niet else R.ok)("bevroren", f"rotatie/schaal niet toegepast op: {niet}" if niet
                                else "alle nodes: rotatie identiteit, schaal 1 (alleen translatie)")

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
        is_rig = n in rig_exact or any(p.match(n) for p in rig_pat)
        if is_rig:
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
    (R.ok if abs(lo[1]) <= TOL_GROND else R.fout)("nulpunt", f"laagste punt Y = {lo[1]:.4f} m (grond = 0)")
    wortel = sc.wortels[0]
    wp = scene_punten(sc, wm, [wortel])
    if len(wp):
        wl, wh = bbox(wp)
        mxz = np.array([(wl[0] + wh[0]) / 2, (wl[2] + wh[2]) / 2])
        (R.ok if np.abs(mxz).max() <= 0.02 else R.fout)(
            "nulpunt", f"midden onderwagen ({sc.nodes[wortel].naam}) op X,Z = {mxz.round(3).tolist()}")
    vk, via = voorkant_schatting(sc, wm)
    if vk:
        (R.ok if vk == CONV["doel_voorkant"] else R.fout)(
            "voorkant", f"voorkant {vk} (afgeleid van {via}); conventie {CONV['doel_voorkant']}")
    else:
        R.warn("voorkant", "voorkant niet automatisch te bepalen (geen Giek/Bak/Voorwagen)")
    if CONV.get("referentie_voorlopig"):
        R.warn("voorkant", "voorkant/nulpunt niet geverifieerd tegen de referentie-GLB's")

    ref_paden = list(referenties) or sorted(glob.glob(os.path.join(os.path.dirname(HIER), "referentie", "*.glb")))
    refs = controleer_referenties(ref_paden, {"voorkant": vk}, R) if ref_paden else []
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
            lengte = np.linalg.norm(p.normalen, axis=1)
            if np.any(np.abs(lengte - 1) > 1e-3):
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
            randen = trimesh.grouping.group_rows(m.edges_sorted, require_count=1)
            open_randen[n.naam] = int(len(randen))
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
    joint_nodes = [n for n in sc.nodes if SL["as"] in n.extras]
    for n in joint_nodes:
        p = wm[n.index][:3, 3]
        eigen = scene_punten(sc, wm, [n.index])
        tekst = f"{n.naam}: draaipunt {p.round(3).tolist()}"
        if n.naam in dp_man:
            d = float(np.linalg.norm(p - np.array(dp_man[n.naam])))
            if d > TOL_PIVOT:
                R.fout("pivot", f"{tekst} wijkt {d:.3f} m af van manifest {dp_man[n.naam]}")
                continue
        if len(eigen):
            elo, ehi = bbox(eigen)
            diag = float(np.linalg.norm(ehi - elo))
            buiten = float(np.linalg.norm(np.maximum(0, np.maximum(elo - p, p - ehi))))
            if buiten > max(0.05, 0.05 * diag):
                (R.fout if buiten > 0.25 else R.warn)("pivot", f"{tekst} ligt {buiten:.3f} m buiten de eigen geometrie")
                continue
            is_wiel = "straal_band" in n.extras or re.match(r"^(Wiel|Kettingwiel|Leiwiel)_", n.naam)
            if (n.extras[SL["as"]] != "Z" and not is_wiel
                    and np.linalg.norm(p - (elo + ehi) / 2) < 0.02 * diag):
                R.warn("pivot", f"{tekst} ligt in het midden van de eigen bbox: scharnier niet gezet?")
                continue
        if n.parent is not None:
            ouder = scene_punten(sc, wm, [n.parent])
            if len(ouder):
                olo, ohi = bbox(ouder)
                buiten = float(np.linalg.norm(np.maximum(0, np.maximum(olo - p, p - ohi))))
                if buiten > 0.05:
                    (R.fout if buiten > 0.25 else R.warn)("pivot", f"{tekst} ligt {buiten:.3f} m buiten de parent ({sc.nodes[n.parent].naam})")
                    continue
        R.ok("pivot", f"{tekst} (= manifest, binnen eigen geometrie en parent)")

    # ---------------- extras: joints en wielen ----------------
    man_j = {j["naam"]: j for j in man.get("joints", [])}
    for n in joint_nodes:
        ontbreekt = [SL[k] for k in ("type", "as", "min", "max") if SL[k] not in n.extras]
        if ontbreekt:
            R.fout("extras", f"{n.naam}: ontbrekende sleutels {ontbreekt}")
        mj = man_j.get(n.naam)
        if mj is None:
            R.fout("extras", f"{n.naam}: joint niet in manifest")
        elif (mj["min"], mj["max"], mj["as"]) != (n.extras[SL["min"]], n.extras[SL["max"]], n.extras[SL["as"]]):
            R.fout("extras", f"{n.naam}: extras en manifest verschillen")
        verw = rigdef.get("assen", {}).get(n.naam)
        if verw and verw != n.extras[SL["as"]]:
            R.fout("extras", f"{n.naam}: as {n.extras[SL['as']]}, rig {rig} verwacht {verw}")
    for naam in man_j:
        if sc.node(naam) is None or SL["as"] not in sc.node(naam).extras:
            R.fout("extras", f"manifest-joint {naam} zonder extras in GLB")
    for n in sc.nodes:
        if re.match(r"^(Wiel|Kettingwiel|Leiwiel)_", n.naam):
            mis = [k for k in ("straal_band", "straal_steekcirkel") if k not in n.extras]
            (R.warn if mis else R.ok)("extras", f"{n.naam}: {'mist ' + str(mis) if mis else 'stralen aanwezig'}")
    if joint_nodes and not [i for i in R.items if i["groep"] == "extras" and i["status"] == "FOUT"]:
        R.ok("extras", f"{len(joint_nodes)} joints met type/as/min/max in node-extras = manifest")
    if CONV.get("extras_sleutels_voorlopig") and joint_nodes:
        R.warn("extras", "sleutelnamen joint-limieten voorlopig (referentie ontbreekt)")

    # ---------------- joint-test ----------------
    joint_rijen = []
    mesh_nodes = [n.index for n in sc.nodes if n.mesh is not None]

    def botsingen(wmx):
        """{(a, b): grootste indringdiepte in m} voor elk botsend paar."""
        cm = trimesh.collision.CollisionManager()
        for i, (v, f) in sc.wereld_driehoeken(mesh_nodes, wmx).items():
            cm.add_object(str(i), trimesh.Trimesh(v, f, process=False))
        _, paren, data = cm.in_collision_internal(return_names=True, return_data=True)
        uit = {tuple(sorted((int(a), int(b)))): 0.0 for a, b in paren}
        for d in data:
            k = tuple(sorted(int(x) for x in d.names))
            uit[k] = max(uit.get(k, 0.0), float(d.depth))
        return uit

    try:
        rust = botsingen(wm)
        fcl_ok = True
    except Exception as e:  # python-fcl ontbreekt
        R.warn("joint-test", f"botsingstest niet beschikbaar ({e}); alleen standen berekend")
        rust, fcl_ok = {}, False
    if rust:
        R.ok("joint-test", "contact in ruststand (verwacht bij scharnieren): " +
             ", ".join(f"{sc.nodes[a].naam}/{sc.nodes[b].naam}" for a, b in sorted(rust)))

    for n in joint_nodes:
        as_g = n.extras.get(SL["as_gltf"]) or CONV["as_naar_gltf"][n.extras[SL["as"]]]
        bewegend = set(sc.afstammelingen(n.index))
        rust_pts = scene_punten(sc, wm, list(bewegend))
        rust_c = rust_pts.mean(0) if len(rust_pts) else wm[n.index][:3, 3]
        standen = {}
        for label, hoek in (("min", n.extras[SL["min"]]), ("max", n.extras[SL["max"]])):
            if n.extras.get(SL["type"], "rotatie") == "translatie":
                rm = np.eye(4)
                rm[:3, 3] = np.asarray(as_g, float) * hoek
            else:
                rm = rotatie_matrix(as_g, hoek)
            wmx = sc.wereld_matrices({n.index: rm})
            bp = scene_punten(sc, wmx, list(bewegend))
            c = bp.mean(0) if len(bp) else wmx[n.index][:3, 3]
            nieuw = []
            if fcl_ok:
                for (a, b), diepte in sorted(botsingen(wmx).items()):
                    if (a in bewegend) == (b in bewegend):
                        continue
                    extra = diepte - rust.get((a, b), 0.0)
                    if (a, b) not in rust or extra > TOL_DOORSNIJDING:
                        nieuw.append(f"{sc.nodes[a].naam}/{sc.nodes[b].naam} ({diepte:.3f} m)")
            onder_grond = float(bp[:, 1].min()) if len(bp) else 0.0
            if onder_grond < -TOL_GROND:
                nieuw.append(f"onder maaiveld (Y={onder_grond:.2f})")
            standen[label] = {"hoek": hoek, "zwaartepunt": c.round(3).tolist(), "doorsnijding": nieuw}
        # teken: positief = heffen (X) of links (Z, glTF +X bij voorkant +Z)
        klein = rotatie_matrix(as_g, 5.0)
        kp = scene_punten(sc, sc.wereld_matrices({n.index: klein}), list(bewegend))
        dc = (kp.mean(0) if len(kp) else rust_c) - rust_c
        if n.extras[SL["as"]] == "X":
            teken = "heffen" if dc[1] > 1e-4 else ("zakken" if dc[1] < -1e-4 else "n.v.t.")
        elif n.extras[SL["as"]] == "Z":
            links = np.cross([0, 1, 0], np.array(ASSEN_VOOR[CONV["doel_voorkant"]]))  # links = up × voor
            teken = "links" if dc @ links > 1e-4 else ("rechts" if dc @ links < -1e-4 else "n.v.t. (zwaartepunt op as)")
        else:
            teken = "n.v.t."
        joint_rijen.append({"naam": n.naam, "as": n.extras[SL["as"]], "as_gltf": as_g,
                            "positief": teken, **{k: v for k, v in standen.items()}})
        doorsn = standen["min"]["doorsnijding"] + standen["max"]["doorsnijding"]
        (R.warn if doorsn else R.ok)("joint-test", f"{n.naam}: {'doorsnijding ' + str(doorsn) if doorsn else 'geen nieuwe doorsnijding'} "
                                     f"op min {standen['min']['hoek']} / max {standen['max']['hoek']}; positief = {teken}")
        if teken in ("zakken", "rechts"):
            R.warn("joint-test", f"{n.naam}: positieve rotatie om lokale +{n.extras[SL['as']]} = {teken}; "
                                 "afspraak is heffen/links (zie spiegeling Unreal)")
    if joint_nodes:
        R.ok("joint-test", "geen vervorming mogelijk: alle nodes rigide (schaal 1, geen skinning)"
             if not niet and not sc.gltf.skins else "controleer schaal/skinning")

    # ---------------- testanimatie ----------------
    anim_pad = None
    if animatie and joint_nodes:
        kanalen, t0 = [], 0.0
        for n in joint_nodes:
            as_g = n.extras.get(SL["as_gltf"]) or CONV["as_naar_gltf"][n.extras[SL["as"]]]
            if n.extras.get(SL["type"], "rotatie") != "rotatie":
                continue
            hoeken = [0, n.extras[SL["min"]], n.extras[SL["max"]], 0]
            kanalen.append({"node": n.index, "tijden": [t0, t0 + 1, t0 + 3, t0 + 4],
                            "quats": [as_hoek_naar_quat(as_g, h) for h in hoeken]})
            t0 += 4.0
        anim_pad = os.path.join(map_, "test_animatie.glb")
        voeg_animaties_toe(sc, [{"naam": "Jointtest", "kanalen": kanalen}], anim_pad)
        R.ok("animatie", f"test_animatie.glb: per joint 0 → min → max → 0 ({t0:.0f} s)")

    # ---------------- rapport ----------------
    schrijf_rapport(map_, man, sc, wm, R, joint_rijen, tri_n, refs, afm, lo, hi)
    with open(os.path.join(map_, "controle.json"), "w", encoding="utf-8") as f:
        json.dump({"code": code, "resultaten": R.items, "joints": joint_rijen, "referenties": refs},
                  f, indent=2, ensure_ascii=False)
    fouten = R.fouten
    print(f"[{code}] controle: {len(R.items)} punten, {len(fouten)} FOUT, "
          f"{sum(i['status'] == 'WAARSCHUWING' for i in R.items)} waarschuwing")
    for i in R.items:
        if i["status"] != "OK":
            print(f"  {i['status']:12s} {i['groep']}: {i['tekst']}")
    return 1 if fouten else 0


ASSEN_VOOR = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


def boom(sc, i, diepte=0, wm=None):
    n = sc.nodes[i]
    p = wm[i][:3, 3].round(3).tolist()
    j = f"  joint {n.extras[SL['as']]} [{n.extras[SL['min']]}..{n.extras[SL['max']]}]" if SL["as"] in n.extras else ""
    regels = [f"{'  ' * diepte}- {n.naam}  (draaipunt {p}){j}"]
    for k in n.kinderen:
        regels += boom(sc, k, diepte + 1, wm)
    return regels


def schrijf_rapport(map_, man, sc, wm, R, joints, tri_n, refs, afm, lo, hi):
    r = [f"# Rapport {man.get('code', '')}: {man.get('merk', '')} {man.get('typeaanduiding', '')}", "",
         f"- Machinetype: {man.get('machinetype')}  |  detailniveau: {man.get('detailniveau')}  |  "
         f"rig: {man.get('rig') or '—'}  |  categorie: {man.get('categorie')}",
         f"- Bron: {man.get('bron_url') or '—'}  |  licentie: {man.get('licentie')}  |  formaat: {man.get('bronformaat')}",
         f"- Afmetingen (m): lengte {afm[2]:.3f}, breedte {afm[0]:.3f}, hoogte {afm[1]:.3f}  "
         f"(bbox {lo.round(3).tolist()} … {hi.round(3).tolist()})",
         f"- Polygonen: {sum(tri_n.values())}",
         f"- Bibliotheek-CSV: `{man.get('bibliotheek_csv')}`", "",
         "## Controle", "", "| Groep | Status | Bevinding |", "|---|---|---|"]
    for i in R.items:
        r.append(f"| {i['groep']} | {i['status']} | {i['tekst'].replace('|', '/')} |")
    r += ["", "## Hiërarchie", "", "```"]
    for w in sc.wortels:
        r += boom(sc, w, 0, wm)
    r += ["```", "", "## Joints: begin- en eindstand", ""]
    if joints:
        r += ["| Joint | As (4DVis) | As glTF | Positief | Min | Zwaartepunt bij min | Max | Zwaartepunt bij max | Doorsnijding |",
              "|---|---|---|---|---|---|---|---|---|"]
        for j in joints:
            d = (j["min"]["doorsnijding"] + j["max"]["doorsnijding"]) or ["geen"]
            r.append(f"| {j['naam']} | {j['as']} | {j['as_gltf']} | {j['positief']} | {j['min']['hoek']}° | "
                     f"{j['min']['zwaartepunt']} | {j['max']['hoek']}° | {j['max']['zwaartepunt']} | {', '.join(d)} |")
        r += ["", "Testanimatie: `test_animatie.glb` (animatie `Jointtest`)."]
    else:
        r.append("Geen joints (detailniveau eenvoudig).")
    r += ["", "## Polygonen per node", "", "| Node | Driehoeken |", "|---|---|"]
    r += [f"| {k} | {v} |" for k, v in tri_n.items()]
    if refs:
        r += ["", "## Vergelijking met referentie", "", "| Bestand | Afm X×Y×Z (m) | Min | Midden X,Z | Voorkant | Wortels | Extras-sleutels |",
              "|---|---|---|---|---|---|---|"]
        for x in refs:
            r.append(f"| {x['bestand']} | {x['afm']} | {x['min']} | {x['midden_xz']} | {x['voorkant']} ({x['via']}) | {x['wortels']} | {x['extras_sleutels']} |")
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
