"""Optie C: een vereenvoudigd rupsgraafmachinemodel uit datasheetmaten (eigen werk).

    python3 tools/genereer_graafmachine.py machines/EX_SY265E_maten.json

Schrijft:
  gegenereerd/<CODE>_bron.glb   bronmodel (meter, Y-up, voorkant -Z), één mesh per bouwdeel
  machines/<CODE>.json          machineconfig voor tools/bouw.py (onderdelen, pennen, joints)

Opbouw en namen volgen Cat336E_LongReach.glb: lege wortel, daaronder Onderwagen en
Bovenwagen; Giek → Stick → Bak; cilinderparen <naam>_Huis / <naam>_Stang.
De gieklimieten worden afgestemd op de datasheet (max graafdiepte en -hoogte).
"""
import json
import os
import sys

import numpy as np
import trimesh
from shapely.geometry import LineString, Point, Polygon

HIER = os.path.dirname(os.path.abspath(__file__))
BASIS = os.path.dirname(HIER)
sys.path.insert(0, HIER)
from gltf_io import rotatie_van_naar  # noqa: E402

# zijaanzicht: (f, y) met f = vooruit = -Z;  bovenaanzicht: (x, f)
M_ZIJ = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)     # (f, y, x) -> (x, y, z)
M_BOVEN = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)   # (x, f, y) -> (x, y, z)


def w(f, y, x=0.0):
    """Zijaanzicht-punt naar wereld (x, y, z)."""
    return np.array([x, y, -f], float)


def richting(a_deg):
    a = np.radians(a_deg)
    return np.array([np.cos(a), np.sin(a)])


def kruis(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def normaal_boven(a_deg):
    a = np.radians(a_deg)
    return np.array([-np.sin(a), np.cos(a)])


def kleur(mesh, rgba):
    mesh.visual.face_colors = rgba
    return mesh


def doos(lo, hi, rgba):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    m = trimesh.creation.box(extents=hi - lo)
    m.apply_translation((lo + hi) / 2)
    return kleur(m, rgba)


def zij_prisma(punten_fy, x0, x1, rgba):
    """Zijprofiel (f, y) uitgetrokken over x0..x1."""
    m = trimesh.creation.extrude_polygon(Polygon(punten_fy).buffer(0), x1 - x0)
    T = np.eye(4)
    T[:3, :3] = M_ZIJ
    T[:3, 3] = [x0, 0, 0]
    m.apply_transform(T)
    return kleur(m, rgba)


def boven_prisma(punten_xf, y0, y1, rgba):
    """Bovenaanzicht (x, f) uitgetrokken over y0..y1."""
    m = trimesh.creation.extrude_polygon(Polygon(punten_xf).buffer(0), y1 - y0)
    T = np.eye(4)
    T[:3, :3] = M_BOVEN
    T[:3, 3] = [0, y0, 0]
    m.apply_transform(T)
    return kleur(m, rgba)


def balk(a_fy, b_fy, breedte, hoogte, xc, rgba):
    """Balk van a naar b in het zijvlak, doorsnede breedte (x) × hoogte."""
    A, B = w(*a_fy, xc), w(*b_fy, xc)
    m = trimesh.creation.box(extents=[breedte, hoogte, np.linalg.norm(B - A)])
    T = np.eye(4)
    T[:3, :3] = rotatie_van_naar([0, 0, 1], B - A)
    T[:3, 3] = (A + B) / 2
    m.apply_transform(T)
    return kleur(m, rgba)


def staaf(p0, p1, straal, rgba, secties=20):
    m = trimesh.creation.cylinder(radius=straal, segment=[p0, p1], sections=secties)
    return kleur(m, rgba)


def pen(fy, x0, x1, straal, rgba):
    return staaf(w(*fy, x0), w(*fy, x1), straal, rgba, 16)


def oogbok(pen_fy, naar_fy, x, rgba, lengte=0.35, hoogte=0.24, gat=0.11):
    """Vork van twee platen om een cilinderoog, met pen; platen lopen van de pen naar `naar_fy`."""
    pen_fy, naar_fy = np.asarray(pen_fy, float), np.asarray(naar_fy, float)
    e = (naar_fy - pen_fy) / np.linalg.norm(naar_fy - pen_fy)
    uit = [balk(pen_fy - 0.08 * e, pen_fy + lengte * e, 0.05, hoogte, x + xs * gat, rgba) for xs in (-1, 1)]
    uit.append(pen(pen_fy, x - gat - 0.025, x + gat + 0.025, 0.04, (150, 152, 155, 255)))
    return uit


# --------------------------------------------------------------------------
class Kinematica:
    """Zijaanzicht-kinematica: absolute hoeken vanaf horizontaal-vooruit, omhoog positief."""

    def __init__(self, voet_fy, Lg, Ls, Lb):
        self.F, self.Lg, self.Ls, self.Lb = np.asarray(voet_fy, float), Lg, Ls, Lb

    def standen(self, ag, as_, ab):
        S = self.F + self.Lg * richting(ag)
        K = S + self.Ls * richting(as_)
        T = K + self.Lb * richting(ab)
        return S, K, T

    def werkbereik(self, giek, steel_rel, bak_rel, zw_f=0.0, n=25):
        diepte = hoogte = reik = 0.0
        for ag in np.linspace(*giek, n):
            for bs in np.linspace(*steel_rel, n):
                for bb in np.linspace(*bak_rel, 9):
                    _, _, T = self.standen(ag, ag + bs, ag + bs + bb)
                    diepte, hoogte = max(diepte, -T[1]), max(hoogte, T[1])
                    if abs(T[1]) < 0.05:
                        reik = max(reik, T[0] - zw_f)
        return diepte, hoogte, reik


def pas_giek_aan(kin, ds, hoeken):
    """Zoek giek min/max (absoluut) die max graafdiepte en -hoogte van de datasheet raken."""
    beste = None
    for gmin in np.arange(-60, -30, 0.5):
        d, _, _ = kin.werkbereik((gmin, gmin + 1), hoeken["steel_relatief"], hoeken["bak_relatief"], n=13)
        if beste is None or abs(d - ds["max_graafdiepte_m"]) < beste[1]:
            beste = (gmin, abs(d - ds["max_graafdiepte_m"]))
    gmin = beste[0]
    beste = None
    for gmax in np.arange(40, 75, 0.5):
        _, h, _ = kin.werkbereik((gmax - 1, gmax), hoeken["steel_relatief"], hoeken["bak_relatief"], n=13)
        if beste is None or abs(h - ds["max_graafhoogte_m"]) < beste[1]:
            beste = (gmax, abs(h - ds["max_graafhoogte_m"]))
    return float(gmin), float(beste[0])


# --------------------------------------------------------------------------
def genereer(maten_pad):
    with open(maten_pad, encoding="utf-8") as f:
        M = json.load(f)
    D, G = M["datasheet"]["afmetingen"], M["geschat"]
    O, WB = M["datasheet"]["overig"], M["datasheet"]["werkbereik"]
    K_ = {k: v for k, v in G["kleuren"].items()}
    code = f"EX_{M['modelcode']}"
    pre = code + "_"
    delen = {}             # naam -> lijst meshes (bron)

    def voeg(naam, mesh):
        delen.setdefault(naam, []).append(mesh)

    # ---------------- onderwagen ----------------
    spoor, plaat = D["spoorbreedte_m"], D["rupsplaat_m"]
    r_eind = (D["rupslengte_m"] - D["rupscontactlengte_m"]) / 2
    fc = D["rupscontactlengte_m"] / 2
    bv = D["bodemvrijheid_m"]
    voeg("Onderwagen", doos([-0.9, bv, -1.3], [0.9, 0.95, 1.3], K_["donkergrijs"]))          # middenstuk
    voeg("Onderwagen", doos([-spoor / 2 + 0.25, 0.5, -0.6], [spoor / 2 - 0.25, 0.85, 0.6], K_["donkergrijs"]))
    voeg("Onderwagen", staaf([0, 0.95, 0], [0, D["contragewicht_vrije_hoogte_m"], 0], 0.85, K_["staal"], 32))
    rups_poly = LineString([(-fc, r_eind), (fc, r_eind)]).buffer(r_eind, quad_segs=10)
    wielen = {}
    for zijde, xs in (("L", -1), ("R", 1)):
        xc = xs * spoor / 2
        voeg(f"Rups_{zijde}", zij_prisma(list(rups_poly.exterior.coords), xc - plaat / 2, xc + plaat / 2, K_["rups"]))
        # rupsframe tussen de rollen (zichtbaar aan de binnenkant)
        binnen = xc - xs * plaat / 2
        voeg(f"Rups_{zijde}", doos([min(binnen, binnen - xs * 0.12), 0.22, -fc + 0.42],
                                   [max(binnen, binnen - xs * 0.12), 0.62, fc - 0.42], K_["donkergrijs"]))
        xw0, xw1 = sorted([binnen - xs * 0.13, binnen - xs * 0.01])           # 1 cm vrij van de rups
        xw = (xw0 + xw1) / 2
        # kettingwiel achter (+Z = f negatief), leiwiel voor
        tanden = Polygon([(np.cos(a) * (0.36 if i % 2 else 0.31), np.sin(a) * (0.36 if i % 2 else 0.31))
                          for i, a in enumerate(np.linspace(0, 2 * np.pi, 23)[:-1])])
        kw = [(p[0] - fc, p[1] + r_eind) for p in tanden.exterior.coords]
        voeg(f"Kettingwiel_{zijde}", zij_prisma(kw, xw0, xw1, K_["staal"]))
        wielen[f"Kettingwiel_{zijde}"] = w(-fc, r_eind, xw)
        lw = [(p[0] + fc, p[1] + r_eind) for p in Point(0, 0).buffer(0.34, quad_segs=8).exterior.coords]
        voeg(f"Leiwiel_{zijde}", zij_prisma(lw, xw0, xw1, K_["staal"]))
        wielen[f"Leiwiel_{zijde}"] = w(fc, r_eind, xw)

    # ---------------- bovenwagen ----------------
    bw, r_staart = D["bovenwagenbreedte_m"] / 2, D["staartdraaistraal_m"]
    y_onder, y_dek = D["contragewicht_vrije_hoogte_m"], G["dek_hoogte_m"]
    f_voor = -G["dek_voorkant_z"]

    boog = [(x, -np.sqrt(r_staart ** 2 - x ** 2)) for x in np.linspace(bw, -bw, 17)]
    gx = G["giekvoet"][0]
    sleuf = G["giek_doorsnede"][0] / 2 + 0.07                     # uitsparing in het dek voor de giek
    dek = [(-bw, f_voor), (gx - sleuf, f_voor), (gx - sleuf, 0.85), (gx + sleuf, 0.85),
           (gx + sleuf, f_voor), (bw, f_voor)] + boog
    voeg("Bovenwagen", boven_prisma(dek, y_onder, y_dek, K_["donkergrijs"]))
    f_cw = -2.2
    cw = [(-bw, f_cw), (bw, f_cw)] + boog
    voeg("Bovenwagen", boven_prisma(cw, y_onder, G["contragewicht_hoogte_m"], K_["geel"]))
    voeg("Bovenwagen", doos([0.15, y_dek, 0.25], [bw, G["accukap_hoogte_m"], -f_cw], K_["geel"]))      # accukap rechts
    voeg("Bovenwagen", doos([bw - 0.005, 1.7, 0.4], [bw + 0.01, 1.85, -f_cw - 0.1], K_["accent"]))  # accentstreep
    voeg("Bovenwagen", doos([0.75, y_dek, -f_voor], [bw, 1.9, 0.25], K_["geel"]))                   # gereedschapskist
    voeg("Bovenwagen", doos([-bw, y_dek, 0.45], [-0.25, 2.2, -f_cw], K_["geel"]))                   # achter cabine
    for xs in (-1, 1):                                                                              # giekvoetbok (vork)
        x_in = gx + xs * (G["giek_doorsnede"][0] / 2 + 0.01)
        voeg("Bovenwagen", doos([min(x_in, x_in + xs * 0.1), y_dek, -0.85], [max(x_in, x_in + xs * 0.1), 2.3, 0.1],
                                K_["geel"]))
    voeg("Bovenwagen", doos([gx - sleuf, y_dek, -0.15], [gx + sleuf, 1.45, 0.3], K_["geel"]))        # bok achter de voet
    dx = G["giekcilinder_x_afstand_m"]

    # cabine (links = -X)
    cx, cz = G["cabine"]["x"], G["cabine"]["z"]
    y_dak = D["hoogte_boven_cabine_m"]
    voeg(pre + "Cabine", doos([cx[0], y_dek, cz[0]], [cx[1], 1.95, cz[1]], K_["donkergrijs"]))
    voeg(pre + "Cabine", doos([cx[0] - 0.01, y_dak - 0.1, cz[0] - 0.02], [cx[1] + 0.01, y_dak, cz[1] + 0.01], K_["donkergrijs"]))
    for px in (cx[0], cx[1] - 0.07):
        for pz in (cz[0], cz[1] - 0.07):
            voeg(pre + "Cabine", doos([px, 1.95, pz], [px + 0.07, y_dak - 0.1, pz + 0.07], K_["donkergrijs"]))
    voeg(pre + "Cabine", doos([cx[0] + 0.3, 1.95, cz[0] + 0.7], [cx[1] - 0.3, 2.45, cz[1] - 0.25], K_["donkergrijs"]))  # stoel
    voeg(pre + "Cabine_Glas", doos([cx[0] + 0.015, 1.95, cz[0] + 0.015], [cx[1] - 0.015, y_dak - 0.1, cz[1] - 0.015], K_["glas"]))

    # ---------------- giek, steel, bak ----------------
    F = np.array([-G["giekvoet"][2], G["giekvoet"][1]])            # (f, y)
    kin = Kinematica(F, O["gieklengte_m"], O["steellengte_m"], G["bak_straal_m"])
    H = G["hoeken_absoluut"]
    gmin, gmax = pas_giek_aan(kin, WB, H)
    rs = G["ruststand"]
    S, Kp, T = kin.standen(rs["giek"], rs["steel"], rs["bak"])
    dg, ds_, db = richting(rs["giek"]), richting(rs["steel"]), richting(rs["bak"])
    ng = normaal_boven(rs["giek"])
    kp, kh = G["giek_knik"]["positie"] * O["gieklengte_m"], G["giek_knik"]["hoogte"]
    knie = F + kp * dg + kh * ng
    gw, gh1, gh2 = G["giek_doorsnede"]
    voeg("Giek", balk(F - 0.25 * richting(rs["giek"] + 25), knie, gw, gh1, gx, K_["geel"]))
    e2 = (S - knie) / np.linalg.norm(S - knie)
    vork = S - G["giek_vork_m"] * e2
    voeg("Giek", balk(knie, vork + 0.05 * e2, gw, gh2, gx, K_["geel"]))
    half_s0 = G["steel_doorsnede"][0] / 2
    for xs in (-1, 1):                                                          # vork om de stick
        voeg("Giek", balk(vork, S + 0.25 * e2, 0.07, gh2 * 0.9, gx + xs * (half_s0 + 0.045), K_["geel"]))
    voeg("Giek", pen(F, gx - 0.42, gx + 0.42, 0.13, K_["staal"]))
    sw, sh, _ = G["steel_doorsnede"]
    hef = G["steel_hefboom_m"]
    hefrichting = richting(rs["steel"] + 180 + G["steel_hefboom_hoek"])        # hefboom schuin naar voren
    voeg("Stick", balk(S - 0.15 * ds_, Kp, sw, sh, gx, K_["geel"]))
    voeg("Stick", balk(S, S + (hef - 0.18) * hefrichting, sw, 0.4, gx, K_["geel"]))
    voeg("Stick", pen(S, gx - 0.36, gx + 0.36, 0.11, K_["staal"]))
    # bak: zijprofiel in (u, v), u = pen -> tand, v = bolle achterkant
    u_k, v_k = db, normaal_boven(rs["bak"])
    prof = [tuple(p) for p in G["bak_profiel_uv"]]          # wand begint buiten de pen; oren dragen de pen
    sch = G["bak_straal_m"] / 1.56                  # tandpunt (profiel 1,44 + tand 0,12) precies op bak_straal
    bak_fy = [tuple(Kp + sch * (u * u_k + v * v_k)) for u, v in prof]
    bb = G["bak_breedte_m"]
    voeg("Bak", zij_prisma(bak_fy, gx - bb / 2, gx + bb / 2, K_["donkergrijs"]))
    for i in range(5):
        xt = gx - bb / 2 + 0.12 + i * (bb - 0.24) / 4
        voeg("Bak", balk(Kp + sch * 1.42 * u_k, Kp + sch * 1.56 * u_k, 0.08, 0.07, xt, K_["staal"]))
    for xs in (-1, 1):                                                          # oren voor de bakpen
        voeg("Bak", balk(Kp - 0.08 * u_k, Kp + sch * 0.42 * u_k, 0.05, 0.26, gx + xs * (half_s0 + 0.035),
                         K_["donkergrijs"]))
    voeg("Bak", pen(Kp, gx - half_s0 - 0.07, gx + half_s0 + 0.07, 0.1, K_["staal"]))

    # ---------------- bakkoppeling (vierstangen, zoals Tuimelaar/Bakstang in de referentie) ----------------
    no_s = normaal_boven(rs["steel"])
    KP = G["koppeling"]
    B = Kp - KP["tuimelaar_pen_m"] * ds_ + KP["tuimelaar_pen_uit_m"] * no_s       # tuimelaar op de stick
    D = Kp + KP["bakoor_m"] * richting(rs["steel"] + KP["bakoor_hoek"])           # bakstang op de bak
    r_t, l_b = KP["tuimelaar_m"], KP["bakstang_m"]

    def snijpunt(Bp, Dp, kant):
        d = np.linalg.norm(Dp - Bp)
        if d > r_t + l_b or d < abs(r_t - l_b):
            return None
        a_ = (r_t ** 2 - l_b ** 2 + d ** 2) / (2 * d)
        h = np.sqrt(max(r_t ** 2 - a_ ** 2, 0))
        e = (Dp - Bp) / d
        m_ = Bp + a_ * e
        opl = [m_ + h * np.array([-e[1], e[0]]), m_ - h * np.array([-e[1], e[0]])]
        return max(opl, key=lambda c: kant * kruis(Dp - Bp, c - Bp))

    kant = 1.0 if kruis(D - B, (B + D) / 2 + no_s - B) > 0 else -1.0     # C aan de buitenkant
    C = snijpunt(B, D, kant)

    def draai(p, om, hoek):
        c, s_ = np.cos(np.radians(hoek)), np.sin(np.radians(hoek))
        d = p - om
        return om + np.array([c * d[0] - s_ * d[1], s_ * d[0] + c * d[1]])

    grenzen = {"Giek": (gmin - rs["giek"], gmax - rs["giek"]),
               "Stick": (H["steel_relatief"][0] - (rs["steel"] - rs["giek"]),
                         H["steel_relatief"][1] - (rs["steel"] - rs["giek"])),
               "Bak": (H["bak_relatief"][0] - (rs["bak"] - rs["steel"]),
                       H["bak_relatief"][1] - (rs["bak"] - rs["steel"]))}
    waarschuwingen = []
    for hoek in np.linspace(*grenzen["Bak"], 61):
        if snijpunt(B, draai(D, Kp, hoek), kant) is None:
            waarschuwingen.append(f"bakkoppeling niet sluitend bij bakhoek {hoek:.0f}°")
            break

    # ---------------- cilinders ----------------
    def giekhart(langs):
        """Hoogte van de giek-hartlijn boven de koorde op `langs` m vanaf de voet."""
        return kh * langs / kp if langs <= kp else kh - (langs - kp) * kh / (O["gieklengte_m"] - kp)

    gc_langs, sc_langs = 3.0, 2.75
    gc_pen = F + gc_langs * dg + (giekhart(gc_langs) - gh2 / 2 - 0.1) * ng         # bok onder de giek
    sc_pen = F + sc_langs * dg + (giekhart(sc_langs) + gh2 / 2 + 0.32) * ng        # bok op de giek, na de knik
    pennen = {
        # naam: (huis-pen (f,y), stang-pen (f,y), x-posities, huisstraal, stangstraal, joint)
        "Giekcilinder": (np.array([f_voor - 0.05, 1.62]), gc_pen, None, 0.09, 0.06, "Giek"),
        "Stickcilinder": (sc_pen, S + hef * hefrichting, [gx], 0.09, 0.055, "Stick"),
        "Bakcilinder": (S + 0.45 * ds_ + 0.42 * no_s, C, [gx], 0.08, 0.05, "Bak"),
    }
    draaipunt_van = {"Giek": F, "Stick": S, "Bak": Kp}

    def stangpen(naam, joint, hoek):
        sp = pennen[naam][1]
        if naam == "Bakcilinder":
            return snijpunt(B, draai(D, Kp, hoek), kant)
        return draai(sp, draaipunt_van[joint], hoek)

    cil_cfg = []
    for naam, (hp, sp, xs, rh, rr, joint) in pennen.items():
        afst = [np.linalg.norm(stangpen(naam, joint, a) - hp) for a in np.linspace(*grenzen[joint], 41)]
        lo, hi = min(afst), max(afst)
        rust = np.linalg.norm(sp - hp)
        huis_len = lo - 0.22
        stang_len = hi - huis_len + 0.12
        if stang_len > lo - 0.05 or hi / lo > 1.8:
            waarschuwingen.append(f"{naam}: slag {lo:.2f}..{hi:.2f} m is mechanisch krap (verhouding {hi / lo:.2f})")
        paren = [("", xs[0])] if xs else [("_L", gx - dx), ("_R", gx + dx)]
        for suf, x in paren:
            H3, S3 = w(*hp, x), w(*sp, x)
            e = (S3 - H3) / np.linalg.norm(S3 - H3)
            n_ = naam + suf
            voeg(n_ + "_Huis", staaf(H3, H3 + e * huis_len, rh, K_["geel"]))
            voeg(n_ + "_Huis", staaf(H3 - [0.06, 0, 0], H3 + [0.06, 0, 0], rh * 1.15, K_["staal"], 12))
            voeg(n_ + "_Stang", staaf(S3, S3 - e * stang_len, rr, K_["chroom"]))
            voeg(n_ + "_Stang", staaf(S3 - [0.06, 0, 0], S3 + [0.06, 0, 0], rr * 1.6, K_["staal"], 12))
            cil_cfg.append((n_, H3, S3, joint, rust, round(lo, 4), round(hi, 4)))
    # pennen en bokken
    voeg("Giek", pen(gc_pen, gx - dx - 0.08, gx + dx + 0.08, 0.07, K_["staal"]))
    voeg("Giek", balk(gc_pen, gc_pen + 0.3 * ng, gw * 0.8, 0.3, gx, K_["geel"]))                  # bok giekcilinders
    for m_ in oogbok(sc_pen, sc_pen - ng, gx, K_["geel"], lengte=0.42):                          # bok stickcilinder
        voeg("Giek", m_)
    for m_ in oogbok(pennen["Stickcilinder"][1], S, gx, K_["geel"], lengte=0.3):                 # vork op de hefboom
        voeg("Stick", m_)
    for m_ in oogbok(pennen["Bakcilinder"][0], pennen["Bakcilinder"][0] - no_s, gx, K_["geel"], lengte=0.3):
        voeg("Stick", m_)                                                                         # bok bakcilinder
    hp_g = pennen["Giekcilinder"][0]
    for xs in (-1, 1):                                                                            # bokken giekcilinders
        for m_ in oogbok(hp_g, hp_g + np.array([-0.35, -0.45]), gx + xs * dx, K_["donkergrijs"], lengte=0.45):
            voeg("Bovenwagen", m_)
    half_s = G["steel_doorsnede"][0] / 2
    for xs in (-1, 1):                      # tuimelaar en bakstang: platen weerszijden van de stick
        voeg("Tuimelaar", balk(B, C, 0.05, 0.16, gx + xs * (half_s + 0.04), K_["donkergrijs"]))
        voeg("Bakstang", balk(C, D, 0.05, 0.14, gx + xs * (half_s + 0.1), K_["donkergrijs"]))
        voeg("Bak", balk(Kp, D, 0.06, 0.22, gx + xs * (half_s + 0.17), K_["donkergrijs"]))          # bakoren
    voeg("Tuimelaar", pen(B, gx - half_s - 0.07, gx + half_s + 0.07, 0.055, K_["staal"]))
    voeg("Tuimelaar", pen(C, gx - half_s - 0.07, gx + half_s + 0.07, 0.05, K_["staal"]))
    voeg("Bakstang", pen(D, gx - half_s - 0.2, gx + half_s + 0.2, 0.05, K_["staal"]))

    # ---------------- bron-GLB ----------------
    sc = trimesh.Scene()
    for naam, ms in delen.items():
        for i, m in enumerate(ms):
            sc.add_geometry(m, node_name=f"{naam}__{i}", geom_name=f"{naam}__{i}")
    uitdir = os.path.join(BASIS, "gegenereerd")
    os.makedirs(uitdir, exist_ok=True)
    bron = os.path.join(uitdir, f"{code}_bron.glb")
    sc.export(bron)

    # ---------------- machineconfig ----------------
    def pat(n):
        return [rf"/{n}__\d+$"]

    def lst(p):
        return [round(float(c), 4) for c in p]

    rig_ao = {"Giek": "giek", "Stick": "stick", "Bak": "bak"}
    tw = {"Tuimelaar": ("Stick", B, "tuimelaar"), "Bakstang": ("Tuimelaar", C, "bakstang")}
    on = [{"naam": pre + "Onderwagen", "bron": pat("Onderwagen"),
           "extras": {"ao_groep": "onderwagen", "functie": "Middenstuk, rupsframes en draaikrans (vast)"}}]
    for zijde in ("L", "R"):
        on.append({"naam": f"{pre}Rups_{zijde}", "parent": pre + "Onderwagen", "bron": pat(f"Rups_{zijde}"),
                   "extras": {"ao_groep": "onderwagen", "functie": "Rups, stilstaand"}})
        for soort in ("Kettingwiel", "Leiwiel"):
            n = f"{soort}_{zijde}"
            on.append({"naam": n, "parent": pre + "Onderwagen", "bron": pat(n), "draaipunt": lst(wielen[n]),
                       "joint": {"as": "X", "min": -180, "max": 180, "bron": "geschat",
                                 "positief": "machine rijdt achteruit"},
                       "wiel": {"straal_band": round(r_eind, 4), "straal_steekcirkel": G["kettingwiel_steekcirkel_m"]},
                       "extras": {"ao_groep": "onderwagen"}})
    on.append({"naam": "Bovenwagen", "bron": pat("Bovenwagen"), "draaipunt": [0, 0, 0],
               "joint": {"as": "Z", "min": -180, "max": 180, "bron": "geschat", "positief": "naar links"},
               "extras": {"ao_groep": "boven", "functie": "Bovenwagen; zwenkt om lokale Z (Godot: Y) door de oorsprong"}})
    on.append({"naam": pre + "Cabine", "parent": "Bovenwagen", "bron": pat(pre + "Cabine"),
               "extras": {"ao_groep": "boven", "functie": "Cabine (vereenvoudigd)"}})
    on.append({"naam": pre + "Cabine_Glas", "parent": "Bovenwagen", "bron": pat(pre + "Cabine_Glas"),
               "extras": {"ao_groep": "", "functie": "Vaste ruiten"}})
    ouder = {"Giek": "Bovenwagen", "Stick": "Giek", "Bak": "Stick"}
    pos = {"Giek": "heffen", "Stick": "stick uit (van de machine af)", "Bak": "bak openen (lossen)"}
    for n in ("Giek", "Stick", "Bak"):
        on.append({"naam": n, "parent": ouder[n], "bron": pat(n),
                   "draaipunt": lst(w(*draaipunt_van[n], gx)),
                   "joint": {"as": "X", "min": round(grenzen[n][0], 1), "max": round(grenzen[n][1], 1),
                             "bron": "geschat", "positief": pos[n]},
                   "extras": {"ao_groep": rig_ao[n]}})
    for n, (par, p, ao) in tw.items():
        on.append({"naam": n, "parent": par, "bron": pat(n), "draaipunt": lst(w(*p, gx)),
                   "extras": {"ao_groep": ao, "draaias": "lokaal X (Godot: X)",
                              "functie": "Bakkoppeling (vierstangen); volgt de bakhoek"}})
    huis_ouder = {"Giek": "Bovenwagen", "Stick": "Giek", "Bak": "Stick"}
    stang_ouder = {"Giek": "Giek", "Stick": "Stick", "Bak": "Tuimelaar"}
    for n_, H3, S3, joint, _, lo, hi in cil_cfg:
        cil = {"paar": n_, "joint": joint, "penafstand_min": lo, "penafstand_max": hi}
        on.append({"naam": n_ + "_Huis", "parent": huis_ouder[joint], "bron": pat(n_ + "_Huis"),
                   "draaipunt": lst(H3), "richt_naar": lst(S3), "cilinder": cil, "extras": {"ao_groep": ""}})
        on.append({"naam": n_ + "_Stang", "parent": stang_ouder[joint], "bron": pat(n_ + "_Stang"),
                   "draaipunt": lst(S3), "richt_naar": lst(H3), "cilinder": cil, "extras": {"ao_groep": ""}})

    kin_tekst = (f"giek {gmin:.1f}..{gmax:.1f}° absoluut, steel {H['steel_relatief']}° en bak "
                 f"{H['bak_relatief']}° relatief")
    d_, h_, r_ = kin.werkbereik((gmin, gmax), H["steel_relatief"], H["bak_relatief"])
    cfg = {
        "_uitleg": f"Gegenereerd door tools/genereer_graafmachine.py uit {os.path.basename(maten_pad)}; niet met de hand bewerken.",
        "merk": M["merk"], "typeaanduiding": M["typeaanduiding"], "machinetype": "graafmachine",
        "modelcode": M["modelcode"], "detailniveau": "volledig", "wortel": code,
        "bron": {"bestand": os.path.relpath(bron, os.path.join(BASIS, "machines")),
                 "url": "", "formaat": "GLB (procedureel, m, Y-up)",
                 "licentie": "Eigen werk: procedureel gegenereerd uit openbare datasheetmaten; vrij te gebruiken en "
                             "herdistribueren. Bevat geen merklogo's of fabrikantgeometrie."},
        "bron_eenheid": "m", "bron_up": "+Y", "bron_voorkant": "-Z", "nulpunt_m": [0, 0, 0],
        "nulpunt_onderdeel": pre + "Onderwagen",
        "doel_polygonen": 50000,
        "onderdelen": on,
        "behoud": [pre + "Cabine", "Giek", "Stick", "Bak"],
        "aannames": [
            "Vereenvoudigd model (optie C): maten uit de datasheet, vormen schematisch; geen fabrikantgeometrie",
            "Giek 5,9 m en steel 2,95 m van de SY265C overgenomen: het werkbereik van de E is gelijk aan dat van de C",
            f"Werkhoeken {kin_tekst}; giekgrenzen afgestemd op max graafdiepte en -hoogte uit de datasheet",
            f"Punt-kinematica (tandpunt): diepte {d_:.2f} m, hoogte {h_:.2f} m, reikwijdte op de grond {r_:.2f} m",
            "Ligging pennen (giekvoet, cilinderpennen, bakkoppeling), cabine, accukap en contragewichthoogte geschat "
            "naar de verhoudingen in het Cat336E-referentiemodel; cilinderpennen en koppeling zo gekozen dat er over "
            "het hele bereik geen doorsnijding is en de slagverhouding ≤ 1,6 blijft",
            "Bakinhoud van het model ca. 0,9 m³ afgestreken (datasheet 1,35 m³ gestort); bakbreedte 1,35 m geschat",
            "Kleuren: SANY-geel met donkergrijs; blauwe accentstreep als kenmerk van de elektrische uitvoering (geschat)",
        ],
        "openstaande_punten": [
            "Bakkoppeling Tuimelaar/Bakstang volgt de opbouw van de referentie; of 4DVis de vierstangen zelf "
            "oplost (zoals controleer.py doet) is in de referentie niet vastgelegd: lokaal bevestigen",
            "Transportlengte (10,565 m) en transporthoogte (3,51 m) niet getoetst: de transportstand van de giek "
            "is niet gepubliceerd",
            "Gewone onderdelen hebben het voorvoegsel EX_SY265E_ (conventie stap 5); de Cat336E-referentie gebruikt "
            "kale namen (Onderwagen, Cabine, Rups_L). Bevestigen dat 4DVis daar niet op matcht",
            "Officiële SANY-datasheet SY265E niet zelf ingezien (download geblokkeerd in de cloud); waarden komen "
            "uit zoekresultaten en dealerpagina's die elkaar op punten tegenspreken (zie datasheet.bronnen)",
        ] + waarschuwingen,
        "voorbeeldstanden": {"graven": {"Giek": -77, "Stick": 25, "Bak": 10},
                             "lossen": {"Giek": 25, "Stick": 80, "Bak": 95, "Bovenwagen": 60}},
        "manifest_extra": {"datasheet": M["datasheet"],
                           "koppeling": {"stick": "Stick", "bak": "Bak", "tuimelaar": "Tuimelaar",
                                         "bakstang": "Bakstang", "pen_bakstang_bak_m": lst(w(*D))}},
    }
    cfg_pad = os.path.join(BASIS, "machines", f"{code}.json")
    with open(cfg_pad, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
    print(f"bron: {bron}\nconfig: {cfg_pad}\n{kin_tekst}\npunt-werkbereik: diepte {d_:.3f}, hoogte {h_:.3f}, "
          f"reik grond {r_:.3f} (datasheet {WB})")
    for x in waarschuwingen:
        print("LET OP:", x)
    return cfg_pad


if __name__ == "__main__":
    genereer(sys.argv[1])
