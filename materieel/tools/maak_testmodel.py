"""Stap 0: testmodel (kubus + tweedelige hiërarchie) maken, bouwen en controleren.

De bron wordt aangeleverd zoals een typische CAD-export: millimeter, Z-up,
voorkant +X, nulpunt verschoven, plus een dubbel object en een intern blokje.
Daarna bouwt tools/bouw.py het 4DVis-model en wordt het resultaat hier
onafhankelijk nagerekend (hiërarchie, pivot, maten) én door controleer.py.

    python3 tools/maak_testmodel.py
"""
import os
import subprocess
import sys

import numpy as np
import trimesh

HIER = os.path.dirname(os.path.abspath(__file__))
BASIS = os.path.dirname(HIER)
sys.path.insert(0, HIER)
from gltf_io import lees_glb  # noqa: E402

OFFSET = np.array([3000.0, -1500.0, 250.0])   # mm, verschoven nulpunt in de bron


def doos(afm, midden, kleur):
    m = trimesh.creation.box(extents=afm)
    m.apply_translation(midden)
    m.visual.face_colors = kleur
    return m


def maak_bron(pad):
    sc = trimesh.Scene()
    assy = trimesh.transformations.translation_matrix(OFFSET)
    sc.graph.update(frame_to="Samenstelling", frame_from=sc.graph.base_frame, matrix=assy)
    voet = doos([1000, 1000, 1000], [0, 0, 500], [230, 180, 20, 255])          # kubus
    arm = doos([1200, 200, 200], [100, 0, 1100], [90, 90, 90, 255])            # ligt erop, naar voren
    intern = doos([200, 200, 200], [0, 0, 500], [255, 0, 0, 255])              # binnen de kubus
    sc.add_geometry(voet, node_name="Voet", geom_name="Voet", parent_node_name="Samenstelling")
    sc.add_geometry(voet.copy(), node_name="Voet_kopie", geom_name="Voet_kopie",
                    parent_node_name="Samenstelling")                           # dubbel object
    sc.add_geometry(intern, node_name="Intern_blok", geom_name="Intern_blok",
                    parent_node_name="Samenstelling")
    sc.add_geometry(arm, node_name="Arm", geom_name="Arm", parent_node_name="Samenstelling")
    os.makedirs(os.path.dirname(pad), exist_ok=True)
    sc.export(pad)


def nareken(glb):
    sc = lees_glb(glb)
    fouten = []
    namen = [n.naam for n in sc.nodes]
    if namen != ["OV_TEST_Voet", "OV_TEST_Arm"]:
        fouten.append(f"nodes {namen}")
    voet, arm = sc.node("OV_TEST_Voet"), sc.node("OV_TEST_Arm")
    if arm.parent != voet.index or sc.wortels != [voet.index]:
        fouten.append("hiërarchie Voet -> Arm klopt niet")
    wm = sc.wereld_matrices()
    # verwachting: bron (-500, 0, 1000) mm rond de kubus -> (0, 1.0, -0.5) m, voorkant +Z
    if not np.allclose(wm[arm.index][:3, 3], [0, 1.0, -0.5], atol=1e-4):
        fouten.append(f"pivot arm {wm[arm.index][:3, 3]}, verwacht [0, 1, -0.5]")
    lok = np.vstack([p.posities for p in sc.meshes[arm.mesh]])
    if not (np.allclose(lok.min(0), [-0.1, 0, 0], atol=1e-4) and np.allclose(lok.max(0), [0.1, 0.2, 1.2], atol=1e-4)):
        fouten.append(f"arm-geometrie niet relatief aan draaipunt: {lok.min(0)} {lok.max(0)}")
    vp = np.vstack([p.posities for p in sc.meshes[voet.mesh]])
    if not (np.allclose(vp.min(0), [-0.5, 0, -0.5], atol=1e-4) and np.allclose(vp.max(0), [0.5, 1, 0.5], atol=1e-4)):
        fouten.append(f"kubus niet 1 m, midden op de grond: {vp.min(0)} {vp.max(0)}")
    if sum(len(p.indices) for p in sc.meshes[voet.mesh]) != 12:
        fouten.append("dubbel object of interne geometrie niet verwijderd (kubus heeft geen 12 driehoeken)")
    for n in sc.nodes:
        if n.rotatie or n.schaal or n.heeft_matrix:
            fouten.append(f"{n.naam}: rotatie/schaal niet bevroren")
    return fouten


def main():
    bron = os.path.join(BASIS, "testmodel", "bron_testmodel.glb")
    maak_bron(bron)
    print(f"bron geschreven: {bron}")
    cfg = os.path.join(BASIS, "machines", "OV_TEST.json")
    uit = os.path.join(BASIS, "lever")
    subprocess.run([sys.executable, os.path.join(HIER, "bouw.py"), cfg, "--uit", uit,
                    "--geen-controle"], check=True)
    fouten = nareken(os.path.join(uit, "OV_TEST", "model.glb"))
    print("nareken:", "OK" if not fouten else fouten)
    rc = subprocess.run([sys.executable, os.path.join(HIER, "controleer.py"),
                         os.path.join(uit, "OV_TEST")]).returncode
    return 1 if fouten or rc else 0


if __name__ == "__main__":
    sys.exit(main())
