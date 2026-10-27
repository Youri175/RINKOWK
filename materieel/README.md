# Materieel voor 4DVis

Werkmap voor 3D-bouwmachinemodellen die 4DVis als materieel laadt (GLB, meter, Y-up).
Werkt zonder Blender: trimesh + pygltflib (+ cascadio voor STEP). Alleen FBX/DAE heeft
een lokale Blender-voorstap nodig.

```
pip install -r requirements.txt
python3 tools/maak_testmodel.py            # stap 0: testmodel bouwen en controleren
python3 tools/bouw.py machines/A.json machines/B.json   # batch; fout = overslaan + overgeslagen.json
python3 tools/controleer.py lever/<CODE>   # stap 7 los draaien
```

## Inhoud

| Pad | Doel |
|---|---|
| `tools/conventies.json` | Vaste 4DVis-eisen: typecodes, rignamen, rigketens, categorieën, as-richtingen, extras-sleutels |
| `tools/bouw.py` | Stap 2–5 en 8: inlezen, eenheid/oriëntatie, bevriezen, opschonen, decimeren, draaipunten, namen, GLB + manifest |
| `tools/controleer.py` | Stap 7: hiërarchie, pivots, bbox, nulpunt, voorkant, polygonen, normalen, texturen, rignamen, joint-test (botsing per min/max-stand), testanimatie, `rapport.md` |
| `tools/gltf_io.py` | GLB lezen/schrijven (textures als bestand naast het GLB) |
| `tools/blender_naar_glb.py` | Lokaal: FBX/DAE/BLEND naar GLB met behoud van hiërarchie |
| `tools/maak_testmodel.py` | Testmodel: kubus + tweedelige hiërarchie, bewust aangeleverd in mm, Z-up, verschoven |
| `machines/*.json` | Eén config per machine; `_sjabloon_*.json` als startpunt |
| `referentie/` | Hier komen `Cat336E_LongReach.glb` en `A40G.glb` (niet committen) |
| `lever/<CODE>/` | Levering: `model.glb`, `textures/`, `manifest.json`, `rapport.md`, `test_animatie.glb` |

## Werkwijze per machine

1. **Bron (stap 1)**: zoek op de exacte typeaanduiding (fabrikant → CAD-bibliotheken → Sketchfab e.d.).
   Leg `url`, `licentie` en `formaat` vast in de config. Bij een lege of onduidelijke licentie
   slaat `bouw.py` de machine over.
2. **Inspecteren (stap 2)**: `bouw.py` meldt het aantal meshes, de driehoeken en de afmetingen in bron-
   en meter-eenheden. Kies `bron_eenheid` (of `auto`), `bron_up` en `bron_voorkant`.
3. **Onderdelen**: wijs bron-meshes met regex (`bron`) toe aan onderdelen, met `parent`,
   `draaipunt` en `joint`. Wat niet matcht valt in het eerste onderdeel (`standaard_onderdeel`).
   Rig-onderdelen krijgen de exacte 4DVis-namen zonder voorvoegsel, alle andere `[TYPE]_[MODEL]_[ONDERDEEL]`.
4. **Bouwen**: `bouw.py` past de rotatie en schaal toe op de geometrie, zet elke node op zijn draaipunt
   (alleen translatie), verwijdert dubbele en interne objecten, decimeert naar `doel_polygonen`
   (behalve `behoud`), legt het nulpunt op het midden van de onderwagen op de grond en schrijft de joints en
   wielstralen in de node-extras.
5. **Controle**: draait automatisch. Een FOUT geeft exitcode 1. Lees `rapport.md`.

## Conventies

- Doelassen glTF: Y omhoog, voorkant **+Z**, +X = links (glTF-standaard).
- Joint-as in het manifest in de 4DVis-notatie (Z = verticaal, X = dwars). De echte richting staat
  in `extras.joint_as_gltf`, zo gekozen dat positief = heffen/open (X: glTF −X) en links (Z: glTF +Y).
  De controle meet het teken na.
- Repo is **publiek**: `lever/` en `referentie/` staan in `.gitignore` (behalve het eigen testmodel).
  Lever licentie-beperkte modellen alleen rechtstreeks aan de gebruiker.

## Openstaand zolang de referentie-GLB's ontbreken

- Voorkant (+Z) en nulpunt zijn niet tegen `Cat336E_LongReach.glb` / `A40G.glb` geverifieerd
  (`referentie_voorlopig` in `conventies.json`).
- Sleutelnamen voor joint-limieten in de node-extras zijn voorlopig (`joint_type`, `joint_as`,
  `joint_min`, `joint_max`). Zet de referenties in `referentie/`: `controleer.py` toont hun
  extras-sleutels. Pas daarna `extras_sleutels` aan en bouw opnieuw.
