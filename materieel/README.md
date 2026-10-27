# Materieel voor 4DVis

Werkmap voor 3D-bouwmachinemodellen die 4DVis als materieel laadt (GLB, meter, Y-up, voorkant −Z).
Werkt zonder Blender: trimesh + pygltflib (+ cascadio voor STEP). Alleen FBX/DAE heeft een lokale
Blender-voorstap nodig.

```
pip install -r requirements.txt
python3 tools/maak_testmodel.py                               # stap 0: testmodel bouwen en controleren
python3 tools/bouw.py machines/A.json machines/B.json         # batch; fout = overslaan + overgeslagen.json
python3 tools/controleer.py lever/<CODE>                      # stap 7 los draaien
python3 tools/render.py lever/<CODE> --stand Giek=-60         # voorbeeldplaatje in een stand

# zonder bruikbaar bronmodel (optie C): graafmachine uit datasheetmaten
python3 tools/genereer_graafmachine.py machines/EX_SY265E_maten.json
python3 tools/bouw.py machines/EX_SY265E.json
```

## Geleverd

| Map | Machine | Status |
|---|---|---|
| `lever/OV_TEST/` | Testmodel: kubus + tweedelige hiërarchie (stap 0) | controle 0 fouten |
| `lever/EX_SY265E/` | SANY SY265E, elektrische rupsgraafmachine, rig `Graafmachine`, vereenvoudigd uit de datasheet | controle 0 fouten, 0 waarschuwingen; werkbereik binnen 0,3% van de datasheet |

Per machine: `model.glb`, `manifest.json` (met de regel voor de bibliotheek-CSV), `rapport.md`,
`test_animatie.glb` (alle joints 0 → min → max → 0, cilinders en bakkoppeling volgen mee),
`voorbeeld_*.png`, `controle.json` en `bouwlog.txt`.

## Inhoud

| Pad | Doel |
|---|---|
| `tools/conventies.json` | Vaste 4DVis-eisen, afgeleid uit de referenties: assen, rignamen en -ketens, extras-sleutels, categorieën |
| `tools/bouw.py` | Stap 2–5 en 8: inlezen, eenheid/oriëntatie, bevriezen, opschonen, decimeren, draaipunten, cilinders richten, namen, GLB + manifest |
| `tools/controleer.py` | Stap 7: hiërarchie, pivots, bbox, nulpunt, voorkant, polygonen, normalen, texturen, rignamen, cilinders, joint-test, werkbereik, referentievergelijking, testanimatie, `rapport.md` |
| `tools/genereer_graafmachine.py` | Optie C: parametrische rupsgraafmachine uit een `*_maten.json` (datasheet + schattingen) |
| `tools/render.py` | Orthografische voorbeeldplaatjes (links, voor, boven, schuin) in een willekeurige stand |
| `tools/gltf_io.py` | GLB lezen en schrijven (textures als bestand naast het GLB) |
| `tools/blender_naar_glb.py` | Lokaal: FBX/DAE/BLEND naar GLB met behoud van hiërarchie (niet in de cloud getest) |
| `machines/*.json` | Config per machine; `_sjabloon_*.json` als startpunt |
| `referentie/` | `Cat336E_LongReach.glb` en `A40G.glb` (niet committen; staan in `.gitignore`) |

## Conventies (afgeleid uit Cat336E_LongReach.glb en A40G.glb)

- **Assen:** Y omhoog, voorkant **−Z**, links −X. Lege wortelnode (`EX_SY265E`) met `extras.driehoeken`.
- **Nulpunt:** zwenkas (graafmachine) of knikas (knikdumper) op de grond.
- **Opbouw graafmachine:** wortel → `<CODE>_Onderwagen` (rupsen, `Kettingwiel_*`, `Leiwiel_*`) en, als broer,
  `Bovenwagen` → `Giek` → `Stick` → `Bak`. Bakkoppeling `Tuimelaar` → `Bakstang` onder de stick.
- **Joints:** `extras.draaias` in Blender-notatie (bv. `lokaal X (Godot: X); positieve hoek = heffen`),
  grenzen in `min_hoek_graden` / `max_hoek_graden`. Positief = heffen, uit, open, naar links.
- **Cilinders:** `<naam>_Huis` en `<naam>_Stang`, oorsprong in de eigen pen, lokale −Z wijst naar de pen
  van de andere helft (de enige nodes met een rotatie). `penafstand_rust/min/max` in de extras.
- **Namen:** rig-onderdelen exact zoals 4DVis ze matcht, zonder voorvoegsel. Gewone delen `[TYPE]_[MODEL]_[ONDERDEEL]`.
- **Joint-test:** een doorsnijding is een nieuw contact tussen losse meshdelen (> 1 cm), of een contact dat
  in rust al bestaat (pen in oog) en meer dan 5 cm dieper wordt.
- **Publieke repo:** `lever/` staat in `.gitignore`. Alleen eigen werk (`OV_TEST`, `EX_SY265E`) gaat mee;
  modellen met een beperkte licentie gaan alleen rechtstreeks naar de gebruiker.

## Openstaand

- `min_hoek_graden` bestaat niet in de referentie: die kent alleen `max_hoek_graden` / `max_knik_graden`, en
  bij de graafmachine geen grenzen per joint (die volgen uit de cilinderslag). Bevestigen in 4DVis.
- Of 4DVis de bakkoppeling (Tuimelaar/Bakstang) zelf oplost, staat niet in de referentie. `controleer.py`
  doet het wel (vierstangen in het vlak van de stick).
- De referentie noemt gewone delen zonder voorvoegsel (`Onderwagen`, `Cabine`, `Rups_L`); wij volgen de
  conventie met voorvoegsel.
- De referentie heeft de textures ingebed; de eis is textures als bestand naast het GLB (de SY265E gebruikt geen textures).
