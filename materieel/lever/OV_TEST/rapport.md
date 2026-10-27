# Rapport OV_TEST: Test Kubus-testmodel

- Machinetype: overig  |  detailniveau: volledig  |  rig: —  |  categorie: Overig
- Bron: —  |  licentie: Eigen werk, gegenereerd door tools/maak_testmodel.py; vrij te gebruiken en herdistribueren  |  formaat: GLB (mm, Z-up)
- Afmetingen (m): lengte 1.200, breedte 1.000, hoogte 1.200  (bbox [-0.5, 0.0, -0.5] … [0.5, 1.2, 0.7])
- Polygonen: 24
- Bibliotheek-CSV: `OV_TEST/model.glb;0;1;`

## Controle

| Groep | Status | Bevinding |
|---|---|---|
| manifest | OK | alle sjabloonsleutels aanwezig |
| manifest | OK | bibliotheek_csv: OV_TEST/model.glb;0;1; |
| hiërarchie | OK | 2 nodes, parent-child gelijk aan manifest |
| bevroren | OK | alle nodes: rotatie identiteit, schaal 1 (alleen translatie) |
| namen | OK | alle namen uniek |
| afmetingen | OK | bbox breedte(X) 1.000 × hoogte(Y) 1.200 × lengte(Z) 1.200 m |
| afmetingen | OK | manifest-afmetingen wijken max 0.000 m af |
| nulpunt | OK | laagste punt Y = 0.0000 m (grond = 0) |
| nulpunt | OK | midden onderwagen (OV_TEST_Voet) op X,Z = [0.0, 0.0] |
| voorkant | WAARSCHUWING | voorkant niet automatisch te bepalen (geen Giek/Bak/Voorwagen) |
| voorkant | WAARSCHUWING | voorkant/nulpunt niet geverifieerd tegen de referentie-GLB's |
| referentie | WAARSCHUWING | geen referentie-GLB's gevonden (materieel/referentie/) |
| polygonen | OK | 24 driehoeken (doel ≤ 50000) |
| normalen | OK | alle primitieven hebben eenheidsnormalen, consistent met de winding |
| faces | OK | alle meshes gesloten (geen open randen) |
| texturen | OK | geen texturen gebruikt |
| pivot | OK | OV_TEST_Arm: draaipunt [0.0, 1.0, -0.5] (= manifest, binnen eigen geometrie en parent) |
| extras | OK | 1 joints met type/as/min/max in node-extras = manifest |
| extras | WAARSCHUWING | sleutelnamen joint-limieten voorlopig (referentie ontbreekt) |
| joint-test | OK | contact in ruststand (verwacht bij scharnieren): OV_TEST_Voet/OV_TEST_Arm |
| joint-test | OK | OV_TEST_Arm: geen nieuwe doorsnijding op min 0 / max 90; positief = heffen |
| joint-test | OK | geen vervorming mogelijk: alle nodes rigide (schaal 1, geen skinning) |
| animatie | OK | test_animatie.glb: per joint 0 → min → max → 0 (4 s) |

## Hiërarchie

```
- OV_TEST_Voet  (draaipunt [0.0, 0.0, 0.0])
  - OV_TEST_Arm  (draaipunt [0.0, 1.0, -0.5])  joint X [0..90]
```

## Joints: begin- en eindstand

| Joint | As (4DVis) | As glTF | Positief | Min | Zwaartepunt bij min | Max | Zwaartepunt bij max | Doorsnijding |
|---|---|---|---|---|---|---|---|---|
| OV_TEST_Arm | X | [-1, 0, 0] | heffen | 0° | [0.0, 1.1, 0.1] | 90° | [0.0, 1.6, -0.6] | geen |

Testanimatie: `test_animatie.glb` (animatie `Jointtest`).

## Polygonen per node

| Node | Driehoeken |
|---|---|
| OV_TEST_Voet | 12 |
| OV_TEST_Arm | 12 |

## Afwijkingen van de conventie

- bron in auto-0.001: geschaald met factor 0.001 naar meter
- bron-oriëntatie up +Z, voorkant +X: gedraaid naar Y-up, voorkant +Z
- midden onderwagen op de grond lag in de bron op [-1.5, 0.25, 3.0] m (na eenheid/as-conversie): naar 0 verschoven
- spiegeling: glTF is rechtshandig, Unreal linkshandig; de importer spiegelt één as. Niet in het model gecorrigeerd

## Aannames

- bron-eenheid automatisch bepaald: grootste maat 1200.0 eenheden -> factor 0.001
- joint-assen in de 4DVis-notatie (Z = verticaal, X = dwars, Y = langs); de richting in glTF staat in extras.joint_as_gltf, zo gekozen dat positief = heffen/open (X) en links (Z)
- joint OV_TEST_Arm: limieten geschat (0..90)

## Openstaande punten

- rig ontbreekt in 4DVis voor machinetype 'overig': keten en joints zitten in het model en dit manifest
- sleutelnamen voor joint-limieten in node-extras zijn voorlopig (joint_type, joint_as, joint_min, joint_max); referentie-GLB's niet beschikbaar om ze over te nemen
- referentiemodellen (Cat336E_LongReach.glb, A40G.glb) niet beschikbaar: voorkant +Z en nulpunt volgen de glTF-conventie, niet geverifieerd tegen de referentie
