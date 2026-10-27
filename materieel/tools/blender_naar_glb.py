"""Lokale voorstap voor bronnen die trimesh niet leest (FBX, DAE, 3DS, BLEND).

    blender -b -P tools/blender_naar_glb.py -- bron.fbx bron.glb

Exporteert zonder transformaties toe te passen, zodat de objecthiërarchie
en namen behouden blijven; bouw.py bevriest daarna rotatie/schaal en zet de
draaipunten. Niet in de cloud getest (Blender is daar niet beschikbaar).
"""
import os
import sys

import bpy

args = sys.argv[sys.argv.index("--") + 1:]
bron, doel = args[0], args[1]
bpy.ops.wm.read_factory_settings(use_empty=True)
ext = os.path.splitext(bron)[1].lower()
if ext == ".fbx":
    bpy.ops.import_scene.fbx(filepath=bron)
elif ext == ".dae":
    bpy.ops.wm.collada_import(filepath=bron)
elif ext == ".obj":
    bpy.ops.wm.obj_import(filepath=bron)
elif ext == ".blend":
    bpy.ops.wm.open_mainfile(filepath=bron)
else:
    raise SystemExit(f"niet ondersteund: {ext}")

# verborgen objecten (stap 4) niet meenemen
for ob in list(bpy.data.objects):
    if ob.hide_get() or ob.hide_render:
        print("verborgen, verwijderd:", ob.name)
        bpy.data.objects.remove(ob)

bpy.ops.export_scene.gltf(filepath=doel, export_format="GLB", export_yup=True,
                          export_apply=False, export_extras=True, use_visible=False)
print("geschreven:", doel)
