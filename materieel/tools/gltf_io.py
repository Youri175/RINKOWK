"""Lezen en schrijven van GLB-bestanden voor de 4DVis-materieelpijplijn.

Werkt alleen met pygltflib en numpy, zodat het zonder Blender draait.
Lezen levert de ruwe node-structuur (inclusief niet-bevroren TRS), zodat de
controle kan zien wat er werkelijk in het bestand staat.
"""
from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field

import numpy as np
import pygltflib

COMPONENT_DTYPE = {
    5120: np.int8, 5121: np.uint8, 5122: np.int16,
    5123: np.uint16, 5125: np.uint32, 5126: np.float32,
}
TYPE_SIZE = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


# --------------------------------------------------------------------------
# Matrices en quaternions (glTF: kolom-major, quaternion als x, y, z, w)
# --------------------------------------------------------------------------
def quat_naar_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def as_hoek_naar_quat(as_vec, hoek_deg):
    a = np.asarray(as_vec, float)
    a = a / np.linalg.norm(a)
    h = np.radians(hoek_deg) / 2.0
    s = np.sin(h)
    return [float(a[0] * s), float(a[1] * s), float(a[2] * s), float(np.cos(h))]


def trs_matrix(t=None, r=None, s=None):
    m = np.eye(4)
    if s is not None:
        m[:3, :3] = np.diag(s)
    if r is not None:
        m[:3, :3] = quat_naar_matrix(r) @ m[:3, :3]
    if t is not None:
        m[:3, 3] = t
    return m


def rotatie_matrix(as_vec, hoek_deg, punt=(0, 0, 0)):
    """4x4 rotatie om een as door `punt`."""
    r = trs_matrix(r=as_hoek_naar_quat(as_vec, hoek_deg))
    t = trs_matrix(t=punt)
    ti = trs_matrix(t=-np.asarray(punt, float))
    return t @ r @ ti


def transformeer(punten, m):
    p = np.asarray(punten, float)
    return p @ m[:3, :3].T + m[:3, 3]


# --------------------------------------------------------------------------
# Lezen
# --------------------------------------------------------------------------
@dataclass
class Primitief:
    posities: np.ndarray
    indices: np.ndarray            # (M, 3)
    normalen: np.ndarray | None
    uv: np.ndarray | None
    materiaal: int | None


@dataclass
class Node:
    index: int
    naam: str
    kinderen: list
    parent: int | None
    matrix: np.ndarray              # lokale 4x4
    translatie: list | None
    rotatie: list | None
    schaal: list | None
    heeft_matrix: bool
    mesh: int | None
    extras: dict


@dataclass
class Scene:
    pad: str
    gltf: pygltflib.GLTF2
    nodes: list
    meshes: list                    # lijst van lijsten Primitief
    wortels: list
    afbeeldingen: list = field(default_factory=list)   # dicts: naam, uri, ingebed

    def node(self, naam):
        for n in self.nodes:
            if n.naam == naam:
                return n
        return None

    def wereld_matrices(self, extra_lokaal=None):
        """Wereldmatrix per node; `extra_lokaal` {index: 4x4} wordt na de
        lokale transformatie toegepast (gebruikt voor de joint-test)."""
        extra_lokaal = extra_lokaal or {}
        uit = {}

        def loop(i, ouder):
            n = self.nodes[i]
            lok = n.matrix @ extra_lokaal.get(i, np.eye(4))
            uit[i] = ouder @ lok
            for k in n.kinderen:
                loop(k, uit[i])

        for w in self.wortels:
            loop(w, np.eye(4))
        return uit

    def afstammelingen(self, i):
        uit = [i]
        for k in self.nodes[i].kinderen:
            uit += self.afstammelingen(k)
        return uit

    def voorouders(self, i):
        uit = []
        p = self.nodes[i].parent
        while p is not None:
            uit.append(p)
            p = self.nodes[p].parent
        return uit

    def wereld_driehoeken(self, node_indices, wm):
        """Eén (V, F) per node in wereldcoördinaten."""
        uit = {}
        for i in node_indices:
            n = self.nodes[i]
            if n.mesh is None:
                continue
            vs, fs, off = [], [], 0
            for p in self.meshes[n.mesh]:
                vs.append(transformeer(p.posities, wm[i]))
                fs.append(p.indices + off)
                off += len(p.posities)
            if vs:
                uit[i] = (np.vstack(vs), np.vstack(fs))
        return uit


def _lees_accessor(gltf, blob, idx):
    acc = gltf.accessors[idx]
    bv = gltf.bufferViews[acc.bufferView]
    dtype = COMPONENT_DTYPE[acc.componentType]
    n_comp = TYPE_SIZE[acc.type]
    start = (bv.byteOffset or 0) + (acc.byteOffset or 0)
    itemsize = np.dtype(dtype).itemsize * n_comp
    stride = bv.byteStride or itemsize
    if stride == itemsize:
        data = np.frombuffer(blob, dtype=dtype, count=acc.count * n_comp, offset=start)
        data = data.reshape(acc.count, n_comp) if n_comp > 1 else data
    else:
        rows = [np.frombuffer(blob, dtype=dtype, count=n_comp, offset=start + r * stride)
                for r in range(acc.count)]
        data = np.array(rows)
    return np.array(data)


def lees_glb(pad) -> Scene:
    gltf = pygltflib.GLTF2().load(pad)
    blob = gltf.binary_blob() or b""
    if not blob and gltf.buffers and gltf.buffers[0].uri:
        uri = gltf.buffers[0].uri
        if uri.startswith("data:"):
            import base64
            blob = base64.b64decode(uri.split(",", 1)[1])
        else:
            with open(os.path.join(os.path.dirname(pad), uri), "rb") as f:
                blob = f.read()

    meshes = []
    for m in gltf.meshes or []:
        prims = []
        for p in m.primitives:
            if p.mode not in (None, 4):
                continue        # alleen driehoeken
            pos = _lees_accessor(gltf, blob, p.attributes.POSITION).astype(float)
            if p.indices is not None:
                idx = _lees_accessor(gltf, blob, p.indices).astype(np.int64).reshape(-1, 3)
            else:
                idx = np.arange(len(pos)).reshape(-1, 3)
            nrm = (_lees_accessor(gltf, blob, p.attributes.NORMAL).astype(float)
                   if p.attributes.NORMAL is not None else None)
            uv = (_lees_accessor(gltf, blob, p.attributes.TEXCOORD_0).astype(float)
                  if p.attributes.TEXCOORD_0 is not None else None)
            prims.append(Primitief(pos, idx, nrm, uv, p.material))
        meshes.append(prims)

    nodes = []
    for i, n in enumerate(gltf.nodes or []):
        if n.matrix:
            mat = np.array(n.matrix, float).reshape(4, 4).T
        else:
            mat = trs_matrix(n.translation, n.rotation, n.scale)
        nodes.append(Node(i, n.name or f"node_{i}", list(n.children or []), None, mat,
                          n.translation, n.rotation, n.scale, bool(n.matrix), n.mesh,
                          dict(n.extras or {}) if isinstance(n.extras, dict) else {}))
    for n in nodes:
        for k in n.kinderen:
            nodes[k].parent = n.index

    sc = gltf.scenes[gltf.scene or 0] if gltf.scenes else None
    wortels = list(sc.nodes) if sc else [n.index for n in nodes if n.parent is None]

    afb = []
    for im in gltf.images or []:
        afb.append({"naam": im.name, "uri": im.uri, "ingebed": im.bufferView is not None})
    return Scene(pad, gltf, nodes, meshes, wortels, afb)


# --------------------------------------------------------------------------
# Schrijven
# --------------------------------------------------------------------------
@dataclass
class Materiaal:
    naam: str
    kleur: tuple = (0.8, 0.8, 0.8, 1.0)
    metallic: float = 0.0
    roughness: float = 0.8
    texture_bestand: str | None = None      # relatief pad naast het GLB
    dubbelzijdig: bool = False


@dataclass
class Deel:
    naam: str
    parent: str | None
    translatie: np.ndarray                    # lokaal t.o.v. parent
    primitieven: list = field(default_factory=list)  # (pos, nrm, uv|None, idx, materiaal_idx)
    extras: dict = field(default_factory=dict)


class _Buffer:
    def __init__(self):
        self.data = bytearray()
        self.views = []
        self.accessors = []

    def _view(self, raw, target=None):
        while len(self.data) % 4:
            self.data += b"\x00"
        off = len(self.data)
        self.data += raw
        self.views.append(pygltflib.BufferView(buffer=0, byteOffset=off,
                                               byteLength=len(raw), target=target))
        return len(self.views) - 1

    def accessor(self, arr, soort, target=None, minmax=False):
        arr = np.ascontiguousarray(arr)
        comp = {np.dtype(np.float32): 5126, np.dtype(np.uint32): 5125,
                np.dtype(np.uint16): 5123}[arr.dtype]
        v = self._view(arr.tobytes(), target)
        acc = pygltflib.Accessor(bufferView=v, componentType=comp,
                                 count=len(arr), type=soort)
        if minmax:
            a2 = arr.reshape(len(arr), -1)
            acc.min = [float(x) for x in a2.min(axis=0)]
            acc.max = [float(x) for x in a2.max(axis=0)]
        self.accessors.append(acc)
        return len(self.accessors) - 1


def schrijf_glb(pad, delen, materialen, wortel_extras=None, scene_naam="Scene"):
    """Schrijf delen (volgorde: parents vóór kinderen) naar een GLB.

    Textures worden als extern bestand (uri) gerefereerd, niet ingebed.
    """
    buf = _Buffer()
    g = pygltflib.GLTF2(asset=pygltflib.Asset(version="2.0", generator="4DVis-materieelpijplijn"))
    g.scenes = [pygltflib.Scene(name=scene_naam, nodes=[])]
    g.scene = 0

    # materialen en textures
    g.materials, g.textures, g.images, g.samplers = [], [], [], []
    for m in materialen:
        pbr = pygltflib.PbrMetallicRoughness(baseColorFactor=list(map(float, m.kleur)),
                                             metallicFactor=float(m.metallic),
                                             roughnessFactor=float(m.roughness))
        if m.texture_bestand:
            if not g.samplers:
                g.samplers.append(pygltflib.Sampler(magFilter=9729, minFilter=9987,
                                                    wrapS=10497, wrapT=10497))
            g.images.append(pygltflib.Image(uri=m.texture_bestand.replace(os.sep, "/"),
                                            name=os.path.basename(m.texture_bestand)))
            g.textures.append(pygltflib.Texture(sampler=0, source=len(g.images) - 1))
            pbr.baseColorTexture = pygltflib.TextureInfo(index=len(g.textures) - 1)
        g.materials.append(pygltflib.Material(
            name=m.naam, pbrMetallicRoughness=pbr, doubleSided=m.dubbelzijdig,
            alphaMode="BLEND" if m.kleur[3] < 1 else "OPAQUE"))

    naam_idx = {}
    g.nodes, g.meshes = [], []
    for d in delen:
        node = pygltflib.Node(name=d.naam, children=[])
        t = np.asarray(d.translatie, float)
        if np.any(np.abs(t) > 0):
            node.translation = [float(x) for x in t]
        if d.extras:
            node.extras = d.extras
        if d.primitieven:
            prims = []
            for pos, nrm, uv, idx, mat in d.primitieven:
                attrs = pygltflib.Attributes(
                    POSITION=buf.accessor(pos.astype(np.float32), "VEC3", 34962, minmax=True),
                    NORMAL=buf.accessor(nrm.astype(np.float32), "VEC3", 34962))
                if uv is not None:
                    attrs.TEXCOORD_0 = buf.accessor(uv.astype(np.float32), "VEC2", 34962)
                ind_type = np.uint16 if len(pos) < 65535 else np.uint32
                ind = buf.accessor(idx.reshape(-1).astype(ind_type), "SCALAR", 34963)
                prims.append(pygltflib.Primitive(attributes=attrs, indices=ind, material=mat))
            g.meshes.append(pygltflib.Mesh(name=d.naam, primitives=prims))
            node.mesh = len(g.meshes) - 1
        g.nodes.append(node)
        naam_idx[d.naam] = len(g.nodes) - 1
        if d.parent is None:
            g.scenes[0].nodes.append(naam_idx[d.naam])
        else:
            g.nodes[naam_idx[d.parent]].children.append(naam_idx[d.naam])
    for n in g.nodes:
        if not n.children:
            n.children = []

    if wortel_extras:
        g.scenes[0].extras = wortel_extras

    g.bufferViews = buf.views
    g.accessors = buf.accessors
    g.buffers = [pygltflib.Buffer(byteLength=len(buf.data))]
    g.set_binary_blob(bytes(buf.data))
    os.makedirs(os.path.dirname(os.path.abspath(pad)), exist_ok=True)
    g.save_binary(pad)
    return naam_idx


def voeg_animaties_toe(scene: Scene, animaties, pad):
    """Schrijf een kopie van `scene` met extra rotatie-animaties naar `pad`.

    `animaties`: lijst {naam, kanalen: [{node: index, tijden: [...], quats: [[x,y,z,w], ...]}]}.
    """
    g = copy.deepcopy(scene.gltf)
    blob = bytearray(g.binary_blob() or b"")
    g.animations = list(g.animations or [])

    def accessor(arr, soort, minmax=False):
        arr = np.ascontiguousarray(arr, np.float32)
        while len(blob) % 4:
            blob.extend(b"\x00")
        off = len(blob)
        blob.extend(arr.tobytes())
        g.bufferViews.append(pygltflib.BufferView(buffer=0, byteOffset=off, byteLength=arr.nbytes))
        acc = pygltflib.Accessor(bufferView=len(g.bufferViews) - 1, componentType=5126,
                                 count=len(arr), type=soort)
        if minmax:
            acc.min, acc.max = [float(arr.min())], [float(arr.max())]
        g.accessors.append(acc)
        return len(g.accessors) - 1

    for anim in animaties:
        samplers, channels = [], []
        for k in anim["kanalen"]:
            samplers.append(pygltflib.AnimationSampler(
                input=accessor(k["tijden"], "SCALAR", True), output=accessor(k["quats"], "VEC4"),
                interpolation="LINEAR"))
            channels.append(pygltflib.AnimationChannel(
                sampler=len(samplers) - 1,
                target=pygltflib.AnimationChannelTarget(node=k["node"], path="rotation")))
        g.animations.append(pygltflib.Animation(name=anim["naam"], samplers=samplers,
                                                channels=channels))
    g.buffers[0].byteLength = len(blob)
    g.set_binary_blob(bytes(blob))
    g.save_binary(pad)
