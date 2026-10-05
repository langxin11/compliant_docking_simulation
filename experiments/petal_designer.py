"""Loopback-only interactive guide preview, using the reviewed geometry functions.

No physics runs or validated assets are modified. Height/clearance changes stretch
the original stop in the viewer only; saved designs explicitly require rebuilding
that solid, its collision geometry, seating references and assembly inertia.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
import math
import os
import re
import struct
import sys
import tempfile
import threading
from datetime import datetime, timezone
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from uuid import uuid4

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import prepare_petal_guidance as geometry  # noqa: E402

UI_ROOT = Path(__file__).with_name("petal_designer_ui")
PREVIEW_SLOTS = threading.BoundedSemaphore(2)
READ_TIMEOUT_SECONDS = 10
SAVE_ROOT = geometry.REPO_ROOT / "runs/designs/petal_guidance"
PARAMETERS = {
    "guide_tip_half_angle_deg": (1., 15., .03125, "平顶半角", "°"),
    "guide_blend_fraction": (.02, .30, .01, "斜坡过渡比例", ""),
    "guide_height_mm": (10., 24., .25, "导向高度", "mm"),
    "guide_edge_round_mm": (.25, 2., .25, "边缘圆滑", "mm"),
    "guide_axial_clearance_mm": (.05, 1., .05, "落座导面轴向余量", "mm"),
}
FIXED_RADIAL = {"radial_lead_width_mm": 0., "radial_crest_drop_mm": 0.}
POSE_LIMITS = {
    "opening_mm": (0., 30.), "x_mm": (-8., 8.), "y_mm": (-8., 8.),
    "yaw_deg": (-20., 20.), "tilt_deg": (-8., 8.),
}
ORIGINAL = json.loads((geometry.SOURCE / "model_info.json").read_text())
NARROW = geometry.candidate_parameters(ORIGINAL["parameters"], "narrow")
PRESETS = {
    "narrow": dict({k: NARROW[k] for k in PARAMETERS},
                   guide_tip_half_angle_deg=1., guide_blend_fraction=.3),
}
DEFAULT_PARAMETERS = dict(PRESETS["narrow"])
PRESET_LABELS = {"narrow": "窄平顶＋角向斜坡（半角1°，过渡0.3）"}


def select_design(path):
    """Show a user-selected saved candidate without replacing the baseline."""
    record = json.loads(Path(path).read_text())
    if record.get("schema") != "petal-guidance-design/v1":
        raise ValueError("请使用调形工具保存的候选文件。")
    values = validate_parameters(record["design_parameters"])
    PRESETS["selected"] = values
    PRESET_LABELS["selected"] = (f"用户候选：{values['guide_tip_half_angle_deg']:g}°／"
                                 f"过渡{values['guide_blend_fraction']:g}")
    DEFAULT_PARAMETERS.clear()
    DEFAULT_PARAMETERS.update(values)


def validate_parameters(values):
    if not isinstance(values, dict) or set(values) - (PARAMETERS.keys() | FIXED_RADIAL.keys()):
        raise ValueError("参数字段不正确，请使用本工具保存的参数文件。")
    for key in FIXED_RADIAL:
        if key in values and (isinstance(values[key], bool) or not isinstance(values[key], (int, float)) or values[key] != 0):
            raise ValueError("径向导面功能已从交互工具移除；请使用两项径向参数均为零的候选。")
    result = dict(PRESETS["narrow"])
    for key, value in values.items():
        if key in FIXED_RADIAL:
            continue
        low, high, *_ = PARAMETERS[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{PARAMETERS[key][3]}必须是数值。")
        if not low <= value <= high or not math.isfinite(value):
            raise ValueError(f"{PARAMETERS[key][3]}需在 {low:g}–{high:g} 之间。")
        result[key] = float(value)
    return result


def full_parameters(values):
    return dict(NARROW, **FIXED_RADIAL, **validate_parameters(values))


def preview_metadata(values):
    p = full_parameters(values)
    angles = np.linspace(-45., 45., 181)
    radius = (p["guide_inner_diameter_mm"] + p["outer_diameter_mm"]) / 4
    heights = geometry.surface_height(radius, angles, p)
    ri, ro = p["guide_inner_diameter_mm"] / 2, p["outer_diameter_mm"] / 2
    radii = np.linspace(ri, ro, 73)
    _, factor = geometry.shoulder_profile(p)
    # The pairing residual describes the analytic, unrounded surface only.
    check_r = np.linspace(ri, ro, 25)[:, None]
    check_a = np.linspace(-180., 180., 721)[None, :]
    residual = geometry.surface_height(check_r, check_a, p)
    residual += geometry.surface_height(check_r, 45. - check_a, p)
    residual -= p["guide_height_mm"]
    a = ORIGINAL["parameters"]["guide_tip_half_angle_deg"]
    d = abs((angles + 45.) % 90. - 45.)
    t = np.clip((45. - a - d) / (45. - 2*a), 0, 1)
    original_heights = ORIGINAL["parameters"]["guide_height_mm"] * t**3 * (10+t*(-15+6*t))
    base = p["adapter_thickness_mm"] + p["head_thickness_mm"]
    original_span = ORIGINAL["parameters"]["guide_height_mm"] + ORIGINAL["parameters"]["guide_axial_clearance_mm"]
    span = p["guide_height_mm"] + p["guide_axial_clearance_mm"]
    warnings = []
    if span != original_span:
        warnings.append("止挡按高度缩放预览；正式模型需重建止挡、惯量及落座基准。")
    return dict(
        protocol="petal-preview/v1", parameters={k: p[k] for k in PARAMETERS}, units="mm", base_height_mm=base,
        nominal_separation_mm=2*base+span, stop_scale=span/original_span,
        analytic_pair_residual_mm=float(np.max(abs(residual))),
        curve=dict(angles=angles.tolist(), heights=heights.tolist(),
                   baseline=original_heights.tolist(), radii=radii.tolist(),
                   crest=geometry.surface_height(radii, 0., p).tolist(),
                   valley=geometry.surface_height(radii, 45., p).tolist()),
        warnings=warnings, preview_only=True, physics_validated=False,
        edge_rounding_in_3d=True, curve_includes_edge_rounding=False,
        shoulder_factors_finite=bool(np.all(np.isfinite(factor))),
    )


def triangle_buffer(mesh):
    vertices, faces = mesh
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(lengths[:, None], 1e-30)
    # Browser positions are in mm; all existing simulation assets remain in m.
    packed = np.concatenate((triangles * 1000., np.repeat(normals[:, None], 3, axis=1)), axis=2)
    return packed.astype("<f4").tobytes()


def preview_packet(values):
    p = full_parameters(values)
    mesh = geometry.guide_mesh(0., p, angular_samples=97, radial_samples=7)
    data = triangle_buffer(mesh)
    meta = preview_metadata(values)
    meta["vertex_count"] = len(data) // 24
    meta["preview_angular_samples"] = 97
    raw = json.dumps(meta, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode()
    raw += b" " * (-len(raw) % 4)
    return struct.pack("<I", len(raw)) + raw + data


def read_preview_obj(path):
    """读取固定预览部件：仅接受有限坐标和正索引三角面。

    Args:
        path: 米制 OBJ 文件；材质、纹理坐标和输入法线不参与渲染。

    Returns:
        米制顶点数组 (N, 3) 与三角面索引 (M, 3)，不修改输入文件。

    Raises:
        ValueError: 输入为空、坐标非法、索引越界或使用不支持的几何记录。
    """
    vertices, faces = [], []
    counts = {"vt": 0, "vn": 0}
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        fields = line.split("#", 1)[0].split()
        if not fields:
            continue
        kind, *items = fields
        try:
            if kind in {"v", "vn", "vt"}:
                coordinates = [float(x) for x in items]
                lengths = {"v": {3}, "vn": {3}, "vt": {1, 2, 3}}
                if len(coordinates) not in lengths[kind] or not all(math.isfinite(x) for x in coordinates):
                    raise ValueError("invalid coordinates")
                if kind == "v":
                    vertices.append(coordinates)
                else:
                    counts[kind] += 1
            elif kind == "f":
                if len(items) != 3:
                    raise ValueError("only triangles are supported")
                face = []
                for item in items:
                    if not re.fullmatch(r"[1-9]\d*(?:/[1-9]\d*(?:/[1-9]\d*)?|//[1-9]\d*)?", item):
                        raise ValueError("only positive indices are supported")
                    parts = item.split("/")
                    for offset, attribute in ((1, "vt"), (2, "vn")):
                        if len(parts) > offset and parts[offset] and int(parts[offset]) > counts[attribute]:
                            raise ValueError("attribute index out of range")
                    index = int(parts[0])-1
                    if index >= len(vertices):
                        raise ValueError("vertex index out of range")
                    face.append(index)
                if len(set(face)) != 3:
                    raise ValueError("degenerate face")
                faces.append(face)
            elif kind not in {"o", "g", "s", "usemtl", "mtllib"}:
                raise ValueError("unsupported OBJ record")
        except ValueError as error:
            raise ValueError(f"OBJ {Path(path).name}:{number}: {error}") from error
    if not vertices or not faces:
        raise ValueError("OBJ mesh is empty")
    mesh = np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int)
    triangles = mesh[0][mesh[1]]
    if np.any(np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]), axis=1) == 0):
        raise ValueError("OBJ contains zero-area triangles")
    return mesh


@lru_cache(maxsize=3)
def base_buffer(name):
    if name not in ("adapter", "head_plate", "stop_land"):
        raise ValueError("未知部件。")
    return triangle_buffer(read_preview_obj(geometry.SOURCE / f"meshes/visual/{name}.obj"))


def save_design(values, pose=None, destination=SAVE_ROOT):
    p = full_parameters(values)
    if pose is None:
        pose = {}
    if not isinstance(pose, dict) or set(pose) - POSE_LIMITS.keys():
        raise ValueError("预览姿态字段不正确。")
    for key, value in pose.items():
        lo, hi = POSE_LIMITS[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not lo <= value <= hi or not math.isfinite(value):
            raise ValueError("预览姿态超出范围。")
    now = datetime.now(timezone.utc)
    name = f"candidate_{now:%Y%m%dT%H%M%SZ}_{uuid4().hex}.json"
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    record = dict(
        schema="petal-guidance-design/v1", status="UNVALIDATED_DESIGN",
        created_at_utc=now.isoformat(), design_parameters=dict({k: p[k] for k in PARAMETERS}, **FIXED_RADIAL),
        parameters=p, preview_pose=pose,
        derived_preview=preview_metadata(values),
        fixed_geometry=dict(outer_diameter_mm=100., guide_inner_diameter_mm=64., petals=4,
                            mating_yaw_deg=45., flip_axis="X", flip_deg=180.),
        geometry_source_sha256=hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest(),
        original_info_sha256=hashlib.sha256((geometry.SOURCE / "model_info.json").read_bytes()).hexdigest(),
        manufacturing_cad_generated=False, collision_model_generated=False,
        requires=["manufacturing solids and stops", "collision geometry", "assembly inertia",
                  "seating metadata and scene", "contact and load validation"],
    )
    raw = (json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()
    path = destination / name
    temporary = None
    published = False
    try:
        # 同目录硬链接原子发布完整文件，遇到同名候选不会覆盖。
        with tempfile.NamedTemporaryFile(dir=destination, prefix=".candidate_", delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(raw)
        for attempt in range(3):
            try:
                os.link(temporary, path)
                published = True
                break
            except FileExistsError:
                if attempt == 2:
                    raise
                path = destination / f"candidate_{now:%Y%m%dT%H%M%SZ}_{uuid4().hex}.json"
    finally:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                logging.exception("Could not remove candidate temporary file %s", temporary)
                if not published:
                    raise
    return path


def loopback_authority(value):
    """Validate a browser's local authority independently of the remote port.

    SSH and desktop previews can expose 8766 through a different local port. Keep
    the loopback hostname restriction, without equating client and server ports.
    """
    if not isinstance(value, str) or not value or any(c.isspace() for c in value):
        return None
    try:
        parsed = urlsplit("//" + value)
        if (parsed.hostname not in {"localhost", "127.0.0.1", "::1"}
                or parsed.username is not None or parsed.password is not None
                or parsed.path or parsed.query or parsed.fragment):
            return None
        port = parsed.port
        if port is not None and not 1 <= port <= 65535:
            return None
        return parsed.hostname, port or 80
    except ValueError:
        return None


class DesignerHandler(BaseHTTPRequestHandler):
    server_version = "PetalDesigner/1.0"

    def setup(self):
        super().setup()
        self.connection.settimeout(READ_TIMEOUT_SECONDS)

    def reply(self, status, body, kind="application/json; charset=utf-8", **headers):
        if not isinstance(body, bytes):
            body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
        for key, value in headers.items():
            self.send_header(key.replace("_", "-"), value)
        self.end_headers()
        self.wfile.write(body)

    def allowed(self):
        host = self.headers.get("Host", "")
        authority = loopback_authority(host)
        if authority is None:
            self.log_message("Rejected preview Host: %r", host[:160])
            self.reply(403, {"error": "此工具只接受本机访问。"})
            return False
        origin = self.headers.get("Origin")
        if origin:
            try:
                parsed = urlsplit(origin)
                origin_authority = loopback_authority(parsed.netloc)
                origins = {authority}
                if (parsed.scheme != "http" or parsed.path or parsed.query or parsed.fragment
                        or (origin_authority not in origins and origin not in getattr(self.server, "external_origins", set()))):
                    raise ValueError("foreign origin")
            except ValueError:
                self.log_message("Rejected preview Origin: %r; Host: %r", origin[:160], host[:160])
                self.reply(403, {"error": "不接受其他页面提交的参数。"})
                return False
        return True

    def do_GET(self):  # noqa: N802
        try:
            self.get_response()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except OSError as error:
            self.log_error("File I/O failed: %s", error)
            self.reply(500, {"error": "文件读写失败，请检查文件并重试。"})

    def get_response(self):
        if not self.allowed():
            return
        files = {"/": ("index.html", "text/html; charset=utf-8"),
                 "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                 "/state.js": ("state.js", "text/javascript; charset=utf-8"),
                 "/renderer.js": ("renderer.js", "text/javascript; charset=utf-8"),
                 "/style.css": ("style.css", "text/css; charset=utf-8")}
        if self.path in files:
            name, kind = files[self.path]
            self.reply(200, (UI_ROOT / name).read_bytes(), kind)
        elif self.path == "/api/info":
            self.reply(200, dict(parameters=PARAMETERS, presets=PRESETS,
                                 preset_labels=PRESET_LABELS, default=DEFAULT_PARAMETERS))
        elif self.path.startswith("/api/base/"):
            try:
                self.reply(200, base_buffer(self.path.removeprefix("/api/base/")), "application/octet-stream")
            except ValueError as error:
                self.reply(404, {"error": str(error)})
        elif self.path.startswith("/api/candidate/"):
            name = self.path.removeprefix("/api/candidate/")
            if not re.fullmatch(r"candidate_\d{8}T\d{6}Z_[a-f0-9]{8}(?:[a-f0-9]{24})?\.json", name) or not (SAVE_ROOT / name).is_file():
                self.reply(404, {"error": "候选文件不存在。"})
            else:
                self.reply(200, (SAVE_ROOT / name).read_bytes(),
                           Content_Disposition=f'attachment; filename="{name}"')
        elif self.path == "/favicon.ico":
            self.reply(204, b"")
        else:
            self.reply(404, {"error": "页面不存在。"})

    def do_POST(self):  # noqa: N802
        if not self.allowed():
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 16384 or self.headers.get("Content-Type") != "application/json":
                raise ValueError("请提交有效的参数。")
            raw = self.rfile.read(size)
            if len(raw) != size:
                raise ValueError("请求内容不完整。")
            payload = json.loads(raw)
            if not isinstance(payload, dict) or set(payload) - {"parameters", "pose"}:
                raise ValueError("请求字段不正确。")
            values = validate_parameters(payload.get("parameters", {}))
            if self.path == "/api/preview":
                if not PREVIEW_SLOTS.acquire(blocking=False):
                    self.reply(503, {"error": "预览繁忙，请稍后调整参数重试。"}, Retry_After="1")
                    return
                try:
                    packet = preview_packet(values)
                finally:
                    PREVIEW_SLOTS.release()
                self.reply(200, packet, "application/octet-stream")
            elif self.path == "/api/save":
                path = save_design(values, payload.get("pose"), destination=SAVE_ROOT)
                self.reply(201, dict(name=path.name, path=str(path), url=f"/api/candidate/{path.name}",
                                     status="UNVALIDATED_DESIGN"))
            else:
                self.reply(404, {"error": "操作不存在。"})
        except (ValueError, TypeError) as error:
            self.reply(400, {"error": str(error)})
        except TimeoutError:
            self.reply(408, {"error": "请求读取超时。"})
        except (BrokenPipeError, ConnectionResetError):
            pass  # a newer slider change cancelled the old preview
        except OSError as error:
            self.log_error("File I/O failed: %s", error)
            self.reply(500, {"error": "文件读写失败，请检查保存目录并重试。"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--design", type=Path, help="以保存的用户候选作为初始预览")
    parser.add_argument("--proxy-origin", action="append", default=[], help="显式允许本地反代的 http Origin，可重复；不信任转发头")
    args = parser.parse_args()
    for origin in args.proxy_origin:
        parsed = urlsplit(origin)
        if (parsed.scheme != "http" or loopback_authority(parsed.netloc) is None
                or parsed.path or parsed.query or parsed.fragment):
            parser.error("--proxy-origin 必须是完整的本地 http Origin，无路径")
    if args.design:
        select_design(args.design)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), DesignerHandler)
    server.external_origins = set(args.proxy_origin)
    print(f"PetalDock 交互调形：http://127.0.0.1:{server.server_port}", flush=True)
    print(f"保存候选：{SAVE_ROOT}；原模型和已验证结果保持不变。", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
