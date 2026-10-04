"""Loopback-only interactive guide preview, using the reviewed geometry functions.

No physics runs or validated assets are modified. Height/clearance changes stretch
the original stop in the viewer only; saved designs explicitly require rebuilding
that solid, its collision geometry, seating references and assembly inertia.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import struct
import sys
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
SAVE_ROOT = geometry.REPO_ROOT / "runs/designs/petal_guidance"
PARAMETERS = {
    "guide_tip_half_angle_deg": (1., 15., .03125, "平顶半角", "°"),
    "guide_blend_fraction": (.02, .30, .01, "斜坡过渡比例", ""),
    "guide_height_mm": (10., 24., .25, "导向高度", "mm"),
    "guide_edge_round_mm": (.25, 2., .25, "边缘圆滑", "mm"),
    "radial_lead_width_mm": (0., 8., .25, "径向导面宽度", "mm"),
    "radial_crest_drop_mm": (0., 5., .25, "径向峰顶下降", "mm"),
    "guide_axial_clearance_mm": (.05, 1., .05, "落座导面轴向余量", "mm"),
}
POSE_LIMITS = {
    "opening_mm": (0., 30.), "x_mm": (-8., 8.), "y_mm": (-8., 8.),
    "yaw_deg": (-20., 20.), "tilt_deg": (-8., 8.),
}
ORIGINAL = json.loads((geometry.SOURCE / "model_info.json").read_text())
NARROW = geometry.candidate_parameters(ORIGINAL["parameters"], "narrow")
PRESETS = {
    "narrow": {k: NARROW[k] for k in PARAMETERS},
    "radial": {k: geometry.candidate_parameters(ORIGINAL["parameters"], "radial")[k]
               for k in PARAMETERS},
}
DEFAULT_PARAMETERS = dict(PRESETS["narrow"])
PRESET_LABELS = {"narrow": "窄平顶＋角向斜坡", "radial": "径向实验对照（载荷超限）"}


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
    if not isinstance(values, dict) or set(values) - PARAMETERS.keys():
        raise ValueError("参数字段不正确，请使用本工具保存的参数文件。")
    result = dict(PRESETS["narrow"])
    for key, value in values.items():
        low, high, *_ = PARAMETERS[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{PARAMETERS[key][3]}必须是数值。")
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{PARAMETERS[key][3]}需在 {low:g}–{high:g} 之间。")
        result[key] = float(value)
    if result["radial_lead_width_mm"] == 0 and result["radial_crest_drop_mm"] != 0:
        raise ValueError("径向导面宽度为零时，峰顶下降也必须为零。")
    if 0 < result["radial_lead_width_mm"] < result["guide_edge_round_mm"]:
        raise ValueError("径向导面宽度需不小于边缘圆滑尺寸，或设为零。")
    return result


def full_parameters(values):
    return dict(NARROW, **validate_parameters(values))


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
    if p["radial_lead_width_mm"] and p["radial_crest_drop_mm"]:
        warnings.append("径向导面尚未验证；已有径向对照曾出现瞬时载荷超限。")
    return dict(
        parameters={k: p[k] for k in PARAMETERS}, units="mm", base_height_mm=base,
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


@lru_cache(maxsize=3)
def base_buffer(name):
    if name not in ("adapter", "head_plate", "stop_land"):
        raise ValueError("未知部件。")
    return triangle_buffer(geometry.read_obj(geometry.SOURCE / f"meshes/visual/{name}.obj"))


def save_design(values, pose=None, destination=SAVE_ROOT):
    p = full_parameters(values)
    if pose is None:
        pose = {}
    if not isinstance(pose, dict) or set(pose) - POSE_LIMITS.keys():
        raise ValueError("预览姿态字段不正确。")
    for key, value in pose.items():
        lo, hi = POSE_LIMITS[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError("预览姿态超出范围。")
    now = datetime.now(timezone.utc)
    name = f"candidate_{now:%Y%m%dT%H%M%SZ}_{uuid4().hex[:8]}.json"
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    record = dict(
        schema="petal-guidance-design/v1", status="UNVALIDATED_DESIGN",
        created_at_utc=now.isoformat(), design_parameters={k: p[k] for k in PARAMETERS},
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
    path = destination / name
    with path.open("x") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
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
                # A local reverse proxy may rewrite Host but retain Origin.
                forwarded = loopback_authority(self.headers.get("X-Forwarded-Host"))
                if forwarded is not None:
                    origins.add(forwarded)
                if (parsed.scheme != "http" or parsed.path or parsed.query or parsed.fragment
                        or origin_authority not in origins):
                    raise ValueError("foreign origin")
            except ValueError:
                self.log_message("Rejected preview Origin: %r; Host: %r", origin[:160], host[:160])
                self.reply(403, {"error": "不接受其他页面提交的参数。"})
                return False
        return True

    def do_GET(self):  # noqa: N802
        if not self.allowed():
            return
        files = {"/": ("index.html", "text/html; charset=utf-8"),
                 "/app.js": ("app.js", "text/javascript; charset=utf-8"),
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
            if not re.fullmatch(r"candidate_\d{8}T\d{6}Z_[a-f0-9]{8}\.json", name) or not (SAVE_ROOT / name).is_file():
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
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict) or set(payload) - {"parameters", "pose"}:
                raise ValueError("请求字段不正确。")
            values = validate_parameters(payload.get("parameters", {}))
            if self.path == "/api/preview":
                self.reply(200, preview_packet(values), "application/octet-stream")
            elif self.path == "/api/save":
                path = save_design(values, payload.get("pose"), destination=SAVE_ROOT)
                self.reply(201, dict(name=path.name, path=str(path), url=f"/api/candidate/{path.name}",
                                     status="UNVALIDATED_DESIGN"))
            else:
                self.reply(404, {"error": "操作不存在。"})
        except (ValueError, TypeError) as error:
            self.reply(400, {"error": str(error)})
        except (BrokenPipeError, ConnectionResetError):
            pass  # a newer slider change cancelled the old preview


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--design", type=Path, help="以保存的用户候选作为初始预览")
    args = parser.parse_args()
    if args.design:
        select_design(args.design)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), DesignerHandler)
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
