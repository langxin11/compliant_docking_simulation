"""Interactive preview uses paired real surfaces and preserves validated assets."""
import importlib
import json
import struct
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))
designer = importlib.import_module("petal_designer")


def unpack(packet):
    length = struct.unpack("<I", packet[:4])[0]
    return json.loads(packet[4:4+length]), np.frombuffer(packet, dtype="<f4", offset=4+length).reshape(-1, 6)


@pytest.mark.parametrize("preset", ["narrow", "radial"])
def test_preview_packet_contains_real_surface_in_mm_and_unit_normals(preset):
    values = designer.PRESETS[preset]
    meta, triangles = unpack(designer.preview_packet(values))
    assert meta["vertex_count"] == len(triangles)
    assert meta["units"] == "mm" and meta["physics_validated"] is False
    assert np.isfinite(triangles).all()
    np.testing.assert_allclose(np.linalg.norm(triangles[:, 3:], axis=1), 1., atol=1e-6)
    p = designer.full_parameters(values)
    vertices, _ = designer.geometry.guide_mesh(0., p, 97, 7)
    np.testing.assert_allclose(triangles[:, :3].min(axis=0), (vertices*1000).min(axis=0), atol=3e-6)
    np.testing.assert_allclose(triangles[:, :3].max(axis=0), (vertices*1000).max(axis=0), atol=3e-6)
    assert meta["analytic_pair_residual_mm"] < 1e-12
    assert meta["nominal_separation_mm"] == pytest.approx(46.4)
    assert meta["stop_scale"] == pytest.approx(1.)


def test_changed_height_and_gap_move_both_stop_surfaces_to_same_seating_plane():
    values = dict(designer.PRESETS["narrow"], guide_height_mm=22., guide_axial_clearance_mm=.75)
    meta = designer.preview_metadata(values)
    base = meta["base_height_mm"]
    original_stop_top = designer.ORIGINAL["mating_site_z_m"] * 1000
    lower_stop_top = base+(original_stop_top-base)*meta["stop_scale"]
    upper_stop_bottom = meta["nominal_separation_mm"] - lower_stop_top
    assert lower_stop_top == pytest.approx(upper_stop_bottom)
    assert 2*lower_stop_top == pytest.approx(meta["nominal_separation_mm"])
    assert meta["analytic_pair_residual_mm"] < 1e-12
    assert any("重建止挡" in text for text in meta["warnings"])


@pytest.mark.parametrize("value", [float("nan"), float("inf"), "4", True, 40.])
def test_invalid_numeric_parameters_rejected(value):
    with pytest.raises(ValueError):
        designer.validate_parameters({"guide_tip_half_angle_deg": value})


def test_radial_transition_and_unknown_fields_are_rejected():
    for values in [{"not_a_parameter": 1}, {"radial_crest_drop_mm": 2},
                   {"radial_lead_width_mm": .5, "guide_edge_round_mm": 1.}]:
        with pytest.raises(ValueError):
            designer.validate_parameters(values)


def test_extreme_valid_shapes_remain_closed_and_have_positive_mass():
    for height, angle, width, edge in [(10., 1., 0., 2.), (24., 15., 8., .25)]:
        values = dict(designer.PRESETS["narrow"], guide_height_mm=height,
                      guide_tip_half_angle_deg=angle, radial_lead_width_mm=width,
                      radial_crest_drop_mm=5. if width else 0., guide_edge_round_mm=edge)
        p = designer.full_parameters(values)
        mesh = designer.geometry.guide_mesh(0., p, 97, 7)
        _, faces = mesh
        edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        assert np.all(counts == 2)
        assert designer.geometry.mesh_moments(mesh, p["density_kg_m3"])[0] > 0
        assert designer.preview_metadata(values)["analytic_pair_residual_mm"] < 1e-12


def test_save_is_unique_unvalidated_and_keeps_geometry_provenance(tmp_path):
    first = designer.save_design(designer.PRESETS["narrow"], destination=tmp_path)
    before = first.read_bytes()
    second = designer.save_design(designer.PRESETS["narrow"], destination=tmp_path)
    assert first != second and first.read_bytes() == before
    record = json.loads(before)
    assert record["schema"] == "petal-guidance-design/v1"
    assert record["status"] == "UNVALIDATED_DESIGN"
    assert record["collision_model_generated"] is False
    assert record["manufacturing_cad_generated"] is False
    assert len(record["geometry_source_sha256"]) == 64
    assert record["design_parameters"] == designer.PRESETS["narrow"]
    with pytest.raises(ValueError):
        designer.save_design({}, {"opening_mm": -1}, destination=tmp_path)


@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setattr(designer, "SAVE_ROOT", tmp_path)
    # The default argument is only used for direct calls; HTTP saves use SAVE_ROOT explicitly.
    instance = ThreadingHTTPServer(("127.0.0.1", 0), designer.DesignerHandler)
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{instance.server_port}", tmp_path
    instance.shutdown()
    instance.server_close()
    thread.join(timeout=2)


def test_http_preview_validation_and_foreign_origin_rejection(server):
    base, _ = server
    with urlopen(base) as response:
        assert response.status == 200
        assert b"/app.js" in response.read()
    payload = json.dumps({"parameters": designer.PRESETS["narrow"]}).encode()
    request = Request(base+"/api/preview", data=payload, headers={"Content-Type": "application/json"})
    with urlopen(request) as response:
        meta, triangles = unpack(response.read())
        assert meta["vertex_count"] == len(triangles)
    request.add_header("Origin", "https://example.com")
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 403
    request = Request(base+"/api/preview", data=b'{"parameters":{"guide_height_mm":null}}',
                      headers={"Content-Type": "application/json"})
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 400


def test_http_save_and_download_stay_in_configured_directory(server):
    base, root = server
    request = Request(base+"/api/save", data=b'{"parameters":{}}',
                      headers={"Content-Type": "application/json"})
    with urlopen(request) as response:
        assert response.status == 201
        saved = json.load(response)
    assert Path(saved["path"]).parent == root
    with urlopen(base+saved["url"]) as response:
        assert json.load(response)["status"] == "UNVALIDATED_DESIGN"
    with pytest.raises(HTTPError) as error:
        urlopen(base+"/api/candidate/../../README.md")
    assert error.value.code == 404


@pytest.mark.parametrize("host", ["localhost:56343", "127.0.0.1:18766", "[::1]:56343"])
def test_ssh_forwarded_local_port_can_load_preview_and_save(server, host):
    base, root = server
    with urlopen(Request(base, headers={"Host": host})) as response:
        assert response.status == 200
    headers = {"Host": host, "Origin": "http://"+host, "Content-Type": "application/json"}
    with urlopen(Request(base+"/api/preview", data=b'{"parameters":{}}', headers=headers)) as response:
        meta, _ = unpack(response.read())
        assert meta["nominal_separation_mm"] == pytest.approx(46.4)
    with urlopen(Request(base+"/api/save", data=b'{"parameters":{}}', headers=headers)) as response:
        saved = json.load(response)
        assert Path(saved["path"]).parent == root


def test_local_reverse_proxy_keeps_forwarded_origin(server):
    base, _ = server
    headers = {"Origin": "http://localhost:56343", "X-Forwarded-Host": "localhost:56343",
               "Content-Type": "application/json"}
    with urlopen(Request(base+"/api/preview", data=b'{"parameters":{}}', headers=headers)) as response:
        assert response.status == 200


@pytest.mark.parametrize("host", ["example.com:8766", "localhost.example.com:8766",
                                  "example.com@localhost:8766", "localhost:bad",
                                  "localhost:70000", "localhost:8766/path"])
def test_nonlocal_or_malformed_forwarded_host_still_rejected(server, host):
    base, _ = server
    with pytest.raises(HTTPError) as error:
        urlopen(Request(base, headers={"Host": host}))
    assert error.value.code == 403


@pytest.mark.parametrize("origin,forwarded", [("http://localhost:56344", ""),
                                             ("https://example.com", "example.com"),
                                             ("null", ""),
                                             ("http://localhost:56343/path", "localhost:56343")])
def test_origin_restriction_survives_forwarded_port_fix(server, origin, forwarded):
    base, _ = server
    headers = {"Host": "localhost:56343", "Origin": origin, "Content-Type": "application/json"}
    if forwarded:
        headers["X-Forwarded-Host"] = forwarded
    with pytest.raises(HTTPError) as error:
        urlopen(Request(base+"/api/save", data=b'{"parameters":{}}', headers=headers))
    assert error.value.code == 403
