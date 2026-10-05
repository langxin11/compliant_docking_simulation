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


def test_saved_candidates_follow_refactored_runtime_layout():
    assert designer.SAVE_ROOT == designer.geometry.REPO_ROOT / "runs/designs/petal_guidance"


def unpack(packet):
    length = struct.unpack("<I", packet[:4])[0]
    return json.loads(packet[4:4+length]), np.frombuffer(packet, dtype="<f4", offset=4+length).reshape(-1, 6)


def test_preview_packet_contains_real_surface_in_mm_and_unit_normals():
    values = designer.PRESETS["narrow"]
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


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 10**400, "4", True, 40.])
def test_invalid_numeric_parameters_rejected(value):
    with pytest.raises(ValueError):
        designer.validate_parameters({"guide_tip_half_angle_deg": value})


def test_removed_radial_parameters_and_unknown_fields_are_rejected():
    for values in [{"not_a_parameter": 1}, {"radial_crest_drop_mm": 2},
                   {"radial_lead_width_mm": .5, "guide_edge_round_mm": 1.}]:
        with pytest.raises(ValueError):
            designer.validate_parameters(values)


def test_extreme_valid_shapes_remain_closed_and_have_positive_mass():
    for height, angle, width, edge in [(10., 1., 0., 2.), (24., 15., 0., .25)]:
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
    assert record["design_parameters"] == dict(designer.PRESETS["narrow"], **designer.FIXED_RADIAL)
    with pytest.raises(ValueError):
        designer.save_design({}, {"opening_mm": -1}, destination=tmp_path)


@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setattr(designer, "SAVE_ROOT", tmp_path)
    # The default argument is only used for direct calls; HTTP saves use SAVE_ROOT explicitly.
    instance = ThreadingHTTPServer(("127.0.0.1", 0), designer.DesignerHandler)
    instance.external_origins = {"http://localhost:56444"}
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
        assert meta["protocol"] == "petal-preview/v1"
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


def test_forwarded_headers_do_not_enable_proxy_mode(server):
    base, _ = server
    headers = {"Origin": "http://localhost:56343", "X-Forwarded-Host": "localhost:56343",
               "Content-Type": "application/json"}
    with pytest.raises(HTTPError) as error:
        urlopen(Request(base+"/api/preview", data=b'{"parameters":{}}', headers=headers))
    assert error.value.code == 403


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


@pytest.mark.parametrize("face", ["f -1 -2 -3", "f 0 2 3", "f 1 2 4", "f 1 2 3 1", "f 1 1 3", "f 1/1 2/1 3/1", "l 1 2"])
def test_preview_obj_rejects_unsupported_geometry(tmp_path, face):
    path = tmp_path / "bad.obj"
    path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\n" + face)
    with pytest.raises(ValueError):
        designer.read_preview_obj(path)


def test_preview_obj_existing_buffers_unchanged():
    import hashlib
    expected = {"adapter": "8a875ae547bff4792b87a61c52e090b147bc690910903cd02fe3d949af8bc83c", "head_plate": "3245c3f31d7b8fb8179c988a40d86b10b05952602bcba729b0c3c5cfc12ac65e", "stop_land": "cff997d00c6ac8aceed7499546dcd6ad5259d6d12a7578c03bc933ce241dd21a"}
    for name, digest in expected.items():
        assert hashlib.sha256(designer.base_buffer(name)).hexdigest() == digest


def test_atomic_publish_failure_leaves_no_partial_candidate(tmp_path, monkeypatch):
    def failure(*args):
        raise OSError("simulated disk failure")
    monkeypatch.setattr(designer.os, "link", failure)
    with pytest.raises(OSError):
        designer.save_design({}, destination=tmp_path)
    assert list(tmp_path.iterdir()) == []


def test_atomic_publish_retries_without_overwriting(tmp_path, monkeypatch):
    original = designer.os.link
    attempts = []
    def collision(source, destination):
        attempts.append(destination)
        if len(attempts) == 1:
            destination.write_text("existing")
        original(source, destination)
    monkeypatch.setattr(designer.os, "link", collision)
    saved = designer.save_design({}, destination=tmp_path)
    assert attempts[0].read_text() == "existing"
    assert saved == attempts[1]
    assert len(saved.stem.rsplit("_", 1)[1]) == 32
    assert not list(tmp_path.glob(".candidate_*"))


def test_bounded_preview(server):
    base, _ = server
    designer.PREVIEW_SLOTS.acquire()
    designer.PREVIEW_SLOTS.acquire()
    try:
        with pytest.raises(HTTPError) as error:
            urlopen(Request(base+"/api/preview", data=b'{"parameters":{}}', headers={"Content-Type": "application/json"}))
        assert error.value.code == 503
    finally:
        designer.PREVIEW_SLOTS.release()
        designer.PREVIEW_SLOTS.release()


def test_explicit_proxy_origin(server):
    base, _ = server
    headers = {"Origin": "http://localhost:56444", "Content-Type": "application/json"}
    with urlopen(Request(base+"/api/preview", data=b'{"parameters":{}}', headers=headers)) as response:
        assert response.status == 200
    headers["Origin"] = "http://localhost:56445"
    with pytest.raises(HTTPError) as error:
        urlopen(Request(base+"/api/preview", data=b'{"parameters":{}}', headers=headers))
    assert error.value.code == 403


def test_http_huge_parameters_and_save_io_errors(server, monkeypatch):
    base, _ = server
    payload = json.dumps({"parameters": {"guide_height_mm": 10**400}}).encode()
    with pytest.raises(HTTPError) as error:
        urlopen(Request(base+"/api/preview", data=payload, headers={"Content-Type": "application/json"}))
    assert error.value.code == 400
    def failure(*args, **kwargs):
        raise OSError("simulated save failure")
    monkeypatch.setattr(designer, "save_design", failure)
    with pytest.raises(HTTPError) as error:
        urlopen(Request(base+"/api/save", data=b'{"parameters":{}}', headers={"Content-Type": "application/json"}))
    assert error.value.code == 500
    assert "文件读写失败" in json.load(error.value)["error"]


def test_incomplete_body_times_out(server, monkeypatch):
    import socket
    from urllib.parse import urlsplit
    monkeypatch.setattr(designer, "READ_TIMEOUT_SECONDS", .1)
    base, _ = server
    port = urlsplit(base).port
    with socket.create_connection(("127.0.0.1", port), timeout=2) as connection:
        connection.sendall(f"POST /api/preview HTTP/1.0\r\nHost: 127.0.0.1:{port}\r\nContent-Type: application/json\r\nContent-Length: 16\r\n\r\n{{".encode())
        response = connection.recv(4096)
        assert b"408" in response


@pytest.mark.parametrize("faces", [[[0, 1, 2], [2, 1, 0]], [[0, 1, 2], [0, 1, 2]], [[0, 1, 2]]*3])
def test_clean_mesh_removes_entire_coincident_group(faces):
    vertices, cleaned = designer.geometry.clean_mesh(np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]), faces)
    assert len(vertices) == len(cleaned) == 0


def test_clean_mesh_quantizes_coordinates_and_filters_degenerate_area():
    vertices = np.array([[0., 0., 0.], [1.+4e-14, 0., 0.], [0., 1., 0.], [0., 1e-19, 0.]])
    cleaned_vertices, faces = designer.geometry.clean_mesh(vertices, [[0, 1, 2], [0, 1, 3]])
    assert len(faces) == 1
    np.testing.assert_array_equal(cleaned_vertices, [[0., 0., 0.], [0., 1., 0.], [1., 0., 0.]])


def test_parallel_saves_publish_complete_unique_records(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=4) as executor:
        paths = list(executor.map(lambda _: designer.save_design({}, destination=tmp_path), range(8)))
    assert len(set(paths)) == 8
    assert all(json.loads(path.read_text())["status"] == "UNVALIDATED_DESIGN" for path in paths)
    assert not list(tmp_path.glob(".candidate_*"))


def test_write_failure_cleans_temporary_file(tmp_path, monkeypatch):
    original = designer.tempfile.NamedTemporaryFile
    class FailedWrite:
        def __init__(self, **kwargs):
            self.handle = original(**kwargs)
            self.name = self.handle.name
        def __enter__(self):
            return self
        def __exit__(self, *args):
            self.handle.close()
        def write(self, raw):
            self.handle.write(raw[:20])
            raise OSError("simulated partial write")
    monkeypatch.setattr(designer.tempfile, "NamedTemporaryFile", FailedWrite)
    with pytest.raises(OSError):
        designer.save_design({}, destination=tmp_path)
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("face", ["f 1 2 3", "f 1/1 2/1 3/1", "f 1//1 2//1 3//1", "f 1/1/1 2/1/1 3/1/1"])
def test_preview_obj_accepts_supported_index_forms(tmp_path, face):
    path = tmp_path / "supported.obj"
    path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\n"+face)
    vertices, faces = designer.read_preview_obj(path)
    np.testing.assert_array_equal(faces, [[0, 1, 2]])
    assert np.isfinite(vertices).all()


@pytest.mark.parametrize("text", ["", "v nan 0 0", "v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3"])
def test_preview_obj_rejects_empty_nonfinite_and_zero_area(tmp_path, text):
    path = tmp_path / "invalid.obj"
    path.write_text(text)
    with pytest.raises(ValueError):
        designer.read_preview_obj(path)


def test_ui_defaults_and_presets_show_selected_narrow_shape(server):
    base, _ = server
    with urlopen(base+"/api/info") as response:
        info = json.load(response)
    assert set(info["presets"]) == {"narrow"}
    assert set(info["preset_labels"]) == {"narrow"}
    assert info["default"] == info["presets"]["narrow"]
    assert info["default"]["guide_tip_half_angle_deg"] == 1.
    assert info["default"]["guide_blend_fraction"] == .3
    assert designer.validate_parameters({}) == info["default"]


def test_selected_design_remains_available_without_radial_preset(tmp_path, monkeypatch):
    monkeypatch.setattr(designer, "PRESETS", dict(designer.PRESETS))
    monkeypatch.setattr(designer, "PRESET_LABELS", dict(designer.PRESET_LABELS))
    monkeypatch.setattr(designer, "DEFAULT_PARAMETERS", dict(designer.DEFAULT_PARAMETERS))
    path = designer.save_design({"guide_tip_half_angle_deg": 2.}, destination=tmp_path)
    designer.select_design(path)
    assert set(designer.PRESETS) == {"narrow", "selected"}
    assert designer.DEFAULT_PARAMETERS["guide_tip_half_angle_deg"] == 2.
    assert designer.DEFAULT_PARAMETERS["guide_blend_fraction"] == .3


def test_zero_radial_legacy_designs_load_but_nonzero_designs_are_rejected(tmp_path):
    values = dict(designer.DEFAULT_PARAMETERS, **designer.FIXED_RADIAL)
    assert designer.validate_parameters(values) == designer.DEFAULT_PARAMETERS
    for key in designer.FIXED_RADIAL:
        with pytest.raises(ValueError, match="功能已从交互工具移除"):
            designer.validate_parameters(dict(values, **{key: 1.}))
    path = tmp_path / "old_radial.json"
    path.write_text(json.dumps({"schema": "petal-guidance-design/v1", "design_parameters": dict(values, radial_lead_width_mm=8.)}))
    with pytest.raises(ValueError, match="功能已从交互工具移除"):
        designer.select_design(path)


def test_default_mesh_unchanged_after_removing_radial_controls(server):
    import hashlib
    base, _ = server
    with urlopen(base+"/api/info") as response:
        info = json.load(response)
    assert not (designer.FIXED_RADIAL.keys() & info["parameters"].keys())
    assert not (designer.FIXED_RADIAL.keys() & info["default"].keys())
    p = designer.full_parameters({})
    assert all(p[key] == 0 for key in designer.FIXED_RADIAL)
    mesh = designer.geometry.guide_mesh(0., p, 97, 7)
    assert hashlib.sha256(designer.triangle_buffer(mesh)).hexdigest() == "fbdc3f63cc159ae2c178ea5a7d5db83f6b1a86510e9af1b721806e67f7cdba06"
    assert (len(mesh[0]), len(mesh[1])) == (2688, 5372)
    assert designer.geometry.mesh_moments(mesh, p["density_kg_m3"])[0] == pytest.approx(.02812964069581886)
