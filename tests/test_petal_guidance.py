"""Geometry pairing, closed guide solids and physical mass integration."""
import importlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from compliant_docking.collision_geometry import oriented_hull
from compliant_docking.docking_task import build_docking_trajectory
from compliant_docking.scene import load_scene

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"experiments"))
generate = importlib.import_module("prepare_petal_guidance")
study = importlib.import_module("petal_guidance_study")
suite = importlib.import_module("petal_insertion_suite")


def parameters(variant):
    info = json.loads((generate.SOURCE/"model_info.json").read_text())
    return generate.candidate_parameters(info["parameters"],variant)


def test_mass_integration_matches_analytic_cube_and_translated_origin():
    vertices = np.array([[x,y,z] for x in (0.,1.) for y in (0.,1.) for z in (0.,1.)])
    mass,first,inertia = generate.mesh_moments(oriented_hull(vertices),2.)
    assert mass == pytest.approx(2.)
    np.testing.assert_allclose(first,[1.,1.,1.],atol=1e-14)
    np.testing.assert_allclose(inertia,np.full((3,3),-.5)+np.eye(3)*(4/3+.5),atol=1e-14)
    moved = vertices+np.array([.2,-.7,1.1])
    m,f,moved_inertia = generate.mesh_moments(oriented_hull(moved),2.)
    c = f/m
    np.testing.assert_allclose(moved_inertia-m*(np.dot(c,c)*np.eye(3)-np.outer(c,c)),np.eye(3)/3,atol=1e-13)


@pytest.mark.parametrize("variant",generate.VARIANTS)
def test_surface_pairing_and_height_bounds_preserve_nominal_mate(variant):
    p = parameters(variant)
    r,a = np.linspace(32.,50.,81)[:,None],np.linspace(-360.,360.,2305)[None,:]
    h = generate.surface_height(r,a,p)
    np.testing.assert_allclose(h+generate.surface_height(r,45.-a,p),18.,atol=1e-12)
    assert h.min() >= 0. and h.max() <= 18.
    np.testing.assert_allclose(h,generate.surface_height(r,a+90.,p),atol=1e-12)


@pytest.mark.parametrize("variant",generate.VARIANTS)
def test_guide_mesh_is_closed_oriented_and_resolution_converges(variant):
    p = parameters(variant)
    coarse = generate.guide_mesh(0.,p,129,9)
    v,f = coarse
    edges = np.sort(np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]])),axis=1)
    _,count = np.unique(edges,axis=0,return_counts=True)
    assert np.all(count == 2)
    m,_,_ = generate.mesh_moments(coarse,2700.)
    fine = generate.guide_mesh(0.,p,257,17)
    mf,_,_ = generate.mesh_moments(fine,2700.)
    assert m > 0. and abs(m-mf) < 5e-6
    assert np.linalg.det(generate.mesh_moments(fine,2700.)[2]) > 0.


@pytest.mark.parametrize("geometry",study.GEOMETRIES)
def test_geometry_selection_retains_reference_and_all_control_parameters(geometry):
    base = load_scene("scenes/iiwa14_petal_insertion.yaml")
    candidate = study.geometry_scene(base,geometry)
    assert candidate.se3_impedance is base.se3_impedance and candidate.physics is base.physics
    left = suite.variant(base,"probe","lateral_released",(-.006,.006,15.))
    right = suite.variant(candidate,"probe","lateral_released",(-.006,.006,15.))
    ta = build_docking_trajectory(left.task,left.docking,left.trajectory)
    tb = build_docking_trajectory(right.task,right.docking,right.trajectory)
    for p,q in zip(ta.poses,tb.poses,strict=True):
        np.testing.assert_array_equal(p.homogeneous,q.homogeneous)
    np.testing.assert_array_equal(ta.times,tb.times)
