import pytest


def compute_d_safe(v, k=1.25, d_min=0.65):
    """Stage 3 safety distance formula: d_safe = k * v^2 + d_min."""
    return k * (v ** 2) + d_min


def calculate_costs(length, is_ramp, payload_kg, slope_mult=4.0, max_limit=35.0):
    """Stage 2 path cost formula balancing distance against slope penalty."""
    if not is_ramp:
        return length
    if payload_kg > max_limit:
        return 1e6
    elevation_rise = 2.8
    slope_cost = elevation_rise * slope_mult * (payload_kg / 15.0)
    return length + slope_cost


def test_safety_equation_at_rest():
    assert pytest.approx(compute_d_safe(0.0), 0.001) == 0.65


def test_safety_equation_at_speed():
    # At v=1.0 m/s: d_safe = 1.25 * 1.0^2 + 0.65 = 1.90m
    assert pytest.approx(compute_d_safe(1.0), 0.001) == 1.90


def test_adaptive_path_selection_light_payload():
    len_ramp = 34.0
    len_zigzag = 55.0

    # Light payload (15kg) -> Cost = 34.0 + (2.8 * 4.0 * 1.0) = 45.2 < 55.0
    cost_ramp = calculate_costs(len_ramp, is_ramp=True, payload_kg=15.0, slope_mult=4.0)
    cost_zigzag = calculate_costs(len_zigzag, is_ramp=False, payload_kg=15.0)

    assert cost_ramp < cost_zigzag


def test_adaptive_path_selection_heavy_payload():
    len_ramp = 34.0
    len_zigzag = 55.0

    # Heavy payload (30kg) -> Cost = 34.0 + (2.8 * 4.0 * 2.0) = 56.4 > 55.0
    cost_ramp = calculate_costs(len_ramp, is_ramp=True, payload_kg=30.0, slope_mult=4.0)
    cost_zigzag = calculate_costs(len_zigzag, is_ramp=False, payload_kg=30.0)

    assert cost_ramp > cost_zigzag


def test_adaptive_path_selection_overload_payload():
    len_ramp = 34.0
    len_zigzag = 55.0

    # Overloaded payload (>35kg) -> Infeasible for ramp
    cost_ramp = calculate_costs(len_ramp, is_ramp=True, payload_kg=40.0)
    cost_zigzag = calculate_costs(len_zigzag, is_ramp=False, payload_kg=40.0)

    assert cost_ramp == 1e6
    assert cost_ramp > cost_zigzag
