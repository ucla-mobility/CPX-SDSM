"""
Unit tests for _RunningStat, the accumulator behind the agent's latency probe.

The probe runs on every flush for the life of the node, so it summarises
D_tx and W_batch without retaining the samples. That trade is only safe if
the summary is exact: these tests pin mean and max against the values a
retained series would have produced.

Requires sdsm_interfaces (container-only), same as the other pipeline tests.
"""

import statistics

import pytest

from global_trust_perception.pipeline.agent import _RunningStat


def test_empty_stat_reports_zero_rather_than_dividing():
    stat = _RunningStat()
    assert stat.count == 0
    assert stat.mean == 0.0
    assert stat.max == 0.0


def test_single_value():
    stat = _RunningStat()
    stat.add(0.25)
    assert (stat.count, stat.mean, stat.max) == (1, 0.25, 0.25)


def test_mean_and_max_match_the_retained_series():
    values = [0.004, 0.021, 0.013, 0.049, 0.002, 0.031]
    stat = _RunningStat()
    for v in values:
        stat.add(v)
    assert stat.count == len(values)
    assert stat.mean == pytest.approx(statistics.mean(values))
    assert stat.max == max(values)


def test_max_survives_a_later_smaller_value():
    """The peak is what a latency budget is argued from, so it must not be
    overwritten by whatever happened to arrive last."""
    stat = _RunningStat()
    for v in (0.05, 0.001):
        stat.add(v)
    assert stat.max == 0.05


def test_zero_is_a_real_sample_not_an_absence():
    """A loopback sender genuinely measures ~0 transmission latency; that has
    to count toward the mean rather than be skipped."""
    stat = _RunningStat()
    stat.add(0.0)
    stat.add(0.02)
    assert stat.count == 2
    assert stat.mean == pytest.approx(0.01)


def test_accumulates_without_retaining_samples():
    """Memory must not grow with the number of flushes -- the probe never
    stops running."""
    stat = _RunningStat()
    for i in range(10_000):
        stat.add(i / 10_000.0)
    assert stat.count == 10_000
    assert not any(isinstance(v, (list, tuple, set, dict))
                   for v in vars(stat).values())
