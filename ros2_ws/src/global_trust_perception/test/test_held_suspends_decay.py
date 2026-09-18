"""
A held detection must not drag reputation toward the baseline.

`N_total = C + I` excludes `held`, so a frame in which everything the agent
reported is still inside the deferred ledger's grace window scores
`N_total == 0` -- the same value as a frame in which the agent reported
nothing at all. Solution-C decay then pulled R toward REPUTATION_DEFAULT in
both cases, which penalised an agent for the pipeline's own adjudication
latency rather than for anything it did.

It scaled the wrong way, too: lengthening the grace window (so honest reports
with a unique vantage are not charged as ghosts too early) keeps detections
pending for longer, which produced MORE undecided frames and so pulled
reputation down harder. The observable signature was a sawtooth -- R rising
+0.1/frame on matches, then sagging asymptotically to exactly 0.5 and
flattening there, since the drift cannot cross its own baseline.

Pure math over `reputation_update`; no ROS, no sdsm_interfaces.
"""

from global_trust_perception.trust_calculations.reputation import (
    DECAY_RATE,
    REPUTATION_DEFAULT,
    reputation_update,
)


def test_undecided_frame_freezes_reputation():
    """Everything held, nothing scored: R must not move at all."""
    r_new, s_frame = reputation_update(1.0, C=0.0, I=0.0, N_total=0.0, held=4)

    assert r_new == 1.0
    assert s_frame == 0.0


def test_silent_frame_still_drifts_to_baseline():
    """The case Solution-C was written for is untouched: nothing held, decay."""
    r_new, s_frame = reputation_update(1.0, C=0.0, I=0.0, N_total=0.0, held=0)

    assert r_new == 1.0 + DECAY_RATE * (REPUTATION_DEFAULT - 1.0)
    assert r_new < 1.0
    assert s_frame == 0.0


def test_held_never_rescues_a_scored_frame():
    """held is only consulted when nothing was scored.

    A frame carrying real evidence is judged on that evidence; a pending
    detection alongside it must not suppress the penalty for an established
    mistake, which would give an attacker a way to mute its own bad frames
    by keeping something perpetually pending.
    """
    r_new, s_frame = reputation_update(0.8, C=0.0, I=2.0, N_total=2.0, held=99)

    assert s_frame == -1.0
    assert r_new < 0.8


def test_a_long_undecided_stretch_does_not_sag():
    """The regression itself: the sawtooth's falling edge is gone.

    340 frames is ~17 s at the replay flush rate, which took a saturated
    agent from 1.0 to ~0.68 before -- and to the 0.5 floor over a longer run.
    """
    r = 1.0
    for _ in range(340):
        r, _ = reputation_update(r, C=0.0, I=0.0, N_total=0.0, held=3)

    assert r == 1.0


def test_held_must_be_nonnegative():
    """A negative held is a caller bug, not something to interpret."""
    try:
        reputation_update(0.5, C=0.0, I=0.0, N_total=0.0, held=-1)
    except ValueError:
        return
    raise AssertionError('negative held should raise')
