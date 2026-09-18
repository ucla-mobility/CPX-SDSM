"""
Unit tests for deferred.PendingVerdicts: the delayed-judgment ledger for
uncorroborated other_only detections. Pure state machine, runs anywhere
(no numpy / shapely).

Lifecycle pinned here (see deferred.py):
  - clock starts at FIRST sighting: a sender's claim that an object exists is
    taken at face value the moment it makes it
  - held within grace          -> nothing charged, entry pending
  - corroborated in time       -> full per-frame back-pay as Correct
  - deadline expires           -> back-charge every held frame at pen(c), then
                                   each further uncorroborated frame is charged
                                   pen(c) instantly
  - track death                -> back-charge held frames as Incorrect IF the
                                   track outlived the deadline; a death inside
                                   the grace window is forgiven
  - vindication (ego-matched)  -> back-pay PRIOR held frames only
Each held frame settles exactly once (Correct xor Incorrect). Detections are fed
as (track_id, certainty, credit, support); most tests below use
credit == certainty (no kinematic discount). See deferred.py's _Entry docstring
for why certainty (drives INCORRECT amounts) and credit (drives CORRECT
amounts) are separate: a kinematically-implausible detection should earn less
reward if genuine, never a softer penalty if a ghost.
"""

from global_trust_perception.trust_calculations.consistency import (
    PENALTY_FLOOR_BETA as BETA,
    SUPPORT_THRESHOLD_THETA as THETA,
    T_DEADLINE_FLUSHES as DEADLINE,
    pen,
)
from global_trust_perception.trust_calculations.deferred import PendingVerdicts


def approx(a, b, tol=1e-9):
    return abs(a - b) < tol


def test_within_grace_charges_nothing():
    L = PendingVerdicts()
    for f in range(1, DEADLINE):          # stay strictly inside the window
        d = L.observe('a', f, [(7, 0.9, 0.9, 0.0)], [])
        assert d.correct == 0.0 and d.incorrect == 0.0 and d.pending == 1


def test_corroboration_within_grace_full_backpay():
    L = PendingVerdicts()
    for f in range(1, 4):                 # 3 held frames, no support
        L.observe('a', f, [(7, 0.9, 0.9, 0.0)], [])
    d = L.observe('a', 4, [(7, 0.9, 0.9, THETA)], [])   # support >= theta -> corroborated
    assert approx(d.correct, 0.9 * 4)     # 3 held + current frame back-paid
    assert d.incorrect == 0.0 and d.settled_correct == 1


def test_expiry_backcharges_all_held_frames():
    L = PendingVerdicts()
    total_inc = 0.0
    first_charge = None
    for f in range(1, DEADLINE + 3):
        d = L.observe('b', f, [(9, 1.0, 1.0, 0.0)], [])
        total_inc += d.incorrect
        if d.incorrect > 0 and first_charge is None:
            first_charge = f
    assert first_charge == 1 + DEADLINE   # entry made at f=1, expires at f=1+DEADLINE
    assert total_inc > 0                  # persistent ghost bled


def test_post_expiry_instant_charge_each_frame():
    L = PendingVerdicts()
    for f in range(1, 1 + DEADLINE):      # build up to just before expiry
        L.observe('b', f, [(9, 1.0, 1.0, 0.0)], [])
    d_expire = L.observe('b', 1 + DEADLINE, [(9, 1.0, 1.0, 0.0)], [])
    assert d_expire.incorrect > 0         # expiry back-charge
    d_next = L.observe('b', 2 + DEADLINE, [(9, 1.0, 1.0, 0.0)], [])
    assert approx(d_next.incorrect, pen(1.0))   # each later frame charged instantly


def test_track_death_inside_grace_is_forgiven():
    """A track that dies before its grace window closes is forgiven: dying does
    not make an uncorroborated report a fabrication, and a real tracker re-IDs
    often enough that charging young deaths would drain an honest sender."""
    L = PendingVerdicts()
    L.observe('c', 1, [(3, 0.6, 0.6, 0.0)], [])
    L.observe('c', 2, [(3, 0.6, 0.6, 0.0)], [])
    d = L.observe('c', 3, [], [])         # track 3 no longer reported -> death
    assert d.incorrect == 0.0
    assert d.settled_incorrect == 0


def test_track_death_past_deadline_backcharges():
    """A track that outlived the grace window and then vanished is charged on
    death -- the spawn-and-abandon ghost-cycling hole stays closed."""
    L = PendingVerdicts()
    for f in range(1, 1 + DEADLINE):      # lives right up to the deadline
        L.observe('c2', f, [(3, 0.6, 0.6, 0.0)], [])
    d = L.observe('c2', DEADLINE + 1, [], [])       # dies having lived DEADLINE
    n = DEADLINE                          # one held frame per observe() above
    assert approx(d.incorrect, n * BETA + (1 - BETA) * 0.6 * n)
    assert d.settled_incorrect == 1


def test_vindication_backpays_prior_frames_only():
    L = PendingVerdicts()
    L.observe('d', 1, [(5, 0.8, 0.8, 0.0)], [])
    L.observe('d', 2, [(5, 0.8, 0.8, 0.0)], [])   # sum_certainty = 1.6
    d = L.observe('d', 3, [], vindicated_tids=[5])   # now ego-matched
    assert approx(d.correct, 1.6)         # this frame credited by matched path
    assert d.incorrect == 0.0


def test_corroborated_on_first_sight_credits_current_frame():
    L = PendingVerdicts()
    d = L.observe('e', 1, [(2, 0.7, 0.7, THETA)], [])   # supported immediately
    assert approx(d.correct, 0.7) and d.pending == 0


def test_independent_agents_do_not_interfere():
    L = PendingVerdicts()
    L.observe('a', 1, [(1, 0.5, 0.5, 0.0)], [])
    d = L.observe('b', 1, [(1, 0.5, 0.5, THETA)], [])   # same track id, different agent
    assert approx(d.correct, 0.5)          # b's is corroborated; a's still pending


# --- corroboration after expiry stops the billing but refunds nothing -------

_C = 0.4                                 # pen(0.4) = 0.7, distinct from credit
_PROOF_F = 1 + DEADLINE + 4              # opens at f=1, expires, drains 3, then proof


def test_late_corroboration_stops_the_drain_without_refunding():
    """A track corroborated only AFTER expiry is credited for the settling
    frame alone: the expiry back-charge and every per-frame drain since stand.
    Sitting a whole grace window uncorroborated is the finding, and later
    proof does not unmake it."""
    L = PendingVerdicts()
    net = 0.0
    for f in range(1, _PROOF_F):
        d = L.observe('q', f, [(1, _C, _C, 0.0)], [])
        net += d.correct - d.incorrect
    assert net < 0                                          # charged, as intended

    d = L.observe('q', _PROOF_F, [(1, _C, _C, THETA)], [])  # proof arrives late
    assert approx(d.correct, _C)                            # this frame's credit ONLY
    assert d.incorrect == 0.0                               # but the billing stops
    assert net + d.correct < 0                              # still net-punished


def test_late_vindication_of_an_expired_track_pays_nothing():
    """Ego seeing the object after expiry likewise stops the billing without
    returning it -- the expired entry is dropped, not paid out."""
    L = PendingVerdicts()
    for f in range(1, DEADLINE + 4):
        L.observe('r', f, [(2, 1.0, 1.0, 0.0)], [])
    d = L.observe('r', DEADLINE + 4, [], vindicated_tids=[2])
    assert d.correct == 0.0
    assert d.settled_correct == 0


# --- credit (KDS-discounted) vs certainty (raw) are independent --------------

def test_low_credit_reduces_backpay_but_not_penalty():
    """A detection with certainty=1.0 but a heavily KDS-discounted credit
    (e.g. 0.2, as if other_score * kds) should back-pay only the discounted
    amount if corroborated -- but if it instead expires as a ghost, the
    penalty must be computed from the FULL raw certainty, not the discounted
    credit. Same held history, opposite fates, to isolate each amount."""
    # Fate A: corroborated -> back-pay uses credit, not certainty.
    L_good = PendingVerdicts()
    for f in range(1, 4):
        L_good.observe('j', f, [(4, 1.0, 0.2, 0.0)], [])   # 3 held frames
    d = L_good.observe('j', 4, [(4, 1.0, 0.2, THETA)], [])  # now corroborated
    assert approx(d.correct, 0.2 * 4)      # credit-based back-pay, NOT 1.0*4
    assert d.incorrect == 0.0

    # Fate B: same held history, but it expires as a ghost instead -> penalty
    # uses the RAW certainty (1.0), unaffected by the low credit. The loop
    # runs DEADLINE calls (n=DEADLINE held), then the (DEADLINE+1)-th call
    # itself also accumulates before the deadline check fires, so n=DEADLINE+1
    # at expiry (same off-by-one pattern as test_expiry_backcharges_all_held_frames).
    L_bad = PendingVerdicts()
    for f in range(1, 1 + DEADLINE):
        L_bad.observe('k', f, [(5, 1.0, 0.2, 0.0)], [])
    d_expire = L_bad.observe('k', 1 + DEADLINE, [(5, 1.0, 0.2, 0.0)], [])
    n = DEADLINE + 1
    assert approx(d_expire.incorrect, _backcharge_of(n=n, sum_certainty=1.0 * n))
    # Confirm this is strictly MORE than it would be if the penalty had
    # (wrongly) used credit instead of certainty -- KDS must never soften it.
    discounted_wrong = _backcharge_of(n=n, sum_certainty=0.2 * n)
    assert d_expire.incorrect > discounted_wrong


def _backcharge_of(n: int, sum_certainty: float) -> float:
    """Mirrors deferred._backcharge without importing a private function."""
    return n * BETA + (1.0 - BETA) * sum_certainty
