// ================================================================
// kerbin_orbit.ks  (v5)
// kOS  –  n-stage liquid rocket, circular equatorial Kerbin orbit
//
// How to use
//   Copy to Ships/Script/, open kOS terminal, type:  run kerbin_orbit.
//
// Changelog
//   v5 – Revert gravity turn to original controlled pitch schedule
//        (linear 90°→0° from TURN_START to TURN_END altitude).
//        Add auto-staging inside the circularisation burn so a
//        stage boundary mid-burn is handled gracefully.
//   v4 – Circularisation steers via maneuver node BURNVECTOR
//        instead of raw PROGRADE (prevents apoapsis creep).
//   v3 – Apoapsis timing check + correction burn for low-TWR craft.
//   v2 – Natural gravity turn (follow prograde).
// ================================================================

CLEARSCREEN.

// ── Configuration ────────────────────────────────────────────────
LOCAL TARGET_ALT     IS 100000.  // m – target circular orbit altitude
LOCAL TURN_START     IS  10000.  // m – begin pitching east
LOCAL TURN_END       IS  45000.  // m – pitch-over complete (0°)
LOCAL MAX_Q_THROTTLE IS    0.7.  // throttle cap through max-Q band
LOCAL MAX_Q_ALT_LO   IS  20000.  // m – start throttle reduction
LOCAL MAX_Q_ALT_HI   IS  35000.  // m – restore full throttle
LOCAL ATMO_ALT       IS  70000.  // m – coast above this before circularising

// ── Helpers ──────────────────────────────────────────────────────

FUNCTION engines_flameout {
    FOR eng IN SHIP:ENGINES {
        IF eng:IGNITION AND eng:FLAMEOUT { RETURN TRUE. }
    }
    RETURN FALSE.
}

FUNCTION avg_isp {
    LOCAL s IS 0. LOCAL n IS 0.
    FOR eng IN SHIP:ENGINES {
        IF eng:IGNITION AND NOT eng:FLAMEOUT {
            SET s TO s + eng:ISP. SET n TO n + 1.
        }
    }
    IF n = 0 { RETURN 300. }
    RETURN s / n.
}

// Linear pitch interpolation from 90° (vertical) to 0° (horizontal).
FUNCTION turn_pitch {
    IF SHIP:ALTITUDE <= TURN_START { RETURN 90. }
    IF SHIP:ALTITUDE >= TURN_END   { RETURN  0. }
    LOCAL frac IS (SHIP:ALTITUDE - TURN_START) / (TURN_END - TURN_START).
    RETURN 90 * (1 - frac).
}

// Throttle cap through the max-Q band.
FUNCTION safe_throttle {
    PARAMETER nominal.
    IF SHIP:ALTITUDE >= MAX_Q_ALT_LO AND SHIP:ALTITUDE <= MAX_Q_ALT_HI {
        RETURN MIN(nominal, MAX_Q_THROTTLE).
    }
    RETURN nominal.
}

// Circularisation Δv at current apoapsis (vis-viva).
FUNCTION circ_dv {
    LOCAL ra IS SHIP:APOAPSIS  + BODY:RADIUS.
    LOCAL rp IS SHIP:PERIAPSIS + BODY:RADIUS.
    LOCAL mu IS BODY:MU.
    RETURN ABS(SQRT(mu / ra) - SQRT(mu * (2/ra - 2/(ra + rp)))).
}

// Tsiolkovsky burn-time estimate.
FUNCTION burn_time {
    PARAMETER dv.
    LOCAL F IS AVAILABLETHRUST.
    IF F <= 0 { RETURN 9999. }
    LOCAL ve IS avg_isp() * 9.80665.
    LOCAL m1 IS SHIP:MASS / (CONSTANT:E ^ (dv / ve)).
    RETURN (SHIP:MASS - m1) * ve / F.
}

// ================================================================
//  PHASE 0 – Pre-launch
// ================================================================
PRINT "╔══════════════════════════════════════════════╗".
PRINT "║  KERBIN ORBITAL LAUNCH  v5  –  kOS           ║".
PRINT "╠══════════════════════════════════════════════╣".
PRINT "║  Target orbit : " + TARGET_ALT/1000 + " km circular              ║".
PRINT "║  Gravity turn : " + TURN_START/1000 + " km → " + TURN_END/1000 + " km                 ║".
PRINT "╚══════════════════════════════════════════════╝".
PRINT "".

SAS OFF. RCS OFF.
LOCK THROTTLE TO 0.
LOCK STEERING TO HEADING(90, 90).  // due east, straight up

FROM { LOCAL t IS 5. } UNTIL t = 0 STEP { SET t TO t-1. } DO {
    PRINT "  T-" + t + "…". WAIT 1.
}
PRINT "[T-0] IGNITION".
PRINT "".

// ================================================================
//  PHASE 1 – Vertical ascent to TURN_START
// ================================================================
STAGE.
LOCK THROTTLE TO 1.0.
WAIT UNTIL SHIP:ALTITUDE > 100.  // clear the pad

PRINT "[PHASE 1] Vertical ascent to " + TURN_START/1000 + " km".
WAIT UNTIL SHIP:ALTITUDE >= TURN_START.

// ================================================================
//  PHASE 2 – Gravity turn + staging
// ================================================================
PRINT "[PHASE 2] Gravity turn (" + TURN_START/1000 + " km → " + TURN_END/1000 + " km)".

UNTIL SHIP:APOAPSIS >= TARGET_ALT {

    // ── Pitch schedule ─────────────────────────────────────────
    LOCK STEERING TO HEADING(90, turn_pitch()).

    // ── Throttle (max-Q cap + apoapsis taper) ──────────────────
    LOCAL ratio IS SHIP:APOAPSIS / TARGET_ALT.
    LOCAL nominal IS 1.0.
    IF ratio > 0.90 {
        SET nominal TO MAX(0.05, (1.0 - ratio) * 10).
    }
    LOCK THROTTLE TO safe_throttle(nominal).

    // ── Staging ────────────────────────────────────────────────
    IF engines_flameout() AND STAGE:READY {
        PRINT "  [STAGE] Burnout – separating stage.".
        LOCK THROTTLE TO 0.
        WAIT 1.
        STAGE.
        WAIT UNTIL AVAILABLETHRUST > 0 OR NOT STAGE:READY.
        LOCK THROTTLE TO 1.0.
        PRINT "  [STAGE] Next stage ignition.".
    }

    WAIT 0.05.
}

// ================================================================
//  PHASE 3 – Cut engines, coast above atmosphere
// ================================================================
LOCK THROTTLE TO 0.
LOCK STEERING TO PROGRADE.

PRINT "[PHASE 3] Apoapsis secured – coasting.".
PRINT "  Apoapsis  : " + ROUND(SHIP:APOAPSIS  / 1000, 1) + " km".
PRINT "  Periapsis : " + ROUND(SHIP:PERIAPSIS / 1000, 1) + " km".

WAIT UNTIL SHIP:ALTITUDE >= ATMO_ALT.

LOCAL dv IS circ_dv().
LOCAL bt IS burn_time(dv).
PRINT "  Circ Δv   : " + ROUND(dv, 1) + " m/s".
PRINT "  Burn time : " + ROUND(bt,  1) + " s".
PRINT "  ETA apo   : " + ROUND(ETA:APOAPSIS, 0) + " s".

// ── Apoapsis timing check ────────────────────────────────────────
// The burn must be centred on apoapsis, requiring at least
// (burn_time/2 + 30 s) of lead time when the burn starts.
// If ETA is short (common for low-TWR rockets), fire a brief
// prograde correction to raise the apoapsis and extend the window.
// Guard: skip if already within 20 s of apoapsis – a prograde burn
// there raises periapsis instead.
LOCAL lead_needed IS bt / 2 + 30.

IF ETA:APOAPSIS < lead_needed AND ETA:APOAPSIS > 20 {
    PRINT "[TIMING] ETA " + ROUND(ETA:APOAPSIS, 0) + " s < required " + ROUND(lead_needed, 0) + " s.".
    PRINT "  Extending apoapsis…".
    LOCK STEERING TO PROGRADE.
    LOCK THROTTLE TO 0.2.
    WAIT UNTIL ETA:APOAPSIS > burn_time(circ_dv()) + 60
            OR SHIP:APOAPSIS > TARGET_ALT * 3.
    LOCK THROTTLE TO 0.
    SET dv TO circ_dv().
    SET bt TO burn_time(dv).
    PRINT "  New ETA: " + ROUND(ETA:APOAPSIS, 0) + " s  |  Apo: " + ROUND(SHIP:APOAPSIS / 1000, 1) + " km".
}

// ================================================================
//  PHASE 4 – Circularisation burn via maneuver node
// ================================================================
// Steer to nd:BURNVECTOR, NOT raw PROGRADE.  Raw PROGRADE drifts
// as the orbit evolves mid-burn and raises the apoapsis instead of
// holding it fixed while periapsis climbs.  BURNVECTOR always
// points toward the remaining Δv needed to hit the node's target
// orbit, so it self-corrects throughout the burn.

LOCAL nd IS NODE(TIME:SECONDS + ETA:APOAPSIS, 0, 0, dv).
ADD nd.

PRINT "[PHASE 4] Circularisation node created.".
PRINT "  Node Δv   : " + ROUND(nd:DELTAV:MAG, 1) + " m/s".
PRINT "  Node ETA  : " + ROUND(nd:ETA, 0) + " s".
PRINT "  Burn time : " + ROUND(burn_time(nd:DELTAV:MAG), 1) + " s".

LOCK STEERING TO nd:BURNVECTOR.
WAIT UNTIL nd:ETA <= burn_time(nd:DELTAV:MAG) / 2 + 5.
LOCK STEERING TO nd:BURNVECTOR.  // re-acquire after potential drift
WAIT UNTIL nd:ETA <= burn_time(nd:DELTAV:MAG) / 2.

PRINT "[PHASE 4] Circularisation – IGNITION.".
LOCK THROTTLE TO 1.0.

UNTIL nd:DELTAV:MAG < 0.5 {

    // ── Auto-staging mid-burn ─────────────────────────────────
    // If a stage is exhausted during the circularisation burn,
    // cut throttle briefly, fire the decoupler/next stage, wait
    // for new engines to spool up, then re-lock the burn vector
    // and continue.  Without this the burn stops silently and
    // leaves the orbit incomplete.
    IF engines_flameout() AND STAGE:READY {
        PRINT "  [STAGE] Stage boundary during circ burn.".
        LOCK THROTTLE TO 0.
        WAIT 0.3.
        STAGE.
        WAIT UNTIL AVAILABLETHRUST > 0 OR NOT STAGE:READY.
        LOCK STEERING TO nd:BURNVECTOR.
        LOCK THROTTLE TO 1.0.
        PRINT "  [STAGE] Resumed.".
    }

    LOCAL rem IS nd:DELTAV:MAG.
    IF      rem < 5  { LOCK THROTTLE TO 0.02. }
    ELSE IF rem < 20 { LOCK THROTTLE TO 0.10. }
    ELSE IF rem < 80 { LOCK THROTTLE TO 0.35. }
    ELSE             { LOCK THROTTLE TO 1.0.  }
    WAIT 0.05.
}
LOCK THROTTLE TO 0.
REMOVE nd.

// ================================================================
//  Complete
// ================================================================
PRINT "".
PRINT "╔══════════════════════════════════════════════╗".
PRINT "║           ORBIT ACHIEVED                     ║".
PRINT "╠══════════════════════════════════════════════╣".
PRINT "║  Apoapsis    : " + ROUND(SHIP:APOAPSIS  / 1000, 2) + " km             ║".
PRINT "║  Periapsis   : " + ROUND(SHIP:PERIAPSIS / 1000, 2) + " km             ║".
PRINT "║  Eccentricity: " + ROUND(ORBIT:ECCENTRICITY, 5) + "              ║".
PRINT "╚══════════════════════════════════════════════╝".

UNLOCK THROTTLE.
UNLOCK STEERING.
SAS ON.
