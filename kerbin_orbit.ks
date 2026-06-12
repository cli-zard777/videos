// ================================================================
// kerbin_orbit.ks  (v8)
// kOS  –  n-stage liquid rocket, circular equatorial Kerbin orbit
//
// How to use
//   Copy to Ships/Script/, open kOS terminal, type:  run kerbin_orbit.
//
// Changelog
//   v8 – Fix auto-staging in two places:
//        (1) Phase 1 (vertical ascent) had no staging loop at all –
//            it was a bare WAIT UNTIL that never checked for flameout.
//        (2) Both staging blocks used WAIT UNTIL AVAILABLETHRUST > 0
//            OR NOT STAGE:READY which exits immediately after STAGE.
//            because NOT STAGE:READY is true the instant staging fires,
//            so throttle was restored before new engines ignited.
//        Fixed by extracting do_stage() with a 5 s timeout wait on
//        AVAILABLETHRUST and adding a proper staging loop to Phase 1.
//   v7 – Fix LOCK STEERING before SAS OFF to eliminate attitude gap.
//   v6 – Remove forced RCS OFF (broke probe core attitude control).
//   v5 – Revert gravity turn to linear pitch schedule; add circ staging.
//   v4 – Circularisation steers via maneuver node BURNVECTOR.
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

// Separate a depleted stage and wait for the next engines to confirm
// thrust.  Uses a 5 s timeout so the script never hangs if the next
// stage has no engines (e.g. a coast / fairing stage).
// Do NOT use STAGE:READY as the wait condition – it goes true again
// almost immediately after STAGE. fires, causing throttle to be
// restored before the new engines have ignited.
FUNCTION do_stage {
    PRINT "  [STAGE] Burnout – separating stage.".
    LOCK THROTTLE TO 0.
    WAIT 0.5.                            // brief coast for clean separation
    STAGE.
    LOCAL t IS TIME:SECONDS.
    WAIT UNTIL AVAILABLETHRUST > 0 OR (TIME:SECONDS - t) > 5.
    IF AVAILABLETHRUST > 0 {
        LOCK THROTTLE TO 1.0.
        PRINT "  [STAGE] Next stage ignited.".
    } ELSE {
        PRINT "  [STAGE] WARNING: no thrust after stage – check vehicle.".
    }
}

// ================================================================
//  PHASE 0 – Pre-launch
// ================================================================
PRINT "╔══════════════════════════════════════════════╗".
PRINT "║  KERBIN ORBITAL LAUNCH  v8  –  kOS           ║".
PRINT "╠══════════════════════════════════════════════╣".
PRINT "║  Target orbit : " + TARGET_ALT/1000 + " km circular              ║".
PRINT "║  Gravity turn : " + TURN_START/1000 + " km → " + TURN_END/1000 + " km                 ║".
PRINT "╚══════════════════════════════════════════════╝".
PRINT "".

// Establish steering BEFORE dropping SAS so kOS takes authority
// with no gap – even one uncontrolled physics frame can start a spin.
LOCK THROTTLE TO 0.
LOCK STEERING TO HEADING(90, 90).  // due east, straight up
SAS OFF.

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

// Loop (not a bare WAIT) so flameout during vertical ascent is caught.
UNTIL SHIP:ALTITUDE >= TURN_START {
    IF engines_flameout() AND STAGE:READY { do_stage(). }
    WAIT 0.05.
}

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
    IF engines_flameout() AND STAGE:READY { do_stage(). }

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
LOCAL nd IS NODE(TIME:SECONDS + ETA:APOAPSIS, 0, 0, dv).
ADD nd.

PRINT "[PHASE 4] Circularisation node created.".
PRINT "  Node Δv   : " + ROUND(nd:DELTAV:MAG, 1) + " m/s".
PRINT "  Node ETA  : " + ROUND(nd:ETA, 0) + " s".
PRINT "  Burn time : " + ROUND(burn_time(nd:DELTAV:MAG), 1) + " s".

LOCK STEERING TO nd:BURNVECTOR.
WAIT UNTIL nd:ETA <= burn_time(nd:DELTAV:MAG) / 2 + 5.
LOCK STEERING TO nd:BURNVECTOR.
WAIT UNTIL nd:ETA <= burn_time(nd:DELTAV:MAG) / 2.

PRINT "[PHASE 4] Circularisation – IGNITION.".
LOCK THROTTLE TO 1.0.

UNTIL nd:DELTAV:MAG < 0.5 {

    // ── Auto-staging mid-burn ─────────────────────────────────
    IF engines_flameout() AND STAGE:READY {
        PRINT "  [STAGE] Stage boundary during circ burn.".
        LOCK THROTTLE TO 0.
        WAIT 0.5.
        STAGE.
        LOCAL st IS TIME:SECONDS.
        WAIT UNTIL AVAILABLETHRUST > 0 OR (TIME:SECONDS - st) > 5.
        IF AVAILABLETHRUST > 0 {
            LOCK STEERING TO nd:BURNVECTOR.
            LOCK THROTTLE TO 1.0.
            PRINT "  [STAGE] Circ burn resumed.".
        }
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
