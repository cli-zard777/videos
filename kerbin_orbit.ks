// ================================================================
// kerbin_orbit.ks  (v3 – guaranteed circ burn window)
// kOS  –  n-stage liquid rocket, circular equatorial Kerbin orbit
//
// How to use
//   Copy to Ships/Script/, open kOS terminal, type:  run kerbin_orbit.
//
// v2 → v3 change
//   Added an apoapsis-timing check after the craft exits the
//   atmosphere.  The circularisation burn must be centred on
//   apoapsis, which requires at least (burn_time / 2 + margin)
//   seconds of lead time when the burn starts.  Low-TWR rockets
//   with long burn times can exit the atmosphere with less ETA
//   to apoapsis than they need.  If that happens, a short prograde
//   correction burn raises the apoapsis (and therefore the ETA)
//   until the window is wide enough.  A guard prevents the
//   correction from firing if the craft is already within 20 s of
//   apoapsis, where a prograde burn raises periapsis instead.
//
// v1 → v2 change
//   Replaced the prescribed linear pitch schedule with a natural
//   gravity turn (follow prograde after a small kick).  Keeps
//   thrust aligned with the velocity vector → minimum AoA drag
//   and gravity losses; self-adapts to any TWR.
// ================================================================

CLEARSCREEN.

// ── Configuration ────────────────────────────────────────────────
LOCAL TARGET_ALT IS 100000.  // m  – target circular orbit altitude
LOCAL KICK_SPEED IS 100.     // m/s surface speed to trigger gravity turn
LOCAL KICK_PITCH IS 80.      // °   – 80° = 10° from vertical (east)
LOCAL ATMO_ALT   IS 70000.   // m  – coast above this before circularising

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
PRINT "║  KERBIN ORBITAL LAUNCH  v3  –  kOS           ║".
PRINT "╠══════════════════════════════════════════════╣".
PRINT "║  Target orbit : " + TARGET_ALT/1000 + " km circular              ║".
PRINT "║  Turn trigger : " + KICK_SPEED + " m/s surface speed         ║".
PRINT "║  Kick pitch   : " + KICK_PITCH + "° (" + (90-KICK_PITCH) + "° from vertical)         ║".
PRINT "╚══════════════════════════════════════════════╝".
PRINT "".

SAS OFF. RCS OFF.
LOCK THROTTLE TO 0.
LOCK STEERING TO HEADING(90, 90).   // due east, straight up

FROM { LOCAL t IS 5. } UNTIL t = 0 STEP { SET t TO t-1. } DO {
    PRINT "  T-" + t + "…". WAIT 1.
}
PRINT "[T-0] IGNITION".
PRINT "".

// ================================================================
//  PHASE 1 – Vertical ascent
// ================================================================
STAGE.
LOCK THROTTLE TO 1.0.
WAIT UNTIL SHIP:ALTITUDE > 100.     // clear the pad

PRINT "[PHASE 1] Vertical ascent to " + KICK_SPEED + " m/s…".
WAIT UNTIL SHIP:VELOCITY:SURFACE:MAG >= KICK_SPEED.

// ================================================================
//  PHASE 2 – Gravity turn initiation (pitch kick + prograde lock)
// ================================================================
PRINT "[PHASE 2] Pitch kick → prograde lock".

// Apply the kick and wait for the rocket to physically rotate to
// that heading before handing control to the prograde vector.
// Cap the wait at 8 s so a very slow rocket doesn't stall here.
LOCK STEERING TO HEADING(90, KICK_PITCH).
LOCAL kick_deadline IS TIME:SECONDS + 8.
WAIT UNTIL VANG(SHIP:FACING:FOREVECTOR, HEADING(90, KICK_PITCH):FOREVECTOR) < 3
       OR  TIME:SECONDS > kick_deadline.

// Hand off – the prograde vector now guides the entire ascent.
LOCK STEERING TO SRFPROGRADE.

// ================================================================
//  PHASE 3 – Gravity turn ascent + staging
// ================================================================
PRINT "[PHASE 3] Following prograde – natural gravity turn".

UNTIL SHIP:APOAPSIS >= TARGET_ALT {

    // ── Staging ────────────────────────────────────────────────
    IF engines_flameout() AND STAGE:READY {
        PRINT "  [STAGE] Burnout detected – separating.".
        LOCK THROTTLE TO 0.
        WAIT 0.5.
        STAGE.
        WAIT UNTIL AVAILABLETHRUST > 0 OR NOT STAGE:READY.
        LOCK THROTTLE TO 1.0.
        PRINT "  [STAGE] Next stage ignition.".
    }

    // ── Steering reference ─────────────────────────────────────
    // Surface prograde and orbital prograde converge above ~50 km;
    // switch to the orbital frame for a cleaner lock in thin air.
    IF SHIP:ALTITUDE > 50000 {
        LOCK STEERING TO PROGRADE.
    } ELSE {
        LOCK STEERING TO SRFPROGRADE.
    }

    // ── Throttle taper as apoapsis closes on target ─────────────
    // Full power until 90 % of target, then linear ramp to near-
    // zero.  Prevents apoapsis overshoot without choking climb rate
    // prematurely.
    LOCAL ratio IS SHIP:APOAPSIS / TARGET_ALT.
    IF ratio >= 0.90 {
        LOCK THROTTLE TO MAX(0.03, (1.0 - ratio) / 0.10).
    } ELSE {
        LOCK THROTTLE TO 1.0.
    }

    WAIT 0.05.
}

// ================================================================
//  PHASE 4 – Cut engines, coast above atmosphere
// ================================================================
LOCK THROTTLE TO 0.
LOCK STEERING TO PROGRADE.

PRINT "[PHASE 4] Apoapsis secured – coasting out of atmosphere.".
PRINT "  Apoapsis  : " + ROUND(SHIP:APOAPSIS  / 1000, 1) + " km".
PRINT "  Periapsis : " + ROUND(SHIP:PERIAPSIS / 1000, 1) + " km".

// Burning inside the atmosphere wastes Δv on drag.  Wait until
// above ATMO_ALT before starting the circularisation timer.
WAIT UNTIL SHIP:ALTITUDE >= ATMO_ALT.

LOCAL dv IS circ_dv().
LOCAL bt IS burn_time(dv).
PRINT "  Circ Δv   : " + ROUND(dv, 1) + " m/s".
PRINT "  Burn time : " + ROUND(bt,  1) + " s".
PRINT "  ETA apo   : " + ROUND(ETA:APOAPSIS, 0) + " s".

// ── Apoapsis timing check ────────────────────────────────────────
// The burn must be centred on apoapsis, so it must START at
// (burn_time / 2) seconds before apoapsis.  If ETA:APOAPSIS is
// smaller than that required lead time we won't reach the start
// cue before the window closes — especially likely for low-TWR
// rockets with long burn times.
//
// Guard: skip correction if already within 20 s of apoapsis —
// a prograde burn there raises periapsis rather than apoapsis and
// would corrupt the orbit.  In that edge case just burn immediately
// (slightly off-centre) and let the eccentricity check tighten it.
LOCAL lead_needed IS bt / 2 + 30.  // half-burn + 30 s comfort margin

IF ETA:APOAPSIS < lead_needed AND ETA:APOAPSIS > 20 {
    PRINT "[TIMING] ETA " + ROUND(ETA:APOAPSIS, 0) + " s < required " + ROUND(lead_needed, 0) + " s.".
    PRINT "  Prograde correction burn – extending apoapsis ETA…".
    LOCK STEERING TO PROGRADE.
    LOCK THROTTLE TO 0.2.
    // Burn until there is enough lead time, or until apoapsis has
    // risen to 3× target (safety cap to prevent runaway burn).
    WAIT UNTIL ETA:APOAPSIS > burn_time(circ_dv()) + 60
            OR SHIP:APOAPSIS > TARGET_ALT * 3.
    LOCK THROTTLE TO 0.
    SET dv TO circ_dv().
    SET bt TO burn_time(dv).
    PRINT "  Correction done. ETA: " + ROUND(ETA:APOAPSIS, 0) + " s  |  Apo: " + ROUND(SHIP:APOAPSIS / 1000, 1) + " km".
    PRINT "  Updated circ Δv : " + ROUND(dv, 1) + " m/s  |  burn: " + ROUND(bt, 1) + " s".
}

// ================================================================
//  PHASE 5 – Circularisation burn (centred on apoapsis)
// ================================================================
// Re-sample burn time live so the trigger stays accurate as mass
// decreases during coast.
WAIT UNTIL ETA:APOAPSIS <= burn_time(circ_dv()) / 2 + 5.
LOCK STEERING TO PROGRADE.
WAIT UNTIL ETA:APOAPSIS <= burn_time(circ_dv()) / 2.

PRINT "[PHASE 5] Circularisation burn.".
LOCK THROTTLE TO 1.0.

UNTIL ORBIT:ECCENTRICITY < 0.005 AND SHIP:PERIAPSIS >= TARGET_ALT * 0.95 {
    LOCAL rem IS circ_dv().
    IF      rem < 5  { LOCK THROTTLE TO 0.02. }
    ELSE IF rem < 20 { LOCK THROTTLE TO 0.10. }
    ELSE IF rem < 80 { LOCK THROTTLE TO 0.35. }
    ELSE             { LOCK THROTTLE TO 1.0.  }
    WAIT 0.05.
}
LOCK THROTTLE TO 0.

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
