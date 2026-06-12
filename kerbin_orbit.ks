// ================================================================
// kerbin_orbit.ks  (v2 – Δv-optimal ascent)
// kOS  –  n-stage liquid rocket, circular equatorial Kerbin orbit
//
// How to use
//   Copy to Ships/Script/, open kOS terminal, type:  run kerbin_orbit.
//
// What changed from v1
//   The old script used a linear pitch schedule (altitude → pitch).
//   That forced a fixed attitude regardless of the actual velocity
//   vector, creating angle-of-attack drag and gravity losses.
//
//   v2 uses a NATURAL GRAVITY TURN:
//     1. Fly straight up until the kick speed is reached.
//     2. Apply a small one-time pitch kick toward east.
//     3. Lock steering to the surface-prograde vector and let
//        physics guide the pitch-over naturally.
//   Thrust stays aligned with the velocity vector for the entire
//   ascent, minimising both AoA drag and gravity losses.  The
//   profile also self-adapts: a high-TWR rocket pitches over fast,
//   a low-TWR rocket stays more vertical – no manual tuning needed.
//
//   Above 50 km the switch from surface-prograde to orbital prograde
//   is negligible (atmosphere is thin) but gives a cleaner lock.
//   Engines are cut once apoapsis is secured and the craft coasts
//   out of the atmosphere before the circularisation burn, avoiding
//   any Δv spent fighting residual drag.
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
PRINT "║  KERBIN ORBITAL LAUNCH  v2  –  kOS           ║".
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

// ================================================================
//  PHASE 5 – Circularisation burn (centred on apoapsis)
// ================================================================
// Re-sample Δv and burn time just before the burn window so the
// estimate uses the actual (post-coast) orbit and engine state.
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
