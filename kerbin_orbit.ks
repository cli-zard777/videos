// ================================================================
// kerbin_orbit.ks
// kOS (KerboScript) – 2-stage liquid rocket
// Target: circular equatorial orbit of Kerbin
//
// How to use
//   1. Copy this file into Ships/Script/ (or use the kOS editor).
//   2. Open the kOS terminal on your launch vehicle.
//   3. Type:  run kerbin_orbit.
//
// Assumptions
//   – Stage 1 : first-stage engines fired by the initial STAGE call.
//   – Stage 2 : second-stage engines are armed in the next action group
//               and are separated from stage 1 by at least one additional
//               STAGE event (decoupler/fairing/etc. before or after).
//   – The rocket is pointing straight up on the pad and the launch site
//     is at (or very near) the equator.
//   – Sufficient Δv exists in each stage for the chosen target altitude.
// ================================================================

CLEARSCREEN.

// ── Configuration ────────────────────────────────────────────────
LOCAL TARGET_ALT    IS 100000.  // m  – desired circular orbit altitude
LOCAL TURN_START    IS  10000.  // m  – begin pitching east
LOCAL TURN_END      IS  45000.  // m  – pitch-over complete (0 °)
LOCAL MAX_Q_THROTTLE IS    0.7. // throttle cap through max-Q band
LOCAL MAX_Q_ALT_LO  IS  20000.  // m  – start throttle reduction
LOCAL MAX_Q_ALT_HI  IS  35000.  // m  – restore full throttle
LOCAL COAST_MARGIN  IS      10. // s  – extra coast before circ burn

// ── Pretty header ─────────────────────────────────────────────────
PRINT "╔═══════════════════════════════════════════╗".
PRINT "║  KERBIN ORBITAL LAUNCH SEQUENCE  –  kOS   ║".
PRINT "╠═══════════════════════════════════════════╣".
PRINT "║  Target orbit : " + TARGET_ALT/1000 + " km circular           ║".
PRINT "║  Gravity turn : " + TURN_START/1000 + " km → " + TURN_END/1000 + " km              ║".
PRINT "╚═══════════════════════════════════════════╝".
PRINT "".

// ================================================================
//  Helper functions
// ================================================================

// Returns TRUE when at least one ignited engine has flamed out.
FUNCTION engines_flameout {
    FOR eng IN SHIP:ENGINES {
        IF eng:IGNITION AND eng:FLAMEOUT { RETURN TRUE. }
    }
    RETURN FALSE.
}

// Gravity-turn pitch angle (degrees above horizon) as a function of
// altitude.  Linear interpolation from 90° (vertical) to 0° (horizontal).
FUNCTION turn_pitch {
    IF SHIP:ALTITUDE <= TURN_START { RETURN 90. }
    IF SHIP:ALTITUDE >= TURN_END   { RETURN  0. }
    LOCAL frac IS (SHIP:ALTITUDE - TURN_START) / (TURN_END - TURN_START).
    RETURN 90 * (1 - frac).
}

// Circularisation Δv needed at current apoapsis (vis-viva).
FUNCTION circ_dv {
    LOCAL r_apo IS SHIP:APOAPSIS + BODY:RADIUS.
    LOCAL r_peri IS SHIP:PERIAPSIS + BODY:RADIUS.
    LOCAL a      IS (r_apo + r_peri) / 2.
    LOCAL mu     IS BODY:MU.
    LOCAL v_circ IS SQRT(mu / r_apo).
    LOCAL v_apo  IS SQRT(mu * (2 / r_apo - 1 / a)).
    RETURN MAX(v_circ - v_apo, 0).
}

// Estimated engine burn time for a given Δv using Tsiolkovsky.
FUNCTION burn_time {
    PARAMETER dv.
    LOCAL F IS AVAILABLETHRUST.
    IF F <= 0 { RETURN 9999. }
    // Average ISP across active engines
    LOCAL isp_sum IS 0.
    LOCAL eng_count IS 0.
    FOR eng IN SHIP:ENGINES {
        IF eng:IGNITION AND NOT eng:FLAMEOUT {
            SET isp_sum   TO isp_sum   + eng:ISP.
            SET eng_count TO eng_count + 1.
        }
    }
    IF eng_count = 0 { RETURN 9999. }
    LOCAL ve IS (isp_sum / eng_count) * 9.80665.
    LOCAL m0 IS SHIP:MASS.
    LOCAL m1 IS m0 / (CONSTANT:E ^ (dv / ve)).
    RETURN (m0 - m1) * ve / F.
}

// Throttle fraction for max-Q band.
FUNCTION safe_throttle {
    PARAMETER nominal.
    IF SHIP:ALTITUDE >= MAX_Q_ALT_LO AND SHIP:ALTITUDE <= MAX_Q_ALT_HI {
        RETURN MIN(nominal, MAX_Q_THROTTLE).
    }
    RETURN nominal.
}

// ================================================================
//  PHASE 0 – Pre-launch
// ================================================================
PRINT "[T-5] Pre-launch checks".
SAS OFF.
RCS OFF.
LOCK THROTTLE TO 0.
LOCK STEERING TO HEADING(90, 90).   // due east, straight up

FROM { LOCAL t IS 5. } UNTIL t = 0 STEP { SET t TO t - 1. } DO {
    PRINT "  T-" + t + "…".
    WAIT 1.
}
PRINT "[T-0] IGNITION – LIFTOFF".
PRINT "".

// ================================================================
//  PHASE 1 – Vertical ascent
// ================================================================
STAGE.                              // ignite stage-1 engines
LOCK THROTTLE TO 1.0.

WAIT UNTIL SHIP:ALTITUDE > 200.     // clear the launch clamps
PRINT "[PHASE 1] Vertical ascent to " + TURN_START/1000 + " km".

WAIT UNTIL SHIP:ALTITUDE >= TURN_START.

// ================================================================
//  PHASE 2 – Gravity turn + staging
// ================================================================
PRINT "[PHASE 2] Gravity turn  (" + TURN_START/1000 + " km → " + TURN_END/1000 + " km)".

UNTIL SHIP:APOAPSIS >= TARGET_ALT {

    // Pitch profile
    LOCAL pitch IS turn_pitch().
    LOCK STEERING TO HEADING(90, pitch).

    // Throttle management (max-Q reduction + final taper)
    LOCAL apo_ratio IS SHIP:APOAPSIS / TARGET_ALT.
    LOCAL nominal IS 1.0.
    IF apo_ratio > 0.90 {
        // Taper throttle as apoapsis approaches target to avoid overshoot.
        SET nominal TO MAX(0.05, (1.0 - apo_ratio) * 10).
    }
    LOCK THROTTLE TO safe_throttle(nominal).

    // ── Staging ──────────────────────────────────────────────────
    IF engines_flameout() AND STAGE:READY {
        PRINT "[STAGING] Stage-1 burnout detected.".
        LOCK THROTTLE TO 0.
        WAIT 1.                     // short separation coast
        STAGE.                      // fire decoupler / ignite stage 2
        PRINT "[STAGING] Stage-2 ignition.".
        WAIT UNTIL AVAILABLETHRUST > 0 OR STAGE:READY.
        LOCK THROTTLE TO 1.0.
    }

    WAIT 0.1.
}

// ================================================================
//  PHASE 3 – Cut throttle, coast to apoapsis
// ================================================================
LOCK THROTTLE TO 0.
PRINT "[PHASE 3] Apoapsis target reached – coasting.".
PRINT "  Current apoapsis  : " + ROUND(SHIP:APOAPSIS / 1000, 1) + " km".
LOCK STEERING TO PROGRADE.

LOCAL dv  IS circ_dv().
LOCAL bt  IS burn_time(dv).
LOCAL eta IS ORBIT:ETA:APOAPSIS.

PRINT "  Circularisation Δv : " + ROUND(dv, 1) + " m/s".
PRINT "  Estimated burn time: " + ROUND(bt, 1) + " s".
PRINT "  ETA to apoapsis    : " + ROUND(eta, 0) + " s".
PRINT "  Waiting for node…".

// Wait until half the burn time before apoapsis (centred burn).
WAIT UNTIL ORBIT:ETA:APOAPSIS <= (burn_time(circ_dv()) / 2) + COAST_MARGIN.

// ================================================================
//  PHASE 4 – Circularisation burn
// ================================================================
PRINT "[PHASE 4] Circularisation burn – IGNITION.".
LOCK STEERING TO PROGRADE.

WAIT UNTIL ORBIT:ETA:APOAPSIS <= burn_time(circ_dv()) / 2.
LOCK THROTTLE TO 1.0.

UNTIL SHIP:PERIAPSIS >= TARGET_ALT * 0.99 {
    LOCAL remaining IS circ_dv().
    // Throttle down for fine control near completion.
    IF      remaining < 5  { LOCK THROTTLE TO 0.02. }
    ELSE IF remaining < 20 { LOCK THROTTLE TO 0.10. }
    ELSE IF remaining < 80 { LOCK THROTTLE TO 0.40. }
    ELSE                   { LOCK THROTTLE TO 1.0.  }
    WAIT 0.05.
}

LOCK THROTTLE TO 0.

// ================================================================
//  Mission complete
// ================================================================
PRINT "".
PRINT "╔═══════════════════════════════════════════╗".
PRINT "║         ORBIT ACHIEVED                    ║".
PRINT "╠═══════════════════════════════════════════╣".
PRINT "║  Apoapsis   : " + ROUND(SHIP:APOAPSIS  / 1000, 2) + " km            ║".
PRINT "║  Periapsis  : " + ROUND(SHIP:PERIAPSIS / 1000, 2) + " km            ║".
PRINT "║  Eccentricity: " + ROUND(ORBIT:ECCENTRICITY, 5) + "              ║".
PRINT "╚═══════════════════════════════════════════╝".

UNLOCK THROTTLE.
UNLOCK STEERING.
SAS ON.
