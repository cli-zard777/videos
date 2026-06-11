// ================================================================
// mun_transfer.ks
// kOS (KerboScript)  –  Trans-Munar Injection  ›  Mun Orbit
//                        Insertion  ›  Circularisation
//                        Single stage  |  no STAGE calls
//
// How to use
//   From a stable circular equatorial Kerbin parking orbit, open
//   the kOS terminal and type:  run mun_transfer.
//
// Prerequisites
//   – Circular equatorial Kerbin parking orbit (kerbin_orbit.ks).
//   – ~900 m/s+ Δv remaining (TMI ≈ 860 m/s, MOI ≈ 280 m/s,
//     circ ≈ 40 m/s from a 100 km Kerbin / 30 km Mun orbit).
//   – Maneuver nodes enabled in KSP difficulty settings.
// ================================================================

CLEARSCREEN.

// ── Configuration ────────────────────────────────────────────────
LOCAL MUN_ORBIT_ALT IS 30000.   // m – target circular Mun orbit altitude
LOCAL DO_WARP       IS TRUE.    // auto time-warp between manoeuvres

// ── Helpers ──────────────────────────────────────────────────────

// Average ISP of currently ignited engines.
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

// Tsiolkovsky burn-time estimate for a given Δv.
FUNCTION burn_time {
    PARAMETER dv.
    LOCAL F IS AVAILABLETHRUST.
    IF F <= 0 { RETURN 9999. }
    LOCAL ve IS avg_isp() * 9.80665.
    LOCAL m1 IS SHIP:MASS / (CONSTANT:E ^ (dv / ve)).
    RETURN (SHIP:MASS - m1) * ve / F.
}

// Execute a maneuver node, centering the burn on the node's ETA.
FUNCTION exec_node {
    PARAMETER nd.
    LOCAL bt IS burn_time(nd:DELTAV:MAG).
    PRINT "  Δv: " + ROUND(nd:DELTAV:MAG, 1) + " m/s  |  burn: " + ROUND(bt, 1) + " s".
    LOCK STEERING TO nd:BURNVECTOR.
    IF DO_WARP AND nd:ETA > bt / 2 + 60 {
        WARPTO(TIME:SECONDS + nd:ETA - bt / 2 - 30).
        WAIT 2.
    }
    WAIT UNTIL nd:ETA <= bt / 2.
    LOCK THROTTLE TO 1.0.
    UNTIL nd:DELTAV:MAG < 0.5 {
        LOCAL r IS nd:DELTAV:MAG.
        IF      r < 5  { LOCK THROTTLE TO 0.02. }
        ELSE IF r < 20 { LOCK THROTTLE TO 0.10. }
        ELSE IF r < 80 { LOCK THROTTLE TO 0.35. }
        WAIT 0.05.
    }
    LOCK THROTTLE TO 0.
    REMOVE nd.
}

// Prograde phase angle from ship to a target body (0–360°).
// Positive values mean the target is ahead of the ship in its orbit.
FUNCTION phase_to {
    PARAMETER tgt.
    LOCAL rs IS -BODY:POSITION.
    LOCAL rt IS tgt:POSITION - BODY:POSITION.
    LOCAL a  IS VANG(rs, rt).
    IF VDOT(VCRS(rs, rt), VCRS(rs, SHIP:VELOCITY:ORBIT)) < 0 {
        SET a TO 360 - a.
    }
    RETURN a.
}

// Required Mun lead angle at the moment of TMI burn for a Hohmann
// transfer: θ = 180° − ω_Mun × t_transfer.
FUNCTION mun_phase_needed {
    LOCAL rp IS SHIP:ALTITUDE + BODY:RADIUS.
    LOCAL rm IS MUN:ORBIT:SEMIMAJORAXIS.
    LOCAL t  IS CONSTANT:PI * SQRT(((rp + rm) / 2)^3 / BODY:MU).
    RETURN ((180 - (360 / MUN:ORBIT:PERIOD) * t + 360) MOD 360).
}

// Seconds until the next optimal TMI launch window.
FUNCTION eta_tmi_window {
    LOCAL gap  IS ((mun_phase_needed() - phase_to(MUN)) + 360) MOD 360.
    LOCAL wrel IS (360 / SHIP:ORBIT:PERIOD) - (360 / MUN:ORBIT:PERIOD).
    IF wrel <= 0 { RETURN 0. }
    RETURN gap / wrel.
}

// Δv for a Hohmann TMI burn from the current (assumed circular) orbit.
FUNCTION tmi_dv {
    LOCAL rp IS SHIP:ALTITUDE + BODY:RADIUS.
    LOCAL rm IS MUN:ORBIT:SEMIMAJORAXIS.
    LOCAL mu IS BODY:MU.
    RETURN SQRT(mu * (2/rp - 2/(rp + rm))) - SQRT(mu / rp).
}

// Circularisation Δv at the current apoapsis (vis-viva; works in any SOI).
FUNCTION circ_dv_at_apo {
    LOCAL ra IS SHIP:APOAPSIS  + BODY:RADIUS.
    LOCAL rp IS SHIP:PERIAPSIS + BODY:RADIUS.
    LOCAL mu IS BODY:MU.
    RETURN ABS(SQRT(mu / ra) - SQRT(mu * (2/ra - 2/(ra + rp)))).
}

// ================================================================
//  MAIN
// ================================================================
PRINT "╔════════════════════════════════════════╗".
PRINT "║   TRANS-MUNAR INJECTION  –  kOS        ║".
PRINT "╠════════════════════════════════════════╣".
PRINT "║  Target Mun orbit : " + MUN_ORBIT_ALT/1000 + " km          ║".
PRINT "╚════════════════════════════════════════╝".
PRINT "".

IF SHIP:BODY:NAME <> "Kerbin" {
    PRINT "ERROR: Must be in Kerbin orbit to run this script.".
    UNLOCK ALL.
} ELSE {
    SAS OFF. RCS OFF.
    LOCK STEERING TO PROGRADE.

    // ============================================================
    //  PHASE 1 – Trans-Munar Injection
    // ============================================================
    PRINT "[PHASE 1] Trans-Munar Injection".

    LOCAL dv_tmi  IS tmi_dv().
    LOCAL eta_win IS eta_tmi_window().

    PRINT "  Mun phase now   : " + ROUND(phase_to(MUN), 1) + "°".
    PRINT "  Mun phase needed: " + ROUND(mun_phase_needed(), 1) + "°".
    PRINT "  Window ETA      : " + ROUND(eta_win / 60, 1) + " min".
    PRINT "  TMI Δv          : " + ROUND(dv_tmi, 1) + " m/s".
    PRINT "".

    LOCAL nd_tmi IS NODE(TIME:SECONDS + eta_win, 0, 0, dv_tmi).
    ADD nd_tmi.
    exec_node(nd_tmi).

    PRINT "  TMI complete. Apoapsis: " + ROUND(SHIP:APOAPSIS / 1000, 0) + " km".
    PRINT "".

    // ============================================================
    //  PHASE 2 – Coast to Mun SOI
    // ============================================================
    PRINT "[PHASE 2] Coasting to Mun SOI…".
    LOCK STEERING TO PROGRADE.

    IF DO_WARP AND ETA:APOAPSIS > 180 {
        PRINT "  Warping " + ROUND(ETA:APOAPSIS / 3600, 1) + " hrs to arrival…".
        // Warp to ~90 s before the transfer-orbit apoapsis; SOI entry
        // happens near this point on a well-timed TMI burn.
        WARPTO(TIME:SECONDS + ETA:APOAPSIS - 90).
        WAIT 2.
    }

    WAIT UNTIL SHIP:BODY:NAME = "Mun".
    SET WARP TO 0.
    PRINT "  Mun SOI entered.".
    PRINT "  Periapsis: " + ROUND(SHIP:PERIAPSIS / 1000, 1) + " km".
    PRINT "".

    // ============================================================
    //  PHASE 3 – Mun Orbit Insertion  (retrograde burn at periapsis)
    // ============================================================
    PRINT "[PHASE 3] Mun Orbit Insertion".
    LOCK STEERING TO RETROGRADE.

    // Use a 350 m/s estimate to centre the MOI burn on periapsis.
    LOCAL est_bt IS burn_time(350).
    IF DO_WARP AND ETA:PERIAPSIS > est_bt + 90 {
        WARPTO(TIME:SECONDS + ETA:PERIAPSIS - est_bt / 2 - 30).
        WAIT 2.
    }
    WAIT UNTIL ETA:PERIAPSIS <= est_bt / 2 + 5.
    LOCK STEERING TO RETROGRADE.           // re-lock after warp
    WAIT UNTIL ETA:PERIAPSIS <= est_bt / 2.

    PRINT "  Burning retrograde to capture.".
    LOCK THROTTLE TO 1.0.

    // On the hyperbolic approach SHIP:APOAPSIS is negative (undefined).
    // Burn retrograde until the orbit is captured with apoapsis close
    // to the target altitude; throttle down progressively to avoid
    // overshooting into a dangerously low orbit.
    UNTIL SHIP:APOAPSIS > 0 AND SHIP:APOAPSIS < MUN_ORBIT_ALT * 3 {
        LOCAL apo IS SHIP:APOAPSIS.
        IF apo > 0 {
            IF      apo < MUN_ORBIT_ALT * 1.5 { LOCK THROTTLE TO 0.08. }
            ELSE IF apo < MUN_ORBIT_ALT * 2.0 { LOCK THROTTLE TO 0.25. }
            ELSE                               { LOCK THROTTLE TO 0.65. }
        }
        WAIT 0.05.
    }
    LOCK THROTTLE TO 0.

    PRINT "  Captured!".
    PRINT "  Apoapsis  : " + ROUND(SHIP:APOAPSIS  / 1000, 1) + " km".
    PRINT "  Periapsis : " + ROUND(SHIP:PERIAPSIS / 1000, 1) + " km".
    PRINT "".

    // ============================================================
    //  PHASE 4 – Circularise at apoapsis
    // ============================================================
    PRINT "[PHASE 4] Mun circularisation".
    LOCK STEERING TO PROGRADE.

    LOCAL cdv IS circ_dv_at_apo().
    LOCAL cbt IS burn_time(cdv).
    PRINT "  Circ Δv : " + ROUND(cdv, 1) + " m/s  |  burn: " + ROUND(cbt, 1) + " s".

    IF DO_WARP AND ETA:APOAPSIS > cbt + 60 {
        WARPTO(TIME:SECONDS + ETA:APOAPSIS - cbt / 2 - 30).
        WAIT 2.
    }
    WAIT UNTIL ETA:APOAPSIS <= cbt / 2 + 5.
    LOCK STEERING TO PROGRADE.
    WAIT UNTIL ETA:APOAPSIS <= cbt / 2.

    LOCK THROTTLE TO 1.0.
    UNTIL SHIP:PERIAPSIS >= MUN_ORBIT_ALT * 0.98 {
        LOCAL r IS circ_dv_at_apo().
        IF      r < 5  { LOCK THROTTLE TO 0.02. }
        ELSE IF r < 20 { LOCK THROTTLE TO 0.10. }
        ELSE IF r < 80 { LOCK THROTTLE TO 0.35. }
        WAIT 0.05.
    }
    LOCK THROTTLE TO 0.

    // ── Mission complete ──────────────────────────────────────────
    PRINT "".
    PRINT "╔════════════════════════════════════════╗".
    PRINT "║       MUN ORBIT ESTABLISHED            ║".
    PRINT "╠════════════════════════════════════════╣".
    PRINT "║  Apoapsis    : " + ROUND(SHIP:APOAPSIS  / 1000, 2) + " km     ║".
    PRINT "║  Periapsis   : " + ROUND(SHIP:PERIAPSIS / 1000, 2) + " km     ║".
    PRINT "║  Eccentricity: " + ROUND(ORBIT:ECCENTRICITY, 5) + "        ║".
    PRINT "╚════════════════════════════════════════╝".

    UNLOCK THROTTLE.
    UNLOCK STEERING.
    SAS ON.
}
