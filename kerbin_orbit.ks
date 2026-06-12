// ================================================================
// kerbin_orbit.ks  (v9 – GUI config + optimised)
// kOS  –  n-stage liquid rocket, circular equatorial Kerbin orbit
//
// How to use
//   Copy to Ships/Script/, open kOS terminal, type:  run kerbin_orbit.
//   A configuration window appears – set parameters, press LAUNCH.
//
// Changelog
//   v9 – Add kOS GUI for orbit/turn parameter selection before
//        launch.  Refactor helpers to accept parameters (no globals
//        shared between functions).  Consolidate staging into a
//        single do_stage() called from every phase.
//   v8 – Fix staging in vertical ascent (bare WAIT had no loop);
//        fix STAGE:READY race condition in wait-for-thrust logic.
//   v7 – Fix LOCK STEERING before SAS OFF (eliminate attitude gap).
//   v6 – Remove forced RCS OFF (broke probe core attitude control).
//   v5 – Revert gravity turn to linear pitch; add circ staging.
//   v4 – Circularisation steers via maneuver node BURNVECTOR.
//   v3 – Apoapsis timing check + correction burn for low-TWR craft.
// ================================================================

CLEARSCREEN.

// ================================================================
//  Helper functions
// ================================================================

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

// Pitch angle during gravity turn (degrees above horizon).
FUNCTION turn_pitch {
    PARAMETER ts. PARAMETER te.
    IF SHIP:ALTITUDE <= ts { RETURN 90. }
    IF SHIP:ALTITUDE >= te { RETURN  0. }
    RETURN 90 * (1 - (SHIP:ALTITUDE - ts) / (te - ts)).
}

// Throttle reduced during max-Q band.
FUNCTION safe_throttle {
    PARAMETER nominal.
    PARAMETER mq_cap.
    PARAMETER mq_lo.
    PARAMETER mq_hi.
    IF SHIP:ALTITUDE >= mq_lo AND SHIP:ALTITUDE <= mq_hi {
        RETURN MIN(nominal, mq_cap).
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

// Separate a depleted stage; wait up to 5 s for the next engines.
// Uses a timeout rather than STAGE:READY – STAGE:READY goes true
// again almost instantly after STAGE. fires, so it cannot be used
// as a reliable "wait for engines" signal.
FUNCTION do_stage {
    PRINT "  [STAGE] Burnout – separating stage.".
    LOCK THROTTLE TO 0.
    WAIT 0.5.
    STAGE.
    LOCAL t IS TIME:SECONDS.
    WAIT UNTIL AVAILABLETHRUST > 0 OR (TIME:SECONDS - t) > 5.
    IF AVAILABLETHRUST > 0 {
        LOCK THROTTLE TO 1.0.
        PRINT "  [STAGE] Next stage ignited.".
    } ELSE {
        PRINT "  [STAGE] WARNING: no thrust after staging.".
    }
}

// ================================================================
//  Configuration GUI
// ================================================================

FUNCTION show_config_gui {
    LOCAL win IS GUI(390).
    SET win:X TO 100.
    SET win:Y TO 80.

    // ── Title ────────────────────────────────────────────────────
    LOCAL ttl IS win:ADDLABEL("  KERBIN ORBITAL LAUNCH  v9  ").
    SET ttl:STYLE:ALIGN TO "CENTER".
    SET ttl:STYLE:FONTSIZE TO 13.
    win:ADDSPACING(6).

    // ── Target orbit altitude ────────────────────────────────────
    LOCAL r1 IS win:ADDHBOX().
    LOCAL l1 IS r1:ADDLABEL("Target Orbit Altitude (km):").
    SET l1:STYLE:WIDTH TO 230.
    LOCAL alt_field IS r1:ADDTEXTFIELD("100").
    SET alt_field:STYLE:WIDTH TO 80.

    win:ADDSPACING(4).

    // ── Gravity turn ─────────────────────────────────────────────
    LOCAL r2 IS win:ADDHBOX().
    LOCAL l2 IS r2:ADDLABEL("Gravity Turn Start (km):").
    SET l2:STYLE:WIDTH TO 230.
    LOCAL ts_field IS r2:ADDTEXTFIELD("10").
    SET ts_field:STYLE:WIDTH TO 80.

    LOCAL r3 IS win:ADDHBOX().
    LOCAL l3 IS r3:ADDLABEL("Gravity Turn End (km):").
    SET l3:STYLE:WIDTH TO 230.
    LOCAL te_field IS r3:ADDTEXTFIELD("45").
    SET te_field:STYLE:WIDTH TO 80.

    win:ADDSPACING(4).

    // ── Max-Q throttle ───────────────────────────────────────────
    LOCAL r4 IS win:ADDHBOX().
    LOCAL l4 IS r4:ADDLABEL("Max-Q Throttle Cap (%):").
    SET l4:STYLE:WIDTH TO 230.
    LOCAL maxq_field IS r4:ADDTEXTFIELD("70").
    SET maxq_field:STYLE:WIDTH TO 80.

    win:ADDSPACING(8).

    // ── Validation error label ───────────────────────────────────
    LOCAL err_lbl IS win:ADDLABEL("").
    SET err_lbl:STYLE:ALIGN TO "CENTER".
    SET err_lbl:STYLE:TEXTCOLOR TO RGB(1, 0.3, 0.3).

    win:ADDSPACING(4).

    // ── Buttons ──────────────────────────────────────────────────
    LOCAL btn_row IS win:ADDHBOX().
    btn_row:ADDSPACING(30).
    LOCAL launch_btn IS btn_row:ADDBUTTON("   LAUNCH   ").
    btn_row:ADDSPACING(20).
    LOCAL cancel_btn IS btn_row:ADDBUTTON("   CANCEL   ").

    win:SHOW().

    LOCAL cfg IS LEXICON("ok", FALSE).

    UNTIL cfg["ok"] OR cancel_btn:PRESSED {
        IF launch_btn:PRESSED {
            SET err_lbl:TEXT TO "".

            LOCAL t_alt   IS alt_field:TEXT:TONUMBER(-1)  * 1000.
            LOCAL t_start IS ts_field:TEXT:TONUMBER(-1)   * 1000.
            LOCAL t_end   IS te_field:TEXT:TONUMBER(-1)   * 1000.
            LOCAL t_maxq  IS maxq_field:TEXT:TONUMBER(-1) / 100.

            // Validate inputs
            IF t_alt < 0 {
                SET err_lbl:TEXT TO "Invalid target altitude.".
            } ELSE IF t_alt < 75000 {
                SET err_lbl:TEXT TO "Target altitude must be above 75 km (atmosphere).".
            } ELSE IF t_start < 0 OR t_end < 0 {
                SET err_lbl:TEXT TO "Invalid turn altitude.".
            } ELSE IF t_start >= t_end {
                SET err_lbl:TEXT TO "Turn start must be below turn end.".
            } ELSE IF t_maxq < 0 OR t_maxq > 1 {
                SET err_lbl:TEXT TO "Max-Q throttle must be between 1 and 100.".
            } ELSE {
                SET cfg["ok"]         TO TRUE.
                SET cfg["target_alt"] TO t_alt.
                SET cfg["turn_start"] TO t_start.
                SET cfg["turn_end"]   TO t_end.
                SET cfg["max_q"]      TO t_maxq.
            }
        }
        WAIT 0.
    }

    win:HIDE().
    win:DISPOSE().
    RETURN cfg.
}

// ================================================================
//  Main
// ================================================================

LOCAL cfg IS show_config_gui().

IF NOT cfg["ok"] {
    PRINT "Launch cancelled.".
} ELSE {

    LOCAL TARGET_ALT     IS cfg["target_alt"].
    LOCAL TURN_START     IS cfg["turn_start"].
    LOCAL TURN_END       IS cfg["turn_end"].
    LOCAL MAX_Q_THROTTLE IS cfg["max_q"].
    LOCAL MAX_Q_LO       IS 20000.
    LOCAL MAX_Q_HI       IS 35000.
    LOCAL ATMO_ALT       IS 70000.

    CLEARSCREEN.
    PRINT "╔══════════════════════════════════════════════╗".
    PRINT "║  KERBIN ORBITAL LAUNCH  v9  –  kOS           ║".
    PRINT "╠══════════════════════════════════════════════╣".
    PRINT "║  Target orbit  : " + TARGET_ALT/1000 + " km                      ║".
    PRINT "║  Gravity turn  : " + TURN_START/1000 + " km → " + TURN_END/1000 + " km              ║".
    PRINT "║  Max-Q cap     : " + ROUND(MAX_Q_THROTTLE*100,0) + "%                        ║".
    PRINT "╚══════════════════════════════════════════════╝".
    PRINT "".

    // ── Phase 0: Pre-launch ──────────────────────────────────────
    LOCK THROTTLE TO 0.
    LOCK STEERING TO HEADING(90, 90).
    SAS OFF.

    FROM { LOCAL t IS 5. } UNTIL t = 0 STEP { SET t TO t-1. } DO {
        PRINT "  T-" + t + "…". WAIT 1.
    }
    PRINT "[T-0] IGNITION".
    PRINT "".

    // ── Phase 1: Vertical ascent ─────────────────────────────────
    STAGE.
    LOCK THROTTLE TO 1.0.
    WAIT UNTIL SHIP:ALTITUDE > 100.

    PRINT "[PHASE 1] Vertical ascent to " + TURN_START/1000 + " km".
    UNTIL SHIP:ALTITUDE >= TURN_START {
        IF engines_flameout() AND STAGE:READY { do_stage(). }
        WAIT 0.05.
    }

    // ── Phase 2: Gravity turn ────────────────────────────────────
    PRINT "[PHASE 2] Gravity turn (" + TURN_START/1000 + " km → " + TURN_END/1000 + " km)".
    UNTIL SHIP:APOAPSIS >= TARGET_ALT {
        LOCK STEERING TO HEADING(90, turn_pitch(TURN_START, TURN_END)).
        LOCAL ratio IS SHIP:APOAPSIS / TARGET_ALT.
        LOCAL nominal IS 1.0.
        IF ratio > 0.90 { SET nominal TO MAX(0.05, (1.0 - ratio) * 10). }
        LOCK THROTTLE TO safe_throttle(nominal, MAX_Q_THROTTLE, MAX_Q_LO, MAX_Q_HI).
        IF engines_flameout() AND STAGE:READY { do_stage(). }
        WAIT 0.05.
    }

    // ── Phase 3: Coast above atmosphere ──────────────────────────
    LOCK THROTTLE TO 0.
    LOCK STEERING TO PROGRADE.
    PRINT "[PHASE 3] Coasting – apo " + ROUND(SHIP:APOAPSIS/1000,1) + " km".
    WAIT UNTIL SHIP:ALTITUDE >= ATMO_ALT.

    LOCAL dv IS circ_dv().
    LOCAL bt IS burn_time(dv).
    PRINT "  Circ Δv: " + ROUND(dv,1) + " m/s  burn: " + ROUND(bt,1) + " s  ETA: " + ROUND(ETA:APOAPSIS,0) + " s".

    // Apoapsis timing correction for low-TWR craft.
    IF ETA:APOAPSIS < bt/2 + 30 AND ETA:APOAPSIS > 20 {
        PRINT "[TIMING] Insufficient lead time – extending apoapsis…".
        LOCK STEERING TO PROGRADE.
        LOCK THROTTLE TO 0.2.
        WAIT UNTIL ETA:APOAPSIS > burn_time(circ_dv()) + 60
                OR SHIP:APOAPSIS > TARGET_ALT * 3.
        LOCK THROTTLE TO 0.
        SET dv TO circ_dv().
        SET bt TO burn_time(dv).
        PRINT "  New apo: " + ROUND(SHIP:APOAPSIS/1000,1) + " km  ETA: " + ROUND(ETA:APOAPSIS,0) + " s".
    }

    // ── Phase 4: Circularisation via maneuver node ────────────────
    LOCAL nd IS NODE(TIME:SECONDS + ETA:APOAPSIS, 0, 0, dv).
    ADD nd.
    PRINT "[PHASE 4] Circ node: " + ROUND(nd:DELTAV:MAG,1) + " m/s  ETA: " + ROUND(nd:ETA,0) + " s".

    LOCK STEERING TO nd:BURNVECTOR.
    WAIT UNTIL nd:ETA <= burn_time(nd:DELTAV:MAG) / 2 + 5.
    LOCK STEERING TO nd:BURNVECTOR.
    WAIT UNTIL nd:ETA <= burn_time(nd:DELTAV:MAG) / 2.

    PRINT "[PHASE 4] Circularisation – IGNITION.".
    LOCK THROTTLE TO 1.0.

    UNTIL nd:DELTAV:MAG < 0.5 {
        // Auto-staging mid-burn (inline to allow nd:BURNVECTOR re-lock).
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

    // ── Complete ─────────────────────────────────────────────────
    PRINT "".
    PRINT "╔══════════════════════════════════════════════╗".
    PRINT "║           ORBIT ACHIEVED                     ║".
    PRINT "╠══════════════════════════════════════════════╣".
    PRINT "║  Apoapsis    : " + ROUND(SHIP:APOAPSIS  /1000,2) + " km             ║".
    PRINT "║  Periapsis   : " + ROUND(SHIP:PERIAPSIS /1000,2) + " km             ║".
    PRINT "║  Eccentricity: " + ROUND(ORBIT:ECCENTRICITY,5) + "              ║".
    PRINT "╚══════════════════════════════════════════════╝".

    UNLOCK THROTTLE.
    UNLOCK STEERING.
    SAS ON.
}
