# multi_z_home.py — Klipper extras module
#
# Provides HOME_Z_MOTORS gcode command that homes all Z steppers
# CONCURRENTLY, each stopping independently when its own endstop triggers.
#
# Installation:
#   cp multi_z_home.py ~/klipper/klippy/extras/multi_z_home.py
#   sudo service klipper restart
#
# Configuration (add to printer.cfg or z_axis.cfg):
#   [multi_z_home]
#   velocity: 10
#   accel: 100
#   distance: 450
#
# Usage (inside [homing_override] or any macro):
#   HOME_Z_MOTORS
#   HOME_Z_MOTORS VELOCITY=5 ACCEL=50 DISTANCE=450
#
# How it works:
#   1. All Z steppers are detached from kinematic control and each given
#      their own trapq (motion queue).
#   2. All three MCU endstop watchers are armed simultaneously.
#   3. Trapezoidal moves are appended to all three trapqs at the same
#      print_time, so they start concurrently on the MCU.
#   4. Because each endstop_pin is exclusively associated with its own
#      stepper in Klipper's trsync system, when PF2 triggers it stops
#      only stepper_z; PF1 stops only stepper_z1; PF0 stops only stepper_z2.
#   5. home_wait() is called for each endstop to confirm trigger and
#      collect any timeout errors.
#   6. All steppers are restored to kinematic control.
#
# After HOME_Z_MOTORS, call SET_KINEMATIC_POSITION Z=<position_endstop>
# to re-establish the kinematic Z reference.

import logging
import math
import chelper

ENDSTOP_SAMPLE_TIME  = .000015
ENDSTOP_SAMPLE_COUNT = 4
ENDSTOP_REST_TIME    = .001
STALL_TIME           = 0.100


class MultiZHome:
    def __init__(self, config):
        self.printer  = config.get_printer()
        self.velocity = config.getfloat('velocity', 10.,  above=0.)
        self.accel    = config.getfloat('accel',    100., minval=0.)
        self.distance = config.getfloat('distance', 450., above=0.)

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'HOME_Z_MOTORS',
            self.cmd_HOME_Z_MOTORS,
            desc="Concurrently home all Z motors to their own endstops")

        ffi_main, ffi_lib = chelper.get_ffi()
        self.ffi_main = ffi_main
        self.ffi_lib  = ffi_lib

        # Populated in _handle_connect
        self._pairs  = []  # [(stepper, mcu_endstop), ...]
        self._trapqs = []  # one independent trapq per stepper
        self._sks    = []  # one cartesian stepper-kinematics per stepper

    # ------------------------------------------------------------------
    # Startup: discover Z stepper ↔ endstop pairs from kinematics
    # ------------------------------------------------------------------

    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        ffi_main, ffi_lib = self.ffi_main, self.ffi_lib

        # Iterate every rail in the kinematics and collect Z steppers.
        # Works for CartKinematics and IDEX (which extends CartKinematics).
        rails = getattr(kin, 'rails', [])
        seen  = set()

        for rail in rails:
            for stepper in rail.get_steppers():
                name = stepper.get_name()
                # Only process stepper_z, stepper_z1, stepper_z2, …
                if not name.startswith('stepper_z') or name in seen:
                    continue
                seen.add(name)

                # Find the endstop that is exclusively associated with
                # this stepper (each extra Z stepper has its own endstop
                # object when endstop_pin is set in its config section).
                matched_endstop = None
                for mcu_endstop, es_name in rail.get_endstops():
                    if stepper in mcu_endstop.get_steppers():
                        matched_endstop = mcu_endstop
                        logging.info(
                            "multi_z_home: %s -> endstop '%s'",
                            name, es_name)
                        break

                if matched_endstop is None:
                    logging.warning(
                        "multi_z_home: %s has no exclusive endstop, skipping",
                        name)
                    continue

                # Allocate a dedicated trapq and stepper-kinematics object
                # so this stepper can be moved independently of the others.
                tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
                sk = ffi_main.gc(
                    ffi_lib.cartesian_stepper_alloc(b'z'), ffi_lib.free)

                self._pairs.append((stepper, matched_endstop))
                self._trapqs.append(tq)
                self._sks.append(sk)

        if not self._pairs:
            logging.warning(
                "multi_z_home: no Z stepper/endstop pairs found — "
                "ensure stepper_z1/z2 each have an endstop_pin configured")

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _calc_move_profile(distance, velocity, accel):
        """Return (accel_t, cruise_t, decel_t, peak_v) for a trap move."""
        if accel <= 0.:
            return 0., distance / velocity, 0., velocity
        accel_d = velocity * velocity / (2. * accel)
        if accel_d * 2. >= distance:
            # Triangular — can't reach full velocity
            velocity = math.sqrt(accel * distance)
            accel_d  = distance / 2.
        accel_t  = velocity / accel
        cruise_t = (distance - 2. * accel_d) / velocity
        return accel_t, cruise_t, accel_t, velocity

    # ------------------------------------------------------------------
    # Gcode command
    # ------------------------------------------------------------------

    def cmd_HOME_Z_MOTORS(self, gcmd):
        velocity = gcmd.get_float('VELOCITY', self.velocity, above=0.)
        accel    = gcmd.get_float('ACCEL',    self.accel,   minval=0.)
        distance = gcmd.get_float('DISTANCE', self.distance, above=0.)

        if not self._pairs:
            raise gcmd.error(
                "HOME_Z_MOTORS: no Z stepper/endstop pairs configured. "
                "Ensure stepper_z1/z2 each have an endstop_pin set.")

        toolhead = self.printer.lookup_object('toolhead')
        ffi_lib  = self.ffi_lib

        accel_t, cruise_t, decel_t, peak_v = self._calc_move_profile(
            distance, velocity, accel)
        move_t = accel_t + cruise_t + decel_t

        # ----------------------------------------------------------------
        # 1. Detach all Z steppers from kinematic control
        # ----------------------------------------------------------------
        toolhead.flush_step_generation()
        prev_sk = []
        prev_tq = []
        for i, (stepper, _) in enumerate(self._pairs):
            prev_sk.append(stepper.set_stepper_kinematics(self._sks[i]))
            prev_tq.append(stepper.set_trapq(self._trapqs[i]))
            stepper.set_position((0., 0., 0.))

        print_time = toolhead.get_last_move_time()

        # ----------------------------------------------------------------
        # 2. Arm all endstops simultaneously
        # ----------------------------------------------------------------
        for stepper, mcu_endstop in self._pairs:
            mcu_endstop.home_start(
                print_time,
                ENDSTOP_SAMPLE_TIME, ENDSTOP_SAMPLE_COUNT, ENDSTOP_REST_TIME)

        # ----------------------------------------------------------------
        # 3. Queue trapezoidal moves on all trapqs at the SAME print_time
        #    so they start concurrently on the MCU.
        #    axes_r = (0, 0, 1) → positive Z direction.
        # ----------------------------------------------------------------
        for tq in self._trapqs:
            ffi_lib.trapq_append(
                tq,
                print_time,
                accel_t, cruise_t, decel_t,  # time phases
                0., 0., 0.,                  # start position (x, y, z)
                0., 0., 1.,                  # direction — positive Z
                0., peak_v, accel)           # start_v, cruise_v, accel

        # ----------------------------------------------------------------
        # 4. Dwell long enough for all moves to complete / endstops to fire
        # ----------------------------------------------------------------
        toolhead.dwell(move_t + STALL_TIME)
        toolhead.flush_step_generation()

        # ----------------------------------------------------------------
        # 5. Confirm each endstop triggered; collect any timeout errors
        # ----------------------------------------------------------------
        home_end_time = print_time + move_t + STALL_TIME
        errors = []
        for stepper, mcu_endstop in self._pairs:
            try:
                mcu_endstop.home_wait(home_end_time)
            except Exception as e:
                errors.append("%s: %s" % (stepper.get_name(), e))

        # ----------------------------------------------------------------
        # 6. Restore all Z steppers to kinematic control
        # ----------------------------------------------------------------
        toolhead.flush_step_generation()
        for i, (stepper, _) in enumerate(self._pairs):
            stepper.set_stepper_kinematics(prev_sk[i])
            stepper.set_trapq(prev_tq[i])
            # Clean up the independent trapq
            ffi_lib.trapq_finalize_moves(
                self._trapqs[i], home_end_time + 100.)
        toolhead.flush_step_generation()

        if errors:
            raise gcmd.error(
                "HOME_Z_MOTORS — endstop(s) did not trigger:\n" +
                "\n".join(errors))

        gcmd.respond_info(
            "HOME_Z_MOTORS: all %d Z endstops triggered OK" %
            len(self._pairs))


def load_config(config):
    return MultiZHome(config)
