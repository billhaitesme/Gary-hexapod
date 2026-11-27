"""DOOR GYM — Bill (2026-08-23): "maybe just have him practice the
door-kick / rear-up press / pop, so then when he finally gets to the door,
he's practiced before, and knows exactly how to open doors."

His spectator autopsy of the maze ladder: "he seemed to kind of lower his
front and then do a little charge at it, scooting his fronts and middles —
he never reared back and lunged, his feet never left the ground." The
choreography shipped untested; this gym is where it earns its look.

One door, no maze. Each drill is a rep with hard metrics:
  kick   - foot-lift height (did the leg actually leave the ground?),
           strike contact, door swing (deg)
  press  - body pitch ACHIEVED (is he visibly reared?), sustained swing
  pop    - net opening after N ram pulses
  ladder - the full sequence as the maze runs it

Run:  uv run --with "mujoco<3.12" --with numpy python door_gym.py
      ... --watch          (Bill's windows on the drills)
      ... --drill kick     (one drill only)
"""
import argparse, json, math, os, time
import numpy as np
import mujoco
import gary_mujoco as G

HERE = os.path.dirname(os.path.abspath(__file__))

TPC = dict(arc_s=0.8, rise_s=0.8, rc=0.014, mt=0.028,
           dy=0.028, v_hold=0.004, v_rise=0.016, v_sum=0.012,
           gather=0.010)
# rc/mt = Bill's WEDGE: rears crouch by rc, mids tall by mt — the
# stance itself is the ramp; walking in it drives him up the wall
TPC_BOUNDS = dict(arc_s=(0.5, 1.4), rise_s=(0.5, 1.6), rc=(0.0, 0.022),
                  mt=(0.0, 0.045), dy=(0.012, 0.04),
                  v_hold=(0.0, 0.014), v_rise=(0.006, 0.03),
                  v_sum=(0.004, 0.022),
                  gather=(0.0, 0.028))


def rp_of(q):
    w, x, y, z = q
    return (math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y - x*z)))))



class BreachFSM:
    """THE BREACH as a reusable phase machine -- the gym drill and the
    field navigator (nav_maze door ladder) drive this same object, so the
    choreography Bill certified in the gym IS what runs in the field.

    Caller per tick:  status = fsm.tick(t, drv, door_deg, yaw_err, log)
      door_deg() -> door hinge angle, degrees (abs)
      yaw_err(rev) -> signed yaw to APPLY (rad, + = CCW/omega+) to face
                      the door (rev=False) or dead away (rev=True); the
                      caller owns the door's heading so the FSM works on
                      any wall orientation
      log(tuple)  -> stage event sink, (round(t,1), text)
    Status: 'working' until terminal; then 'through' (door open, he has
    pressed through / come about) or 'defeated' (full tragedy performed,
    disengaged). Terminal status holds; caller resumes its own control.

    Full spec + Bill quotes: notes/BREACH-SPEC-2026-08-21.md. Phases:
    0 settle / 1 rear-up / 2 liberate / 3 SLAM / 4 assess / 31 re-cock /
    32 BUMP / 5 dismount / 6 forward re-jam / 40 look it up-and-down /
    41 probe-knock / 42 head shake / 43 turn away / 61 close tail-first /
    62 square-up / (1..4 again, reversed = the MULE-KICK) / 45 sad
    walk-off / 9 press through (spin + RUSH after a mule-kick win;
    knock-probe allowed on an ajar entry).

    Bill's order (2026-08-21, "i know im being pedantic"): two forward
    evolutions; then look it up and down, probe-knock it again, shake
    his head, THEN turn away and mule-kick it -- the walk-off doubles as
    the kick approach. "it most likely wont [open], but if it does, then
    he can spin around quickly, and rush toward the now open door."
    """
    OPEN_DEG = 16.0

    def __init__(self, t0, hint=None):
        # hint: the move that opened this door LAST time (Bill: "it
        # should just keep trying to use whatever got the door to open.
        # it doesn't need to cycle through all three tries every time")
        self.gainer = hint         # 'knock' | 'sweep' | 'shove' | 'slam'
        self._sweep_pre = None
        self._mount_pre = None
        self.ph, self.t0 = 0, t0
        self.start_t = t0
        self.slam_n = 0
        self.rev = False
        self.bumped = False
        self.knock_fired = False
        self.defeat_noise = False
        self.after_dismount = 6
        self.through_t = None
        self.ajar_knocks = 0
        self._probe_done = False
        self.best_deg = 0.0        # widest the door has ever been
        self.stuck_bar = None      # a stuck-ajar shove must beat this
        self.stuck_shoves = 0      # GAINLESS shoves (a shove that
        self._sweeps = 0           # shoulder-sweep attempts this door
        self._shove_pre = None     # gains >=6 deg resets the count)
        self._T0 = None
        self.status = 'working'

    def _wedge(self, drv, rp, rev):
        # ASS DOWN FACE UP toward the door -- mirrored when it's behind
        lo, mid, hi = -0.028 * rp, 0.042 * rp, 0.055 * rp
        if rev:
            drv.leg_h[1] = drv.leg_h[2] = lo    # fronts crouch
            drv.leg_h[0] = drv.leg_h[3] = mid
            drv.leg_h[4] = drv.leg_h[5] = hi    # rears tall
        else:
            drv.leg_h[4] = drv.leg_h[5] = lo
            drv.leg_h[0] = drv.leg_h[3] = mid
            drv.leg_h[1] = drv.leg_h[2] = hi
        drv.body_dy = (0.060 if rev else -0.060) * rp
        drv.ram_full = True     # hips allowed to their envelope

    def _spin_start(self, drv):
        # fast in-place spin cadence (Bill: "he really should be
        # turning-in-place a lot faster"): T 0.38 + the raised pure-spin
        # stroke clamp = ~56 deg/s, 3x the old rate. The caller's
        # cadence is restored the moment the spin ends.
        if self._T0 is None:
            self._T0 = drv.T
            drv.T = 0.38

    def _spin_end(self, drv):
        if self._T0 is not None:
            drv.T = self._T0
            self._T0 = None

    def _turn_to(self, drv, e, tol):
        # proportional stop, floor for the stepping gait
        if abs(e) <= tol:
            drv.omega = 0.0
            self._spin_end(drv)
            return True
        self._spin_start(drv)
        mag = max(0.8, min(3.5, 5.0 * abs(e)))
        drv.v = 0.0
        drv.omega = mag if e > 0 else -mag
        drv.lift = 0.014
        return False

    def tick(self, t, drv, door_deg, yaw_err, log, shove_pose=None):
        # shove_pose: optional callable -> (fwd_err, lat_err, yaw_err)
        # in body frame, toward a station squared onto the door panel's
        # CURRENT face (an ajar panel has rotated: position AND normal).
        # Field callers supply it; the gym passes None (its certified
        # geometry already connects).
        if self.status != 'working':
            return self.status
        # THE REACTIVE DOCTRINE (Bill: "anytime it opens, or swings open
        # wider, he resumes trying to get through it"): door movement is
        # an interrupt. Watched from every STANDING phase; the mounted
        # phases already route through assess, and cutting away mid-mount
        # would dump him off the wedge.
        _d_now = door_deg()
        if (self.ph in (0, 1, 6, 40, 41, 42, 43, 61, 62)
                and _d_now > max(8.0, self.best_deg + 5.0)):
            log((round(t, 1),
                 "door MOVED (%.0f deg) -> resume entry" % _d_now))
            drv.press = 0
            drv.leg_h = [0.0] * 6
            drv.body_dy = 0.0
            drv.body_pitch = 0.0
            drv.body_yaw = 0.0
            drv.height = 0.0
            drv.v = 0.0
            drv.vx = 0.0
            drv.omega = 0.0          # the interrupt can fire MID-TURN --
            self._spin_end(drv)      # without these he 'resumed entry'
            self.ph, self.t0 = 9, t  # carrying a full-rate spin (Bill,
            # live: 'gets doors partway open, then just spinning around
            # a lot' -- every 5-deg door creep re-fired the watch and
            # re-caught him mid-pirouette)
        self.best_deg = max(self.best_deg, _d_now)
        sgn = -1.0 if self.rev else 1.0             # dy mirror
        if self.ph == 0:                   # settle
            drv.v = 0.0
            if t - self.start_t > 0.8:
                if shove_pose is not None:
                    # SQUARE UP FIRST (Bill, watching the field: 'his
                    # first attempt at the door-attack is an air-ball.
                    # he full sends and misses -- he doesn't square up
                    # close to it to begin with, he does the second
                    # time'): the first mount was the only one that
                    # never stationed -- it trusted wherever the
                    # close-in creep stopped. Reuse the shove's
                    # pose-seek (ph 92) before the first rear-up.
                    self.ph, self.t0 = 92, t
                else:
                    self.ph, self.t0 = 1, t
        elif self.ph == 1:                 # rear up (pitch back)
            self.bumped = False
            drv.v = 0.0
            drv.vx = 0.0
            drv.omega = 0.0
            self._wedge(drv, min(1.0, (t - self.t0) / 1.4), self.rev)
            if t - self.t0 > 1.8:
                self.ph, self.t0 = 2, t
        elif self.ph == 2:                 # liberate onto the panel
            drv.press = 4 if self.rev else 1
            drv.press_hi = 0.032
            if t - self.t0 > 0.45:
                self.ph, self.t0 = 3, t
        elif self.ph == 3:                 # SLAM
            if self._mount_pre is None:
                self._mount_pre = door_deg()
            sp = min(1.0, (t - self.t0) / 0.30)
            ext = 0.020 * min(1.0, sp * 1.6)
            if self.rev:
                drv.leg_h[1] = drv.leg_h[2] = -0.028 + ext
            else:
                drv.leg_h[4] = drv.leg_h[5] = -0.028 + ext
            drv.leg_h[0] = drv.leg_h[3] = 0.042 + ext * 0.5
            drv.body_dy = sgn * (-0.060 + 0.122 * sp)
            if sp >= 1.0:
                self.slam_n += 1
                log((round(t, 1),
                     "SLAM %d%s" % (self.slam_n, " (REVERSED)" if self.rev else "")))
                self.ph, self.t0 = 4, t
        elif self.ph == 4:                 # assess -- a recognizable
            drv.body_dy = sgn * 0.050      # event, not a hope
            _bar = max(self.OPEN_DEG, self.stuck_bar or 0.0)
            if door_deg() > _bar:
                self.through_t = self.through_t or round(t, 1)
                self.stuck_bar = None
                log((round(t, 1),
                     'THROUGH -- door gave at %.1f deg' % door_deg()))
                self.ph, self.t0 = 9, t
            elif t - self.t0 > 0.7:
                if not self.bumped:
                    # Bill (watching): "his paws are still against the
                    # wall. you think he could press in, and then shove
                    # it again?" -- a second hit from the same mount is
                    # nearly free
                    self.bumped = True
                    self.ph, self.t0 = 31, t
                else:
                    _mg = door_deg() - (self._mount_pre or 0.0)
                    self._mount_pre = None
                    if self.rev:
                        self.after_dismount = 45   # mule-kick failed: done
                    elif _mg >= 6.0 and self.slam_n < 5:
                        # THE SLAMS ARE WORKING: keep slamming (Bill:
                        # "keep trying whatever got the door to open")
                        self.gainer = 'slam'
                        self.after_dismount = 6
                    elif self.slam_n < 2:
                        self.after_dismount = 6    # next forward evolution
                    else:
                        self.after_dismount = 40   # the third act
                    self.ph, self.t0 = 5, t
        elif self.ph == 31:                # RE-COCK, paws ON the panel
            cp = min(1.0, (t - self.t0) / 0.55)
            drv.body_dy = sgn * (0.050 - 0.110 * cp)
            if cp >= 1.0:
                self.ph, self.t0 = 32, t
        elif self.ph == 32:                # THE BUMP: shove it again
            sp = min(1.0, (t - self.t0) / 0.30)
            drv.body_dy = sgn * (-0.060 + 0.122 * sp)
            if sp >= 1.0:
                log((round(t, 1),
                     "BUMP %d (paws never left)" % self.slam_n))
                self.ph, self.t0 = 4, t
        elif self.ph == 5:                 # DISMOUNT -- come off the
            dp = t - self.t0               # wall the way he got on
            if dp < 0.4:
                drv.body_dy = sgn * 0.050 * (1.0 - dp / 0.4)
            elif dp < 1.1:
                drv.press = 0
                self._wedge(drv, max(0.0, 1.0 - (dp - 0.4) / 0.7), self.rev)
            elif dp < 1.6:                 # small unload-hop back --
                drv.leg_h = [0.0] * 6      # NOT a retreat (Bill: "he
                drv.body_dy = 0.0          # backs up too far on his
                drv.v = 0.045 if self.rev else -0.045   # second
                drv.lift = 0.014           # attempt... almost tip
            else:                          # over after contact")
                drv.v = 0.0
                drv.ram_full = False
                self.ph, self.t0 = self.after_dismount, t
        elif self.ph == 6:                 # forward re-jam: SLOW,
            if t - self.t0 < 0.7:          # no arrival momentum
                drv.v = 0.030
                drv.lift = 0.014
            else:
                drv.v = 0.0
                self.ph, self.t0 = 1, t
        elif self.ph == 40:                # THE THIRD ACT opens: look
            # it up and down (Bill's order: look, probe-knock, shake,
            # THEN turn away and mule-kick as the parting shot)
            drv.v = 0.0
            drv.omega = 0.0
            drv.lift = 0.0
            hs = t - self.t0
            drv.body_pitch = (0.30 * math.sin(3.14159 * hs / 1.1)
                              if hs < 1.1 else
                              -0.14 * math.sin(3.14159 * (hs - 1.1) / 0.5))
            if hs > 1.6:
                drv.body_pitch = 0.0
                log((round(t, 1), "LOOKS IT UP AND DOWN"))
                self.ph, self.t0 = 41, t
        elif self.ph == 41:                # PROBE-KNOCK: one more rap,
            drv.v = 0.0                    # watching for a give
            drv.lift = 0.0
            if t - self.t0 > 0.3 and not self._probe_done:
                self._probe_done = True
                drv.kick_leg = 1
                log((round(t, 1), "PROBE-KNOCK"))
            if door_deg() > max(self.OPEN_DEG, self.best_deg + 4.0):
                # the bar moves with best_deg: a door already ratcheted
                # to 42 must OPEN FURTHER to count, or the third act
                # cycles 'through -> not through' forever
                self.through_t = self.through_t or round(t, 1)
                log((round(t, 1),
                     'THROUGH -- the probe-knock did it (%.1f deg)' % door_deg()))
                self.ph, self.t0 = 9, t
            elif t - self.t0 > 1.7:
                self.ph, self.t0 = 42, t
        elif self.ph == 42:                # the head SHAKE, quick --
            drv.v = 0.0                    # feet planted, body wagging
            drv.omega = 0.0
            drv.lift = 0.0
            drv.height = -0.010
            drv.body_yaw = 0.35 * math.sin(2 * 3.14159 * (t - self.t0) / 0.55)
            if t - self.t0 > 1.0:
                drv.body_yaw = 0.0
                drv.height = 0.0
                log((round(t, 1), "HEAD SHAKE"))
                self.ph, self.t0 = 43, t
        elif self.ph == 43:                # TURN AWAY -- which is also
            # the mule-kick approach: he seems to give up, shows it his
            # tail... and bucks it as a parting shot
            if self._turn_to(drv, yaw_err(True), 0.05) or t - self.t0 > 8.0:
                self.rev = True
                log((round(t, 1), "TURNED AWAY -- the mule-kick is coming"))
                self.ph, self.t0 = 61, t
        elif self.ph == 45:                # sad walk-off: tail to the
            # door, head low, the defeat noise -- he never looks back
            hs = t - self.t0
            drv.press = 0
            drv.leg_h = [0.0] * 6
            drv.body_dy = 0.0
            if not self.defeat_noise:
                self.defeat_noise = True
                log((round(t, 1), "DEFEAT NOISE: sad descending tone"))
            if hs < 2.2:
                drv.body_pitch = -0.12 + 0.05 * hs   # head rising slowly
                drv.height = -0.014
                drv.v = 0.035                        # forward = away
                drv.lift = 0.014
            else:
                drv.body_pitch = 0.0
                drv.height = 0.0
                drv.v = 0.0
                log((round(t, 1), "WALKED OFF -- rep over"))
                self.status = 'defeated'
        elif self.ph == 61:                # close tail-first to jam
            # (Bill: "reduce the distance" -- the slam must CONNECT,
            # not fly: back in until the door stops him)
            if t - self.t0 < 2.4:
                drv.v = -0.035
                drv.lift = 0.014
            else:
                drv.v = 0.0
                self.ph, self.t0 = 62, t
        elif self.ph == 62:                # SQUARE-UP: the close can
            # drift the heading; re-true it at the door
            if self._turn_to(drv, yaw_err(True), 0.05) or t - self.t0 > 1.5:
                drv.omega = 0.0
                log((round(t, 1),
                     "SQUARED at door (err %.1f deg)" % math.degrees(yaw_err(True))))
                self.ph, self.t0 = 1, t
        elif self.ph == 9:                 # breached: press through
            drv.press = 0
            drv.leg_h = [0.0] * 6
            if self.rev:                   # mule-kick win: "he can spin
                # around quickly, and rush toward the now open door"
                if math.cos(yaw_err(False)) < 0.92:
                    self._spin_start(drv)
                    drv.v = 0.0
                    drv.omega = -3.5
                    drv.lift = 0.014
                else:
                    drv.omega = 0.0
                    self._spin_end(drv)
                    self.rev = False
                    log((round(t, 1), "SPUN AROUND -- rushing the open door"))
                    self.t0 = t
            else:
                drv.body_dy = 0.02
                drv.v = 0.045              # the rush -- DETERMINED:
                _ye9 = yaw_err(False)      # aim at the gap and drive
                drv.omega = max(-0.5, min(0.5, 1.2 * _ye9))
                drv.lift = 0.016
                # ajar on entry? "he is completely allowed to give it
                # another knock-probe as he enters... probe-knocks it
                # the rest of the way open"
                if (door_deg() < 45.0
                        and t - self.t0 > 0.9 + 1.3 * self.ajar_knocks
                        and drv.kick_leg is None):
                    # (knocks above ~45 deg whiff -- the panel has swung
                    # out of the strike zone; the shove finishes it)
                    self.ajar_knocks += 1
                    drv.v = 0.0
                    drv.kick_leg = 1
                    log((round(t, 1),
                         "AJAR (%.0f deg) -- knock-probe on entry" % door_deg()))
                if t - self.t0 > 2.8 and door_deg() > 55.0:
                    if self.stuck_shoves == 0 and self._shove_pre is not None:
                        if door_deg() - self._shove_pre >= 6.0 and self.gainer is None:
                            self.gainer = 'shove'
                    self.status = 'through'   # swung WIDE -- caller
                    # resumes with the gap actually clear (a 45-deg
                    # panel across a corridor cost 280 s of grinding in
                    # the field; Bill's ratchet doctrine finishes the
                    # job: probe till stuck, square up, shove, repeat)
                elif t - self.t0 > 4.5 and door_deg() < 55.0:
                    # STUCK ajar. Frame audit: entry rushes push him
                    # INTO the doorway -- the ajar panel sits beside his
                    # shoulder, so a re-mount slams along its face into
                    # air. First remedy is the SHOULDER SWEEP: lean into
                    # the panel and keep walking, sweeping it open as he
                    # passes. The square-up shove stays as the heavy-
                    # door fallback (gym-certified from a clean stance).
                    if self.gainer == 'shove' and shove_pose is not None:
                        # the shove opened it before -- straight there
                        log((round(t, 1),
                             "STUCK at %.0f deg -> the SHOVE worked before" % door_deg()))
                        self.ph, self.t0 = 91, t
                    elif shove_pose is not None and self._sweeps < 2:
                        self._sweeps += 1
                        self._sweep_pre = door_deg()
                        self._shove_pre = door_deg()
                        log((round(t, 1),
                             "STUCK at %.0f deg -> SHOULDER SWEEP" % door_deg()))
                        self.ph, self.t0 = 93, t
                    else:
                        if self._shove_pre is not None:
                            if door_deg() - self._shove_pre < 6.0:
                                self.stuck_shoves += 1   # gained nothing
                            else:
                                self.stuck_shoves = 0    # it's ratcheting
                        log((round(t, 1),
                             "STUCK at %.0f deg -> square up and SHOVE" % door_deg()))
                        self.ajar_knocks = 0
                        self.ph, self.t0 = 91, t
        elif self.ph == 91:                # square up for the shove
            if self._turn_to(drv, yaw_err(False), 0.05) or t - self.t0 > 1.5:
                drv.omega = 0.0
                if self.stuck_shoves >= 2: # two GAINLESS shoves: the
                    self.ph, self.t0 = 40, t   # third act
                elif shove_pose is not None:
                    self.ph, self.t0 = 92, t   # re-pose onto the panel
                else:
                    self.stuck_bar = door_deg() + 10.0
                    self._shove_pre = door_deg()
                    self.rev = False
                    self.ph, self.t0 = 1, t
        elif self.ph == 93:                # SHOULDER SWEEP: lean into
            # the panel, keep walking -- the door opens as he passes
            _fe, _le, _ye = shove_pose() if shove_pose is not None else (0.0, 0.0, 0.0)
            if door_deg() > 55.0:
                drv.vx = 0.0
                self.ph = 9
                self.t0 = t - 2.81         # 'through' fires immediately
            elif t - self.t0 > 6.0:
                drv.vx = 0.0
                if (self._sweep_pre is not None
                        and door_deg() - self._sweep_pre >= 4.0):
                    # the sweep is WORKING: repeat it, free of the cap
                    self.gainer = 'sweep'
                    self._sweeps = max(0, self._sweeps - 1)
                self.ph, self.t0 = 9, t    # reassess (routes by _sweeps)
            else:
                drv.v = 0.038
                drv.vx = (0.028 if _le > 0 else -0.028)
                drv.omega = max(-0.4, min(0.4, 1.0 * _ye))
                drv.lift = 0.016
        elif self.ph == 92:                # RE-POSE: seek the station
            # squared onto the panel's CURRENT face -- the panel rotated
            # toward its hinge, so past ~35 deg both the gap-centred
            # position AND the original heading produce glancing blows
            _fe, _le, _ye = shove_pose() if shove_pose is not None else (0.0, 0.0, 0.0)
            if ((abs(_fe) < 0.035 and abs(_le) < 0.035 and abs(_ye) < 0.14)
                    or t - self.t0 > 4.5):
                drv.v = 0.0
                drv.vx = 0.0
                drv.omega = 0.0
                self.stuck_bar = (door_deg() + 10.0
                                  if door_deg() > 14.0 else None)
                self._shove_pre = door_deg() if door_deg() > 14.0 else None
                self.rev = False
                self.ph, self.t0 = 1, t
            else:
                drv.lift = 0.014
                drv.v = max(-0.030, min(0.030, 1.8 * _fe))
                drv.vx = max(-0.030, min(0.030, 1.8 * _le))
                drv.omega = max(-0.9, min(0.9, 2.0 * _ye))
        return self.status


# THE CRAWL-IN GENES (Bill's laws are structure, these are the
# numbers; hand-tuning found the sled and the servo ceiling, the
# co-adapted constants are evolution's job -- project doctrine).
# CONCURRENT-CLIMB genes (the machine is Bill's spec -- strides,
# independent hand agents, mid conversion -- these are its numbers)
# SURGE-CYCLE genes (Bill's pounce choreography is the structure;
# these are its numbers): gather swing/lift, drive extension/height/
# surge, claw launch times + duration, cycle period, settle height,
# front pull/grip, mid-conversion gap
CRAWL = dict(v=0.020, lift=0.020, lean=0.042, h_base=0.010,
             h_k=0.90, p_base=0.06, p_k=0.9, gate1=0.165, gate2=0.140,
             gate3=0.118, gate4=0.100, promo_cool=2.2, demote_t=2.5,
             seen_t=0.5, reach_cap=0.085, ass_down=0.022,
             tall_f=0.004, f_lift=0.026, m_lift=0.014,
             supp_k=0.55, rear_k=0.45, conf_need=0.30, retry_t=3.0,
             slip_t=4.0, shift_gap=0.094, dig_y=0.020, dig_z=0.014,
             shift_v=0.026, shift_lean=0.062, beat_T=2.2, pull_z=0.014,
             hook_T=0.6,
             # the knee COMMAND envelope (deg interior) is a gene: Bill's
             # real knee straightens and inverts, the sim knee is free
             # (2026-08-22), and how far to extend is a torque trade-off
             # (0.176 N*m servos) for evolution to price, not a constant.
             # 101.7 = KNEE0+45 = the clamp every record was set under.
             knee_max=101.7, knee_climb=101.7, lunge_gate=0.34, pitch_max=1.0,
             hook_y=0.4245,
             # THE ENTRY (rear mount) as genes -- defaults = the hand-tuned
             # values of 2026-08-22, so nothing changes until evolution moves
             # them on the true (unclamped) Gary
             pose_gap=0.118, pose_v=0.014, pose_rear=-0.028, pose_mid=0.042,
             pose_front=0.055, pose_lean=-0.042, lib_v=0.024, ram_T=0.35,
             ram_h=0.022, ram_ext=0.020, ram_mid=0.028, ram_lean=0.084,
             pose_lift=0.0, lib_press=1.0, strike_T=0.8, settle_T=0.4,
             strike_y=0.418, hook_dx=0.045, col_dx=0.0, col_dy=0.0,
             body_dy=0.03, body_dz=0.015, body_T=2.5, body_gap=0.105, body_rest=1.5, col_d=0.075,
             adv_dy=0.04, strike_rung=1.0, stand_h=0.0, stand_T=0.5, pose_Tmax=8.0, col_c=0.0, mid_dy=0.03,
             strike_back=0.02, strike_out=0.04, incl_front=0.0, incl_mid=0.0, incl_rear=0.0, strike_gap=0.140,
             nod_on=0.0, nod_amp=0.02,
             sec_lean=0.042, sec_mid=0.042, sec_rear=-0.008,
             hook_clear=0.042,
             mid_clear=0.028)
_gp_c = os.environ.get("GARY_GENES") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "out", "crawl_genes.json")
# GARY_GENES=<file>: a FROZEN genome for hand probes while an ES is
# rewriting out/crawl_genes.json (2026-08-23: probes silently loaded the
# ES's moving incumbent for an hour).
if os.path.exists(_gp_c):
    CRAWL.update(json.load(open(_gp_c)))


def run_drill(drill="ladder", resist=0.02, verbose=True, frame_cb=None,
              cam_ctl=None, tele_cb=None, genes=None, locked=False,
              max_t=None):
    G.KNEE_CMD_MAX = float(CRAWL.get("knee_max", G.KNEE_CMD_MAX))   # knee gene -> IK clamp
    model = mujoco.MjModel.from_xml_string(G.build_xml(
        door_gym=(drill not in ("wall", "jump", "rear", "getup")),
        wall_gym=(drill in ("wall", "rear"))))
    data = mujoco.MjData(model)
    badr = int(model.jnt_qposadr[model.body("base").jntadr[0]])
    data.qpos[badr + 2] = G.COXA_Z
    data.qpos[badr + 1] = 0.235
    if drill == "getup":
        data.qpos[badr + 2] = 0.016    # spawn BELLY-DOWN     # strike-range spawn — the
    # first gym forgot the approach entirely: every strike hit air and
    # the door read 0.0 deg in all four drills
    if drill in ("wall", "jump", "rear", "getup"):
        dadr = None
    else:
        dj = model.joint("gymdoor_hinge")
        dadr = int(dj.qposadr[0])
        model.dof_frictionloss[int(dj.dofadr[0])] = resist
        if locked:
            # a REAL lock: the hinge itself blocked at ~2 deg. Friction
            # tiers can't stop a connected shove (a position-servo bump
            # is geometric displacement -- resist 3.0/4.5/6.0 all fell
            # identically to the donkey double-tap); only a lock can.
            model.jnt_range[int(dj.id)] = (-0.035, 0.035)
    # Bill's graded resistance: 0.02 = a kick pops it, 0.08 = needs
    # the press, 0.2 = the full pop. Success after each move is a
    # RECOGNIZABLE EVENT that ends the ladder early.
    foot1 = model.geom("foot1").id
    base_body = model.body("base").id
    floor_geom = model.geom("floor").id
    drv = G.GaitDriver()
    drv.foot_lock = None
    drv.v = drv.vx = 0.0; drv.omega = 0.0; drv.lift = 0.0
    renderer = None
    if frame_cb is not None:
        renderer = mujoco.Renderer(model, 400, 640)
    t, s = 0.0, 0
    wall0 = time.perf_counter()
    kick_peak_z = 0.0
    base_peak = 0.0
    base_sum, base_n = 0.0, 0
    nose_id = model.site('ld0').id
    nose_sum = 0.0
    air_s = 0.0
    all_feet = [model.geom(f'foot{i}').id for i in range(6)]
    door_peak = 0.0
    pitch_peak = 0.0
    tpc_state = {}
    belly_ticks, fault = 0, False
    rear_ph, rear_t0 = 0, 0.0
    slam_n, breach_done = 0, None
    defeat_noise = [False]
    breach_fsm = None          # THE BREACH lives in BreachFSM now
    asc_hold = {}              # PERSISTENT anchors: created once at plant
    getup_done = None
    stage_log = []

    def world_to_leg(leg, W):
        """world point -> (coxa, out, down) for the driver's leg IK: the
        REAL world anchor — transformed through the live body pose."""
        p = data.body("base").xpos
        Rm = data.body("base").xmat.reshape(3, 3)
        b = Rm.T @ (np.array(W) - p)
        mx, my = G.MOUNTS[leg]
        rx, ry = G.RADIAL[leg]
        dx, dy = b[0] - mx, b[1] - my
        out = float(np.hypot(dx, dy))
        down = float(-b[2])
        coxa = float(np.arctan2(dy, dx) - np.arctan2(ry, rx))
        while coxa > np.pi:
            coxa -= 2 * np.pi
        while coxa < -np.pi:
            coxa += 2 * np.pi
        return (coxa, out, down)

    rung_geoms = set()
    for _gi in range(model.ngeom):
        _gn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, _gi) or ""
        if _gn.startswith("rung"):
            rung_geoms.add(_gi)

    def foot_on_rung_n(leg, n):
        """TRUE only while this foot touches THE SPECIFIC rung n (bar
        or its lip) AND sits at that shelf's height. The any-rung test
        plus a level COUNTER produced the false summit: a foot on
        rung 1 with the counter at 4 scored 'bears on shelf 4' --
        Bill's eyes caught it on the reel."""
        _fg = model.geom(f"foot{leg}").id
        _tops = [0.053, 0.113, 0.173, 0.233]
        _fz = float(data.geom_xpos[_fg][2])
        # at or ABOVE the rung top: a 6 mm foot seated on the shelf has
        # its centre at top+0.006; allow 4 mm of settle below that and
        # 22 mm above. A foot pressed against the LIP FACE 1.6 cm under
        # the shelf passed the old +-22 mm band and read 'on rung 3'
        # (gatepath agent, 2026-08-23) -- the label law, again.
        _seat = _tops[n - 1] + 0.006
        if shin_on_rung_n(leg, n):
            return True                     # the shin hooks the rung
        if _fz < _seat - 0.004 or _fz > _seat + 0.022:
            return False
        _ok = set()
        for _nm in (f"rung{n}", f"rung{n}_lip"):
            try:
                _ok.add(model.geom(_nm).id)
            except Exception:
                pass
        for _ci in range(data.ncon):
            _c = data.contact[_ci]
            if ((_c.geom1 == _fg and _c.geom2 in _ok) or
                    (_c.geom2 == _fg and _c.geom1 in _ok)):
                return True
        return False

    def hook_x(leg):
        """x for a hook target: a natural offset outboard of this leg's
        coxa (not the foot's live x -- a hand that lands inboard would
        need 90+ deg of hip yaw to retreat, 2026-08-23)."""
        _cx = float(data.body(f"coxa{leg}").xpos[0])
        _bx = float(data.body("base").xpos[0])
        return _cx + math.copysign(CRAWL["hook_dx"], _cx - _bx)

    def shin_slabs(leg):
        out = []
        for k in range(8):
            try:
                out.append(model.geom(f"tibia{leg}_col{k}").id)
            except Exception:
                break
        return out

    _SHIN = {i: shin_slabs(i) for i in range(6)}

    def shin_contacts(leg):
        """(slab index, rung geom name) pairs for this leg's blade slabs
        touching any rung or lip right now."""
        out = []
        for _ci in range(data.ncon):
            _c = data.contact[_ci]
            for _a, _b in ((_c.geom1, _c.geom2), (_c.geom2, _c.geom1)):
                if _a in _SHIN[leg]:
                    _nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, _b) or ""
                    if _nm.startswith("rung"):
                        out.append((_SHIN[leg].index(_a), _nm))
        return out

    def shin_on_rung_n(leg, n):
        return any(nm in (f"rung{n}", f"rung{n}_lip")
                   for _, nm in shin_contacts(leg))

    def foot_on_rung(leg):
        """TRUE only while this foot is really touching a rung. The
        ascent's 'one leg at a time' was a stopwatch; this makes it a
        measured fact."""
        _fg = model.geom(f"foot{leg}").id
        for _ci in range(data.ncon):
            _c = data.contact[_ci]
            if ((_c.geom1 == _fg and _c.geom2 in rung_geoms) or
                    (_c.geom2 == _fg and _c.geom1 in rung_geoms)):
                return True
        return False

    def belly_now():
        # CHASSIS on floor (not shins — folded rear shins on the ground
        # polluted every earlier belly metric)
        for ci in range(data.ncon):
            c = data.contact[ci]
            if floor_geom in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == floor_geom else c.geom1
                if int(model.geom_bodyid[other]) == base_body:
                    return True
        return False

    def door_deg():
        if dadr is None:
            return 0.0
        return abs(math.degrees(float(data.qpos[dadr])))

    # the drill script: settle 1s, then the move(s)
    while t < (16.0 if drill == "ladder" else 14.0 if drill == "wall" else 4.0 if drill == "jump" else 90.0 if drill == "rear" else 5.0 if drill == "getup" else 95.0 if drill == "breach" else 8.0):
        if s % 5 == 0:
            m_pitch, _ = rp_of(data.body("base").xquat)
            pitch_peak = max(pitch_peak, m_pitch)
            # SIGNED, nose-up positive — settled EMPIRICALLY 2026-08-21
            # (commanded body_pitch +0.25 -> measured rp_of[0] = +0.227,
            # mast z rose): the abs() bandage from sign burn #6 let a
            # 22.6-deg NOSEDIVE score as a climb (sign burn #7, caught by
            # Bill watching motion: "the 22.6 degrees is in the wrong
            # direction"). Conventions get experiments now, not guesses.
            if drill in ("kick", "ladder") and abs(t - 1.0) < 0.006:
                drv.kick_leg = 1
                stage_log.append((round(t, 1), "KICK fired"))
            if drill in ("kick", "ladder") and 1.15 < t < 1.75:
                drv.body_dy = 0.022    # surge into the strike — the last
                # centimeter of reach lives in the body, not the leg
            elif drill == "kick":
                drv.body_dy = 0.0
            if drill == "breach":
                # THE BREACH -- the certified choreography lives in
                # BreachFSM (single source: the field navigator drives
                # the same class). Full spec + Bill quotes:
                # notes/BREACH-SPEC-2026-08-21.md
                import math as _m

                def _yaw_err(rev):
                    # signed yaw to APPLY (+ = CCW, verified empirically)
                    # to face the gym door at +y (rev=False) or dead away
                    # (rev=True)
                    R = data.body("base").xmat.reshape(3, 3)
                    if rev:
                        return _m.atan2(-R[0, 1], -R[1, 1])
                    return _m.atan2(R[0, 1], R[1, 1])

                if breach_fsm is None:
                    breach_fsm = BreachFSM(t)
                if breach_fsm.tick(t, drv, door_deg, _yaw_err,
                                   stage_log.append) != 'working':
                    break
            elif drill == "getup":
                # THE GET-UP (Bill: "he needs to be able to recover from
                # that as well. like, in general."): from chassis-down,
                # splay to get the feet under him, then push to stand.
                # The first skill a real animal learns; September's most
                # important reflex.
                if t < 0.6:
                    pass                          # limp — the collapse
                elif t < 1.4:
                    drv.height = -0.022
                    drv.lift = 0.0
                else:
                    drv.height = min(0.01, -0.022 + 0.02 * (t - 1.4))
                if t > 1.0 and getup_done is None:
                    if float(data.body("base").xpos[2]) > 0.044:
                        getup_done = round(t - 0.6, 2)
            elif drill == "rear":
                # CONDITION-DRIVEN MOUNT (Bill: "obviously he needs to BE
                # CLOSER TO THE WALL when he pitches back. the pitch back
                # separates him from the wall as well, so get closer, to
                # account for that."): phases advance on ARRIVAL, not
                # timers — the pose costs ~6 cm of nose-gap (rotation arcs
                # the nose away + the aft cock pulls the chassis back), so
                # creep overshoots to compensate, however long the slippy
                # walk takes.
                if rear_ph == 0:                   # ARRIVAL = FOOT-JAM
                    # (Bill: "he can only get so close to the wall. he
                    # doesn't have to walk into it more than once.") — his
                    # front feet strike the wall at body-y ~22 and he
                    # SPAWNS at 23.5: he starts arrived. The old y>=31.5
                    # demanded a position his feet physically forbid — the
                    # "creep slip" was him plowing the wall forever. Once
                    # he poses and liberates, the jamming feet leave the
                    # ground and the ram's flying fronts cross the plane.
                    drv.v = 0.0
                    if t > 1.2:
                        rear_ph, rear_t0 = 1, t   # the RATIFIED mount
                        # chain (pose/liberate/ram/secure) IS the
                        # transition brief's steps 1-2: aft-cocked COM
                        # frees the fronts, the ram seats the chin,
                        # SECURE anchors rung 2. Deleting it at dawn
                        # was wrong; the brief and the ratified mount
                        # agree.
                if 1 <= rear_ph <= 5 and int(t * 4) != asc_hold.get("ent_dbg"):
                    asc_hold["ent_dbg"] = int(t * 4)
                    _rp_e, _ = rp_of(data.body("base").xquat)
                    _bz_e = float(data.body("base").xpos[2])
                    _gp_e = 0.405 - float(data.body("base").xpos[1])
                    _fl_e = "".join("1" if float(data.geom_xpos[
                        model.geom(f"foot{i}").id][2]) < 0.012 else "0"
                        for i in range(6))
                    _kerr = []
                    _ksat = []
                    for i in range(6):
                        _j = model.joint(f"k{i}").id
                        _a = next(a for a in range(model.nu)
                                  if int(model.actuator_trnid[a][0]) == _j)
                        _q = float(data.qpos[model.jnt_qposadr[_j]])
                        _c = float(data.ctrl[_a])
                        _kerr.append(math.degrees(_c - _q))
                        _ksat.append(abs(float(data.actuator_force[_a]))
                                     >= 0.95 * G.SERVO_TORQUE)
                    stage_log.append((round(t, 2),
                        "ENTRY ph%d pitch %.1f base %.1f gap %.1f floor %s | "
                        "knee err(deg) %s | sat %s"
                        % (rear_ph, math.degrees(_rp_e), _bz_e * 100,
                           _gp_e * 100, _fl_e,
                           " ".join("%+.0f" % e for e in _kerr),
                           "".join("1" if x else "0" for x in _ksat))))
                if False:
                    pass
                elif rear_ph == 1:                 # POSE (costs the 6 cm
                    # creep IN while posing -- WITH A FLOOR: the
                    # ungated creep drove his head under the ladder.
                    # THE ENTRY IS GENES (2026-08-23): every constant
                    # here was hand-tuned under clamped joints; on the
                    # true Gary the mount must be re-found by evolution.
                    drv.v = (CRAWL["pose_v"] if (0.405 - float(
                        data.body("base").xpos[1])) > CRAWL["pose_gap"]
                             else 0.0)
                    drv.lift = CRAWL["pose_lift"]   # 0 = drag (rubber feet shove him back)
                    _rp = min(1.0, (t - rear_t0) / 1.6)
                    drv.leg_h[4] = drv.leg_h[5] = CRAWL["pose_rear"] * _rp
                    drv.leg_h[0] = drv.leg_h[3] = CRAWL["pose_mid"] * _rp
                    drv.leg_h[1] = drv.leg_h[2] = CRAWL["pose_front"] * _rp
                    drv.body_dy = CRAWL["pose_lean"] * _rp
                    _gapp = 0.405 - float(data.body("base").xpos[1])
                    if _rp >= 1.0 and t - rear_t0 > 2.2 and (
                            _gapp <= CRAWL["pose_gap"] or t - rear_t0 > CRAWL["pose_Tmax"]):
                        rear_ph, rear_t0 = 2, t
                elif rear_ph == 2:                 # LIBERATE / STAND UP
                    drv.v = CRAWL["lib_v"]     # keep closing while
                    drv.press = 1 if CRAWL["lib_press"] >= 0.5 else 0   # the fronts fly (or not: hands-first)
                    drv.press_hi = 0.032
                    if CRAWL["lib_press"] < 0.5:
                        # hands-first: the crouch got him in under rung 2;
                        # now STAND UP close, all legs to stand_h, so a
                        # lifted hand has somewhere to dip (2026-08-23)
                        _sr = min(1.0, (t - rear_t0) / CRAWL["stand_T"])
                        drv.v = 0.0
                        # NODSCAN: one down-up sweep of the incline while
                        # sampling the forward beam; collect near returns
                        if CRAWL["nod_on"] >= 0.5:
                            _np9 = min(1.0, (t - rear_t0) / CRAWL["stand_T"])
                            _nod9 = math.sin(_np9 * 2.0 * math.pi) * CRAWL["nod_amp"]
                        else:
                            _nod9 = 0.0
                        # INCLINE (ASTERISK step 2): fronts tall, rears short,
                        # on planted feet -- a rung ahead-and-above becomes
                        # ahead-and-level in the leg frame (a level body's
                        # reach to it folds the femur 150 deg over the back)
                        for _i2, _p2, _g2 in ((1, "pose_front", "incl_front"), (2, "pose_front", "incl_front"),
                                              (0, "pose_mid", "incl_mid"), (3, "pose_mid", "incl_mid"),
                                              (4, "pose_rear", "incl_rear"), (5, "pose_rear", "incl_rear")):
                            drv.leg_h[_i2] = CRAWL[_p2] + (CRAWL["stand_h"] + CRAWL[_g2] - CRAWL[_p2]) * _sr
                            if _i2 in (1, 2):
                                drv.leg_h[_i2] += _nod9
                            elif _i2 in (4, 5):
                                drv.leg_h[_i2] -= _nod9
                        if CRAWL["nod_on"] >= 0.5:
                            _rrf = float(data.sensordata[model.sensor_adr[
                                model.sensor("rf_ld0").id]])
                            if 0.0 < _rrf < 0.5:
                                _sid9 = model.site("ld0").id
                                _pp9 = data.site_xpos[_sid9]
                                _zx9 = data.site_xmat[_sid9].reshape(3, 3)[:, 2]
                                _hy9 = float(_pp9[1] + _zx9[1] * _rrf)
                                _hz9 = float(_pp9[2] + _zx9[2] * _rrf)
                                asc_hold.setdefault("nod_pts", []).append((_hy9, _hz9))
                    if t - rear_t0 > (CRAWL["stand_T"] if CRAWL["lib_press"] < 0.5 else 0.5):
                        _pts9 = asc_hold.pop("nod_pts", [])
                        if _pts9:
                            _ys9 = sorted(y for y, _ in _pts9)
                            _plane9 = _ys9[len(_ys9) // 2]
                            _near9 = sorted(z for y, z in _pts9
                                            if y < _plane9 - 0.004)
                            _cl9s = []
                            for _z9 in _near9:
                                if _cl9s and _z9 - _cl9s[-1][-1] < 0.025:
                                    _cl9s[-1].append(_z9)
                                else:
                                    _cl9s.append([_z9])
                            # the cluster's TOP return is the lip's top
                            # edge, which is the seat height (the +14 mm
                            # mean-correction overshot by 1 cm)
                            _rz9s = [max(c) for c in _cl9s if len(c) >= 2]
                            if len(_rz9s) >= 2:
                                _sp9 = _rz9s[-1] - _rz9s[-2]
                                _ext9 = sorted(set(_rz9s + [_rz9s[-1] + _sp9,
                                                            _rz9s[0] - _sp9]))
                                _ext9 = [z for z in _ext9 if 0.02 < z < 0.30]
                                asc_hold["scan_rsh"] = _ext9
                                asc_hold["scan_plane"] = _plane9
                            stage_log.append((round(t, 1),
                                "NODSCAN: %d pts, plane y %.3f, rungs z %s%s"
                                % (len(_pts9),
                                   _ys9[len(_ys9) // 2] if _ys9 else -1,
                                   ["%.3f" % z for z in _rz9s],
                                   " (adopted+extrapolated)" if len(_rz9s) >= 2 else " (fallback to built-ins)")))
                        rear_ph, rear_t0 = 3, t
                elif rear_ph == 3:                 # THE RAM (one, at the
                    _sp = min(1.0, (t - rear_t0) / CRAWL["ram_T"])   # wall)
                    _ext = CRAWL["ram_ext"] * min(1.0, _sp * 1.6)
                    drv.height = CRAWL["ram_h"] * _sp   # the ram ARCS UP, not
                    # a flat slide (Bill: "shoot UP more, higher arc")
                    drv.leg_h[4] = drv.leg_h[5] = CRAWL["pose_rear"] + _ext
                    # MIDS ARE THE ELEVATOR, not outriggers (Bill: "he
                    # doesnt make full use of his middle legs to pitch
                    # his body back, or elevate it as it moves
                    # forward") -- they extend hard THROUGH the ram so
                    # the body RISES as it advances
                    drv.leg_h[0] = drv.leg_h[3] = (CRAWL["pose_mid"]
                                                   + CRAWL["ram_mid"] * _sp)
                    drv.body_dy = CRAWL["pose_lean"] + CRAWL["ram_lean"] * _sp
                    if _sp >= 1.0:
                        # hands-first: no SECURE arc from the floor --
                        # the stair gait takes rung 1 itself
                        rear_ph, rear_t0 = (5 if CRAWL["lib_press"] < 0.5
                                            else 4), t
                elif rear_ph == 4:                 # SECURE THE RUNG (Bill:
                    # "make sure he didn't slide off the second rung once
                    # he was up there"): the ram lands the feet on the
                    # smooth FACE between rungs (6.5 cm — no shelf) and
                    # face-friction bleeds them down. Each front re-places
                    # UP onto rung 2's ledge — one ratchet arc apiece —
                    # then world-anchors ON the shelf, where gravity pins
                    # instead of pulls.
                    # SEAMLESS CATCH (Bill: "he loses all body
                    # posture, and nose dives, like his front legs stop
                    # holding themselves at the shoulder"): the secure
                    # arcs launch FROM THE LIVE flying feet -- the ram
                    # leaves them at rung-2 height, so the hook happens
                    # ON the lunge momentum ("the second rung that he
                    # really should've been able to get on his initial
                    # lunge"), with no slack beat at the handoff
                    if "sec_from" not in asc_hold:
                        asc_hold["sec_from"] = {
                            1: max(0.050, float(data.geom_xpos[
                                model.geom("foot1").id][2])),
                            2: max(0.050, float(data.geom_xpos[
                                model.geom("foot2").id][2]))}
                    drv.press = 0
                    _t4 = t - rear_t0
                    _R2 = 0.119
                    _tp = {}
                    _hf4 = CRAWL["lib_press"] < 0.5      # hands-first mount
                    if _t4 <= 0.5:
                        _tp[1] = {"arc": {"from": asc_hold["sec_from"][1], "to": _R2,
                                          "pr": _t4 / 0.5}}
                        _tp[2] = (None if _hf4 else
                                  {"rung": asc_hold["sec_from"][2], "plant_h": drv.height})
                    elif _t4 <= 1.0:
                        _tp[1] = {"rung": _R2, "plant_h": drv.height}
                        _tp[2] = {"arc": {"from": asc_hold["sec_from"][2], "to": _R2,
                                          "pr": (_t4 - 0.5) / 0.5}}
                    else:
                        _tp[1] = {"rung": _R2, "plant_h": drv.height}
                        _tp[2] = {"rung": _R2, "plant_h": drv.height}
                    drv.tpc = _tp
                    drv.body_dy = CRAWL["sec_lean"]
                    drv.leg_h[4] = drv.leg_h[5] = CRAWL["sec_rear"]
                    drv.leg_h[0] = drv.leg_h[3] = CRAWL["sec_mid"]
                    if _t4 > 2.4:
                        rear_ph, rear_t0 = 5, t
                        drv.tpc = {1: {"rung": 0.091, "plant_h": drv.height},
                                   2: {"rung": 0.091, "plant_h": drv.height}}
                elif rear_ph == 5:
                    # THE STAIR GAIT (Bill: "think about how gaits work
                    # when he walks... thats how this should work") --
                    # WAVE gait at the wall + per-leg TERRAIN LEVELS.
                    # Promotion = a leg's touchdowns verify on-rung and
                    # the body is close enough for the next shelf;
                    # demotion = a promoted leg loses rung contact too
                    # long. Everything works together because the gait
                    # engine never stops.
                    _RSH6 = [0.059, 0.119, 0.179, 0.239]
                    _srsh = asc_hold.get("scan_rsh")
                    if _srsh is not None and len(_srsh) >= 2:
                        # sensed seats: cluster z is the bar/lip band; the
                        # seat is that band's top + the foot radius
                        _RSH6 = [min(0.30, z) for z in _srsh][:4]
                        while len(_RSH6) < 4:
                            _RSH6.append(_RSH6[-1] + (_RSH6[-1] - _RSH6[-2]))
                    if "stg" not in asc_hold:
                        asc_hold["stg"] = "shift"
                        asc_hold["stg_t"] = t
                        asc_hold["t0"] = t
                        asc_hold["lvl"] = {i: 0 for i in range(6)}
                        asc_hold["promo_t"] = {i: -9.0 for i in range(6)}
                        asc_hold["rung_seen"] = {i: -9.0 for i in range(6)}
                        asc_hold["fanchor"] = {}
                        for _fa6 in (1, 2):
                            if foot_on_rung(_fa6):      # honest anchors only
                                asc_hold["fanchor"][_fa6] = data.geom_xpos[
                                    model.geom(f"foot{_fa6}").id].copy()
                        drv.g = G.GAITS.index("WAVE")
                        drv.beta = G.BETA[drv.g]
                        drv.press = 0
                        drv.tpc = None
                        drv.foot_lock = None
                        stage_log.append((round(t, 1),
                            "SHIFT: secured on rung 2 -> CoG to the wall"))
                    _stg6 = asc_hold["stg"]
                    _gap6 = 0.405 - float(data.body("base").xpos[1])
                    _st6 = t - asc_hold["stg_t"]
                    if _stg6 == "walk":
                        drv.v = 0.030
                        drv.lift = 0.018
                        drv.body_dy = 0.045
                        drv.body_pitch = 0.0
                        drv.height = 0.008
                        drv.leg_h = [0.0] * 6
                        drv.foot_lock = None
                        if _gap6 < 0.115:
                            asc_hold["stg"] = "tilt"
                            asc_hold["stg_t"] = t
                            stage_log.append((round(t, 1),
                                "TILT: rears compress, mids FULL extend (gap %.1f)"
                                % (_gap6 * 100)))
                    elif _stg6 == "tilt":
                        # rears deep-compressed + mids fully extended =
                        # the 30-45 deg pitch, made of legs not math
                        drv.v = 0.010
                        drv.lift = 0.008
                        _r6 = min(1.0, _st6 / 1.0)
                        # graded wedge (the brief's tilt assumes a
                        # rear-heavy insect; Gary's nose carries the
                        # lidar -- tall mids alone make a TENT that
                        # drops the nose). The proven rear-mount pose:
                        # fronts TALLEST, mids middle, rears compressed.
                        drv.leg_h[4] = drv.leg_h[5] = -0.030 * _r6
                        drv.leg_h[0] = drv.leg_h[3] = 0.042 * _r6
                        drv.leg_h[1] = drv.leg_h[2] = 0.055 * _r6
                        drv.body_dy = 0.045
                        if _st6 > 1.4:
                            asc_hold["stg"] = "chinrest"
                            asc_hold["stg_t"] = t
                            stage_log.append((round(t, 1),
                                "CHINREST: seat the chin on rung 3's lip"))
                    elif _stg6 == "chinrest":
                        # THE CHIN IS THE THIRD HAND: the wedge nose
                        # rides at ~13.5-14; rung 3's lip is 14.2.
                        # Drive in and seat the chin ON the lip -- the
                        # wall carries the lidar-heavy nose, killing
                        # the one-handed-while-stretched moment that
                        # collapsed every anchor attempt at the 0.18
                        # N-m ceiling.
                        # LIBERATE THE FRONTS (the ratified mount's
                        # missing step): planted front feet JAM against
                        # the baseboard at gap ~11.5 -- they must FLY
                        # at wall height while mids+rears drive the
                        # last 3 cm and the chin seats
                        drv.press = 1
                        drv.press_hi = 0.032
                        drv.v = 0.022
                        drv.lift = 0.010
                        drv.body_dy = 0.060
                        drv.leg_h[4] = drv.leg_h[5] = -0.030
                        drv.leg_h[0] = drv.leg_h[3] = 0.042
                        if (_gap6 < 0.085 and _st6 > 0.6) or _st6 > 3.5:
                            asc_hold["stg"] = "anchor"
                            asc_hold["stg_t"] = t
                            asc_hold["a_ph"] = "reach"
                            drv.press = 0
                            stage_log.append((round(t, 1),
                                "ANCHOR: BOTH hands together, chin seated (gap %.1f)"
                                % (_gap6 * 100)))
                    elif _stg6 == "anchor":
                        # BOTH FRONTS TOGETHER (roach-style: both in
                        # 75 ms) -- the chin on the lip carries the
                        # nose, hands drop SHORT reaches onto rung 2,
                        # tug-confirm both
                        _ap6 = asc_hold["a_ph"]
                        _fl6 = {}
                        drv.v = 0.012
                        drv.lift = 0.008
                        drv.leg_h[4] = drv.leg_h[5] = -0.030
                        drv.leg_h[0] = drv.leg_h[3] = 0.050
                        drv.body_dy = 0.058
                        if _ap6 == "reach":
                            # THE HOOK STRIKE (Bill): up ABOVE the
                            # rung, forward PAST the ladder plane,
                            # straight DOWN into the trough
                            _pr6 = min(1.0, _st6 / 0.55)
                            for _al6 in (1, 2):
                                _lw6 = data.geom_xpos[
                                    model.geom(f"foot{_al6}").id]
                                if _pr6 < 0.4:
                                    _s7 = _pr6 / 0.4
                                    _ty6 = float(_lw6[1])
                                    _tz6 = _RSH6[1] + 0.030 * _s7
                                elif _pr6 < 0.75:
                                    _s7 = (_pr6 - 0.4) / 0.35
                                    _ty6 = float(_lw6[1]) + (0.4245
                                            - float(_lw6[1])) * _s7
                                    _tz6 = _RSH6[1] + 0.030
                                else:
                                    _s7 = (_pr6 - 0.75) / 0.25
                                    _ty6 = 0.4245
                                    _tz6 = _RSH6[1] + 0.030 * (1 - _s7)
                                _c7, _o7, _d7 = world_to_leg(
                                    _al6, (float(_lw6[0]), _ty6, _tz6))
                                if (0.02 < _o7 < 0.135
                                        and -0.108 < _d7 < 0.108
                                        and abs(_c7) < G.COXA_CMD_MAX):
                                    asc_hold["a_last_%d" % _al6] = (
                                        _c7, _o7, _d7)
                                if asc_hold.get("a_last_%d" % _al6):
                                    _fl6[_al6] = asc_hold[
                                        "a_last_%d" % _al6]
                            if _pr6 >= 1.0:
                                asc_hold["a_ph"] = "tug"
                                asc_hold["stg_t"] = t
                                for _al6 in (1, 2):
                                    asc_hold["a_w%d" % _al6] =                                         data.geom_xpos[model.geom(
                                            f"foot{_al6}").id].copy()
                        elif _ap6 == "tug":
                            # PULL-IN AND SQUEEZE (Bill): pull back
                            # toward the chassis until the foot JAMS
                            # against the trough lip -- the anchor is
                            # a mechanical latch, and the tug and the
                            # jam are the same motion
                            for _al6 in (1, 2):
                                _aw6 = asc_hold["a_w%d" % _al6]
                                _c7, _o7, _d7 = world_to_leg(
                                    _al6, (float(_aw6[0]),
                                           0.4100,
                                           float(_aw6[2]) - 0.010))
                                if (0.02 < _o7 < 0.135
                                        and -0.108 < _d7 < 0.108
                                        and abs(_c7) < G.COXA_CMD_MAX):
                                    _fl6[_al6] = (_c7, _o7, _d7)
                                elif asc_hold.get("a_last_%d" % _al6):
                                    _fl6[_al6] = asc_hold[
                                        "a_last_%d" % _al6]
                            if _st6 > 0.3:
                                _h16 = foot_on_rung(1)
                                _h26 = foot_on_rung(2)
                                stage_log.append((round(t, 1),
                                    "TUG both: leg1 %s, leg2 %s"
                                    % ("HELD" if _h16 else "slip",
                                       "HELD" if _h26 else "slip")))
                                if _h16 or _h26:
                                    for _al6 in (1, 2):
                                        asc_hold["fanchor"][_al6] =                                             data.geom_xpos[model.geom(
                                                f"foot{_al6}").id].copy()
                                    asc_hold["stg"] = "shift"
                                    asc_hold["stg_t"] = t
                                    stage_log.append((round(t, 1),
                                        "SHIFT: push-pull, CoG to the wall"))
                                else:
                                    asc_hold["a_ph"] = "reach"
                                    asc_hold["stg_t"] = t
                        drv.foot_lock = _fl6 if _fl6 else None
                    elif _stg6 == "shift":
                        # DIG: fronts pull DOWN-AND-IN on their rungs
                        # (internal tension locks him to the wall);
                        # ground legs walk the CoG in
                        _fl6 = {}
                        _mz7 = (sum(float(v[2]) for v in
                                    asc_hold["fanchor"].values())
                                / max(1, len(asc_hold["fanchor"])))
                        for _kl6, _kv6 in asc_hold["fanchor"].items():
                            # DYNAMIC TENSION, SYMMETRIC (Bill's
                            # squeeze; asymmetric pull-in rolled him
                            # onto his side): both hands pull at the
                            # MEAN anchor height against the lip
                            _c7, _o7, _d7 = world_to_leg(
                                _kl6, (_kv6[0],
                                       0.4100,
                                       _mz7 - CRAWL["dig_z"]))
                            if _o7 >= 0.135:
                                # TAUT ARM, not a frozen rod: the
                                # anchors are born over-stretched
                                # (secure ends at gap ~15.5) -- clamp
                                # the reach at the envelope, aimed at
                                # the rung, so the servos PULL as the
                                # body closes instead of deadlocking
                                # the whole approach
                                _o7 = 0.132
                            if (0.02 < _o7 < 0.135 and -0.108 < _d7 < 0.108
                                    and abs(_c7) < G.COXA_CMD_MAX):
                                _fl6[_kl6] = (_c7, _o7, _d7)
                            elif _o7 <= 0.02:
                                # TOO CLOSE is the HANDS-UP moment,
                                # not an error: release this hold one
                                # shelf higher instead of freezing it
                                # into a drag-off
                                _nz7 = next((z for z in _RSH6
                                             if z > float(_kv6[2])
                                             + 0.012), None)
                                if _nz7 is not None:
                                    asc_hold["fanchor"][_kl6] = \
                                        _kv6.copy()
                                    asc_hold["fanchor"][_kl6][2] = _nz7
                                    stage_log.append((round(t, 1),
                                        "hold leg%d releases UP to z=%.1f"
                                        % (_kl6, _nz7 * 100)))
                        drv.foot_lock = _fl6 if _fl6 else None
                        drv.v = CRAWL["shift_v"]
                        drv.lift = 0.014
                        drv.body_dy = CRAWL["shift_lean"]
                        drv.leg_h[4] = drv.leg_h[5] = -0.014
                        drv.leg_h[0] = drv.leg_h[3] = 0.050
                        if _gap6 < CRAWL["shift_gap"] or _st6 > 8.0:
                            asc_hold["stg"] = "climb"
                            asc_hold["stg_t"] = t
                            # the CLIMB gets its own knee envelope: the
                            # entry collapses past ~107 deg, the rung-3
                            # reach needs more (hand test 2026-08-22)
                            G.KNEE_CMD_MAX = float(CRAWL["knee_climb"])
                            stage_log.append((round(t, 1),
                                "CLIMB: stair gait takes over (gap %.1f)"
                                % (_gap6 * 100)))
                            asc_hold["climb_t0"] = t
                            drv.foot_lock = None
                            # seed the stair gait: fronts own shelf 2
                            asc_hold.setdefault("lvl", {i: 0 for i in
                                                        range(6)})
                            asc_hold["lvl"].update({1: 2, 2: 1})
                    if _stg6 != "climb":
                        pass
                    else:
                        _SH5 = [0.0, 0.053, 0.113, 0.173, 0.233]
                        # gap gates per target level (higher shelves need a
                        # closer body -- the mid/front reach geometry)
                        _GATE5 = {1: CRAWL["gate1"], 2: CRAWL["gate2"],
                                  3: CRAWL["gate3"], 4: CRAWL["gate4"]}
                        if not asc_hold:
                            asc_hold["lvl"] = {i: 0 for i in range(6)}
                            asc_hold["t0"] = t
                            asc_hold["promo_t"] = {i: -9.0 for i in range(6)}
                            asc_hold["rung_seen"] = {i: -9.0 for i in range(6)}
                            drv.g = G.GAITS.index("WAVE")
                            drv.beta = G.BETA[drv.g]
                            drv.tpc = None
                            drv.foot_lock = None
                            # ENTER DIGGING (Bill: "may have been premature
                            # deleting the launch at the wall... he pushes
                            # himself away from the wall if anything"): the
                            # dig-deep reflex starts SATURATED, so he
                            # drives INTO the wall from the first stride --
                            # sustained aggression instead of a ballistic
                            # ram
                            asc_hold["last_promo"] = t - 20.0
                            # pitch rotates about the FRONT FEET (the
                            # summit's own trick): nose-up then FOLDS the
                            # rear of the body instead of extending the
                            # fronts -- the probe showed the pitch tilt
                            # term (+0.024) cancelling the shelf shortening
                            # (-0.048) leg-for-leg: pitch and terrain were
                            # the same DoF used against each other
                            drv.body_pivot_y = 0.19
                            stage_log.append((round(t, 1),
                                "STAIR GAIT begins (WAVE at the wall)"))
                        _et5 = t - asc_hold["t0"]
                        _lvl5 = asc_hold["lvl"]
                        _nose5 = float(data.site_xpos[nose_id][2])
                        _gap5 = 0.405 - float(data.body("base").xpos[1])
                        if any(foot_on_rung(_fr) for _fr in range(6)):
                            asc_hold["rung_t"] = t
                        if int(_et5 // 2.0) != asc_hold.get("logk"):
                            if asc_hold.get("logk") is not None:
                                _f15 = data.geom_xpos[model.geom("foot1").id]
                                _f05 = data.geom_xpos[model.geom("foot0").id]
                                stage_log.append((round(t, 1),
                                    "stairs: gap %.1f, nose %.1f, lvls %s | f1(y%.1f,z%.1f) f0(y%.1f,z%.1f) rung1 y39.3-41.7 top5.2"
                                    % (_gap5 * 100, _nose5 * 100,
                                       "".join(str(_lvl5[i]) for i in
                                               range(6)),
                                       _f15[1] * 100, _f15[2] * 100,
                                       _f05[1] * 100, _f05[2] * 100)))
                            asc_hold["logk"] = int(_et5 // 2.0)

                        # the WALK (continuous; the gait engine owns timing)
                        drv.v = CRAWL["v"]
                        drv.lift = CRAWL["lift"]
                        drv.body_dy = CRAWL["lean"]
                        # front legs swing HIGHER (Bill) -- clear the next
                        # rung's lip, not just the current shelf
                        drv.leg_lift = [0.0] * 6
                        drv.leg_lift[1] = drv.leg_lift[2] = CRAWL["f_lift"]
                        drv.leg_lift[0] = drv.leg_lift[3] = CRAWL["m_lift"]
                        # promoted legs REACH THE WALL: the gait's neutral
                        # touchdown lands short of the shelves (the mids
                        # were stepping on stilts over empty air -- the
                        # promote/demote churn); forward bias carries the
                        # touchdown onto the rungs, capped at the envelope
                        _NR5 = {0: 0.000, 3: 0.000, 1: 0.094, 2: 0.094,
                                4: -0.107, 5: -0.107}
                        # THE WEDGE UNDER THE GAIT (Bill: "his body
                        # still has to be pitched back so his ass is down
                        # and his head is up" -- the gait is the concept,
                        # the climbing posture stays): rears folded, fronts
                        # tall on their shelves
                        for _i5 in range(6):
                            # TERRAIN SIGN: leg_h positive EXTENDS the leg
                            # (wedge semantics: tall body over a low foot).
                            # Standing ON a shelf = the leg SHORTENS by the
                            # shelf height. The first stair build had this
                            # inverted -- promoted feet pressed 5-14 cm INTO
                            # the floor at the wall base (telemetry: foot1
                            # y40, z0.3 under a commanded 5.2 shelf), and
                            # every 'shelf level' was secretly a taller
                            # wedge brushing rung faces. Bill: 'his front
                            # legs just stopped grasping' -- they never
                            # grasped.
                            drv.leg_h[_i5] = -_SH5[_lvl5[_i5]]
                            _ftw5 = asc_hold.get("ftmax", 0.0)
                            if _i5 in (4, 5):
                                # rears un-fold and EXTEND as the climb
                                # rises -- the entry crouch is not a life
                                # sentence (Bill: "just like the rears
                                # should as the pitch increases")
                                drv.leg_h[_i5] += (-CRAWL["ass_down"]
                                                   + min(0.055, CRAWL["rear_k"]
                                                         * _ftw5))
                            elif _i5 in (0, 3) and _lvl5[_i5] == 0:
                                # MIDS EXTEND TALL UNDERNEATH (Bill: "full
                                # extension for support should be allowed
                                # UNDER the body to SUPPORT and PUSH"):
                                # from firm to full tall stance beneath the
                                # hips as the front terrain rises, until
                                # they graduate to the wall
                                drv.leg_h[_i5] += min(0.045, 0.006
                                                      + CRAWL["supp_k"] * _ftw5)
                            elif _i5 in (1, 2):
                                drv.leg_h[_i5] += CRAWL["tall_f"]
                            if _lvl5[_i5] > 0:
                                _need5 = _gap5 - 0.008 - _NR5[_i5]
                                drv.leg_dy[_i5] = -min(CRAWL["reach_cap"],
                                                       max(0.0, _need5))
                            else:
                                drv.leg_dy[_i5] = 0.0
                        # posture rides the HIGH-WATER front terrain --
                        # a demotion must NOT slump the body (the old
                        # current-average coupling made failure compound:
                        # demote -> slump -> shorter reach -> more demotes,
                        # Bill: "his effort becomes minimal over time...
                        # like he gets tired and gives up")
                        _ft5 = (_SH5[_lvl5[1]] + _SH5[_lvl5[2]]) / 2.0
                        _ft5 = asc_hold["ftmax"] = max(
                            asc_hold.get("ftmax", 0.0), _ft5)
                        # DIG DEEP AND PUSH (Bill): stalled too long with no
                        # promotion -> effort ESCALATES until the next one
                        _stall5 = t - asc_hold.get("last_promo", asc_hold["t0"])
                        _dig5 = min(1.0, max(0.0, (_stall5 - 6.0) / 5.0))
                        drv.v = CRAWL["v"] * (1.0 + 0.6 * _dig5)
                        drv.body_dy = CRAWL["lean"] + 0.014 * _dig5
                        if (CRAWL["lib_press"] < 0.5
                                and sum(1 for _h in (1, 2)
                                        if _h in asc_hold.get("mids", {})) < 2):
                            drv.body_dy = CRAWL["sec_lean"]   # aft until both hands hold
                        drv.leg_lift[1] += 0.008 * _dig5
                        drv.leg_lift[2] += 0.008 * _dig5
                        # the body RISES ONTO the stepped feet -- that
                        # is what climbing stairs IS; the old 0.030 cap and
                        # weak coupling left shelf-shortened legs dangling
                        # off a low chassis (foot z 0.3 under a 5.2 shelf)
                        drv.height = min(0.045, CRAWL["h_base"]
                                         + CRAWL["h_k"] * _ft5)
                        # NO commanded pitch (Bill: "his pitch is
                        # mishapen. instead of just his rear legs being
                        # crouched, his middle legs are too, efectively
                        # putting him on the ground" -- the straight-line
                        # tilt through any pivot folds the mids halfway to
                        # the rears; a line cannot make rears-folded +
                        # mids-FIRM + fronts-high). Pitch now EMERGES from
                        # the terrain: shelved fronts high, firm mids, only
                        # the rears crouched.
                        drv.body_pitch = 0.0
                        if CRAWL["lib_press"] < 0.5:
                            # POSTURE FREEZE (2026-08-23): the stair posture
                            # law keys height/leg heights to the level
                            # counters, which a scheduled strike bumps --
                            # the body was commanded 4.5 cm up the moment a
                            # strike was scheduled. Hands-first: the stance
                            # posture is the stand-up posture; only BODY
                            # changes the body.
                            drv.height = 0.0
                            for _i7, _g7 in ((0, "incl_mid"), (1, "incl_front"), (2, "incl_front"),
                                             (3, "incl_mid"), (4, "incl_rear"), (5, "incl_rear")):
                                drv.leg_h[_i7] = CRAWL["stand_h"] + CRAWL[_g7]
                            drv.leg_lift[1] = drv.leg_lift[2] = 0.0

                        # HOOK STRIKES (Bill's step 1): hands-first, the
                        # fronts do not walk onto rung 1, they HOOK it
                        # mids forward first (lifted steps), then the strikes
                        _mf = asc_hold.setdefault("mids_fwd", set())
                        if CRAWL["mid_dy"] < 0.001:
                            _mf.update((0, 3))              # no mid steps: strikes straight away
                        if (CRAWL["lib_press"] < 0.5 and asc_hold.get("rehook") is None
                                and len(_mf) < 2 and t - asc_hold.get("adv_t", -9.0) > 0.3):
                            _ml = 0 if 0 not in _mf else 3
                            _mw = data.geom_xpos[model.geom(f"foot{_ml}").id].copy()
                            _tym = float(_mw[1]) + CRAWL["mid_dy"]
                            _c5, _o5, _d5 = world_to_leg(_ml, (hook_x(_ml), _tym, 0.006))
                            if (0.02 < _o5 < 0.135 and -0.108 < _d5 < 0.108
                                    and abs(_c5) < G.COXA_CMD_MAX
                                    and G.leg_ik(_o5, _d5) is not None):
                                asc_hold["rehook"] = {
                                    "leg": _ml, "t0": t, "from": _mw, "to": 0.006,
                                    "prev_w": None, "strike": True, "step": True,
                                    "ty": _tym}
                                stage_log.append((round(t, 1),
                                    "MID STEP leg%d -> y %.1f (gap %.1f)"
                                    % (_ml, _tym * 100, _gap5 * 100)))
                            _mf.add(_ml)
                            asc_hold["adv_t"] = t
                        if (CRAWL["lib_press"] < 0.5
                                and asc_hold.get("rehook") is None
                                and len(_mf) >= 2
                                and _gap5 <= CRAWL["strike_gap"]
                                and t - asc_hold.get("strike_t", -9.0) > 1.5):
                            for _hs in (1, 2):
                                if _hs in asc_hold.get("mids", {}):
                                    continue            # already latched
                                if asc_hold.get("strikes", {}).get(_hs, 0) >= 3:
                                    continue            # three tries, then the gait
                                _nz_s = _RSH6[int(CRAWL["strike_rung"]) - 1]
                                _fx_s = hook_x(_hs)
                                _c_s, _o_s, _d_s = world_to_leg(
                                    _hs, (_fx_s, CRAWL["hook_y"], _nz_s - 0.006))
                                if not (0.02 < _o_s < 0.135 and -0.108 < _d_s < 0.108
                                        and abs(_c_s) < G.COXA_CMD_MAX
                                        and G.leg_ik(_o_s, _d_s) is not None):
                                    continue            # drop point out of reach
                                if _nz_s > _RSH6[0] + 0.01:
                                    _c_a, _o_a, _d_a = world_to_leg(
                                        _hs, (_fx_s, 0.390, _nz_s + CRAWL["hook_clear"]))
                                    if not (0.02 < _o_a < 0.135 and -0.108 < _d_a < 0.108
                                            and abs(_c_a) < G.COXA_CMD_MAX
                                            and G.leg_ik(_o_a, _d_a) is not None):
                                        asc_hold["strike_hold"] = asc_hold.get("strike_hold", 0) + 1
                                        if asc_hold["strike_hold"] % 50 == 1:
                                            stage_log.append((round(t, 1),
                                                "STRIKE HOLD leg%d: apex o=%.3f d=%.3f c=%.2f"
                                                % (_hs, _o_a, _d_a, _c_a)))
                                        continue        # apex behind the lips out of reach
                                _fw_s = data.geom_xpos[
                                    model.geom(f"foot{_hs}").id].copy()
                                asc_hold["rehook"] = {
                                    "leg": _hs, "t0": t, "from": _fw_s,
                                    "to": _nz_s, "prev_w": _fw_s.copy(),
                                    "strike": True}
                                asc_hold["strike_t"] = t
                                asc_hold.setdefault("strikes", {})[_hs] = (
                                    asc_hold.get("strikes", {}).get(_hs, 0) + 1)
                                _lvl5[_hs] = max(_lvl5[_hs], int(CRAWL["strike_rung"]))
                                asc_hold["promo_t"][_hs] = t
                                stage_log.append((round(t, 1),
                                    "HOOK STRIKE leg%d -> rung %d (gap %.1f)"
                                    % (_hs, int(CRAWL["strike_rung"]), _gap5 * 100)))
                                break
                        # PROMOTION / DEMOTION (the stair-finding reflex)
                        for _i5 in (1, 2, 0, 3):
                            if _i5 in asc_hold.get("mids", {}):
                                continue   # LATCHED: not the carousel's
                            if foot_on_rung(_i5):
                                asc_hold["rung_seen"][_i5] = t
                                _ca7 = asc_hold.setdefault("conf_acc", {})
                                if (_lvl5[_i5] > 0 and foot_on_rung_n(
                                        _i5, _lvl5[_i5])):
                                    _ca7[_i5] = _ca7.get(_i5, 0.0) + 0.01
                                if (_lvl5[_i5] > 0
                                        and not asc_hold.setdefault(
                                            "owned", {}).get(_i5)
                                        and _ca7.get(_i5, 0.0)
                                        > CRAWL["conf_need"]):
                                    asc_hold["owned"][_i5] = True
                                    stage_log.append((round(t, 1),
                                        "OWNED: leg%d bears on shelf %d"
                                        % (_i5, _lvl5[_i5])))
                                    # a shelf you OWN, you LATCH: the
                                    # leg joins the wall set under
                                    # pull-in tension instead of going
                                    # back to the carousel
                                    asc_hold.setdefault("mids", {})[_i5] = {
                                        "w": data.geom_xpos[model.geom(
                                            f"foot{_i5}").id].copy(),
                                        "last": None}
                                    # THE FOLLOW-THROUGH (Bill's ladder
                                    # diagram; the shelf-3 fall read
                                    # lvls 132000 -- mids still on the
                                    # FLOOR under a rising body): the
                                    # moment a hand owns shelf 2+, a
                                    # mid CHASES onto rung 1 by the
                                    # same over-enclosure hook,
                                    # event-driven, no carousel wait
                                    if (_i5 in (1, 2) and _lvl5[_i5] >= 2
                                            and not any(m in asc_hold["mids"]
                                                        for m in (0, 3))
                                            and asc_hold.get("rehook")
                                            is None):
                                        _mc5 = 0 if 0 not in asc_hold[
                                            "mids"] else 3
                                        asc_hold["rehook"] = {
                                            "leg": _mc5, "t0": t,
                                            "from": data.geom_xpos[
                                                model.geom(
                                                    f"foot{_mc5}").id
                                                ].copy(),
                                            "to": _RSH6[0]}
                                        stage_log.append((round(t, 1),
                                            "MIDS CHASE: leg%d to rung 1"
                                            % _mc5))
                            _cur5 = _lvl5[_i5]
                            _nxt5 = _cur5 + 1
                            _maxl5 = 4 if _i5 in (1, 2) else 1
                            if _nxt5 > _maxl5:
                                _nxt5 = _maxl5   # the ladder has a top
                            _gate6 = _GATE5[_nxt5] + (
                                0.035 if _cur5 >= 2 else 0.0)
                            if _i5 in (0, 3):
                                _gate6 = min(_gate6, 0.118)  # anatomy
                            elif _i5 in (1, 2) and _cur5 >= 2:
                                # protect the MOUNT: no higher reach
                                # until a mid is on the wall (brief
                                # step 4 -- the greedy shelf-3 promo
                                # dragged a REAL rung-2 hold off)
                                if not (_lvl5[0] > 0 or _lvl5[3] > 0):
                                    _gate6 = -1.0
                            if (_nxt5 <= _maxl5
                                    and _gap5 < _gate6
                                    and t - asc_hold.get("last_promo", -9)
                                    > 1.2
                                    and t - asc_hold["promo_t"][_i5]
                                    > CRAWL["promo_cool"]
                                    and (_cur5 == 0 or
                                         t - asc_hold["rung_seen"][_i5]
                                         < CRAWL["seen_t"])):
                                _lvl5[_i5] = _nxt5
                                asc_hold["promo_t"][_i5] = t
                                asc_hold["last_promo"] = t
                                asc_hold.setdefault("conf_acc", {})[_i5] = 0.0
                                asc_hold.setdefault("owned", {})[_i5] = False
                                stage_log.append((round(t, 1),
                                    "PROMOTE leg%d to shelf %d (gap %.1f)"
                                    % (_i5, _nxt5, _gap5 * 100)))
                            elif (_cur5 > 0 and (
                                    (not asc_hold.get("owned", {}).get(_i5)
                                     and t - asc_hold["promo_t"][_i5]
                                     > CRAWL["retry_t"])
                                    or (asc_hold.get("owned", {}).get(_i5)
                                        and t - asc_hold["rung_seen"][_i5]
                                        > CRAWL["slip_t"]))):
                                _lvl5[_i5] = _cur5 - 1
                                asc_hold["promo_t"][_i5] = t
                                asc_hold.setdefault("owned", {})[_i5] = False
                                asc_hold.setdefault("conf_acc", {})[_i5] = 0.0
                                stage_log.append((round(t, 1),
                                    "demote leg%d to shelf %d"
                                    % (_i5, _cur5 - 1)))
                        # --- WALL HOLDS: the latch made real (the old
                        # 'latched' set had no actuation -- the fronts
                        # only held because the SHIFT's stale locks and
                        # the troughs did it physically). Every latched
                        # leg gets pull-in tension; with 2+ latched the
                        # LADDER RHYTHM runs (Bill's playbook: 'Front
                        # legs pull the weight up... Paired Alternating
                        # Hooks'): PULL beat (downward tension deepens,
                        # body height rises) then ONE hand re-hooks a
                        # rung higher via the over-enclosure path.
                        _fl9 = {}
                        _lat9 = asc_hold.setdefault("mids", {})
                        # AFTER he gets on the wall he can hold himself
                        # to it (Bill): below the lunge angle the hold
                        # apparatus is OFF -- commanded pull-in during
                        # the mount tethers him at knee height and the
                        # pitch can never grow. Free legs mount; held
                        # legs climb.
                        _pitchg = rp_of(data.body("base").xquat)[0]
                        # ON THE WALL = past the lunge angle (the IMU law,
                        # Bill) OR both hands honestly bearing on rungs.
                        # The mount's commanded pose is 18 deg and the
                        # gate was 0.34 rad = 19.5 deg, so by construction
                        # the ladder rhythm never ran in any winner
                        # (measured 2026-08-22: pitch peak 18.7, zero
                        # RE-HOOK lines). lunge_gate is a gene.
                        _onw9 = (_pitchg > CRAWL["lunge_gate"]
                                 or (foot_on_rung(1) and foot_on_rung(2)))
                        # (no on-wall clear: a hand that honestly bears is
                        # held whether or not the other is -- 2026-08-23)
                        for _fa9 in (1, 2):
                            if not _onw9:
                                break
                            if (_fa9 not in _lat9
                                    and _fa9 in asc_hold.get(
                                        "fanchor", {})):
                                _lat9[_fa9] = {"w": asc_hold["fanchor"]
                                               [_fa9].copy(),
                                               "last": None}
                        # MIDS LATCH TOO (Bill: fronts pull, mids brace):
                        # a mid that honestly bears on its rung joins the
                        # wall set under the same tension. leg3 bore on
                        # shelf 1 at 41.1 s and was dragged off because
                        # only fronts were ever latched (2026-08-22).
                        for _ma9 in (0, 3, 1, 2):
                            _ml9 = int(_lvl5[_ma9])
                            if _ma9 in _lat9 and foot_on_rung(_ma9):
                                asc_hold["rung_seen"][_ma9] = t
                            if (_ma9 not in _lat9 and _ml9 > 0
                                    and foot_on_rung_n(_ma9, _ml9)):
                                _lat9[_ma9] = {
                                    "w": data.geom_xpos[model.geom(
                                        f"foot{_ma9}").id].copy(),
                                    "last": None}
                                stage_log.append((round(t, 1),
                                    "%s LATCH: leg%d holds rung %d"
                                    % ("MID" if _ma9 in (0, 3) else "HAND",
                                       _ma9, _ml9)))
                            elif (_ma9 in _lat9 and not foot_on_rung(_ma9)
                                  and t - asc_hold["rung_seen"].get(
                                      _ma9, t) > CRAWL["slip_t"]):
                                del _lat9[_ma9]
                                stage_log.append((round(t, 1),
                                    "%s LATCH lost: leg%d"
                                    % ("MID" if _ma9 in (0, 3) else "HAND", _ma9)))
                        _pl9 = 0.0
                        # THE IMU PHASE LAW (Bill: "until his IMU reads
                        # that his pitch has passed his lunge angle, he
                        # should be trying to just get onto the wall,
                        # then he can start CLIMBING... holding himself
                        # against the wall, not push/pulling himself
                        # off the ground"): below the lunge angle =
                        # MOUNTING (tension + wedge + drive, no beats);
                        # past it = the ladder rhythm. Ports to
                        # firmware as-is: the real IMU gates climb mode.
                        _pitch9 = rp_of(data.body("base").xquat)[0]
                        # PITCH CEILING (IMU law, Bill: 'hold himself to
                        # it AFTER he gets on it'): the first live rhythm
                        # pulled him past vertical onto his back (pitch
                        # 176 deg, 2026-08-22). Above pitch_max the hands
                        # HOLD (tension, no pull beat, no drive, no
                        # lean) and the body is left to the mids.
                        _steep9 = _pitch9 > CRAWL["pitch_max"]
                        # RHY trace every 4 s: which gate blocks the beat
                        if int(t // 4) != asc_hold.get("rhy_dbg"):
                            asc_hold["rhy_dbg"] = int(t // 4)
                            stage_log.append((round(t, 1),
                                "RHY: lat=%s fanchor=%s onw=%d steep=%d "
                                "pitch=%.2f gap=%.3f rehook=%s"
                                % (sorted(_lat9),
                                   sorted(asc_hold.get("fanchor", {})),
                                   int(_onw9), int(_steep9), _pitch9,
                                   _gap5, asc_hold.get("rehook") is not None)))
                        if len(_lat9) >= 2 and _onw9 and not _steep9:
                            _lt9 = t - asc_hold.setdefault("rhythm_t", t)
                            _bph9 = _lt9 % CRAWL["beat_T"]
                            # THE RATCHET (2026-08-22): at each new beat
                            # every latched leg's anchor is re-based to
                            # where its foot ACTUALLY holds, so the pull
                            # lifts the body from there instead of
                            # pumping against a stale anchor (leg1 sat on
                            # rung 3 with a rung-2 anchor; the pull reset
                            # to zero every beat). Feet that hold, gain.
                            _bi9 = int(_lt9 // CRAWL["beat_T"])
                            if _bi9 != asc_hold.get("rebase_n"):
                                asc_hold["rebase_n"] = _bi9
                                for _rl9, _rh9_ in list(_lat9.items()):
                                    if foot_on_rung(_rl9):
                                        _rh9_["w"] = data.geom_xpos[
                                            model.geom(f"foot{_rl9}").id
                                        ].copy()
                                        _rh9_["last"] = None
                            _pl9 = CRAWL["pull_z"] * min(1.0, _bph9 / 0.9)
                            drv.height = min(0.045, drv.height + 0.018)
                            # ASS DOWN, HEAD UP, MOVE IN -- during the
                            # pull, the ground drives harder, the lean
                            # deepens, and the rears fold in proportion
                            # to how high the hands are latched: the
                            # wedge steepens AS he climbs, not before
                            _mlz9 = (sum(float(v["w"][2]) for v in
                                         _lat9.values()) / len(_lat9))
                            drv.v = CRAWL["v"] * 1.8
                            drv.body_dy = CRAWL["lean"] + 0.014
                            drv.leg_h[4] = drv.leg_h[5] = (
                                drv.leg_h[4] - 0.30 * max(0.0,
                                                          _mlz9 - 0.05))
                            # a hand only REACHES from tightness (the
                            # Y-miss: right height, 1-2 cm short of the
                            # channel -- 'move in, and pitch up' first)
                            if (_bph9 > CRAWL["beat_T"] * 0.8
                                    and _gap5 < 0.128
                                    and asc_hold.get("rehook") is None
                                    and int(_lt9 // CRAWL["beat_T"])
                                    != asc_hold.get("beat_n")):
                                asc_hold["beat_n"] = int(_lt9 // CRAWL["beat_T"])
                                _hx9 = [l for l in (1, 2) if l in _lat9]
                                if _hx9:
                                    _hl9 = _hx9[asc_hold["beat_n"]
                                                % len(_hx9)]
                                    _hw9 = _lat9[_hl9]["w"]
                                    _nz9 = next((z for z in _RSH6
                                                 if z > float(_hw9[2])
                                                 + 0.012), None)
                                    # REACH GATE: a hand only lets go
                                    # when the next rung's apex AND
                                    # drop point are inside its
                                    # envelope from the live pose --
                                    # otherwise the beat just costs a
                                    # hand (every rung-3 re-hook landed
                                    # back on rung 2, 2026-08-22). When
                                    # it cannot, it HOLDS, and says why
                                    # in leg-frame numbers.
                                    _fx9 = hook_x(_hl9)
                                    _ok9 = True
                                    _why9 = ""
                                    if _nz9 is not None:
                                        # the waypoints the hook will FLY
                                        # (gatepath agent, 2026-08-23: the
                                        # old gate tested the apex at the
                                        # advance y while the lift is flown
                                        # at the retreat y), and the knee
                                        # clamp's TRUE reach: leg_ik clamps
                                        # silently, so a bare leg_ik test
                                        # accepted 11.2 cm the servo cannot
                                        # deliver at knee_climb (10.1)
                                        _hw9 = data.geom_xpos[
                                            model.geom(f"foot{_hl9}").id]
                                        _fy9, _fz9 = float(_hw9[1]), float(_hw9[2])
                                        _lipb9 = 0.4025 - 0.0035
                                        _ry9 = _lipb9 - 0.006 - 0.003
                                        _hy9 = CRAWL["hook_y"]
                                        _apx9 = _nz9 + CRAWL["hook_clear"]
                                        if _fy9 > _ry9 + 0.004:
                                            _wps9 = ((_fx9, _fy9, _fz9 + 0.012, "lift"),
                                                     (_fx9, _ry9, _fz9 + 0.012, "retreat"),
                                                     (_fx9, _ry9, _apx9, "apexR"),
                                                     (_fx9, _hy9, _apx9, "apexA"),
                                                     (_fx9, _hy9, _nz9 - 0.006, "drop"))
                                        else:
                                            _wps9 = ((_fx9, _fy9, _apx9, "apexR"),
                                                     (_fx9, _hy9, _apx9, "apexA"),
                                                     (_fx9, _hy9, _nz9 - 0.006, "drop"))
                                        _dmax9 = math.sqrt(
                                            G.L2 * G.L2 + G.L3 * G.L3
                                            - 2.0 * G.L2 * G.L3 * math.cos(
                                                math.radians(G.KNEE_CMD_MAX)))
                                        for _wx9, _wy9, _wz9, _lbl9 in _wps9:
                                            _c9, _o9, _d9 = world_to_leg(
                                                _hl9, (_wx9, _wy9, _wz9))
                                            _D9 = math.hypot(_o9 - G.L1, _d9)
                                            if not (0.02 < _o9 < 0.135
                                                    and -0.108 < _d9 < 0.108
                                                    and abs(_c9) < G.COXA_CMD_MAX
                                                    and G.leg_ik(_o9, _d9)
                                                    is not None
                                                    and _D9 <= _dmax9):
                                                _ok9 = False
                                                _why9 = ("%s o=%.3f d=%.3f "
                                                         "c=%.2f D=%.3f/%.3f"
                                                         % (_lbl9, _o9, _d9,
                                                            _c9, _D9, _dmax9))
                                                break
                                    if _nz9 is not None and _ok9:
                                        asc_hold["rehook"] = {
                                            "leg": _hl9, "t0": t,
                                            "from": data.geom_xpos[
                                                model.geom(
                                                    f"foot{_hl9}").id
                                                ].copy(),
                                            "to": _nz9,
                                            "prev_w": _lat9[_hl9]["w"].copy()}
                                        del _lat9[_hl9]
                                        stage_log.append((round(t, 1),
                                            "RE-HOOK leg%d -> z %.1f"
                                            % (_hl9, _nz9 * 100)))
                                    elif _nz9 is not None:
                                        stage_log.append((round(t, 1),
                                            "HOLD leg%d: rung z %.1f out of "
                                            "reach (%s)" % (_hl9,
                                            _nz9 * 100, _why9)))
                        # MIDS AS COLUMNS (Bill): a mid that is on no rung
                        # stands at its natural outreach and extends
                        # straight DOWN as far as the leg goes (leg
                        # frame: c=0, o=NEUTRAL_OUT+col_dx, d=max feasible).
                        # The floor stops it; the push is the column.
                        if (CRAWL["lib_press"] < 0.5 and asc_hold.get("body") is None
                                and asc_hold.get("rehook") is None):
                            for _mc in (0, 3):
                                if _mc in _lat9 or _lvl5[_mc] > 0:
                                    continue
                                _oc = G.NEUTRAL_OUT + CRAWL["col_dx"]
                                _dc_best = None
                                for _dc in (CRAWL["col_d"], CRAWL["col_d"] - 0.01, CRAWL["col_d"] - 0.02, 0.055, 0.046):
                                    if G.leg_ik(_oc, _dc) is not None:
                                        _dc_best = _dc
                                        break
                                if _dc_best is not None:
                                    # MIDS FORWARD (ASTERISK step 1): yaw the
                                    # column toward the wall so the mid foot
                                    # supports the front end while a hand lifts
                                    _sgn = 1.0 if float(data.body(f"coxa{_mc}").xpos[0]) > float(
                                        data.body("base").xpos[0]) else -1.0
                                    _fl9[_mc] = (_sgn * CRAWL["col_c"], _oc, _dc_best)
                        # BODY (ASTERISK step 4): move the body on the legs
                        _hands9 = [h for h in (1, 2) if h in _lat9]
                        _bd = asc_hold.get("body")
                        _adv = asc_hold.setdefault("adv", set())
                        if (CRAWL["lib_press"] < 0.5 and len(_hands9) == 2
                                and asc_hold.get("rehook") is None and _bd is None
                                and len(_adv) < 2
                                and t - asc_hold.get("adv_t", -9.0) > 0.3):
                            _rl = 4 if 4 not in _adv else 5
                            _rw = data.geom_xpos[model.geom(f"foot{_rl}").id].copy()
                            _ty = float(_rw[1]) + CRAWL["adv_dy"]
                            _c4, _o4, _d4 = world_to_leg(_rl, (hook_x(_rl), _ty, 0.006))
                            if (0.02 < _o4 < 0.135 and -0.108 < _d4 < 0.108
                                    and abs(_c4) < G.COXA_CMD_MAX
                                    and G.leg_ik(_o4, _d4) is not None):
                                asc_hold["rehook"] = {
                                    "leg": _rl, "t0": t, "from": _rw, "to": 0.006,
                                    "prev_w": None, "strike": True, "step": True,
                                    "ty": _ty}
                                stage_log.append((round(t, 1),
                                    "ADVANCE leg%d -> y %.1f (gap %.1f)"
                                    % (_rl, _ty * 100, _gap5 * 100)))
                            _adv.add(_rl)
                            asc_hold["adv_t"] = t
                        _walkin9 = (len(_hands9) == 0
                                    and asc_hold.get("mids_fwd") is not None
                                    and len(asc_hold.get("mids_fwd", set())) >= 2
                                    and _gap5 > CRAWL["strike_gap"])
                        if (CRAWL["lib_press"] < 0.5
                                and (len(_hands9) == 2 or _walkin9)
                                and asc_hold.get("rehook") is None
                                and _bd is None and (len(_adv) >= 2 or _walkin9)
                                and t - asc_hold.get("body_done_t", -9.0) > CRAWL["body_rest"]
                                and _gap5 > CRAWL["body_gap"]):
                            _bd = {"t0": t, "anch": {}, "bad": 0, "walkin": _walkin9}
                            for _lg in range(6):
                                _aw0 = data.geom_xpos[model.geom(f"foot{_lg}").id].copy()
                                _c0, _o0, _d0 = world_to_leg(_lg, (float(_aw0[0]), float(_aw0[1]), float(_aw0[2])))
                                if (0.02 < _o0 < 0.135 and -0.108 < _d0 < 0.108
                                        and abs(_c0) < G.COXA_CMD_MAX
                                        and G.leg_ik(_o0, _d0) is not None):
                                    _bd["anch"][_lg] = _aw0       # only legs reachable at start
                            asc_hold["body"] = _bd
                            stage_log.append((round(t, 1),
                                "BODY: forward %.1f up %.1f cm on anchors %s (gap %.1f)"
                                % (CRAWL["body_dy"] * 100, CRAWL["body_dz"] * 100,
                                   sorted(_bd["anch"]), _gap5 * 100)))
                        if _bd is not None:
                            _pr = min(1.0, (t - _bd["t0"]) / CRAWL["body_T"])
                            # forward ALONG the inclined body: pure-forward
                            # offsets on a pitched stance drag him downhill
                            # (nose -30 in the walk-in, 2026-08-23)
                            _pb9, _ = rp_of(data.body("base").xquat)
                            _adv9 = CRAWL["body_dy"] * _pr
                            _off = (0.0,
                                    -_adv9 * math.cos(_pb9),
                                    -_adv9 * math.sin(_pb9) - CRAWL["body_dz"] * _pr)
                            _nbad = 0
                            _badl = []
                            for _lg, _aw in _bd["anch"].items():
                                if _lg in (1, 2):
                                    # hands keep their place (a 7 mm lip
                                    # cannot hold a hand pulled back 3 cm);
                                    # the floor legs push the body
                                    _tw = (float(_aw[0]), float(_aw[1]), float(_aw[2]))
                                else:
                                    _tw = (float(_aw[0]) + _off[0], float(_aw[1]) + _off[1],
                                           float(_aw[2]) + _off[2])
                                _cb, _ob, _db = world_to_leg(_lg, _tw)
                                if (0.02 < _ob < 0.135 and -0.108 < _db < 0.108
                                        and abs(_cb) < G.COXA_CMD_MAX
                                        and G.leg_ik(_ob, _db) is not None):
                                    _fl9[_lg] = (_cb, _ob, _db)
                                else:
                                    _nbad += 1
                                    _badl.append("%d:o%.3f d%.3f c%.2f" % (_lg, _ob, _db, _cb))
                            drv.v = 0.0
                            _tgt_gap = (CRAWL["strike_gap"] if _bd.get("walkin")
                                        else CRAWL["body_gap"])
                            _done = (_pr >= 1.0 or _gap5 <= _tgt_gap
                                     or (len(_bd["anch"]) - _nbad) < 2
                                     or (len(_hands9) < 2 and not _bd.get("walkin")))
                            if _done:
                                asc_hold["body"] = None
                                asc_hold["body_done_t"] = t
                                asc_hold["adv"] = set()
                                # re-base the hands where they now hold
                                for _h in _hands9:
                                    if foot_on_rung(_h):
                                        _lat9[_h]["w"] = data.geom_xpos[
                                            model.geom(f"foot{_h}").id].copy()
                                        _lat9[_h]["last"] = None
                                stage_log.append((round(t, 1),
                                    "BODY done: pr %.2f gap %.1f bad %d %s hands %d pitch %.1f base %.1f"
                                    % (_pr, _gap5 * 100, _nbad, _badl, len(_hands9),
                                       math.degrees(rp_of(data.body("base").xquat)[0]),
                                       float(data.body("base").xpos[2]) * 100)))
                        # STANCE FREEZE (2026-08-23): in hands-first climbing
                        # the walking gait is OFF; feet not in flight hold
                        # their BODY-frame positions (friction at the planted
                        # rubber feet carries the body). World anchors had
                        # only held the feet while the body drifted.
                        if CRAWL["lib_press"] < 0.5 and _bd is None:
                            drv.v = 0.0
                            drv.lift = 0.0
                        for _lg9, _lh9 in _lat9.items():
                            if _bd is not None:
                                continue                    # BODY owns the locks
                            _above9 = (float(_lh9["w"][2]) > float(
                                data.body(f"coxa{_lg9}").xpos[2]) + 0.01)
                            if _onw9 and _above9:
                                _lt5 = (float(_lh9["w"][0]), 0.3980,
                                        float(_lh9["w"][2]) - 0.006
                                        - (_pl9 if "_pl9" in dir() else 0.0))
                            elif _onw9:
                                # below the shoulder: squeeze in to the
                                # JAM point (lip front face 0.406 + foot
                                # radius 0.006), not behind the lip --
                                # 0.398 dragged a seated hand back over
                                # the 7 mm lip (collapse in 2 s)
                                _lt5 = (float(_lh9["w"][0]), 0.4120,
                                        float(_lh9["w"][2]))
                            else:
                                # held, not pulled: the anchor itself
                                _lt5 = (float(_lh9["w"][0]),
                                        float(_lh9["w"][1]),
                                        float(_lh9["w"][2]))
                            _c9, _o9, _d9 = world_to_leg(_lg9, _lt5)
                            if _o9 >= 0.135:
                                _o9 = 0.132
                            if (0.02 < _o9 < 0.135
                                    and -0.108 < _d9 < 0.108
                                    and abs(_c9) < G.COXA_CMD_MAX):
                                _lh9["last"] = (_c9, _o9, _d9)
                                _fl9[_lg9] = (_c9, _o9, _d9)
                            elif _lh9.get("last") is not None:
                                _fl9[_lg9] = _lh9["last"]
                        _rh9 = asc_hold.get("rehook")
                        if _rh9 is not None:
                            # a hook that starts ON a shelf flies the
                            # 5-phase over-enclosure path (~2x the
                            # length): give it 2x hook_T. Evolved 0.56 s
                            # was flown in 0.56 s and the foot lagged
                            # into rung 3's face (2026-08-22).
                            _hT9 = CRAWL["hook_T"] * (
                                2.0 if float(_rh9["from"][1])
                                > (0.4025 - 0.0035 - 0.009 + 0.004) else 1.0)
                            if _rh9.get("strike"):
                                _hT9 = CRAWL["strike_T"]
                            _rp9 = (t - _rh9["t0"]) / _hT9
                            _rf9, _rz9 = _rh9["from"], _rh9["to"]
                            # SETTLE: after the flight, keep pressing for
                            # up to settle_T; plant early the moment the
                            # hand honestly bears on its target rung
                            if _rp9 > 1.0:
                                _tn_s = next((n for n in (1, 2, 3, 4)
                                              if abs(_RSH6[n - 1] - _rz9)
                                              < 0.005), 0)
                                _early = (_tn_s > 0 and
                                          foot_on_rung_n(_rh9["leg"], _tn_s))
                                if (not _early and
                                        (t - _rh9["t0"]) < _hT9 + CRAWL["settle_T"]):
                                    _rp9 = 1.0          # hold the press target
                            # HOOK PATH INSTRUMENT: the highest the foot
                            # actually got, and any rung geom it touched
                            # on the way (a hook that lifts straight up
                            # from inside a trough meets the shelf above)
                            _wf9 = data.geom_xpos[model.geom(
                                f"foot{_rh9['leg']}").id]
                            if float(_wf9[2]) > _rh9.get("zmax", -1.0):
                                _rh9["zmax"] = float(_wf9[2])
                                _rh9["y_at_zmax"] = float(_wf9[1])
                            _rh9["ymin"] = min(_rh9.get("ymin", 9.0),
                                               float(_wf9[1]))
                            _fg9 = model.geom(f"foot{_rh9['leg']}").id
                            for _ci9 in range(data.ncon):
                                _cg9 = data.contact[_ci9]
                                _og9 = (_cg9.geom2 if _cg9.geom1 == _fg9
                                        else _cg9.geom1
                                        if _cg9.geom2 == _fg9 else -1)
                                if _og9 >= 0:
                                    _on9 = mujoco.mj_id2name(
                                        model, mujoco.mjtObj.mjOBJ_GEOM,
                                        _og9) or ""
                                    if "rung" in _on9 or "lip" in _on9:
                                        _rh9.setdefault("touched", set()
                                            ).add(_on9)
                            # mids cannot over-reach past the plane the
                            # way fronts can (their chase hooks fell to
                            # the floor at z 0.2): they hook to the
                            # trough CENTER with lower clearance
                            # mids land at the trough FRONT, just
                            # inside the lip: the center is 5 mm past
                            # their reach envelope at climbing gap and
                            # every center-target was silently
                            # infeasible (the leg simply dropped)
                            _oy9 = CRAWL["hook_y"] if _rh9["leg"] in (1, 2) else 0.4125
                            if _rh9.get("strike"):
                                _oy9 = max(_oy9, CRAWL["strike_y"])
                            if _rh9.get("ty") is not None:
                                _oy9 = float(_rh9["ty"])
                            # hook clearance over the rung is a GENE: at
                            # 4.2 cm the rung-3 apex sits ~1 cm outside
                            # the front envelope and every re-hook lands
                            # back on rung 2 (measured 2026-08-22)
                            _cl9 = (CRAWL["hook_clear"] if _rh9["leg"] in (1, 2)
                                    else CRAWL["mid_clear"])
                            # lip geometry (gary_mujoco rungN_lip): back
                            # face y = 0.4025 - 0.0035; a 6 mm foot
                            # retreats 3 mm clear of it, and first lifts
                            # 12 mm to clear the lip top (rung top + 7 mm
                            # + foot radius + margin)
                            _lipb9 = 0.4025 - 0.0035
                            _ry9 = _lipb9 - 0.006 - 0.003
                            _onshelf9 = float(_rf9[1]) > _ry9 + 0.004
                            _fx9_, _fy9_, _fz9_ = (hook_x(_rh9["leg"]),
                                                   float(_rf9[1]),
                                                   float(_rf9[2]))
                            _z1_9 = _fz9_ + 0.012
                            if "folded" not in _rh9:
                                _cf, _of, _df = world_to_leg(_rh9["leg"], (_fx9_, _fy9_, _fz9_))
                                _rh9["folded"] = math.hypot(_of - G.L1, _df) < 0.05
                            if _rh9.get("strike") and not _rh9.get("step") and not _rh9["folded"] and _rp9 < 0.18:
                                _rt9 = (_fx9_, _fy9_, _fz9_ + 0.01 * (_rp9 / 0.18))   # not folded: no swing-out
                                _rh9["out"] = _rt9
                            elif _rh9.get("step") and _rp9 < 0.25:
                                # a floor step LIFTS first (a swing-out on
                                # the floor dragged the body 7 cm)
                                _s9 = _rp9 / 0.25
                                _rt9 = (_fx9_, _fy9_, _fz9_ + (_rz9 + _cl9 - _fz9_) * _s9)
                                _rh9["out"] = _rt9
                            elif _rh9.get("step") and _rp9 < 0.75:
                                _s9 = (_rp9 - 0.25) / 0.50
                                _rt9 = (_fx9_, _fy9_ + (_oy9 - _fy9_) * _s9, _rz9 + _cl9)
                            elif _rh9.get("step") and _rp9 <= 1.0:
                                _s9 = (_rp9 - 0.75) / 0.25
                                _rt9 = (_fx9_, _oy9, _rz9 + _cl9 - (_cl9 + 0.006) * _s9)
                            elif _rh9.get("strike") and _rp9 < 0.18:
                                # a folded leg swings OUT before it goes up
                                # (crouched strike: 82/121 targets 'too
                                # close', 2026-08-23): move the foot away
                                # from its coxa, horizontally, 2.5 cm
                                _cw9 = data.body(f"coxa{_rh9['leg']}").xpos
                                _vx9, _vy9 = _fx9_ - float(_cw9[0]), _fy9_ - float(_cw9[1])
                                _vn9 = math.hypot(_vx9, _vy9) or 1.0
                                _s9 = _rp9 / 0.18
                                _rt9 = (_fx9_ + _vx9 / _vn9 * 0.025 * _s9,
                                        _fy9_ + _vy9 / _vn9 * 0.025 * _s9,
                                        _fz9_ + 0.01 * _s9)
                                _rh9["out"] = _rt9
                            elif _rh9.get("strike") and _rz9 > _RSH6[0] + 0.01 and _rp9 < 0.30:
                                # target above rung 1: go OUT SIDEWAYS first
                                # (lifting a foot 8 cm above a coxa 5 cm away
                                # has one IK solution: the femur folded 150
                                # deg over the back, into the chassis). The
                                # rung is 60 cm wide; beside the foot is free.
                                _s9 = (_rp9 - 0.18) / 0.12
                                _o0 = _rh9.get("out", (_fx9_, _fy9_, _fz9_))
                                _sgn_x = 1.0 if float(data.body(f"coxa{_rh9['leg']}").xpos[0]) > float(
                                    data.body("base").xpos[0]) else -1.0
                                _ry9s = (0.4025 - 0.0035) - CRAWL["strike_back"]
                                _rt9 = (_o0[0] + _sgn_x * CRAWL["strike_out"] * _s9,
                                        _o0[1] + (min(_o0[1], _ry9s) - _o0[1]) * _s9,
                                        _o0[2] + 0.005 * _s9)
                                _rh9["back"] = _rt9
                            elif _rh9.get("strike") and _rz9 > _RSH6[0] + 0.01 and _rp9 < 0.45:
                                # vertical lift behind the lips only until
                                # the foot clears rung 1's bar; a foot 10 cm
                                # above its coxa at 4 cm out needs the femur
                                # folded 150-200 deg over the body (IK check)
                                _s9 = (_rp9 - 0.30) / 0.15
                                _b0 = _rh9.get("back", (_fx9_, _fy9_, _fz9_))
                                _zc = _RSH6[0] + 0.012
                                _rt9 = (_b0[0], _b0[1], _b0[2] + (_zc - _b0[2]) * _s9)
                                _rh9["clear"] = _rt9
                            elif _rh9.get("strike") and _rz9 > _RSH6[0] + 0.01 and _rp9 < 0.85:
                                # then UP AND OUT: diagonal to the apex above
                                # the target's lip, the leg extending
                                _s9 = (_rp9 - 0.45) / 0.40
                                _b0 = _rh9.get("clear", (_fx9_, _fy9_, _fz9_))
                                _rt9 = (_b0[0] + (_fx9_ - _b0[0]) * _s9,
                                        _b0[1] + (_oy9 - _b0[1]) * _s9,
                                        _b0[2] + (_rz9 + _cl9 - _b0[2]) * _s9)
                            elif _rh9.get("strike") and _rz9 > _RSH6[0] + 0.01 and _rp9 <= 1.0:
                                _s9 = (_rp9 - 0.85) / 0.15
                                _rt9 = (_fx9_, _oy9, _rz9 + _cl9 - (_cl9 + 0.006) * _s9)
                            elif _rh9.get("strike") and _rp9 < 0.55:
                                # diagonal: from the swung-out foot to the
                                # apex a little behind the advance y
                                _s9 = (_rp9 - 0.18) / 0.37
                                _ay9 = _oy9 - 0.015
                                _o0 = _rh9.get("out", (_fx9_, _fy9_, _fz9_))
                                _rt9 = (_o0[0] + (_fx9_ - _o0[0]) * _s9,
                                        _o0[1] + (_ay9 - _o0[1]) * _s9,
                                        _o0[2] + (_rz9 + _cl9 - _o0[2]) * _s9)
                            elif _rh9.get("strike") and _rp9 < 0.75:
                                _s9 = (_rp9 - 0.55) / 0.20
                                _rt9 = (_fx9_,
                                        (_oy9 - 0.015) + 0.015 * _s9,
                                        _rz9 + _cl9)
                            elif _rh9.get("strike") and _rp9 <= 1.0:
                                _s9 = (_rp9 - 0.75) / 0.25
                                _rt9 = (_fx9_, _oy9,
                                        _rz9 + _cl9 - (_cl9 + 0.006) * _s9)
                            elif _onshelf9 and _rp9 < 0.12:
                                _s9 = _rp9 / 0.12
                                _rt9 = (_fx9_, _fy9_,
                                        _fz9_ + (_z1_9 - _fz9_) * _s9)
                            elif _onshelf9 and _rp9 < 0.30:
                                _s9 = (_rp9 - 0.12) / 0.18
                                _rt9 = (_fx9_,
                                        _fy9_ + (_ry9 - _fy9_) * _s9,
                                        _z1_9)
                            elif _onshelf9 and _rp9 < 0.60:
                                _s9 = (_rp9 - 0.30) / 0.30
                                _rt9 = (_fx9_, _ry9,
                                        _z1_9 + (_rz9 + _cl9 - _z1_9) * _s9)
                            elif _onshelf9 and _rp9 < 0.82:
                                _s9 = (_rp9 - 0.60) / 0.22
                                _rt9 = (_fx9_,
                                        _ry9 + (_oy9 - _ry9) * _s9,
                                        _rz9 + _cl9)
                            elif _onshelf9 and _rp9 <= 1.0:
                                _s9 = (_rp9 - 0.82) / 0.18
                                # PRESS: 6 mm below the seat (the last
                                # 2 mm hovered, touching nothing)
                                _rt9 = (_fx9_, _oy9,
                                        _rz9 + _cl9 - (_cl9 + 0.006) * _s9)
                            elif _rp9 < 0.4:
                                _s9 = _rp9 / 0.4
                                _rt9 = (_fx9_, float(_rf9[1]),
                                        float(_rf9[2]) + (_rz9 + _cl9
                                        - float(_rf9[2])) * _s9)
                            elif _rp9 < 0.7:
                                _s9 = (_rp9 - 0.4) / 0.3
                                _rt9 = (_fx9_,
                                        float(_rf9[1]) + (_oy9
                                        - float(_rf9[1])) * _s9,
                                        _rz9 + _cl9)
                            elif _rp9 <= 1.0:
                                _s9 = (_rp9 - 0.7) / 0.3
                                _rt9 = (_fx9_, _oy9,
                                        _rz9 + _cl9 - (_cl9 + 0.006) * _s9)
                            else:
                                _wl9 = data.geom_xpos[model.geom(
                                    f"foot{_rh9['leg']}").id].copy()
                                _tn9 = next((n for n in (1, 2, 3, 4)
                                             if abs(_RSH6[n - 1] - _rz9)
                                             < 0.005), 0)
                                _hit9 = (_tn9 > 0 and
                                         foot_on_rung_n(_rh9["leg"], _tn9))
                                _ret9 = None
                                if _rh9.get("step"):
                                    pass                        # a step: planted, not latched
                                elif _hit9 or _rh9.get("prev_w") is None:
                                    _lat9[_rh9["leg"]] = {"w": _wl9,
                                                          "last": None}
                                else:
                                    # MISSED: fly the hand BACK to its
                                    # previous hold (a hand left hanging
                                    # at rung 3 lost the wall within 15 s
                                    # in every run, 2026-08-23)
                                    _pw9 = _rh9["prev_w"]
                                    _pz9 = min(_RSH6, key=lambda z:
                                               abs(z - float(_pw9[2])))
                                    _ret9 = {"leg": _rh9["leg"], "t0": t,
                                             "from": _wl9.copy(), "to": _pz9,
                                             "prev_w": None, "ret": True}
                                    stage_log.append((round(t, 1),
                                        "HOOK MISSED leg%d: rung %d not "
                                        "borne -> returning to z %.1f"
                                        % (_rh9["leg"], _tn9, _pz9 * 100)))
                                stage_log.append((round(t, 1),
                                    "RE-HOOK leg%d planted y=%.1f z=%.1f (on rung: %s)"
                                    " | zmax %.1f at y %.1f, ymin %.1f, touched %s"
                                    " | bad %d/%d first: %s | shin %s"
                                    % (_rh9["leg"], _wl9[1] * 100, _wl9[2] * 100,
                                       foot_on_rung(_rh9["leg"]),
                                       _rh9.get("zmax", 0.0) * 100,
                                       _rh9.get("y_at_zmax", 0.0) * 100,
                                       _rh9.get("ymin", 0.0) * 100,
                                       sorted(_rh9.get("touched", ())),
                                       _rh9.get("bad", 0), _rh9.get("ticks", 0),
                                       _rh9.get("first_bad", "-"),
                                       shin_contacts(_rh9["leg"]))))
                                asc_hold["rehook"] = _ret9
                                _rt9 = None
                            if _rt9 is not None:
                                _c9, _o9, _d9 = world_to_leg(
                                    _rh9["leg"], _rt9)
                                _okf = (0.02 < _o9 < 0.135
                                        and -0.108 < _d9 < 0.108
                                        and abs(_c9) < G.COXA_CMD_MAX
                                        and G.leg_ik(_o9, _d9) is not None)
                                _rh9["ticks"] = _rh9.get("ticks", 0) + 1
                                # FLY trace: 10 samples per flight
                                _fk = int(_rp9 * 10)
                                if _rh9.get("strike") and _fk != _rh9.get("fly_k"):
                                    _rh9["fly_k"] = _fk
                                    _fa = data.geom_xpos[model.geom(f"foot{_rh9['leg']}").id]
                                    stage_log.append((round(t, 2),
                                        "FLY leg%d p%.1f tgt y%.1f z%.1f | act y%.1f z%.1f | %s o=%.3f d=%.3f c=%.2f"
                                        % (_rh9["leg"], _rp9, _rt9[1] * 100, _rt9[2] * 100,
                                           float(_fa[1]) * 100, float(_fa[2]) * 100,
                                           "ok" if _okf else "BAD", _o9, _d9, _c9)))
                                if _okf:
                                    asc_hold["rh_last"] = (_c9, _o9, _d9)
                                else:
                                    _rh9["bad"] = _rh9.get("bad", 0) + 1
                                    if "first_bad" not in _rh9:
                                        _rh9["first_bad"] = (
                                            "p%.2f c=%.2f o=%.3f d=%.3f D=%.3f"
                                            % (_rp9, _c9, _o9, _d9,
                                               math.hypot(_o9 - G.L1, _d9)))
                                if asc_hold.get("rh_last") is not None:
                                    _fl9[_rh9["leg"]] = asc_hold["rh_last"]
                        # THE LINE (2026-08-22): the rhythm's commands --
                        # latch tension, pull beats, every hook path --
                        # were handed to the driver only above the 0.34
                        # rad gate, which the mount never reaches. They
                        # were computed and discarded one line before
                        # the servos, all campaign. Same on-wall law as
                        # the block's entry.
                        if (_onw9 or _lat9 or asc_hold.get("rehook") is not None
                                or CRAWL["lib_press"] < 0.5 or asc_hold.get("body") is not None):
                            drv.foot_lock = _fl9 if _fl9 else None
                            _rhl = asc_hold.get("rehook")
                            if _rhl is not None and _rhl.get("strike") and int(t * 8) != asc_hold.get("lock_dbg"):
                                asc_hold["lock_dbg"] = int(t * 8)
                                _lg = _rhl["leg"]
                                stage_log.append((round(t, 2),
                                    "LOCK leg%d in=%s lock=%s keys=%s"
                                    % (_lg, _lg in _fl9,
                                       tuple(round(v, 3) for v in _fl9.get(_lg, ())),
                                       sorted(_fl9))))
            elif drill == "jump":
                # CAN HE JUMP? (Bill: "maybe we should see if we can get him
                # to jump first") — crouch deep, then explode every leg.
                if t < 1.0:
                    drv.height = 0.0
                elif t < 1.35:
                    drv.height = -0.022            # coil
                elif t < 1.55:
                    drv.height = 0.045             # EXPLODE
                else:
                    drv.height = 0.0
            elif drill == "wall":
                # GEN-3: THE THREE-POINT RATCHET (Bill's law + the world-
                # anchor fix). Alternating cycle: anchor one front on a
                # rung, then the BODY RISES PAST THE PLANTED FOOT (world-
                # anchored hold compensates), then the other front anchors
                # higher, release-and-re-anchor higher still. Stillness
                # while a foot steps; motion only during rises.
                if t <= 2.4:
                    drv.press = 0
                    drv.tpc = None
                    drv.leg_h = [0.0] * 6
                    drv.v = 0.024
                else:
                    _ct = t - 2.4
                    g3 = {**TPC, **(genes or {})}   # missing keys fall back
                    # to defaults — gene-file/controller version skew has
                    # crashed the watcher twice
                    GK = 0.028                      # ground-level kdz
                    R = (0.046, 0.091, 0.136, 0.157)
                    _a, _r = g3["arc_s"], g3["rise_s"]
                    _c = 0.0
                    SCHED = []
                    for _i, (_lg, _fr, _to) in enumerate((
                            (1, GK, R[0]), (2, GK, R[1]),
                            (1, R[0], R[2]), (2, R[1], R[3]))):
                        SCHED.append((_c, _c + _a, "arc", _lg, _fr, _to))
                        _c += _a
                        SCHED.append((_c, _c + _r, "walk", _i * 0.25, (_i + 1) * 0.25))
                        _c += _r
                    SCHED.append((_c, 11.5, "summit", 0, 0))
                    # PROGRESSIVE GATHER (Bill, three times: "he's not
                    # doing anything to keep himself close with his FOUR
                    # back legs"): keep-close RAMPS with climb progress —
                    # cumulative haul toward the wall as height builds,
                    # slack during arcs so steps stay free
                    _gath = g3["gather"] * min(1.0, _ct / 6.0)
                    drv.body_dy = g3["dy"] + _gath
                    drv.body_pivot_y = 0.0
                    drv.v = g3["v_hold"]
                    # the advance/standoff trade (Bill: "not walking into
                    # the wall enough" vs arcs jamming when crowded) is
                    # GENE territory — hand-tuning oscillated
                    for seg in SCHED:
                        if seg[0] <= _ct < seg[1]:
                            if seg[2] == "arc":
                                _, _, _, lg, frm, to = seg
                                pr = (_ct - seg[0]) / (seg[1] - seg[0])
                                tpc_state[lg] = {"arc": {"from": frm,
                                                         "to": to, "pr": pr}}
                            elif seg[2] == "summit":
                                # full wedge + keep walking: the summit IS
                                # Bill's wedge at max depth, driven forward
                                # ("his back legs should crouch, his middle
                                # legs tall, and they should walk him
                                # forward")
                                drv.leg_h[4] = drv.leg_h[5] = -g3["rc"]
                                drv.leg_h[0] = drv.leg_h[3] = g3["mt"]
                                drv.body_dy = g3["dy"] + _gath + 0.008
                                drv.v = g3["v_sum"]
                            else:                     # "walk": deepen the
                                p0, p1 = seg[3], seg[4]   # wedge + WALK HIM
                                pr = (_ct - seg[0]) / (seg[1] - seg[0])
                                _wp = p0 + (p1 - p0) * min(1.0, pr)
                                drv.leg_h[4] = drv.leg_h[5] = -g3["rc"] * _wp
                                drv.leg_h[0] = drv.leg_h[3] = g3["mt"] * _wp
                                drv.v = g3["v_rise"]
                            break
                        elif _ct >= seg[1] and seg[2] == "arc":
                            lg, to = seg[3], seg[5]
                            st_now = tpc_state.get(lg)
                            if st_now is None or st_now.get("arc") is not None:
                                tpc_state[lg] = {"rung": to,
                                                 "plant_h": drv.height}
                    drv.tpc = tpc_state
            elif drill == "press" or (drill == "ladder" and 2.6 < t < 6.1):
                _pt = t - (1.0 if drill == "press" else 2.6)
                # v4 = A CLIMB (Bill: "close enough to try to climb it,
                # basically, to put his weight on it"): plant the fronts ON
                # the panel one at a time (press=3 stages them), and only
                # once the wall carries the nose, load and drive. No
                # free-standing rear — the belly-flop era is over.
                if _pt <= 1.6:                     # CREEP: the climb must
                    drv.press = 0                  # begin with CONTACT —
                    drv.v = 0.024                  # started out of reach,
                    drv.body_pitch = 0.0           # the whole climb happened
                    drv.body_dy = 0.0              # in the air and he folded
                else:                              # flat ("just laying down")
                    drv.press = 3
                    drv.v = 0.012                  # rear four walk in under
                    drv.body_pitch = min(0.35, 0.10 * max(0.0, _pt - 3.2))
                    drv.body_dy = min(0.03, 0.015 * max(0.0, _pt - 3.2))
                    # pitch FOLLOWS the feet (they must own the wall first),
                    # never leads them
            elif drill == "pop" or (drill == "ladder" and 6.1 < t < 12.0):
                # PUMP, not ram (Bill: the back-and-forth looked
                # "discombobulated... no real intent"): stay CLIMBED onto
                # the door (press=3 held) and pulse the load — weight
                # surging rhythmically through planted front feet.
                _qt = t - (1.0 if drill == "pop" else 6.1)
                # BODY SLAMS — the honest escalation. The pump discovered a
                # physics law: you cannot push hard through the arms you
                # lean on (braced position-servos cap thrust at ~13 deg).
                # The slam works because the body flies free. What makes it
                # look DELIBERATE (vs Bill's "discombobulated bulldozer") is
                # the clean cycle: straight back-step, neutral posture, then
                # the committed launch. No scooting, no flail.
                _ph = _qt % 1.15
                if _ph < 0.5:                      # clean back-step
                    drv.press = 0
                    drv.body_pitch = 0.0
                    drv.body_dy = 0.0
                    drv.vx = 0.0
                    drv.v = -0.020
                else:                              # committed launch
                    drv.press = 0
                    drv.body_pitch = 0.06
                    drv.v = 0.050
            elif drill != "ladder" or t >= 12.0:
                drv.press = 0
                drv.body_pitch = 0.0
                drv.body_dy = 0.0
                drv.v = 0.0
            if tele_cb is not None and s % 25 == 0:
                tele_cb(dict(t=t, door=door_deg(),
                             pitch=math.degrees(m_pitch),
                             footz=float(data.geom_xpos[foot1][2])))
        if max_t is not None and t >= max_t:
            break
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        s += 1
        kick_peak_z = max(kick_peak_z, float(data.geom_xpos[foot1][2]))
        base_peak = max(base_peak, float(data.body('base').xpos[2]))
        base_sum += float(data.body('base').xpos[2]); base_n += 1
        if (drill == "rear" and rear_ph >= 2 and t - rear_t0 > 1.2
                and abs(rp_of(data.body("base").xquat)[0]) < 0.035
                and float(data.site_xpos[nose_id][2]) < 0.105
                and not (rear_ph >= 5
                         and (asc_hold.get("stg") != "climb"
                              or (CRAWL["lib_press"] < 0.5
                                  and t - asc_hold.get("climb_t0", -9) < 6.0)
                              or t - max(asc_hold.get("rung_t", -9),
                                         max(asc_hold.get("rung_seen",
                                                          {0: -9}).values()))
                              < 1.2))):
            # FELL OFF THE POSE/WALL -- rep dead, restart (Bill: 'if he
            # falls off the wall, restart it'). During the ASCENT the
            # honest fall test is ANCHORS LOST THE RUNGS: the crawl-in
            # legitimately flattens pitch while the body comes in and
            # up, and the old flat-pitch check was killing live climbs.
            stage_log.append((round(t, 1),
                "FAULT flat-pitch: pitch %.1f deg, rung recency %.1fs"
                % (math.degrees(rp_of(data.body("base").xquat)[0]),
                   t - max(asc_hold.get("rung_t", -9),
                           max(asc_hold.get("rung_seen", {0: -9}).values())))))
            fault = True
            break
        if (s % 25 == 0 and t > 1.0 and drill != "getup" and belly_now()
                and not (drill == "rear" and rear_ph >= 5
                         and float(data.site_xpos[nose_id][2]) > 0.12)):
            # tail-touch with the NOSE HIGH is the ass-down wedge
            # working as designed, not a belly collapse -- the fault
            # was executing the commanded posture at 6.6 s every run
            belly_ticks += 1
            if belly_ticks >= 8:           # ~0.4 s of true chassis-
                stage_log.append((round(t, 1), "FAULT belly-down"))
                fault = True               # down = FAULT: the rep is
                break                      # dead (Bill: 'if his
                # belly flattens on the ground, redo')
        elif s % 25 == 0:
            belly_ticks = 0
        if os.environ.get("CRAWL_PROBE") and (abs(t - 12.0) < 0.0011
                                              or abs(t - 21.0) < 0.0011):
            import sys as _sys
            _fc = np.zeros(6)
            for _ci in range(data.ncon):
                _c = data.contact[_ci]
                for _fi in range(6):
                    if model.geom(f"foot{_fi}").id in (_c.geom1, _c.geom2):
                        _F6 = np.zeros(6)
                        mujoco.mj_contactForce(model, data, _ci, _F6)
                        _fc[_fi] += abs(_F6[0])
            print("PROBE t=%.1f body y=%.3f z=%.3f pitch=%.2f" % (
                t, data.body('base').xpos[1], data.body('base').xpos[2],
                rp_of(data.body('base').xquat)[0]), file=_sys.stderr)
            for _fi in range(6):
                _fw = data.geom_xpos[model.geom(f"foot{_fi}").id]
                _tq = data.actuator_force[3*_fi:3*_fi+3]
                print("  leg%d N=%5.1f foot y=%+.3f z=%.3f  sum_dy=%+.3f  tq=(%+.2f,%+.2f,%+.2f)" % (
                    _fi, _fc[_fi], _fw[1], _fw[2],
                    drv.body_dy + drv.leg_dy[_fi],
                    _tq[0], _tq[1], _tq[2]), file=_sys.stderr)
        nose_sum += float(data.site_xpos[nose_id][2])
        if drill == 'jump' and all(float(data.geom_xpos[g][2]) > 0.012
                                   for g in all_feet):
            air_s += model.opt.timestep
        door_peak = max(door_peak, door_deg())
        if renderer is not None and s % 16 == 0:
            cam = mujoco.MjvCamera(); cam.lookat[:] = data.body("base").xpos
            cc = cam_ctl or {}
            cam.distance = cc.get("distance", 0.7)
            cam.elevation = cc.get("elevation", -20)
            cam.azimuth = cc.get("azimuth", 160)
            renderer.update_scene(data, camera=cam)
            frame_cb(renderer.render())
            lag = (wall0 + t) - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
    if drill == "breach" and breach_fsm is not None:
        slam_n = breach_fsm.slam_n
        breach_done = breach_fsm.through_t
        defeat_noise[0] = breach_fsm.defeat_noise
    res = dict(drill=drill, fault=fault, getup_s=getup_done,
               slams=slam_n, breached_at=breach_done,
               defeated=defeat_noise[0], stages=stage_log,
               nose_end_cm=round(float(data.site_xpos[nose_id][2]) * 100, 1),
               gap_end_cm=round((0.405 - float(
                   data.body('base').xpos[1])) * 100, 1),
               foot1_peak_cm=round(kick_peak_z * 100, 1),
               body_pitch_peak_deg=round(math.degrees(pitch_peak), 1),
               door_peak_deg=round(door_peak, 1),
               door_final_deg=round(door_deg(), 1),
               base_peak_cm=round(base_peak * 100, 1),
               airborne_ms=round(air_s * 1000),
               base_mean_cm=round(base_sum / max(1, base_n) * 100, 1),
               nose_mean_cm=round(nose_sum / max(1, base_n) * 100, 1),
               # ENDING STATE, honest: the highest rung each leg bears on
               # when the clock runs out (0 = not on the ladder). The
               # fitness scores THIS, not beat-top touches that ended on
               # the floor (notes/WALL-DEADLOCK-2026-08-22.md, fact 6).
               end_rungs=[next((n for n in (4, 3, 2, 1)
                                if foot_on_rung_n(_l, n)), 0)
                          for _l in range(6)],
               end_pitch_deg=round(math.degrees(
                   rp_of(data.body("base").xquat)[0]), 1))
    if verbose:
        print(json.dumps(res))
    return res


WALL_GENES = dict(creep_t=2.4, climb_v=0.014, dy=0.026, height=0.0,
                  pitch_rate=0.12, pitch_delay=0.8)
WALL_BOUNDS = dict(creep_t=(1.8, 3.4), climb_v=(0.004, 0.026),
                   dy=(0.0, 0.042), height=(-0.012, 0.028),
                   pitch_rate=(0.0, 0.22), pitch_delay=(0.0, 3.0))


def evolve_tpc(gens=12, lam=6, seed=3):
    import numpy as _np
    rng = _np.random.default_rng(seed)
    best = dict(TPC)
    bs = wall_score(run_drill("wall", verbose=False, genes=best))
    print(f"seed score {bs:.1f}", flush=True)
    for g in range(1, gens + 1):
        sig = 0.28 * (0.84 ** (g - 1))
        for _ in range(lam):
            kid = {k: float(_np.clip(best[k] + rng.normal(0, sig * (hi - lo)),
                                     lo, hi))
                   for k, (lo, hi) in TPC_BOUNDS.items()}
            r = run_drill("wall", verbose=False, genes=kid)
            sc = wall_score(r)
            print(f"  g{g} {sc:7.1f} foot={r['foot1_peak_cm']} "
                  f"nose={r['nose_mean_cm']} pitch={r['body_pitch_peak_deg']}",
                  flush=True)
            if sc < bs:
                bs, best = sc, kid
        print(f"g{g} BEST {bs:.1f}", flush=True)
        json.dump(dict(best=best, score=bs),
                  open(os.path.join(HERE, "out", "tpc_climb.json"), "w"))
    return best


def wall_score(r):
    # UP the wall: feet high, NOSE high (the mast's real altitude — a
    # tall-assed nosedive can't fake it), nose-UP pitch only (signed;
    # the abs() fitness let evolution launder a 22.6-deg dive as a
    # climb), and NOT laying down.
    lay_pen = max(0.0, 4.6 - r["base_mean_cm"]) * 4.0
    return -(r["foot1_peak_cm"] * 1.0
             + max(0.0, r["body_pitch_peak_deg"]) * 0.35
             + (r["nose_mean_cm"] - 11.5) * 3.0 - lay_pen)


def evolve_wall(gens=10, lam=8, seed=3):
    import numpy as _np
    rng = _np.random.default_rng(seed)
    best = dict(WALL_GENES)
    bs = wall_score(run_drill("wall", verbose=False, genes=best))
    print(f"seed score {bs:.1f}  {best}", flush=True)
    for g in range(1, gens + 1):
        sig = 0.30 * (0.82 ** (g - 1))
        for _ in range(lam):
            kid = {k: float(_np.clip(best[k] + rng.normal(0, sig * (hi - lo)),
                                     lo, hi))
                   for k, (lo, hi) in WALL_BOUNDS.items()}
            r = run_drill("wall", verbose=False, genes=kid)
            s = wall_score(r)
            print(f"  g{g} {s:7.1f} foot={r['foot1_peak_cm']} pitch={r['body_pitch_peak_deg']} base={r['base_mean_cm']}", flush=True)
            if s < bs:
                bs, best = s, kid
        print(f"g{g} BEST {bs:.1f}  {best}", flush=True)
        json.dump(dict(best=best, score=bs),
                  open(os.path.join(HERE, "out", "wall_climb.json"), "w"))
    return best


def _crawl_fitness(r):
    """Baseline-honest + RATCHET-honest: only NEW-HIGH verified plants
    count (a rung-2 tapping metronome farmed 37 plants for 296 points
    -- pays once now), weighted by rung: 1/2/4/8 for rungs 1-4. Nose
    term is measured against the TALL-STANCE ceiling (~18), endings
    only. Faults are death."""
    import re as _re
    best_by_leg = {}
    ratchet = 0.0
    best_shelf = {}
    for _t, s in r["stages"]:
        mo = _re.search(r"OWNED: leg(\d) bears on shelf (\d)", s)
        if mo is not None:
            leg, sh = int(mo.group(1)), int(mo.group(2))
            if sh > best_shelf.get(leg, 0):
                best_shelf[leg] = sh
                ratchet += float(1 << (sh - 1))
            continue
        m = _re.search(r"(?:CLAW|HAND|leg)\s*leg?(\d) planted at z=(-?\d+\.\d) \(on rung: True\)", s)
        if m is None:
            m = _re.search(r"leg(\d) planted at z=(-?\d+\.\d) cm \(on rung: True\)", s)
        if m is None:
            continue
        leg, z = int(m.group(1)), float(m.group(2))
        if z > best_by_leg.get(leg, 0.0) + 2.0:
            best_by_leg[leg] = z
            ratchet += (8.0 if z >= 17.0 else 4.0 if z >= 13.0
                        else 2.0 if z >= 8.5 else 1.0)
    nose = float(r.get("nose_end_cm", 0.0))
    gap = float(r.get("gap_end_cm", 18.0))
    # ENDING STATE pays; touches only steer. candB's 10-gen ES went
    # 17.5 -> 73.2 on beat-top 'bears on shelf N' touches while nose_end
    # FELL and he ended on the floor. Now a rung counts when the leg is
    # still bearing on it at the end: 1/2/4/8 per leg for rungs 1-4,
    # a mid (0/3) on any rung is the campaign's last centimeter (+15),
    # and the transient ratchet is worth a fifth of what it was.
    end = list(r.get("end_rungs") or [0] * 6)
    end_pts = sum(float(1 << (n - 1)) for n in end if n > 0)
    mid_on = any(end[_l] > 0 for _l in (0, 3))
    plants = sum(1 for n in end if n > 0)
    return (2.0 * (nose - 18.0) + 0.5 * (18.0 - gap)
            + 10.0 * end_pts + 15.0 * mid_on + 2.0 * ratchet
            - 30.0 * bool(r.get("fault"))), gap, nose, plants


def evolve_crawl(gens=40):
    import copy, random
    random.seed(11)
    SPAN = dict(v=(0.008, 0.045), lift=(0.010, 0.035),
                lean=(0.02, 0.07), h_base=(0.0, 0.025),
                h_k=(0.3, 1.5), p_base=(0.06, 0.28), p_k=(0.5, 3.5),
                gate1=(0.13, 0.19), gate2=(0.115, 0.165),
                gate3=(0.10, 0.14), gate4=(0.088, 0.12),
                promo_cool=(0.8, 4.0), demote_t=(1.2, 4.5),
                seen_t=(0.25, 1.0), reach_cap=(0.05, 0.11),
                ass_down=(0.0, 0.042), tall_f=(0.0, 0.030),
                f_lift=(0.0, 0.05), m_lift=(0.0, 0.035),
                supp_k=(0.0, 1.1), rear_k=(0.0, 0.9),
                conf_need=(0.10, 0.8), retry_t=(1.5, 5.0),
                slip_t=(2.0, 6.0), shift_gap=(0.085, 0.115),
                dig_y=(0.008, 0.03), dig_z=(0.004, 0.024),
                shift_v=(0.012, 0.04), shift_lean=(0.04, 0.08),
                beat_T=(1.2, 4.0), pull_z=(0.004, 0.032),
                hook_T=(0.35, 1.2), knee_max=(95.0, 180.0),
                knee_climb=(95.0, 180.0),
                lunge_gate=(0.18, 0.40), pitch_max=(0.45, 1.35),
                hook_y=(0.408, 0.4245),
                pose_gap=(0.095, 0.150), pose_v=(0.0, 0.030),
                pose_rear=(-0.050, 0.0), pose_mid=(0.0, 0.065),
                pose_front=(0.0, 0.075), pose_lean=(-0.075, 0.0),
                lib_v=(0.0, 0.045), ram_T=(0.20, 0.80),
                ram_h=(0.0, 0.045), ram_ext=(0.0, 0.045),
                ram_mid=(0.0, 0.050), ram_lean=(0.030, 0.140),
                pose_lift=(0.0, 0.025), lib_press=(0.0, 1.0), strike_T=(0.4, 1.5),
                settle_T=(0.0, 1.0), strike_y=(0.412, 0.426), hook_dx=(0.02, 0.08),
                col_dx=(-0.03, 0.04), col_dy=(-0.04, 0.04),
                body_dy=(0.01, 0.06), body_dz=(0.0, 0.04), body_T=(1.0, 5.0),
                body_gap=(0.08, 0.125), body_rest=(0.5, 4.0), col_d=(0.05, 0.105),
                adv_dy=(0.01, 0.07), strike_rung=(1.0, 2.0), stand_h=(-0.015, 0.04), stand_T=(0.3, 2.0),
                pose_Tmax=(4.0, 20.0), col_c=(-0.3, 0.9), mid_dy=(0.0, 0.06), strike_back=(0.009, 0.035), strike_out=(0.0, 0.07),
                incl_front=(0.0, 0.05), incl_mid=(-0.02, 0.03), incl_rear=(-0.03, 0.01), strike_gap=(0.100, 0.140),
                nod_on=(0.0, 1.0), nod_amp=(0.005, 0.035),
                sec_lean=(0.0, 0.07), sec_mid=(0.0, 0.06), sec_rear=(-0.03, 0.02),
                hook_clear=(0.015, 0.050),
                mid_clear=(0.008, 0.040))
    best = copy.deepcopy(CRAWL)
    CRAWL_LIVE = CRAWL

    def score(g):
        CRAWL_LIVE.clear()
        CRAWL_LIVE.update(g)
        r = run_drill(drill="rear", verbose=False)
        return _crawl_fitness(r)

    bs, bg, bz, bp = score(best)
    print("gen 0 incumbent: %.1f (gap %.1f, nose %.1f, plants %d)"
          % (bs, bg, bz, bp), flush=True)
    for gen in range(1, gens + 1):
        for k in range(3):
            mut = copy.deepcopy(best)
            for name, (lo, hi) in SPAN.items():
                if random.random() < 0.45:
                    mut[name] = min(hi, max(lo, mut[name]
                                            + random.gauss(0, (hi - lo) * 0.18)))
            s, g_, z_, p_ = score(mut)
            print("  gen %d.%d: %.1f (gap %.1f, nose %.1f, plants %d)"
                  % (gen, k, s, g_, z_, p_), flush=True)
            if s > bs:
                bs, bg, bz, bp = s, g_, z_, p_
                best = mut
                print("  ^ NEW BEST", flush=True)
        json.dump(best, open(_gp_c, "w"), indent=1)
    CRAWL_LIVE.clear()
    CRAWL_LIVE.update(best)
    print("best: %.1f (gap %.1f, nose %.1f, plants %d) -> %s"
          % (bs, bg, bz, bp, _gp_c), flush=True)



if __name__ == "__main__":
    ap = argparse.ArgumentParser()

    ap.add_argument("--drill", default="ladder",
                    choices=["kick", "press", "pop", "ladder", "wall", "jump", "rear", "getup", "breach"])
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--resist", type=float, default=0.02)
    ap.add_argument("--evolve-wall", action="store_true")
    ap.add_argument("--locked", action="store_true", help="tier-3: hinge blocked -- the defeat rehearsal")
    ap.add_argument("--evolve-tpc", action="store_true")
    ap.add_argument("--evolve-crawl", action="store_true",
                    help="(1+3)-ES over the crawl-in genes; fitness = gap closed + altitude + on-rung plants - faults")
    a = ap.parse_args()
    if a.evolve_crawl:
        evolve_crawl()
        raise SystemExit(0)
    genes_w = None
    gp_w = os.path.join(HERE, "out", "tpc_climb.json")
    if a.drill == "wall" and os.path.exists(gp_w):
        genes_w = json.load(open(gp_w))["best"]
        print(f"[GYM] evolved wall genes: {genes_w}")
    if a.evolve_tpc:
        evolve_tpc()
    elif a.evolve_wall:
        evolve_wall()
    elif a.watch:
        import importlib
        from watch_nav import make_world_window, show_frame, SpectatorClosed
        import matplotlib.pyplot as plt
        CAM = dict(azimuth=160.0, elevation=-20.0, distance=0.7)
        WW = make_world_window(CAM)
        tfig, tax = plt.subplots(num="Door gym — telemetry", figsize=(6.4, 3.0))
        d_ln, = tax.plot([], [], color="#b5651d", lw=2, label="door (deg)")
        p_ln, = tax.plot([], [], color="#2d7fe8", lw=1.6, label="body pitch (deg)")
        f_ln, = tax.plot([], [], color="#3fa34d", lw=1.2, label="front foot z (cm)")
        tax.legend(fontsize=8); tfig.tight_layout(); tfig.show()
        T = dict(t=[], door=[], pitch=[], footz=[])

        def tele(d):
            T["t"].append(d["t"]); T["door"].append(d["door"])
            T["pitch"].append(d["pitch"]); T["footz"].append(d["footz"] * 100)
            d_ln.set_data(T["t"], T["door"]); p_ln.set_data(T["t"], T["pitch"])
            f_ln.set_data(T["t"], T["footz"])
            tax.relim(); tax.autoscale_view()
            tfig.canvas.draw_idle(); tfig.canvas.flush_events()

        import door_gym as DG
        rep = 0
        while True:
            importlib.reload(DG)
            rep += 1
            for k in T:
                T[k].clear()
            print(f"[GYM] rep {rep} ({a.drill})...")
            try:
                if a.drill == "wall" and os.path.exists(gp_w):
                    genes_w = json.load(open(gp_w))["best"]
                r = DG.run_drill(a.drill, resist=a.resist, cam_ctl=CAM,
                                 frame_cb=lambda fr: show_frame(WW, fr),
                                 tele_cb=tele, genes=genes_w, locked=a.locked)
            except SpectatorClosed:
                break
            print(f"[GYM] {r}")
            if r.get("defeated"):
                try:
                    import winsound
                    for f in (880, 660, 440, 262):
                        winsound.Beep(f, 210)      # the sad trombone,
                except Exception:                  # as heard by Bill
                    pass
            time.sleep(1.5)
    else:
        run_drill(a.drill, genes=genes_w)
