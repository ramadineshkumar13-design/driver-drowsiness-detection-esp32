# -*- coding: utf-8 -*-
"""
Unit test verification for Phase 3: Hierarchical State Machine & Alert Engine
Validates:
  1. NORMAL state stability
  2. Escalation from NORMAL -> WARNING on apnea + steering inactivity
  3. Recovery from WARNING -> NORMAL on driver steering correction
  4. Escalation from WARNING -> CRITICAL after 5.0s timeout
  5. Emergency steering override recovery in CRITICAL
  6. Escalation from CRITICAL -> EMERGENCY upon PWM reaching 0%
"""

STATE_NORMAL = 0
STATE_WARNING = 1
STATE_CRITICAL = 2
STATE_EMERGENCY = 3

class MockStateMachine:
    def __init__(self):
        self.state = STATE_NORMAL
        self.time_in_state = 0
        self.warning_timer = 0
        self.motor_speed = 100.0
        self.ramp_in_progress = False
        self.ramp_step = 0.0

    def start_ramp(self, target, duration_ms):
        self.target_speed = target
        delta = target - self.motor_speed
        self.ramp_step = delta / float(duration_ms)
        self.ramp_in_progress = True

    def update_ramp(self, delta_ms):
        if not self.ramp_in_progress:
            return True
        new_speed = self.motor_speed + (self.ramp_step * float(delta_ms))
        if self.ramp_step < 0 and new_speed <= self.target_speed:
            self.motor_speed = self.target_speed
            self.ramp_in_progress = False
            return True
        self.motor_speed = new_speed
        return False

    def step(self, irreg_breath, steer_inact, delta_ms):
        self.time_in_state += delta_ms
        prev_state = self.state

        recovered = (not steer_inact) and (not irreg_breath)

        if self.state == STATE_NORMAL:
            if irreg_breath and steer_inact:
                self.state = STATE_WARNING
                self.time_in_state = 0
                self.warning_timer = 0

        elif self.state == STATE_WARNING:
            if recovered or (not steer_inact):
                self.state = STATE_NORMAL
                self.time_in_state = 0
                self.warning_timer = 0
            else:
                self.warning_timer += delta_ms
                if self.warning_timer >= 5000:
                    self.state = STATE_CRITICAL
                    self.time_in_state = 0
                    self.start_ramp(0.0, 4000)

        elif self.state == STATE_CRITICAL:
            if not steer_inact: # Driver override
                self.state = STATE_NORMAL
                self.time_in_state = 0
                self.ramp_in_progress = False
                self.motor_speed = 100.0
            else:
                ramp_done = self.update_ramp(delta_ms)
                if ramp_done or self.motor_speed <= 0.01:
                    self.state = STATE_EMERGENCY
                    self.time_in_state = 0
                    self.motor_speed = 0.0

        elif self.state == STATE_EMERGENCY:
            if recovered and self.time_in_state > 5000:
                self.state = STATE_NORMAL
                self.time_in_state = 0
                self.motor_speed = 100.0

        return self.state

def run_fsm_tests():
    print("===========================================================")
    print("       STATE MACHINE TRANSITION & ESCALATION TESTS        ")
    print("===========================================================")
    fsm = MockStateMachine()
    passed = 0
    total = 0

    # 1. Normal state remains normal
    total += 1
    s = fsm.step(irreg_breath=False, steer_inact=False, delta_ms=100)
    assert s == STATE_NORMAL and fsm.motor_speed == 100.0
    print("  [PASS] Test 1: Normal driving remains in STATE_NORMAL @ 100% speed")
    passed += 1

    # 2. Only irregular breathing, but steering ACTIVE -> stays NORMAL
    total += 1
    s = fsm.step(irreg_breath=True, steer_inact=False, delta_ms=100)
    assert s == STATE_NORMAL
    print("  [PASS] Test 2: Irregular breathing with ACTIVE steering does not falsely escalate")
    passed += 1

    # 3. Irregular breathing AND Steering Inactive -> escalates to WARNING
    total += 1
    s = fsm.step(irreg_breath=True, steer_inact=True, delta_ms=100)
    assert s == STATE_WARNING
    print("  [PASS] Test 3: Irregular breathing + Inactive steering escalates to STATE_WARNING")
    passed += 1

    # 4. Driver twitches steering in WARNING -> recovers to NORMAL
    total += 1
    s = fsm.step(irreg_breath=True, steer_inact=False, delta_ms=100)
    assert s == STATE_NORMAL
    print("  [PASS] Test 4: Driver steering correction in WARNING triggers immediate recovery to NORMAL")
    passed += 1

    # 5. Escalate back to WARNING, let 5.0s timeout elapse -> escalates to CRITICAL
    total += 1
    fsm.step(irreg_breath=True, steer_inact=True, delta_ms=100) # enters WARNING
    for _ in range(49): # 4.9s
        s = fsm.step(irreg_breath=True, steer_inact=True, delta_ms=100)
    assert s == STATE_WARNING, "Should still be warning at 4.9s"
    # 5.0s tick
    s = fsm.step(irreg_breath=True, steer_inact=True, delta_ms=100)
    assert s == STATE_CRITICAL
    print("  [PASS] Test 5: 5.0s unresponded warning escalates to STATE_CRITICAL")
    passed += 1

    # 6. In CRITICAL, motor decelerates over 4.0 seconds down to 0% -> enters EMERGENCY
    total += 1
    for _ in range(39): # 3.9s of ramp
        s = fsm.step(irreg_breath=True, steer_inact=True, delta_ms=100)
        assert s == STATE_CRITICAL
        assert fsm.motor_speed > 0.0
    # 4.0s tick: ramp reaches 0%
    s = fsm.step(irreg_breath=True, steer_inact=True, delta_ms=100)
    assert s == STATE_EMERGENCY and fsm.motor_speed == 0.0
    print("  [PASS] Test 6: Controlled PWM deceleration down to 0% triggers STATE_EMERGENCY")
    passed += 1

    # 7. In CRITICAL, steering override test
    total += 1
    fsm2 = MockStateMachine()
    fsm2.step(True, True, 100) # WARNING
    for _ in range(50): fsm2.step(True, True, 100) # CRITICAL
    assert fsm2.state == STATE_CRITICAL
    # Driver jerks wheel!
    s = fsm2.step(irreg_breath=True, steer_inact=False, delta_ms=100)
    assert s == STATE_NORMAL and fsm2.motor_speed == 100.0
    print("  [PASS] Test 7: Emergency steering jerk in CRITICAL aborts decel and restores NORMAL")
    passed += 1

    print("===========================================================")
    print(" Test Summary: %d / %d Tests Passed (100%%)" % (passed, total))
    print("===========================================================")

if __name__ == '__main__':
    run_fsm_tests()
