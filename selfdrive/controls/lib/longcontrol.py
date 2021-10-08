from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.pid import LongPIController
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.modeld.constants import T_IDXS
from selfdrive.config import Conversions as CV
from selfdrive.car.gm.values import MIN_ACC_SPEED
from selfdrive.ntune import ntune_scc_get

# mpc에서 계산된 타겟속도로 맞추기 위해 PID를 이용한 gas/brake 값을 산출

LongCtrlState = log.ControlsState.LongControlState


#STOPPING_EGO_SPEED = 2.0
#STOPPING_TARGET_SPEED_OFFSET = 1.0
#STARTING_TARGET_SPEED = 1.0

# [Neokii88...]
STOPPING_EGO_SPEED = 0.6
STOPPING_TARGET_SPEED_OFFSET = 0.01
STARTING_TARGET_SPEED = 0.5

BRAKE_THRESHOLD_TO_PID = 0.2
#REGEN_THRESHOLD = 0.02

# apply at least this amount of brake to maintain the vehicle stationary
# 차량을 정지 상태로 유지하려면 최소한 이 정도의 브레이크를 적용하십시오.
BRAKE_STOPPING_TARGET = 0.5

RATE = 100.0
#DEFAULT_LONG_LAG = 0.15


def long_control_state_trans(active, long_control_state, v_ego, v_target, v_pid,
                             output_gb, brake_pressed, cruise_standstill, min_speed_can):
  """Update longitudinal control state machine"""
  stopping_target_speed = min_speed_can + STOPPING_TARGET_SPEED_OFFSET
  stopping_condition = (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and (
                                 v_pid < stopping_target_speed and v_target < stopping_target_speed))

  starting_condition = v_target > STARTING_TARGET_SPEED and not cruise_standstill

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if active:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.starting

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif output_gb >= -BRAKE_THRESHOLD_TO_PID:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP, compute_gb):
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = LongPIController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                                (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                                (CP.longitudinalTuning.kfBP, CP.longitudinalTuning.kfV),
                                rate=RATE,
                                sat_limit=0.8,
                                convert=compute_gb)
    self.v_pid = 0.0
    self.last_output_gb = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, CP, long_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    # TODO estimate car specific lag, use .5s for now
    if len(long_plan.speeds) == CONTROL_N:
      #v_target = interp(DEFAULT_LONG_LAG, T_IDXS[:CONTROL_N], long_plan.speeds)
      #v_target_future = long_plan.speeds[-1]
      #a_target = interp(DEFAULT_LONG_LAG, T_IDXS[:CONTROL_N], long_plan.accels)
      longitudinalActuatorDelay = ntune_scc_get("longitudinalActuatorDelay")
      v_target = interp(longitudinalActuatorDelay, T_IDXS[:CONTROL_N], long_plan.speeds)
      v_target_future = long_plan.speeds[-1]
      a_target = 2 * (v_target - long_plan.speeds[0]) / longitudinalActuatorDelay - long_plan.accels[0]
    else:
      v_target = 0.0
      v_target_future = 0.0
      a_target = 0.0


    # Actuation limits (작동 제한)
    gas_max = interp(CS.vEgo, CP.gasMaxBP, CP.gasMaxV)
    brake_max = interp(CS.vEgo, CP.brakeMaxBP, CP.brakeMaxV)

    # Update state machine
    output_gb = self.last_output_gb
    self.long_control_state = long_control_state_trans(active, self.long_control_state, CS.vEgo,
                                                       v_target_future, self.v_pid, output_gb,
                                                       CS.brakePressed, CS.cruiseState.standstill, CP.minSpeedCan)

    v_ego_pid = max(CS.vEgo, CP.minSpeedCan)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off or CS.gasPressed or not CS.adaptiveCruise or CS.brakePressed \
            or CS.vEgo <= MIN_ACC_SPEED / CV.MS_TO_KPH or ntune_scc_get('adaptiveCruise') == 0:
      self.reset(v_ego_pid)
      output_gb = 0.

    # tracking objects and driving (물체 추적 및 운전)
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target
      self.pid.pos_limit = gas_max
      self.pid.neg_limit = - brake_max

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7
      deadzone = interp(v_ego_pid, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)

      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=prevent_overshoot)

      if prevent_overshoot:
        output_gb = min(output_gb, 0.)

    # Intention is to stop, switch to a different brake control until we stop
    # 의도는 멈출 때까지 다른 브레이크 컨트롤로 전환하는 것입니다.
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped (차가 멈출 때까지 브레이크를 계속 밟으십시오)
      if not CS.standstill or output_gb > -BRAKE_STOPPING_TARGET:
        output_gb -= CP.stoppingBrakeRate / RATE
      output_gb = clip(output_gb, -brake_max, gas_max)

      self.reset(CS.vEgo)

    # Intention is to move again, release brake fast before handing control to PID
    # 다시 이동하려는 의도이며 PID에 제어를 넘기기 전에 브레이크를 빨리 해제하십시오.
    elif self.long_control_state == LongCtrlState.starting:
      if output_gb < -0.2:
        output_gb += CP.startingBrakeRate / RATE
      self.reset(CS.vEgo)

    self.last_output_gb = output_gb
    final_gas = clip(output_gb, 0., gas_max)
    final_brake = -clip(output_gb, -brake_max, 0.)

    return final_gas, final_brake, v_target, a_target
