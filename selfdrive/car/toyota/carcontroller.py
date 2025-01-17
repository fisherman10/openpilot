from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_interceptor_command, make_can_msg
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command, create_lta_steer_command
from selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, EV_HYBRID_CAR
from opendbc.can.packer import CANPacker
from common.realtime import DT_CTRL
VisualAlert = car.CarControl.HUDControl.VisualAlert
from common.features import Features

# EPS faults if you apply torque while the steering rate is above 100deg/s for too long
MAX_STEER_RATE = 100
MAX_STEER_RATE_FRAMES = 18

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

PUMP_RESET_INTERVAL = 1.5
PUMP_RESET_DURATION = 0.1

class BrakingStatus():
  STANDSTILL_INIT = 0
  BRAKE_HOLD = 1
  PUMP_RESET = 2

# reset pump every PUMP_RESET_INTERVAL seconds for. Reset to zero for PUMP_RESET_DURATION
def standstill_brake(min_accel, ts_last, ts_now, prev_status):
  brake = min_accel
  status = prev_status

  dt = ts_now - ts_last
  if prev_status == BrakingStatus.PUMP_RESET and dt > PUMP_RESET_DURATION:
    status = BrakingStatus.BRAKE_HOLD
    ts_last = ts_now

  if prev_status == BrakingStatus.BRAKE_HOLD and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if prev_status == BrakingStatus.STANDSTILL_INIT and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if status == BrakingStatus.PUMP_RESET:
    brake = 0

  return brake, status, ts_last

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_limited = False
    self.steer_rate_counter = 0

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

    # standstill globals
    self.prev_ts = 0.
    self.standstill_status = BrakingStatus.STANDSTILL_INIT
    self.min_standstill_accel = 0

    f = Features()
    self.force_use_stock_acc = f.has("StockAcc")

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart, laneActive):

    ts = frame * DT_CTRL
    lat_active = active and laneActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # gas and brake
    if CS.CP.enableGasInterceptor and active:
      MAX_INTERCEPTOR_GAS = 0.5
      # RAV4 has very sensitive gas pedal
      if CS.CP.carFingerprint in (CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER, CAR.HIGHLANDERH):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif CS.CP.carFingerprint in (CAR.COROLLA,):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.
    pcm_accel_cmd = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    # throttle accel if it goes above 3200 rpm, happens with Toyota Alphards and Vellfires
    if CS.rpm > 3200 and active and CS.CP.carFingerprint not in EV_HYBRID_CAR:
      pcm_accel_cmd = 0.2

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, CarControllerParams)
    self.steer_rate_limited = new_steer != apply_steer

    # allow stock lane departure prevention passthrough
    stockLdw = CS.out.stockAdas.laneDepartureHUD
    if stockLdw:
      apply_steer = CS.out.stockAdas.ldpSteerV


    # count up to MAX_STEER_RATE_FRAMES, at which point we need to cut torque to avoid a steering fault
    if lat_active and abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE:
      self.steer_rate_counter += 1
    else:
      self.steer_rate_counter = 0

    apply_steer_req = 1
    if not stockLdw and not lat_active:
      apply_steer = 0
      apply_steer_req = 0
    elif self.steer_rate_counter > MAX_STEER_RATE_FRAMES:
      apply_steer_req = 0
      self.steer_rate_counter = 0

    # TODO: probably can delete this. CS.pcm_acc_status uses a different signal
    # than CS.cruiseState.enabled. confirm they're not meaningfully different
    if not enabled and CS.pcm_acc_status:
      pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_steer = apply_steer
    self.last_standstill = CS.out.standstill

    can_sends = []

    if frame < 1000:
      can_sends.append(make_can_msg(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))

    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
    if frame % 2 == 0 and CS.CP.carFingerprint in TSS2_CAR:
      can_sends.append(create_lta_steer_command(self.packer, 0, 0, frame // 2))

    # LTA mode. Set ret.steerControlType = car.CarParams.SteerControlType.angle and whitelist 0x191 in the panda
    # if frame % 2 == 0:
    #   can_sends.append(create_steer_command(self.packer, 0, 0, frame // 2))
    #   can_sends.append(create_lta_steer_command(self.packer, actuators.steeringAngleDeg, apply_steer_req, frame // 2))

    # we can spam can to cancel the system even if we are using lat only control
    if (frame % 3 == 0 and CS.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = lead or CS.out.vEgo < 12.    # at low speed we always assume the lead is present so ACC can be engaged

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and CS.CP.carFingerprint in (CAR.LEXUS_IS, CAR.LEXUS_RC):
        can_sends.append(create_acc_cancel_command(self.packer))
      # Lexus 2018 no unplug DSU + using KommuActuator
      elif CS.CP.carFingerprint in (CAR.LEXUS_NX):
        if CS.out.standstill and (pcm_accel_cmd > 0):
          # Send a quick gas command to resume SNG
          can_sends.append(make_can_msg(512, b'\x01\x2F\x01\x2F\x00\x01\x00\x00', 0))
      elif CS.CP.openpilotLongitudinalControl:

        # standstill logic
        if enabled and pcm_accel_cmd > 0 and CS.out.standstill and CS.CP.carFingerprint == CAR.COROLLA_TSS2:
          if self.standstill_status == BrakingStatus.STANDSTILL_INIT:
            self.min_standstill_accel = pcm_accel_cmd - 0.1
          pcm_accel_cmd, self.standstill_status, self.prev_ts = standstill_brake(self.min_standstill_accel, self.prev_ts, ts, self.standstill_status)
        else:
          self.standstill_status = BrakingStatus.STANDSTILL_INIT
          self.prev_ts = ts

        if self.force_use_stock_acc and CS.out.vEgo > 1:
          pcm_accel_cmd = CS.stock_acc_cmd

        can_sends.append(create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type, CS.distance_btn))
        self.accel = pcm_accel_cmd
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type, CS.distance_btn))

    if frame % 2 == 0 and CS.CP.enableGasInterceptor and CS.CP.openpilotLongitudinalControl:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, frame // 2))
      self.gas = interceptor_gas_cmd

    # ui mesg is at 1Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    fcw_alert = hud_alert == VisualAlert.fcw
    steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw) or stockLdw

    send_ui = False
    if ((fcw_alert or steer_alert) and not self.alert_active) or \
       (not (fcw_alert or steer_alert) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    elif pcm_cancel_cmd:
      # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
      send_ui = True

    if (frame % 100 == 0 or send_ui):
      can_sends.append(create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, left_line, right_line, left_lane_depart, right_lane_depart, enabled))

    if frame % 100 == 0 and CS.CP.enableDsu:
      can_sends.append(create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for (addr, cars, bus, fr_step, vl) in STATIC_DSU_MSGS:
      if frame % fr_step == 0 and CS.CP.enableDsu and CS.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.accel = CS.stock_acc_cmd if (self.force_use_stock_acc and CS.out.vEgo > 1) else self.accel
    new_actuators.gas = self.gas

    return new_actuators, can_sends
