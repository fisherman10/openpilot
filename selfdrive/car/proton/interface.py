#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.proton.values import CAR
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

from common.params import Params

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "proton"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.proton)]
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.radarOffCan = True
    ret.enableApgs = False                 # advanced parking guidance system
    ret.enableDsu = False                  # driving support unit

    ret.steerRateCost = 0.85                # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.10          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')
   
    ret.enableGasInterceptor = 0x201 in fingerprint[0] or 0x401 in fingerprint[0]
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.X50:
      ret.wheelbase = 2.6
      ret.steerRatio = 13.8660
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 1370. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1

      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [600]] #maximum is 600 only. Any more it will disengage and reengage on its own.

      ret.lateralTuning.pid.kpBP = [0., 25., 35., 40.]
      ret.lateralTuning.pid.kpV = [0.06, 0.15, 0.186, 0.186]
      ret.lateralTuning.pid.kiBP = [0., 20., 25., 30., 40.]
      ret.lateralTuning.pid.kiV = [0.03, 0.05, 0.06, 0.06, 0.08]
      ret.lateralTuning.pid.kf = 0.000141845

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [0, 0, 0]
      ret.longitudinalActuatorDelayLowerBound = 0.42
      ret.longitudinalActuatorDelayUpperBound = 0.60

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    # Todo
    ret.longitudinalTuning.deadzoneBP = [0., 8.05]
    ret.longitudinalTuning.deadzoneV = [0, 0]
    ret.longitudinalTuning.kiBP = [0., 5., 20.]
    ret.longitudinalTuning.kiV = [0, 0, 0]

    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.001 # reach stopping target smoothly

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # to receive CAN Messages
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)

    # create events for auto lane change below allowable speed
    if ret.vEgo < LANE_CHANGE_SPEED_MIN and (ret.leftBlinker or ret.rightBlinker):
      events.add(EventName.belowLaneChangeSpeed)

    if self.CS.hand_on_wheel_warning and self.CS.is_icc_on:
      events.add(EventName.protonHandOnWheelWarning)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c):

    isLdw = c.hudControl.leftLaneDepart or c.hudControl.rightLaneDepart

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators, c.hudControl.leadVisible, c.hudControl.rightLaneVisible, c.hudControl.leftLaneVisible, c.cruiseControl.cancel, isLdw)

    self.frame += 1
    return can_sends
