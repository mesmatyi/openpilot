#!/usr/bin/env python3
"""Sole bench publisher of explicitly synthetic, stationary vehicle inputs."""
import time

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.system.manager.bench import LEASE_TIMEOUT, fresh, hardware_safe


def get_bench_car_params():
  CP = car.CarParams.new_message()
  CP.brand = "mock"
  CP.carFingerprint = "BENCH CAMERA MODE - NOT A VEHICLE"
  CP.notCar = True
  CP.passive = True
  CP.safetyConfigs = [car.CarParams.SafetyConfig.new_message(safetyModel=car.CarParams.SafetyModel.noOutput)]
  return CP


def main():
  params = Params()
  CP = get_bench_car_params()
  pm = messaging.PubMaster(["carState", "carParams"])
  sm = messaging.SubMaster(["pandaStates"])
  rk = Ratekeeper(100)
  initialized = False
  try:
    while params.get_bool("BenchMode") and params.get_bool("BenchModeRequested"):
      sm.update(0)
      now = time.monotonic()
      if not fresh(params.get("BenchModeLease"), now, LEASE_TIMEOUT):
        break
      if not hardware_safe(sm, now):
        # Allow the first real panda message to arrive, without claiming readiness.
        if sm.seen["pandaStates"]:
          break
        rk.keep_time()
        continue

      cs = messaging.new_message("carState", valid=True)
      cs.carState.standstill = True
      # CAN is absent, not a successfully validated vehicle bus. Cruise remains off.
      cs.carState.canValid = False
      pm.send("carState", cs)
      if not initialized:
        params.put("CarParams", CP.to_bytes(), block=True)
      if not initialized or rk.frame % 100 == 0:
        cp = messaging.new_message("carParams", valid=True)
        cp.carParams = CP
        pm.send("carParams", cp)
      if not initialized or rk.frame % 10 == 0:
        params.put("BenchAdapterHeartbeat", now, block=True)
      initialized = True
      rk.keep_time()
  finally:
    params.remove("BenchAdapterHeartbeat")


if __name__ == "__main__":
  main()
