"""GPIO numbers of the comma SDM845 platform (tizi = comma 3X, mici = comma four).

Mirrors openpilot/common/hardware/comma/pins.py.
"""


class GPIO:
  # both STM_RST_N and LTE_RST_N are misnamed, they are high to reset
  HUB_RST_N = 30
  UBLOX_RST_N = 32
  UBLOX_SAFEBOOT_N = 33
  GNSS_PWR_EN = 34  # schematic label: GPIO_UBLOX_PWR_EN

  STM_RST_N = 124
  STM_BOOT0 = 134
  STM_PWR_EN_N = 41

  SIREN = 42
  SOM_ST_IO = 49  # keeps the fan powered until shutdown

  LTE_RST_N = 50
  LTE_PWRKEY = 116
  LTE_BOOT = 52

  CAM0_AVDD_EN = 8
  CAM0_RSTN = 9
  CAM1_RSTN = 7
  CAM2_RSTN = 12

  # IMU data-ready interrupt (line offset on gpiochip0)
  LSM_INT = 84
