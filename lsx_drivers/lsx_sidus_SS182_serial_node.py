#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi drivers (nepi_drivers) repo
# (see https://github.com/nepi-engine/nepi_drivers)
#
# License: NEPI Drivers repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


### Set the namespace before importing nepi_sdk
import os
import copy
import serial
import serial.tools.list_ports
import time
import re
import sys
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_serial

from nepi_interfaces.msg import DeviceLSXStatus

from nepi_api.device_if_lsx import LSXDeviceIF
from nepi_api.messages_if import MsgIF




PKG_NAME = 'LSX_SIDUS_SS182_SERIAL'
FILE_TYPE = 'NODE'


DEFAULT_MIN = '1'
DEFAULT_MAX = '50'



#########################################
# Sidus SS182 LSX Driver Node Class
#########################################

class SidusSS182SerialNode(object):
  ### LXS Driver Settings
  # Set driver capability parameters

  #######################
  CAP_SETTINGS = dict(
    min_intensity_percent = {"type":"Int","name":"min_intensity_percent","options":["1","100"]},
    max_intensity_percent =  {"type":"Int","name":"max_intensity_percent","options":["1","100"]}
  )

  FACTORY_SETTINGS = dict(
    min_intensity_percent = {"type":"Int","name":"min_intensity_percent","value":str(DEFAULT_MIN)},
    max_intensity_percent =  {"type":"Int","name":"max_intensity_percent","value": str(DEFAULT_MAX)}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  settingFunctions = dict(
    min_intensity_percent = {'get':'getMinIntensityPercent', 'set': 'setMinIntensityPercent'},
    max_intensity_percent = {'get':'getMaxIntensityPercent', 'set': 'setMaxIntensityPercent'}
  )

  #Factory Control Values
  FACTORY_CONTROLS = dict( standby_enabled = False,
  on_off_state = False,
  intensity_ratio = 0.0,
  strobe_enabled = False,
  blink_interval_sec = 2,
  blink_enabled = False
  )

  # Serial timing, measured on an SS182mkII at 9600 baud. A reply arrives 0.1
  # to 0.86 s after the command (the manual allows up to 1 s for a NAK), and
  # the light ignores a command that arrives within about 0.5 s of its previous
  # reply; after a 1.0 s quiet gap it always answered. send_msg waits out
  # QUIET_SEC after every transaction, so the bus carries one command every
  # 1.5 to 2.5 s.
  REPLY_TIMEOUT_SEC = 1.5
  QUIET_SEC = 1.0
  SERIAL_READ_TIMEOUT_SEC = 0.1
  # The check timer sends a pending on/off or intensity change as soon as the
  # bus is free, and otherwise reads the temperature every POLL_INTERVAL_SEC.
  CHECK_INTERVAL_SEC = 0.25
  POLL_INTERVAL_SEC = 3.0
  # The power-on intensity (LPO) follows the last level the light was on at,
  # so after a power cycle it comes back at that brightness before NEPI
  # connects. It is written once the level has been steady this long, so a
  # slider drag does not queue writes to the light's memory.
  LPO_SETTLE_SEC = 5.0
  ALARM_LOG_THROTTLE_SEC = 10

  # Reply DATA codes. ERR0-3 reject the command. ERR4-9 are health alarms: while
  # one is active it replaces the value in every reply.
  ERR_CODES_DICT = {
    0: 'ERR0 invalid command',
    1: 'ERR1 value out of range',
    2: 'ERR2 missing parameters',
    3: 'ERR3 invalid R/W',
    4: 'ERR4 LED intensity reduced for LED temperature',
    5: 'ERR5 LED intensity reduced for ambient temperature',
    6: 'ERR6 humidity inside the light is over 80 percent',
    7: 'ERR7 internal pressure is over 1500 mbar',
    8: 'ERR8 water detected inside the light. Remove power from the light as soon as possible',
    9: 'ERR9 input voltage out of range'
  }

  # LIR selects the intensity scale for LIN, LMX and LIL
  LIR_FULL_SCALE_DICT = {1: 999, 0: 255}
  DEFAULT_FULL_SCALE = 999

  TEMP_MIN_C = -40
  TEMP_MAX_C = 85
  TEMP_SENTINEL = 255 # reported until the first valid reading

  device_info_dict = dict(device_name = "",
                        path = "",
                        serial_number = "",
                        hw_version = "",
                        sw_version = "")

  init_settings_dict = dict()
  settings_dict = dict()

  # Initialize some parameters
  serial_num = ""
  hw_version = ""
  sw_version = ""

  connect_attempts = 0


  serial_port = None
  serial_lock = None
  control_lock = None
  connected = False

  on_off_state = False
  standby_state = False
  intensity_ratio = 0.0
  target_ratio = 0.0
  strobe_state = False

  temp_c = TEMP_SENTINEL

  # Light state the light last confirmed; None until known. The light turns
  # itself on at power-up and no command reads whether it is on.
  applied_on = None
  applied_level = None
  applied_lpo = None   # power-on intensity on the light, read at connect
  last_on_level = None # last level above 0 the light was on at
  last_on_time = 0.0
  rejected_action = None # an ERR0-3 rejected command, not resent until the request changes
  last_bus_end = 0.0
  last_poll_time = 0.0

  self_check_count = 5
  self_check_counter = 0 # track sequencial message failures

  lsx_if = None

  addr_str = ""

  # Device state read at connect
  dak_mode = None
  full_scale = DEFAULT_FULL_SCALE
  max_level = None   # LMX, operator maximum
  limit_level = None # LIL, factory limit

  min_intensity_percent = int(DEFAULT_MIN)
  max_intensity_percent = int(DEFAULT_MAX)

  CONFIGS_DICT = {
        'Standard' : {'data_len': 4},
  }
  config_dict = CONFIGS_DICT['Standard']
  data_len = 4
  ### LXS Driver NODE Initialization
  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()
  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF()
    self.msg_if.pub_info("Starting Node Initialization Processes")

  ##############################
  # Initialize Class Variables
    self.serial_lock = threading.Lock()
    self.control_lock = threading.Lock()
    self.device_info_dict = copy.deepcopy(self.device_info_dict)

    # Get required drv driver dict info

    try:
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        #self.msg_if.pub_warn("Got Drivers_Dict from param server: " + str(self.drv_dict))
        self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
        self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
        self.port_str = self.device_path
        self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str']
        self.baud_int = int(self.baud_str)
        self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str']

        system_config = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
        if system_config in self.CONFIGS_DICT.keys():
            self.config_dict = self.CONFIGS_DICT[system_config]
        self.data_len = self.config_dict['data_len']
    except Exception as e:
        self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
        return

    ################################################
    self.msg_if.pub_info("Connecting to Device on port " + self.port_str + " with baud " + self.baud_str + " at address " + self.addr_str)
    ### Try and connect to device
    while self.connected == False and self.connect_attempts < 5:
        self.connected = self.connect()
        if self.connected == False:
            nepi_sdk.sleep(1)
    if self.connected == False:
        self.msg_if.pub_info("Shutting down node")
        self.msg_if.pub_info("Specified serial port not available")
        nepi_sdk.signal_shutdown("Serial port not available")
    else:
        ################################################
        self.msg_if.pub_info("... Connected!")
        self.dev_info = self.driver_getDeviceInfo()
        self.logDeviceInfo()
        # Initialize settings
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()

        # Launch the LSX interface. Its start-up pushes the on_off_state and
        # intensity_ratio params through turnOnOff and setIntensityRatio, so the
        # light, which turns itself on at power-up, is set to the saved state.
        self.msg_if.pub_info("Launching NEPI LSX () interface...")
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        self.lsx_if = LSXDeviceIF(
                    device_info = self.device_info_dict,
                    getStatusFunction = self.getStatus,
                    getSettingsFunction=self.getSettingsFunction,
                    setSettingFunction=self.setSettingFunction,
                    factoryControls = self.FACTORY_CONTROLS,
                    standbyEnableFunction = None,
                    turnOnOffFunction = self.turnOnOff,
                    setIntensityRatioFunction = self.setIntensityRatio,
                    blinkOnOffFunction = None,
                    reports_temp = True,
                    reports_power = False
                    )

        # Start an activity check process that kills node after some number of failed comms attempts
        self.msg_if.pub_info("Starting an activity check process")
        nepi_sdk.start_timer_process(self.CHECK_INTERVAL_SEC, self.check_timer_callback)
        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever (until object is detected)
        nepi_sdk.spin()




      #**********************
      # Device setting functions

  def logDeviceInfo(self):
      dev_info_string = self.node_name + " Device Info:\n"
      dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
      dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
      dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
      dev_info_string += "Hardware Version: " + self.dev_info["HardwareVersion"] + "\n"
      dev_info_string += "Serial Number: " + self.dev_info["SerialNum"] + "\n"
      self.msg_if.pub_info(dev_info_string)


  def initSettingsDict(self):
      init_settings_dict = dict()
      for setting_name in self.CAP_SETTINGS.keys():
        cap_setting = self.CAP_SETTINGS[setting_name]
        setting_dict = dict()
        setting_dict['type'] = cap_setting['type']
        # The retired cap-settings form carried an Int control's min and max in
        # an 'options' pair. The controls contract calls that 'bounds'.
        if 'options' in cap_setting.keys():
          try:
            setting_dict['bounds'] = [int(cap_setting['options'][0]),int(cap_setting['options'][1])]
          except Exception as e:
            self.msg_if.pub_warn("Invalid bounds for setting: " + setting_name + " : " + str(e))
        default = None
        if setting_name in self.FACTORY_SETTINGS.keys():
          try:
            default = int(self.FACTORY_SETTINGS[setting_name]['value'])
          except Exception as e:
            self.msg_if.pub_warn("Invalid factory value for setting: " + setting_name + " : " + str(e))
        if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
          default = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
        if default is None:
          continue
        setting_dict['default'] = default
        init_settings_dict[setting_name] = setting_dict

      self.init_settings_dict = init_settings_dict
      settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
      settings_dict_values = nepi_controls.get_values_dict(settings_dict)
      self.msg_if.pub_info("Initialized Settings: " + str(settings_dict_values))
      return settings_dict


  def refreshSettingsDict(self):
      # Both settings are held by the driver, not the device, so this only
      # reads current values back. Bounds and options do not move.
      settings_dict = copy.deepcopy(self.settings_dict)
      for setting_name in settings_dict.keys():
        if setting_name not in self.settingFunctions.keys():
          continue
        try:
          get_function = getattr(self, self.settingFunctions[setting_name]['get'])
          val = get_function()
        except Exception as e:
          self.msg_if.pub_warn("Failed to read setting " + setting_name + " : " + str(e))
          continue
        if val is None:
          continue
        try:
          settings_dict = nepi_controls.set_value(settings_dict, setting_name, int(val))
        except Exception as e:
          self.msg_if.pub_warn("Failed to apply read setting " + setting_name + " : " + str(e))
      return settings_dict


  def getSettingsFunction(self):
      return self.settings_dict


  def setSettingFunction(self,setting_name,setting_value):
      setting_str = setting_name + ":" + str(setting_value)
      success = False
      msg = 'Success'
      if setting_name not in self.settings_dict.keys():
        msg = (self.node_name + " Setting name " + setting_str + " is not supported")
        return False, msg, self.settings_dict
      if setting_name not in self.settingFunctions.keys():
        msg = (self.node_name + " Setting name " + setting_str + " has no set function")
        return False, msg, self.settings_dict

      try:
        set_function = getattr(self, self.settingFunctions[setting_name]['set'])
        # The set functions return a bare success flag
        success = (set_function(setting_value) == True)
        if success == True:
          msg = ( self.node_name + " UPDATED SETTINGS " + setting_str)
        else:
          msg = ( self.node_name + " rejected setting " + setting_str)
      except Exception as e:
        msg = "Failed to set " + setting_str + " : " + str(e)
        self.msg_if.pub_warn(msg)

      self.settings_dict = self.refreshSettingsDict()
      return success, msg, self.settings_dict

    ##############
    ### Settings Functions
    # min and max intensity percent map the 0-1 intensity ratio onto the
    # light's intensity scale in the driver. Neither is written to the light:
    # the light's own maximum (LMX) is never changed here.

  def getMinIntensityPercent(self):
    return self.min_intensity_percent


  def setMinIntensityPercent(self, val):
    percent = self.getValidPercent(val)
    if percent is None:
      return False
    self.min_intensity_percent = percent
    return True


  def getMaxIntensityPercent(self):
    return self.max_intensity_percent


  def setMaxIntensityPercent(self, val):
    percent = self.getValidPercent(val)
    if percent is None:
      return False
    self.max_intensity_percent = percent
    return True


  def getValidPercent(self, val):
    try:
      percent = int(float(val))
    except Exception:
      self.msg_if.pub_warn("Intensity percent setting is not a number: " + str(val))
      return None
    if percent < 1 or percent > 100:
      self.msg_if.pub_warn("Intensity percent setting must be 1 to 100, got: " + str(val))
      return None
    return percent


  def getIntensityRange(self):
    min_percent = float(self.min_intensity_percent)
    max_percent = max(float(self.max_intensity_percent), min_percent)
    return min_percent, max_percent


  def ratioToLevel(self, intensity_ratio):
    if intensity_ratio <= 0:
      return 0
    intensity_ratio = min(float(intensity_ratio), 1.0)
    [min_percent, max_percent] = self.getIntensityRange()
    percent = min_percent + intensity_ratio * (max_percent - min_percent)
    level = int(round(percent / 100.0 * self.full_scale))
    for limit in [self.max_level, self.limit_level]:
      if limit is not None and level > limit:
        level = limit
    return max(0, min(level, self.full_scale))



  #######################
  ### LSX IF Get Status Function

  ### Status callback
  def getStatus(self):
    # No serial traffic here: LSXDeviceIF calls this at 1 Hz and one
    # transaction takes 1.5 to 2.5 s. The check timer keeps the values current.
    # Create LSX status message
    status_msg= DeviceLSXStatus()
    status_msg.device_node_name = self.node_name
    status_msg.device_name = self.device_info_dict["device_name"]
    status_msg.device_path= self.device_info_dict["path"]
    status_msg.serial_num = self.device_info_dict["serial_number"]
    status_msg.hw_version = self.device_info_dict["hw_version"]
    status_msg.sw_version = self.device_info_dict["sw_version"]
    status_msg.on_off_state = self.on_off_state
    status_msg.standby_state = self.standby_state
    status_msg.intensity_ratio = self.intensity_ratio
    status_msg.strobe_state = self.strobe_state
    status_msg.blink_state = False
    status_msg.blink_interval = 0
    # DeviceLSXStatus.temp_c is uint8, which cannot carry a sub-zero reading
    status_msg.temp_c = max(0, min(int(self.temp_c), 255))
    status_msg.power_w = 0
    return(status_msg)


  def update_status_values(self):
    """Read the housing temperature from the light.

    Called from the check timer, never from getStatus. temp_c is the DTM 0003
    housing temperature; a failed read or an active health alarm keeps the
    last good value.

    Returns:
        bool: True if the light answered, including with an alarm or a
            rejection; False on a timeout.
    """
    # DTM 0003 is the current housing temperature
    ser_msg = ('&' + self.addr_str + 'DTM' + self.create_zero_prefix_str(3) + 'R')
    response = self.send_msg(ser_msg)
    if response is None:
      return False
    temp_c = None
    if self.getReplyData(ser_msg, response) is not None:
      try:
        temp_c = int(response[6:9])
      except ValueError:
        self.msg_if.pub_warn("Temperature reply was not a number: " + str(response), throttle_s = 10)
    if temp_c is not None and temp_c >= self.TEMP_MIN_C and temp_c <= self.TEMP_MAX_C:
      self.temp_c = temp_c
    return True

  #######################
  ### LSX IF Interface Functions
  # These only store the request. The check timer sends LIN, LON or LOF as
  # soon as the bus is free, so a slider drag becomes one LIN write of the
  # latest value instead of a queue of 2 s transactions.
  def turnOnOff(self,turn_on_off):
    with self.control_lock:
      self.on_off_state = (turn_on_off == True)
      self.rejected_action = None
    return True


  def setIntensityRatio(self,intensity_ratio):
    try:
      intensity_ratio = float(intensity_ratio)
    except Exception:
      self.msg_if.pub_warn("Intensity ratio is not a number: " + str(intensity_ratio))
      return False
    if intensity_ratio < 0:
      intensity_ratio = 0.0
    elif intensity_ratio > 1:
      intensity_ratio = 1.0
    with self.control_lock:
      self.target_ratio = intensity_ratio
      self.intensity_ratio = intensity_ratio
      self.rejected_action = None
    return True


  def getPendingAction(self):
    # The next command that brings the light to the requested state, or None.
    # While off the target level is only stored, because the manual disagrees
    # on whether a LIN write turns the LED on; it is written before LON.
    with self.control_lock:
      want_on = self.on_off_state
      level = self.ratioToLevel(self.target_ratio)
    actions = []
    if want_on == True:
      if self.applied_level != level:
        actions.append(('LIN', level))
      if self.applied_on != True:
        actions.append(('LON', 0))
    elif self.applied_on != False:
      actions.append(('LOF', 0))
    if (len(actions) == 0 and self.last_on_level is not None and self.last_on_level != self.applied_lpo and
        (time.monotonic() - self.last_on_time) >= self.LPO_SETTLE_SEC):
      actions.append(('LPO', self.last_on_level))
    for action in actions:
      if action != self.rejected_action:
        return action
    return None


  def applyAction(self, action):
    # Sends one LIN, LON, LOF or LPO. Returns False only when the light did
    # not answer at all.
    [mnemonic, level] = action
    if mnemonic in ('LIN', 'LPO'):
      ser_msg = ('&' + self.addr_str + mnemonic + self.create_zero_prefix_str(level) + 'W')
    else:
      ser_msg = ('&' + self.addr_str + mnemonic + self.create_blank_str() + 'W')
    response = self.send_msg(ser_msg)
    if response is None:
      return False
    if self.checkValidResponse(ser_msg, response) == True:
      if mnemonic == 'LIN':
        self.applied_level = level
      elif mnemonic == 'LPO':
        self.applied_lpo = level
      else:
        self.applied_on = (mnemonic == 'LON')
      if mnemonic in ('LIN', 'LON') and self.applied_on == True and self.applied_level is not None and self.applied_level > 0:
        self.last_on_level = self.applied_level
        self.last_on_time = time.monotonic()
      self.rejected_action = None
    else:
      self.rejected_action = action
    return True

  #######################
  ### Class Functions
  def check_timer_callback(self, timer):
      """Apply requested light changes, poll status, and count missed replies.

      Each call sends at most one command. A pending on/off or intensity change
      goes first; otherwise the housing temperature is read once every
      POLL_INTERVAL_SEC. Any reply frame counts as alive, including one
      carrying a health alarm or a rejection. Only a timeout counts as a
      failure, and the node shuts down after more than self_check_count
      failures in a row so discovery can re-probe the port.

      Args:
          timer: The timer event passed by the timer process. Unused.
      """
      if nepi_sdk.is_shutdown() or self.serial_port is None:
          return
      action = self.getPendingAction()
      if action is not None:
          answered = self.applyAction(action)
      elif (time.monotonic() - self.last_poll_time) >= self.POLL_INTERVAL_SEC:
          self.last_poll_time = time.monotonic()
          answered = self.update_status_values()
      else:
          return
      if answered == True:
          self.self_check_counter = 0
      else:
          self.self_check_counter = self.self_check_counter + 1
          self.msg_if.pub_warn("No reply from light at address " + self.addr_str + " on " + self.port_str +
                               " (" + str(self.self_check_counter) + " in a row)")

      if self.self_check_counter > self.self_check_count:
          self.msg_if.pub_warn("Shutting down device: " + self.addr_str + " on port " + self.port_str)
          self.msg_if.pub_warn("Too many comm failures")
          nepi_sdk.signal_shutdown("Too many comm failures")


  ### Function to try and connect to device at given port and baudrate
  def connect(self):
      """Open the serial port and read the light's identity and intensity limits.

      Reads DSN, DFW and BS1 for the device info, sets terse acknowledgement
      (DAK 1) for this power cycle if needed, and reads the LIR intensity scale
      and the LMX and LIL limits.

      Returns:
          bool: True if the light answered at the configured address.
      """
      success = False
      self.connect_attempts += 1

      port_check = self.check_port(self.port_str)
      if port_check is False:
          self.msg_if.pub_warn("serial port not active")
          return success
      try:
          # Try and open serial port
          self.msg_if.pub_info("Opening serial port " + self.port_str + " with baudrate: " + self.baud_str)
          self.serial_port = serial.Serial(self.port_str,self.baud_int,timeout = self.SERIAL_READ_TIMEOUT_SEC, exclusive = True)
          self.msg_if.pub_info("Serial port opened")
      except Exception as e:
          self.msg_if.pub_warn("Something went wrong with connect function at serial port: " + self.port_str + " (" + str(e) + ")")
          self.serial_port = None
          return success

      # Send Message
      self.msg_if.pub_info("Requesting info for device: " + self.addr_str)
      ser_msg = ('&' + self.addr_str + 'DSN' + self.create_blank_str() + 'R')
      response = self.send_msg(ser_msg)
      if response is None:
          self.msg_if.pub_warn("No reply from a light at address " + self.addr_str + " on " + self.port_str + " at " + self.baud_str + " baud")
          self.closePort()
          return success

      success = True
      serial_num = self.getReplyData(ser_msg, response)
      self.serial_num = serial_num if serial_num is not None else "unknown"
      sw_version = self.readData('DFW')
      self.sw_version = sw_version if sw_version is not None else "unknown"
      hw_version = self.readData('BS1')
      self.hw_version = hw_version if hw_version is not None else "SS182mkII"

      self.setTerseAck()
      self.readIntensityScale()

      self.msg_if.pub_info("Connected to device at address: " + self.addr_str)
      self.msg_if.pub_info("Serial Number: " + self.serial_num + " Firmware P/N: " + self.sw_version + " Control Board: " + self.hw_version)
      return success


  def setTerseAck(self):
      # Verbose acknowledgement adds text lines after every reply; terse keeps
      # one frame per command. The DAK write lasts for this power cycle only and
      # is never made permanent (no DSS).
      dak = self.readInt('DAK')
      if dak == 1:
          self.dak_mode = 1
          return
      self.msg_if.pub_info("Acknowledgement mode is " + str(dak) + ", setting terse (DAK 1) for this power cycle")
      ser_msg = ('&' + self.addr_str + 'DAK' + self.create_zero_prefix_str(1) + 'W')
      response = self.send_msg(ser_msg)
      confirmed = self.checkValidResponse(ser_msg, response)
      if confirmed == False:
          # In DAK 0 a write gets no reply, so confirm with a read
          dak = self.readInt('DAK')
          confirmed = (dak == 1)
      if confirmed == True:
          self.dak_mode = 1
      else:
          self.dak_mode = None
          self.msg_if.pub_warn("Could not confirm terse acknowledgement mode (DAK read " + str(dak) + ")")


  def readIntensityScale(self):
      lir = self.readInt('LIR')
      if lir in self.LIR_FULL_SCALE_DICT.keys():
          self.full_scale = self.LIR_FULL_SCALE_DICT[lir]
      else:
          self.full_scale = self.DEFAULT_FULL_SCALE
          self.msg_if.pub_warn("Could not read intensity scale (LIR " + str(lir) + "), using 0 to " + str(self.full_scale))
      self.max_level = self.readInt('LMX')
      self.limit_level = self.readInt('LIL')
      self.applied_lpo = self.readInt('LPO')
      self.msg_if.pub_info("Intensity scale 0 to " + str(self.full_scale) +
                           ", operator maximum (LMX) " + str(self.max_level) +
                           ", factory limit (LIL) " + str(self.limit_level) +
                           ", power-on intensity (LPO) " + str(self.applied_lpo))


  def send_msg(self,ser_msg):
    """Send one command and return the light's reply frame.

    Holds the serial lock for the whole write-and-read transaction, so no
    command goes out on the half-duplex bus before the previous reply or its
    timeout, and waits until QUIET_SEC has passed since the previous
    transaction ended, because the light ignores a command sent too soon
    after its last reply. Clears the input buffer, writes the command with CR LF, and reads
    lines until one carries the command's '&' + address + mnemonic prefix.
    Banner and noise lines are discarded, and any verbose acknowledgement lines
    that follow the frame are consumed.

    Args:
        ser_msg (str): The 10-character command, without CR LF.

    Returns:
        str or None: The 10-character reply frame, which may carry ERRn in its
            DATA field, or None if no frame arrived within REPLY_TIMEOUT_SEC.
    """
    response = None
    if self.serial_port is None or nepi_sdk.is_shutdown():
      return response
    ser_str = (ser_msg + '\r\n')
    b=bytearray()
    b.extend(map(ord, ser_str))
    with self.serial_lock:
      wait_sec = self.last_bus_end + self.QUIET_SEC - time.monotonic()
      if wait_sec > 0:
        time.sleep(wait_sec)
      try:
        self.serial_port.reset_input_buffer()
        self.serial_port.write(b)
        response = self.readReply(ser_msg[0:5], self.REPLY_TIMEOUT_SEC)
        if response is not None and self.dak_mode != 1:
          self.drainVerboseLines()
      except Exception as e:
        self.msg_if.pub_warn("Serial transaction failed for " + ser_msg + " (" + str(e) + ")", throttle_s = 5)
        response = None
      self.last_bus_end = time.monotonic()
    return response


  def getFrame(self, line, prefix):
    # A frame is 10 characters from its prefix: & addr MNE DATA R/W. A glitch
    # byte from the RS485 turnaround can sit ahead of it on the same line.
    ind = line.find(prefix)
    if ind < 0:
      return None
    frame = line[ind:ind + 10]
    if len(frame) == 10 and frame[9] in ('R','W'):
      return frame
    return None


  def readReply(self, prefix, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    pending = b''
    while time.monotonic() < deadline:
      chunk = self.serial_port.readline()
      if len(chunk) == 0:
        continue
      pending += chunk
      if pending.endswith(b'\n') == False:
        continue
      line = pending.decode('ascii', errors = 'ignore')
      pending = b''
      frame = self.getFrame(line, prefix)
      if frame is not None:
        return frame
    if len(pending) > 0:
      return self.getFrame(pending.decode('ascii', errors = 'ignore'), prefix)
    return None


  def drainVerboseLines(self):
    # Verbose mode (DAK 2) follows the frame with 40-character text lines. Read
    # until the line goes quiet so the next command does not collide with them.
    deadline = time.monotonic() + self.REPLY_TIMEOUT_SEC
    while time.monotonic() < deadline:
      if len(self.serial_port.readline()) == 0:
        break


  def getErrCode(self, response):
    if response is None or len(response) < 9:
      return None
    data_str = response[5:9]
    if data_str[0:3] == 'ERR' and data_str[3].isdigit():
      return int(data_str[3])
    return None


  def checkValidResponse(self, ser_msg, response):
    # True for a reply to ser_msg that executed: a value frame, or a frame
    # carrying a health alarm (ERR4-9). False for no reply or ERR0-3.
    if response is None or response[0:5] != ser_msg[0:5]:
      return False
    err_code = self.getErrCode(response)
    if err_code is None:
      return True
    if err_code <= 3:
      self.msg_if.pub_warn("Light rejected " + ser_msg + ": " + self.ERR_CODES_DICT[err_code], throttle_s = 5)
      return False
    self.logAlarm(err_code)
    return True


  def logAlarm(self, err_code):
    msg = "Light health alarm: " + self.ERR_CODES_DICT[err_code]
    if err_code == 8:
      self.msg_if.pub_fatal(msg, throttle_s = self.ALARM_LOG_THROTTLE_SEC)
    else:
      self.msg_if.pub_warn(msg + ". Keeping last good values", throttle_s = self.ALARM_LOG_THROTTLE_SEC)


  def getReplyData(self, ser_msg, response):
    # DATA of a reply that carries a value; None for no reply, a rejected
    # command, or an active alarm that replaced the value
    if self.checkValidResponse(ser_msg, response) == False:
      return None
    if self.getErrCode(response) is not None:
      return None
    return response[5:9]


  def readData(self, mnemonic):
    ser_msg = ('&' + self.addr_str + mnemonic + self.create_blank_str() + 'R')
    response = self.send_msg(ser_msg)
    return self.getReplyData(ser_msg, response)


  def readInt(self, mnemonic):
    data_str = self.readData(mnemonic)
    if data_str is None:
      return None
    try:
      return int(data_str)
    except ValueError:
      self.msg_if.pub_warn("Reply to " + mnemonic + " was not a number: " + data_str, throttle_s = 10)
      return None


  def closePort(self):
    if self.serial_port is not None:
      try:
        self.serial_port.close()
      except Exception:
        pass
      self.serial_port = None


  ### Function for checking if port is available
  def check_port(self,port_str):
    """Check that the serial port is present.

    Uses the same port list discovery probes, which adds the Jetson UARTs that
    pyserial does not always enumerate.

    Args:
        port_str (str): The serial device path, for example /dev/ttyUSB0.

    Returns:
        bool: True if the port is present.
    """
    return port_str in nepi_serial.get_serial_ports_list()

  def create_blank_str(self):
      """Return the all-zero DATA field used by read commands.

      Returns:
          str: data_len zeros, '0000' for the standard config.
      """
      data_str = ""
      for z in range(self.data_len):
          data_str += '0'
      return data_str

  def create_zero_prefix_str(self,count_val):
    """Return a value zero-padded to the DATA field length.

    Args:
        count_val (int): The value to pad.

    Returns:
        str: The value as a string, zero-prefixed to data_len characters.
    """
    data_str = str(count_val)
    zero_prefix_len = self.data_len-len(data_str)
    for z in range(zero_prefix_len):
        data_str = ('0' + data_str)
    return data_str

   #######################
    ### Driver Interface Functions

  def driver_getDeviceInfo(self):
      """Return the device info read at connect.

      Returns:
          dict: Manufacturer, Model, FirmwareVersion (DFW part number),
              HardwareVersion (control board serial) and SerialNum (DSN).
      """
      dev_info = dict()
      dev_info["Manufacturer"] = 'Sidus'
      dev_info["Model"] = 'SS182'
      dev_info["FirmwareVersion"] = self.sw_version
      dev_info["HardwareVersion"] = self.hw_version
      dev_info["SerialNum"] = self.serial_num
      return dev_info

  #######################
  ### Cleanup processes on node shutdown
  def cleanup_actions(self):
    """Close the serial port on node shutdown, after any transaction in progress."""
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    locked = False
    if self.serial_lock is not None:
      locked = self.serial_lock.acquire(timeout = 2 * self.REPLY_TIMEOUT_SEC + self.QUIET_SEC)
    try:
      self.closePort()
    finally:
      if locked:
        self.serial_lock.release()

if __name__ == '__main__':
  SidusSS182SerialNode()
