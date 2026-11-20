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

import os
import subprocess
import time
import string
import serial
import serial.tools.list_ports


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_serial

PKG_NAME = 'LSX_SIDUS_SS182_SERIAL' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# LSX Discover Method
#########################################


### Function to try and connect to device and also monitor and clean up previously connected devices
class SidusSS182SerialDiscovery:

  DATA_LENGTH_DICT = {
      'Standard' : 4
  }

  data_len = 4

  # Measured on an SS182mkII at 9600 baud: a DSN reply arrives 0.1 to 0.86 s
  # after the command. drivers_mgr runs every driver's discoveryFunction in
  # turn on one thread, so a silent port costs this wait per probe.
  PROBE_REPLY_TIMEOUT_SEC = 1.2
  PORT_READ_TIMEOUT_SEC = 0.05
  # A port that answered nothing is re-probed at this interval, not every cycle.
  SILENT_RETRY_SEC = 60.0
  # The factory baud. With every baud selected, each pass probes it plus one
  # other baud in turn. A probe at the wrong baud reaches the light as garbage
  # that can swallow the next command, so the first probe at each baud of a
  # multi-baud pass is sent twice.
  DEFAULT_BAUD = '9600'
  # A node that exits soon after launch is not relaunched until this has passed.
  NODE_LOAD_TIME_SEC = 10

  # ttyACM ports belong to Teensy, Pololu Maestro and PX4 boards on these
  # systems and never receive Sidus frames, unless the USB vendor is a
  # USB-UART bridge maker whose chip enumerated under cdc_acm:
  # WCH, FTDI, Silicon Labs, Prolific.
  BRIDGE_USB_VIDS = [0x1A86, 0x0403, 0x10C4, 0x067B]

  active_paths_list = []

  active_devices_dict = dict()
  node_launch_name = "SS182"
  baudrate_list = []
  baud_str = '9600'
  baud_int = 9600
  addr_str = "A"

  dont_retry_list = []

  includeDevices = []
  excludedDevices = ['ttyACM']

  ################################################
  def __init__(self):
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(namespace = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.letters_list = list(string.ascii_uppercase)
    self.active_devices_dict = dict()
    self.launch_time_dict = dict()
    self.silent_ports_dict = dict() # path_str: time of the last probe that got no reply
    self.baud_rotation_dict = dict() # path_str: index of the next non-default baud to probe
    self.retry = True
    self.logger.log_info("Initialization Complete")

  ##########  Nex Standard Discovery Function
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self,available_paths_list, active_paths_list,base_namespace,drv_dict, retry_enabled = True):
    self.logger.log_debug("Entering discovery function with drv_dict: " + str(drv_dict))###
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace
    ########################
    # Get discovery options
    try:
      baudrate_dict = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']
      self.logger.log_debug("Baud Rate dict: " + str(baudrate_dict))###
      #self.logger.log_debug("Starting discovery with drv_dict " + str(drv_dict))#
      baudrate_options = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['options']
      self.logger.log_debug("Baud Rate options: " + str(baudrate_options))###

      baudrate_sel = drv_dict['DISCOVERY_DICT']['OPTIONS']['baud_rate']['value']
      self.logger.log_debug("Baud Rate selected: " + str(baudrate_sel))###
      baudrate_list = []
      if baudrate_sel != "All":
        baudrate_list.append(baudrate_sel)
      else:
          for baudrate in baudrate_options:
            if baudrate != 'All':
              baudrate_list.append(baudrate)
      self.baudrate_list = baudrate_list
      self.logger.log_debug("Baud Rate list: " + str(baudrate_list))###

      start_addr = drv_dict['DISCOVERY_DICT']['OPTIONS']['start_addr']['value']
      try:
        start_ind = self.letters_list.index(start_addr)
      except:
        start_ind = 0
      self.logger.log_debug("Starting discovery addr: " + str(self.letters_list[start_ind]))

      stop_addr = drv_dict['DISCOVERY_DICT']['OPTIONS']['stop_addr']['value']
      try:
        stop_ind = self.letters_list.index(stop_addr)
      except:
        stop_ind = 0
      self.logger.log_debug("Ending discovery addr: " + str(self.letters_list[stop_ind]))

      self.addr_search_list = list(self.letters_list[start_ind:stop_ind + 1])
      self.logger.log_debug("Disc Addr List: " + str(self.addr_search_list))

    except Exception as e:
      self.logger.log_debug("Failed to setupoptions " + str(e))#
      return self.active_paths_list

    system_config = drv_dict['DISCOVERY_DICT']['OPTIONS']['system_config']['value']
    self.logger.log_debug("Got system config type: " + str(system_config))
    try:
      self.data_len = self.DATA_LENGTH_DICT[system_config]
      self.logger.log_debug("Got data length: " + str(self.data_len))
    except:
      self.data_len = 4
      self.logger.log_debug("Using default data length: " + str(self.data_len))
      return self.active_paths_list


    ########################
    # Retry behavior
    self.retry = retry_enabled
    if self.retry == True:
        self.dont_retry_list = []

    # Create path search options
    self.path_list = nepi_serial.get_serial_ports_list()
    #self.logger.log_debug("ports list: " + str(self.path_list))###
    for path_str in list(self.silent_ports_dict.keys()):
      if path_str not in self.path_list:
        del self.silent_ports_dict[path_str]
    bridge_acm_paths = self.getBridgeAcmPaths()

    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict:
        success = self.checkOnDevice(path_str)
        if success == False:
          path_purge_list.append(path_str)
          if self.retry == False:
            self.dont_retry_list.append(path_str)
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del  self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)

    ### Checking for devices on available paths
    for path_str in self.path_list:
      valid_path = path_str not in self.active_paths_list
      if valid_path:
        for id in self.excludedDevices:
          if path_str.find(id) != -1:
            valid_path = False
            if id == 'ttyACM' and path_str in bridge_acm_paths:
              valid_path = True
      if valid_path and path_str in self.silent_ports_dict.keys():
        valid_path = (nepi_sdk.get_time() - self.silent_ports_dict[path_str]) > self.SILENT_RETRY_SEC
      if valid_path:
        self.logger.log_debug("Looking for path: " + path_str)
        self.logger.log_debug("In path_list: " + str(self.active_paths_list))

        found = self.checkForDevice(path_str)
        if found and path_str not in self.dont_retry_list:
          success = self.launchDeviceNode(path_str)
          if success:
            self.active_paths_list.append(path_str)
    return self.active_paths_list
  ################################################

  ##########  Device specific calls
  def getBridgeAcmPaths(self):
    bridge_paths = []
    try:
      for port in serial.tools.list_ports.comports():
        if port.vid is not None and port.vid in self.BRIDGE_USB_VIDS:
          bridge_paths.append(port.device)
    except Exception as e:
      self.logger.log_debug("Failed to read USB vendor ids: " + str(e))
    return bridge_paths


  def getProbeBaudList(self,path_str):
    if len(self.baudrate_list) <= 1:
      return list(self.baudrate_list)
    probe_list = []
    if self.DEFAULT_BAUD in self.baudrate_list:
      probe_list.append(self.DEFAULT_BAUD)
    others = [baud for baud in self.baudrate_list if baud != self.DEFAULT_BAUD]
    if len(others) > 0:
      ind = self.baud_rotation_dict.get(path_str, 0) % len(others)
      self.baud_rotation_dict[path_str] = ind + 1
      probe_list.append(others[ind])
    return probe_list


  def getFrame(self,line,prefix):
    # A reply is 10 characters from its prefix: & addr MNE DATA R/W. A glitch
    # byte from the RS485 turnaround can sit ahead of it on the same line.
    ind = line.find(prefix)
    if ind < 0:
      return None
    frame = line[ind:ind + 10]
    if len(frame) == 10 and frame[9] in ('R','W'):
      return frame
    return None


  def readProbeReply(self,serial_port,prefix,timeout_sec):
    # Lines without the expected prefix (power-up banner, noise) are skipped.
    deadline = time.monotonic() + timeout_sec
    pending = b''
    while time.monotonic() < deadline:
      chunk = serial_port.readline()
      if len(chunk) == 0:
        continue
      pending += chunk
      if pending.endswith(b'\n') == False:
        continue
      line = pending.decode('ascii', errors = 'ignore')
      pending = b''
      frame = self.getFrame(line,prefix)
      if frame is not None:
        return frame
    if len(pending) > 0:
      return self.getFrame(pending.decode('ascii', errors = 'ignore'),prefix)
    return None


  def checkForDevice(self,path_str):
    self.logger.log_debug("Entering check for device function for path: " + str(path_str))###
    found_device = False
    if path_str in self.active_paths_list:
      return found_device
    probe_baud_list = self.getProbeBaudList(path_str)
    multi_baud = len(probe_baud_list) > 1
    for baud_str in probe_baud_list:
      try:
        # Try and open serial port. exclusive takes a flock, so a port some
        # other process holds under flock fails to open here instead of
        # having this probe written into it.
        serial_port = serial.Serial(path_str,int(baud_str),timeout = self.PORT_READ_TIMEOUT_SEC, exclusive = True)
      except Exception as e:
        self.logger.log_debug("Unable to open serial port " + path_str + " with baudrate: " + baud_str + " (" + str(e) + ")")
        return found_device
      #################################################
      try:
        port_error = False
        for addr_ind, addr_str in enumerate(self.addr_search_list):
          data_str = ''
          for i in range(self.data_len):
            data_str = data_str + '0'
          ser_msg= ('&' + addr_str + 'DSN' + data_str + 'R')
          ser_str = (ser_msg + '\r\n')
          ################################################
          # Send Serial String
          b=bytearray()
          b.extend(map(ord, ser_str))
          tries = 2 if (multi_baud and addr_ind == 0) else 1
          response = None
          for try_ind in range(tries):
            self.logger.log_debug("Sending serial message: " + ser_msg + " on " + path_str + " at " + baud_str)
            try:
              serial_port.reset_input_buffer()
              serial_port.write(b)
              response = self.readProbeReply(serial_port,ser_msg[0:5],self.PROBE_REPLY_TIMEOUT_SEC)
            except Exception as e:
              self.logger.log_info("Got a serial read/write error on " + path_str + ": " + str(e))
              port_error = True
              break
            if response is not None:
              break
          if port_error:
            break
          self.logger.log_debug("Got response: " + str(response))
          # A reply carrying ERRn in its DATA still identifies the light
          if response is not None:
            self.baud_str = baud_str
            self.baud_int = int(baud_str)
            self.addr_str = addr_str
            self.logger.log_info("Found SS182 light at path: " + path_str + " baudrate: " + baud_str + " address: " + addr_str + " reply: " + response)
            found_device = True
            break # Don't check any more addresses
      finally:
        # Clean up the serial port
        try:
          serial_port.close()
        except Exception:
          pass
      if found_device:
        break # Don't check any more baud rates

    if found_device:
      self.silent_ports_dict.pop(path_str, None)
    else:
      if path_str not in self.silent_ports_dict.keys():
        self.logger.log_info("No SS182 light answered on " + path_str + " at " + ", ".join(probe_baud_list) +
                             " baud -- will re-check it every " + str(int(self.SILENT_RETRY_SEC)) + " s")
      self.silent_ports_dict[path_str] = nepi_sdk.get_time()
    return found_device


  def checkOnDevice(self,path_str):
    self.logger.log_debug("Entering check on device function for path: " + str(path_str))###
    active = True
    if path_str not in self.available_paths_list:
      active = False
      self.logger.log_info("No longer detecting device on : " + path_str)
    elif path_str in self.active_devices_dict:
      sub_process = self.active_devices_dict[path_str].get('sub_process')
      if sub_process is not None and sub_process.poll() is not None:
        active = False  # node process has exited
        self.logger.log_info("Driver node exited for device on : " + path_str + " -- will re-probe it")
    if active == False:
      if path_str in self.active_devices_dict.keys():
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drvs.killDriverNode(node_name,sub_process)
    return active


  def launchDeviceNode(self, path_str):
    self.logger.log_debug("Entering launch device function for path: " + str(path_str) + ' addr: ' + str(self.addr_str))###
    success = False
    launch_id = path_str

    # Check if should try to launch
    if launch_id in self.launch_time_dict.keys():
      if (nepi_sdk.get_time() - self.launch_time_dict[launch_id]) <= self.NODE_LOAD_TIME_SEC:
        return success

    file_name = self.drv_dict['NODE_DICT']['file_name']
    device_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + str(self.addr_str)
    node_name = nepi_system.get_device_alias(device_name)
    self.logger.log_info("Launching node: " + node_name + " with path: " + path_str + " baudrate: " + self.baud_str + " addr: " + self.addr_str)

    #Setup required param server drv_dict for discovery node
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")
    self.drv_dict['DEVICE_DICT']={'device_name': device_name,
                                  'device_path': path_str}
    self.drv_dict['DEVICE_DICT']['baud_str'] = self.baud_str
    self.drv_dict['DEVICE_DICT']['addr_str'] = self.addr_str
    nepi_sdk.set_param(dict_param_name,self.drv_dict)
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
    if success:
      self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
    else:
      self.logger.log_info("Failed to lauch node: " + node_name + " with msg: " + msg)
      if self.retry == False:
        self.logger.log_info("Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(path_str)
    return success


  def killAllDevices(self,active_paths_list):
    #self.logger.log_debug("Entering Kill All Devices function for path: " + str(path_str))###
    path_purge_list = []
    for key in self.active_devices_dict.keys():
      path_purge_list.append(key)
    self.logger.log_debug("Killing Devices: " + str(path_purge_list))
    for path_str in path_purge_list:
        path_entry = self.active_devices_dict[path_str]
        node_name = path_entry['node_name']
        sub_process = path_entry['sub_process']
        success = nepi_drvs.killDriverNode(node_name,sub_process)

    for path_str in path_purge_list:
        try:
          del  self.active_devices_dict[path_str]
        except Exception as e:
          self.logger.log_debug("Failed to remove driver from active paths dict: " + str(e))

        try:
          self.logger.log_debug("Removing path from active paths list: " + str(path_str))
          self.active_paths_list.remove(path_str)
          active_paths_list.remove(path_str)
          self.logger.log_debug("Updated active paths list: " + str(active_paths_list))
        except Exception as e:
          self.logger.log_debug("Failed to remove driver from active paths list: " + str(e))


        try:
          self.logger.log_debug("Removing path from class active paths list: " + str(path_str))
          self.active_paths_list.remove(path_str)
        except Exception as e:
          #self.logger.log_debug("Failed to remove driver from class active paths list: " + str(e))
          pass


    nepi_sdk.sleep(1)
    return active_paths_list
