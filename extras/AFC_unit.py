# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from configfile import error
class afcUnit:
    def __init__(self, config):
        self.printer    = config.get_printer()
        self.gcode      = self.printer.lookup_object('gcode')
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.AFC = self.printer.lookup_object('AFC')

        self.lanes      = {}

        # Objects
        self.buffer_obj     = None
        self.hub_obj        = None
        self.extruder_obj   = None
        
        # Config get section
        self.full_name          = config.get_name().split()
        self.name               = self.full_name[-1]
        self.screen_mac         = config.get('screen_mac', None)
        self.hub                = config.get("hub", None)
        self.extruder           = config.get("extruder", None)
        self.buffer_name        = config.get('buffer', None)
        self.led_name           = config.get('led_name', self.AFC.led_name)
        self.led_fault          = config.get('led_fault', self.AFC.led_fault)
        self.led_ready          = config.get('led_ready', self.AFC.led_ready)
        self.led_not_ready      = config.get('led_not_ready', self.AFC.led_not_ready)
        self.led_loading        = config.get('led_loading', self.AFC.led_loading)
        self.led_prep_loaded    = config.get('led_loading', self.AFC.led_loading)
        self.led_unloading      = config.get('led_unloading', self.AFC.led_unloading)
        self.led_tool_loaded    = config.get('led_tool_loaded', self.AFC.led_tool_loaded)

        self.long_moves_speed   = config.getfloat("long_moves_speed",  self.AFC.long_moves_speed)          # Speed in mm/s to move filament when doing long moves
        self.long_moves_accel   = config.getfloat("long_moves_accel",  self.AFC.long_moves_accel)         # Acceleration in mm/s squared when doing long moves
        self.short_moves_speed  = config.getfloat("short_moves_speed",  self.AFC.short_moves_speed)      # Speed in mm/s to move filament when doing short moves
        self.short_moves_accel  = config.getfloat("short_moves_accel",  self.AFC.short_moves_accel)      # Acceleration in mm/s squared when doing short moves
        self.short_move_dis     = config.getfloat("short_move_dis",  self.AFC.short_move_dis)               # Move distance in mm for failsafe moves.
    
    def handle_connect(self):
        self.AFC = self.printer.lookup_object('AFC')
        self.AFC.units[self.name] = self

        if self.hub is not None:
            try:
                self.hub_obj = self.printer.lookup_object("AFC_hub {}".format(self.hub))
            except:
                error_string = 'Error: No config found for hub: {hub} in [AFC_{unit_type} {unit_name}]. Please make sure [AFC_hub {hub}] section exists in your config'.format(
                hub=self.hub, unit_type=self.type.replace("_", ""), unit_name=self.name )
                raise error(error_string)

            if self.hub_obj.unit is not None:
                raise error("AFC_hub {} already has a unit {} assigned, can't assign {}. Only one unit can be assigned per AFC_hub".format(self.hub, " ".join(self.hub_obj.unit.full_name), " ".join(self.full_name)))
            else:
                self.hub_obj.unit = self

        if self.extruder is not None:
            try:
                self.extruder_obj = self.printer.lookup_object("AFC_extruder {}".format(self.extruder))
            except:
                error_string = 'Error: No config found for extruder: {extruder} in [AFC_{unit_type} {unit_name}]. Please make sure [AFC_extruder {extruder}] section exists in your config'.format(
                    extruder=self.extruder, unit_type=self.type.replace("_", ""), unit_name=self.name )
                raise error(error_string)

        if self.buffer_name is not None:
            try:
                self.buffer_obj = self.printer.lookup_object('AFC_buffer {}'.format(self.buffer_name))
            except:
                error_string = 'Error: No config found for buffer: {buffer} in [AFC_{unit_type} {unit_name}]. Please make sure [AFC_buffer {buffer}] section exists in your config'.format(
                    buffer=self.buffer_name, unit_type=self.type.replace("_", ""), unit_name=self.name )
                raise error(error_string)

        self.AFC.gcode.respond_info("AFC_{}:ready {} {}".format(self.name, self.hub, self.hub_obj))

        # Send out event so lanes can store units object
        self.printer.send_event("{}:connect".format(self.name), self)

    def system_Test(self, CUR_LANE, delay, assignTcmd, enable_movement):
        msg = ''
        succeeded = True
        
        # Run test reverse/forward on each lane
        CUR_LANE.unsync_to_extruder(False)
        if enable_movement:
            CUR_LANE.move( 5, self.AFC.short_moves_speed, self.AFC.short_moves_accel, True)
            self.AFC.reactor.pause(self.AFC.reactor.monotonic() + delay)
            CUR_LANE.move( -5, self.AFC.short_moves_speed, self.AFC.short_moves_accel, True)
        else:
            self.AFC.reactor.pause(self.AFC.reactor.monotonic() + delay)

        if CUR_LANE.prep_state == False:
            if CUR_LANE.load_state == False:
                self.AFC.afc_led(CUR_LANE.led_not_ready, CUR_LANE.led_index)
                msg += 'EMPTY READY FOR SPOOL'
            else:
                self.AFC.afc_led(CUR_LANE.led_fault, CUR_LANE.led_index)
                msg +="<span class=error--text> NOT READY</span>"
                CUR_LANE.do_enable(False)
                msg = '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                succeeded = False

        else:
            self.AFC.afc_led(CUR_LANE.led_ready, CUR_LANE.led_index)
            msg +="<span class=success--text>LOCKED</span>"
            if CUR_LANE.load_state == False:
                msg +="<span class=error--text> NOT LOADED</span>"
                self.AFC.afc_led(CUR_LANE.led_not_ready, CUR_LANE.led_index)
                succeeded = False
            else:
                CUR_LANE.status = 'Loaded'
                msg +="<span class=success--text> AND LOADED</span>"

                if CUR_LANE.tool_loaded:
                    if CUR_LANE.get_toolhead_sensor_state() == True or CUR_LANE.extruder_obj.tool_start == "buffer":
                        if CUR_LANE.extruder_obj.lane_loaded == CUR_LANE.name:
                            self.AFC.current = CUR_LANE.name
                            CUR_LANE.sync_to_extruder()
                            msg +="<span class=primary--text> in ToolHead</span>"
                            if CUR_LANE.extruder_obj.tool_start == "buffer":
                                msg += "<span class=warning--text>\n Ram sensor enabled, confirm tool is loaded</span>"
                            self.AFC.SPOOL.set_active_spool(CUR_LANE.spool_id)
                            self.AFC.afc_led(CUR_LANE.led_tool_loaded, CUR_LANE.led_index)
                            CUR_LANE.status = 'Tooled'
                            CUR_LANE.enable_buffer()
                            CUR_LANE.extruder_obj.lane_loaded = CUR_LANE.name
                        else:
                            if CUR_LANE.get_toolhead_sensor_state() == True:
                                msg +="<span class=error--text> error in ToolHead. \nLane identified as loaded in AFC.vars.unit file\n but not identified as loaded in AFC.var.tool file</span>"
                                succeeded = False
                    else:
                        lane_check=self.AFC.ERROR.fix('toolhead',CUR_LANE)  #send to error handling
                        if not lane_check:
                            return False

        if assignTcmd: self.AFC.TcmdAssign(CUR_LANE)
        CUR_LANE.do_enable(False)
        self.AFC.gcode.respond_info( '{lane_name} tool cmd: {tcmd:3} {msg}'.format(lane_name=CUR_LANE.name, tcmd=CUR_LANE.map, msg=msg))
        CUR_LANE.set_afc_prep_done()

        return succeeded
    def get_status(self, eventtime=None):
        self.response = {}
        self.response['lanes'] = [lane.name for lane in self.lanes.values()]

        return self.response
