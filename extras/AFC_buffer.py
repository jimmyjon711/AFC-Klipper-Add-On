# 8 Track Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from configparser import Error as error

ADVANCE_STATE_NAME = "Advance"
TRAILING_STATE_NAME = "Trailing"

class AFCtrigger:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split(' ')[-1]
        self.turtleneck = False
        self.belay = False
        # Try and get one of each pin to see how user has configured buffer
        self.advance_pin = config.get('advance_pin', None)
        self.buffer_distance = config.getfloat('distance', None)

        if self.advance_pin is not None and self.buffer_distance is not None:
            # Throw an error as this is not a valid configuration, only Turtle neck or buffer can be configured not both
            msg = "Turtle neck or buffer can be configured not both, please fix buffer configuration"
            self.gcode._respond_error( msg )
            raise error( msg )
        if self.advance_pin is not None:
            self.turtleneck = True
            # self.set_multiplier = config.getfloat('multiplier', 1.0)
            self.advance_pin = config.get('advance_pin')
            self.trailing_pin = config.get('trailing_pin')
            self.multiplier_high = config.getfloat("multiplier_high", default=1.1, minval=1.0)
            self.multiplier_low = config.getfloat("multiplier_low", default=0.9, minval=0.0, maxval=1.0)
        elif self.buffer_distance is not None:
            self.belay = True
            self.pin = config.get('pin')
            self.buffer_distance = config.getfloat('distance', 0)
            self.velocity = config.getfloat('velocity', 0)
            self.accel = config.getfloat('accel', 0)
        else:
            msg = "Buffer is not configured correctly, please fix configuration"
            self.gcode._respond_error( msg )
            raise error( msg )

        self.last_state = False
        self.enable = False
        self.current = ''
        self.AFC = self.printer.lookup_object('AFC')
        self.debug = config.getboolean("debug", False)
        buttons = self.printer.load_object(config, "buttons")
        if self.belay:
            buttons.register_buttons([self.pin], self.sensor_callback)
        if self.turtleneck:
            buttons.register_buttons([self.advance_pin], self.advance_callback)
            buttons.register_buttons([self.trailing_pin], self.trailing_callback)

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_mux_command("QUERY_BUFFER", "BUFFER", self.name, self.cmd_QUERY_BUFFER, desc=self.cmd_QUERY_BUFFER_help)
        if self.turtleneck:
            self.gcode.register_mux_command("SET_ROTATION_FACTOR", "AFC_trigger", None, self.cmd_SET_ROTATION_FACTOR, desc=self.cmd_LANE_ROT_FACTOR_help)
    cmd_LANE_ROT_FACTOR_help = "change rotation distance by factor specified"
    def cmd_SET_ROTATION_FACTOR(self, gcmd):
        if self.enable and self.turtleneck:
            if self.AFC.current != None:
                change_factor = gcmd.get_float('FACTOR', 1.0)
                if change_factor == 1.0:
                    stepper = self.printer.lookup_object('AFC_stepper ' + self.AFC.current).extruder_stepper.stepper
                    stepper.set_rotation_distance(self.base_rotation_dist)
                    self.gcode.respond_info("Rotation distance reset to base value: {}".format(self.base_rotation_dist))
                elif change_factor != 0:
                    self.set_multiplier(change_factor)
                    stepper = self.printer.lookup_object('AFC_stepper ' + self.AFC.current).extruder_stepper.stepper
                    new_rotation_dist = stepper.get_rotation_distance()[0]
                    self.gcode.respond_info("New rotation distance after applying factor: {}".format(new_rotation_dist))
                else:
                    self.gcode.respond_info("FACTOR must not be 0")
            else:
                self.gcode.respond_info("BUFFER {} NOT ENABLED".format(self.name.upper()))
        else:
            self.gcode.respond_info("BUFFER {} CAN'T CHANGE ROTATION DISTANCE".format(self.name.upper()))

    cmd_QUERY_BUFFER_help = "Report Buffer sensor state"

    def cmd_QUERY_BUFFER(self, gcmd):
        if self.turtleneck:
            tool_loaded=self.AFC.current
            LANE = self.printer.lookup_object('AFC_stepper ' + tool_loaded)
            stepper = LANE.extruder_stepper.stepper
            rotation_dist = stepper.get_rotation_distance()[0]
            self.gcode.respond_info("{} Rotation distance: {}".format(LANE.name.upper(), rotation_dist))
            state_info = self.last_state
        else:
            if self.last_state:
                state_info = "compressed"
            else:
                state_info = "expanded"
        self.gcode.respond_info("{} : {}".format(self.name, state_info))

    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 2.

    def _set_extruder_stepper(self):
        if self.printer.state_message == 'Printer is ready' and self.AFC.current != None:
            tool_loaded=self.AFC.current
            LANE = self.printer.lookup_object('AFC_stepper ' + tool_loaded)
            stepper = LANE.extruder_stepper.stepper
            base_rotation_dist = stepper.get_rotation_distance()[0]
            self.base_rotation_dist = base_rotation_dist
            if self.debug == True: self.gcode.respond_info("Base rotation distance for {}: {}".format(LANE.name.upper(),base_rotation_dist))
            self.update_rotation_distance = lambda m: stepper.set_rotation_distance(
                base_rotation_dist / m
            )
        else:
            return

    def enable_buffer(self):
        self.enable = True
        if self.debug == True: self.gcode.respond_info("{} buffer enabled".format(self.name.upper()))
        if self.turtleneck:
            self._set_extruder_stepper()
            multiplier = 1.0
            if self.last_state == ADVANCE_STATE_NAME:
                # self.set_high_multiplier()
                multiplier = self.multiplier_high
            elif self.last_state == TRAILING_STATE_NAME:
                # self.set_low_multiplier()
                multiplier = self.multiplier_low
            
            self.set_multiplier( multiplier )

    def disable_buffer(self):
        self.enable = False
        if self.debug == True: self.gcode.respond_info("{} buffer disabled".format(self.name.upper()))
        if self.turtleneck:
            self.reset_multiplier()

    def set_multiplier(self, multiplier):
        if not self.enable: return
        if self.AFC.current is None : return

        self.update_rotation_distance( multiplier )
        if self.debug == True: 
            stepper = self.printer.lookup_object('AFC_stepper ' + self.AFC.current).extruder_stepper.stepper
            new_rotation_dist = stepper.get_rotation_distance()[0]
            self.gcode.respond_info("New rotation distance after applying factor: {}".format(new_rotation_dist))

    # def set_low_multiplier(self):
    #     if self.enable:
    #         multiplier = self.multiplier_low
    #         self.set_multiplier(multiplier)
    #         if self.debug == True: 
    #             stepper = self.printer.lookup_object('AFC_stepper ' + self.AFC.current).extruder_stepper.stepper
    #             new_rotation_dist = stepper.get_rotation_distance()[0]
    #             self.gcode.respond_info("New rotation distance after applying factor: {}".format(new_rotation_dist))

    # def set_high_multiplier(self):
    #     if self.enable:
    #         multiplier = self.multiplier_high
    #         self.set_multiplier(multiplier)
    #         if self.debug == True: 
    #             stepper = self.printer.lookup_object('AFC_stepper ' + self.AFC.current).extruder_stepper.stepper
    #             new_rotation_dist = stepper.get_rotation_distance()[0]
    #             self.gcode.respond_info("New rotation distance after applying factor: {}".format(new_rotation_dist))

    def reset_multiplier(self):
        if self.debug == True: self.gcode.respond_info("Buffer multiplier reset")
        self.update_rotation_distance(1.0)
    
    def sensor_callback(self, eventtime, state):
        self.last_state = state
        if self.printer.state_message == 'Printer is ready':
            if self.AFC.tool_start.filament_present == True:
                if self.AFC.current != None:
                    tool_loaded=self.AFC.current
                    LANE = self.printer.lookup_object('AFC_stepper ' + tool_loaded)
                    if LANE.status != 'unloading':
                        if self.debug == True: self.gcode.respond_info("Buffer Triggered, State: {}".format(state))
                        LANE.move(self.buffer_distance, self.velocity ,self.accel)

    def advance_callback(self, eventtime, state):
        if self.printer.state_message == 'Printer is ready' and self.enable and self.last_state != ADVANCE_STATE_NAME:
            if self.AFC.tool_start.filament_present == True:
                if self.AFC.current != None:
                    self.set_multiplier( self.multiplier_high )
                    if self.debug == True: self.gcode.respond_info("Buffer Triggered State: Advanced")
                    
        self.last_state = ADVANCE_STATE_NAME
        if self.debug == True: self.gcode.respond_info("Buffer Triggered State: Advanced")

    def trailing_callback(self, eventtime, state):
        if self.printer.state_message == 'Printer is ready' and self.enable and self.last_state != TRAILING_STATE_NAME:
            if self.AFC.tool_start.filament_present == True:
                if self.AFC.current != None:
                    self.set_multiplier( self.multiplier_low )
                    if self.debug == True: self.gcode.respond_info("Buffer Triggered State: Trailing")
        
        self.last_state = TRAILING_STATE_NAME
        if self.debug == True: self.gcode.respond_info("Buffer Triggered State: Trailing")  

def load_config_prefix(config):
    return AFCtrigger(config)
