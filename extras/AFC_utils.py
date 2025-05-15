# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# File is used to hold common functions that can be called from anywhere and don't belong to a class
import json
from sys import settrace
from datetime import datetime

from urllib import request
from urllib.request import (
    Request,
    urlopen
)
from urllib.parse import (
    urlencode,
    urljoin
)

def add_filament_switch( switch_name, switch_pin, printer ):
    """
    Helper function to register pins as filament switch sensor so it will show up in web guis

    :param switch_name: Name of switch to register, should be in the following format: `filament_switch_sensor <name>`
    :param switch_pin: Pin to add to config for switch
    :param printer: printer object

    :return returns filament_switch_sensor object
    """
    import configparser
    import configfile
    ppins = printer.lookup_object('pins')
    ppins.allow_multi_use_pin(switch_pin.strip("!^"))
    filament_switch_config = configparser.RawConfigParser()
    filament_switch_config.add_section( switch_name )
    filament_switch_config.set( switch_name, 'switch_pin', switch_pin)
    filament_switch_config.set( switch_name, 'pause_on_runout', 'False')

    cfg_wrap = configfile.ConfigWrapper( printer, filament_switch_config, {}, switch_name)

    fila = printer.load_object(cfg_wrap, switch_name)
    fila.runout_helper.sensor_enabled = False
    fila.runout_helper.runout_pause = False

    return fila

class AFC_moonraker:
    def __init__(self, port, logger):
        self.port = port
        self.logger = logger
        self.local_host = 'http://localhost{port}'.format( port=port )
        self.database_url = urljoin(self.local_host, "server/database/item")
        self.data_array = {"namespace":"afc_stats", "key":"", "value":""}
        self.afc_stats_values = None

    def _get_results(self, url_string):
        try:
            resp = json.load(urlopen(url_string))
        except:
            resp = None
        return resp        

    def get_spoolman_server(self):
        resp = self._get_results(urljoin(self.local_host, 'server/config'))
        if resp is not None:
            return resp['result']['orig']['spoolman']['server']     # check for spoolman and grab url
        else:
            return None

    def get_file_filament_change_count(self, filename ):
        change_count = 0
        resp = self._get_results(urljoin(self.local_host, 'server/files/metadata?filename={}'.format(filename)))
        if resp is not None and 'filament_change_count' in resp['result']:
            change_count =  resp['result']['filament_change_count']
        return change_count
    
    def get_afc_stats(self):
        resp = None
        req = Request(self.database_url)
        resp = self._get_results(urljoin(self.database_url, "?namespace=afc_stats"))
        if resp is None:
            self.logger.info("AFC_stats not in database")
        else:
            self.afc_stats = resp['result']
        
        return self.afc_stats
    
    def update_afc_stats(self, key, value):
        post_payload = {
            "request_method": "POST",
            "namespace": "afc_stats",
            "key": key,
            "value": value
        }
        req = Request(self.database_url, urlencode(post_payload).encode())
        resp = self._get_results(req)
    
    def get_spool(self, id):

        request_payload = {
            "request_method": "GET",
            "path": f"/v1/spool/{id}"
        }
        spool_url = urljoin(self.local_host, f'server/spoolman/proxy')
        req = Request( spool_url, urlencode(request_payload).encode() )
        resp = self._get_results(req)
        return resp['result']

def check_and_return( value_str, data_values ):
    value = 0
    if value_str in data_values:
        value = data_values[value_str]
    
    return value

class AFCStats_var:
    def __init__(self, parent_name, name, data, moonraker):
        self.parent_name = parent_name
        self.name        = name
        self.moonraker   = moonraker

        if data is not None and self.parent_name in data:
            value = check_and_return( self.name, data[self.parent_name])
            try:
                self._value = int(value)
            except ValueError:
                try :
                    self._value = float(value)
                except:
                    self._value = value
        else:
            self._value = 0
            self.update_database()
    @property
    def value(self):
        return self._value
    @value.setter
    def value(self, value):
        self._value = value
    
    def average_time(self, value):
        if self._value > 0:
            self._value += value
            self._value /= 2
        else:
            self._value = value
    
    def increase_count(self):
        self._value += 1
        self.update_database()
    
    def reset_count(self):
        self._value = 0
        self.update_database()
    
    def update_database(self):
        self.moonraker.update_afc_stats(f"{self.parent_name}.{self.name}", self._value)
    
    def set_current_time(self):
        from datetime import datetime
        time = datetime.now()
        self._value = time.strftime("%Y-%m-%d %H:%M")
        self.update_database()

class AFCStats:
    def __init__(self, moonraker):
        
        self.moonraker = moonraker
        afc_stats = self.moonraker.get_afc_stats()
        
        if afc_stats is not None:
            values = afc_stats["value"]
        else:
            values = None

        self.tc_total           = AFCStats_var("toolchange_count", "total", values, self.moonraker)
        self.tc_tool_unload     = AFCStats_var("toolchange_count", "tool_unload", values, self.moonraker)
        self.tc_tool_load       = AFCStats_var("toolchange_count", "tool_load", values, self.moonraker)
        self.tc_without_error   = AFCStats_var("toolchange_count", "changes_without_error", values, self.moonraker)
        self.tc_last_load_error = AFCStats_var("toolchange_count", "last_load_error", values, self.moonraker) # TimeDateValue
        
        if self.tc_last_load_error.value == 0:
            self.tc_last_load_error.set_current_time()

        self.cut_total                  = AFCStats_var("cut", "cut_total", values, self.moonraker)
        self.cut_total_since_changed    = AFCStats_var("cut", "cut_total_since_changed", values, self.moonraker)
        self.last_blade_changed         = AFCStats_var("cut", "last_blade_changed", values, self.moonraker) # TODO
        # self.cut_threshold_for_warning  = AFCStats_var("cut", "cut_total", values, self.moonraker)

        self.average_toolchange_time    = AFCStats_var("average_time", "tool_change", values, self.moonraker)
        self.average_tool_unload_time   = AFCStats_var("average_time", "tool_unload", values, self.moonraker)
        self.average_tool_load_time     = AFCStats_var("average_time", "tool_load",   values, self.moonraker)
    
    def increase_cut_total(self):
        self.cut_total.increase_count()
        self.cut_total_since_changed.increase_count()

    def increase_toolcount_change(self):
        self.tc_total.increase_count()
        self.tc_without_error.increase_count()
    
    def reset_toolchange_wo_error(self):
        self.tc_without_error.reset_count()
        self.tc_last_load_error.set_current_time()

    def print_stats(self, afc_obj):
        avg_tool_load   = f"Avg Tool Load: {self.average_tool_load_time.value:4.2f}s"
        avg_tool_unload = f"Avg Tool Unload: {self.average_tool_unload_time.value:4.2f}s"
        avg_tool_change = f"Avg Tool Change: {self.average_toolchange_time.value:4.2f}s"
        blade_change = "2025-05-13 21:06"
        
        print_str  = f"{'':{'-'}<86}\n"
        print_str += f"|{'Toolchanges':{' '}^42}|{'Cut':{' '}^41}|\n"
        print_str += f"|{'':{'-'}<84}|\n"
        print_str += f"|{'Total':{' '}>22} : {self.tc_total.value:{' '}<17}|{'Total':{' '}>21} : {self.cut_total.value:{''}<17}|\n"
        print_str += f"|{'Tool Unload':{' '}>22} : {self.tc_tool_unload.value:{' '}<17}|{'Total since changed':{' '}>21} : {self.cut_total_since_changed.value:{''}<17}|\n"
        print_str += f"|{'Tool Load':{' '}>22} : {self.tc_tool_load.value:{' '}<17}|{'Blade last changed':{' '}>21} : {blade_change:{''}<17}|\n"
        print_str += f"|{'Changes without error':{' '}>22} : {self.tc_without_error.value:{' '}<17}|{'':{''}<41}|\n"
        print_str += f"|{'Last error date':{' '}>22} : {self.tc_last_load_error.value:{' '}<17}|{'':{''}<41}|\n"
        print_str += f"{'':{'-'}<86}\n"
        print_str += f"|{avg_tool_load:{' '}^27}|{avg_tool_unload:{' '}^27}|{avg_tool_change:{' '}^28}|\n"
        print_str += f"{'':{'-'}<86}\n"

        for lane in afc_obj.lanes.values():
            print_str += f"| {lane.name:{' '}>7} : Lane change count: {lane.lane_load_count.value:{' '}>6}    N20 active time: FWD-{lane.espooler.stats.n20_runtime_fwd:>8} RWD-{lane.espooler.stats.n20_runtime_rwd:>8}  |\n"
        print_str += f"{'':{'-'}<86}\n"
        afc_obj.logger.raw(print_str)


# toolchange_count
#   total
#   tool_load
#   tool_unload
#   without_error
#   last_load_error
# cut_count
#   total
#   cut_count_since_changed
#   last_blade_changed
#   number_cut_for_warning - warn user when they are 1000 cuts away
# average_toolchange_time
#   tool_load
#   tool_unload

# Lane specific
#   n20_runtime_per_lane
#     lane1
#     lane2
#     etc....
#   change_count_per_lane
#     lane1
#     lane2
#     etc....