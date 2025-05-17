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