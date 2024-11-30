from extras.AFC_unit_base import afcUnitBase

class afcNightOwl(afcUnitBase):
    def __init__(self, config):
        super().__init__(config)

    def handle_connect(self):
        """
        Handle the connection event.
        This function is called when the printer connects. It looks up AFC info
        and assigns it to the instance variable `self.AFC`.
        """
        super().handle_connect()

        self.logo = 'Night Owl Ready'
        self.logo ='R  ,     ,\n'
        self.logo+='E  )\___/(\n'
        self.logo+='A {(@)v(@)}\n'
        self.logo+='D  {|~~~|}\n'
        self.logo+='Y  {/^^^\}\n'
        self.logo+='!   `m-m`\n'

        self.logo_error = 'Night Owl Not Ready\n'

def load_config(config):
    return afcNightOwl(config)