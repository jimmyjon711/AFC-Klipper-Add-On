from extras.AFC_unit import afcUnit

class afcNightOwl(afcUnit):
    def __init__(self, config):
        super().__init__(config)

    def handle_connect(self):
        """
        Handle the connection event.
        This function is called when the printer connects. It looks up AFC info
        and assigns it to the instance variable `self.AFC`.
        """
        self.AFC = self.printer.lookup_object('AFC')

        self.logo = '<span class=success--text>Night Owl Ready</span>'
        self.logo ='<span class=success--text>R  ,     ,\n'
        self.logo+='E  )\___/(\n'
        self.logo+='A {(@)v(@)}\n'
        self.logo+='D  {|~~~|}\n'
        self.logo+='Y  {/^^^\}\n'
        self.logo+='!   `m-m`</span>\n'

        self.logo_error = '<span class=error--text>Night Owl Not Ready</span>\n'

def load_config(config):
    return afcNightOwl(config)