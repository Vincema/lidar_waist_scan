import pigpio
import subprocess
import constants

class bitbang_serial:
    
    def __init__(self, RX, TX, baudr):
        subprocess.call(["sudo", "pigpiod"])
        self.pi_gpio_handler = pigpio.pi()
        self.RX_pin = RX
        self.TX_pin = TX
        self.baudrate = baudr
        self.pi_gpio_handler.set_mode(self.RX_pin, pigpio.INPUT)
        self.pi_gpio_handler.set_mode(self.TX_pin, pigpio.OUTPUT)
        
        pigpio.exceptions = False
        self.pi_gpio_handler.bb_serial_read_close(self.RX_pin) # In case the port is alrdy open
        pigpio.exceptions = True

        # Open serial
        self.pi_gpio_handler.bb_serial_read_open(self.RX_pin, self.baudrate)   

    def close_bb_serial(self):
        self.pi_gpio_handler.bb_serial_read_close(self.RX_pin)
        self.pi_gpio_handler.stop()

    def read_line_bb_serial(self):
        #try:
        str_line = ""
        end = False
        while end != True:
            (count, input_bytes) = self.read_bb_serial()
            if count: 
                input_str = input_bytes.decode("utf-8")
                for i in range(count):
                    #print(input_bytes[i])
                    if input_bytes[i] == 13: # Enter
                        end = True
                        break
                    elif input_bytes[i] == 127: # Backspace
                        if str_line != "":
                            self.write_line_bb_serial('\033[1D',False)
                            self.write_line_bb_serial(" ",False)
                            self.write_line_bb_serial('\033[1D',False)
                            str_line = str_line[:-1]  
                    else:
                        str_line += input_str[i]
                        self.write_line_bb_serial(input_str[i],False)
        return str_line
        #except:
        #    cust_print("Bitbang serial error!")

    def read_bb_serial(self):
        return self.pi_gpio_handler.bb_serial_read(self.RX_pin)

    def write_line_bb_serial(self, msg, new_line=True):
        if new_line:
            msg += "\n\r"
        if msg != "":
            self.pi_gpio_handler.wave_clear()
            self.pi_gpio_handler.wave_add_serial(self.TX_pin, self.baudrate, msg)
            wv = self.pi_gpio_handler.wave_create()
            self.pi_gpio_handler.wave_send_once(wv)
            while self.pi_gpio_handler.wave_tx_busy(): # Wait until all data sent
                pass
            self.pi_gpio_handler.wave_delete(wv)
        

def cust_print(msg):
    if constants.use_bb_serial:
        bb_ser.write_line_bb_serial(msg)
    else:
        print(msg)

def cust_read(msg):
    if constants.use_bb_serial:
        bb_ser.write_line_bb_serial(msg, False)
        usr_input = bb_ser.read_line_bb_serial()
        bb_ser.write_line_bb_serial("", True)
        return usr_input
    else:
        return input(msg)

bb_ser = []
