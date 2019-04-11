import pyautogui
import serial
import argparse
import time
import logging

class MyControllerMap:
    def __init__(self):
        self.button = {'q': 'q', 'w': 'w', 'e': 'e'} # Fast forward (10 seg) pro Youtube

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
    
    def update(self):
        ## Sync protocol
        while self.incoming != b'-':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        button = self.ser.read()
        print(button)
        status = self.ser.read()
        print(status)
        #logging.debug("Received DATA: {}".format(data))

        if button == b'q':

            if status == b'1':
                logging.info("KEYDOWN q")
                pyautogui.keyDown(self.mapping.button['q'])
                
            elif status == b'0':
                logging.info("KEYUP q")
                pyautogui.keyUp(self.mapping.button['q'])

        if button == b'w':

            if status == b'1':
                logging.info("KEYDOWN w")
                pyautogui.keyDown(self.mapping.button['w'])
                
            elif status == b'0':
                logging.info("KEYUP w")
                pyautogui.keyUp(self.mapping.button['w'])

        if button == b'e':

            if status == b'1':
                logging.info("KEYDOWN e")
                pyautogui.keyDown(self.mapping.button['e'])
                
            elif status == b'0':
                logging.info("KEYUP e")
                pyautogui.keyUp(self.mapping.button['e'])

        self.incoming = self.ser.read()
        print(self.incoming)


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
