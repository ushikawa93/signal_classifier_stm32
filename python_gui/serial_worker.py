import serial
import time
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal


class SerialWorker(QThread):
    data_received = Signal(str)
    connection_error = Signal(str)

    def __init__(self):
        super().__init__()
        self.port = None
        self.running = False

    def connect(self, port, baudrate):
        try:
            self.port = serial.Serial(port, baudrate, timeout=1)
            self.running = True
            self.start()
            return True
        except Exception as e:
            self.connection_error.emit(str(e))
            return False

    def disconnect(self):
        self.running = False
        self.wait()
        if self.port and self.port.is_open:
            self.port.close()
        self.port = None

    def send(self, data: str):
        if self.port and self.port.is_open:
            for char in data:
                self.port.write(char.encode())
                time.sleep(0.01)
            self.port.write(('\r').encode())
            self.port.flush()

    def run(self):
        while self.running:
            try:
                if self.port and self.port.in_waiting:
                    line = self.port.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self.data_received.emit(line)
            except Exception as e:
                self.connection_error.emit(str(e))
                self.running = False


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]
