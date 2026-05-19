import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QTextEdit, QFrame, QLineEdit
)
from PySide6.QtCore import Qt, QDateTime
from PySide6.QtGui import QFont
from serial_worker import SerialWorker, list_ports

BAUDRATES = ["9600", "19200", "38400", "57600", "115200", "230400"]

STYLE = """
QMainWindow, QWidget {
    background-color: #1a1c1e;
    color: #e0e0e0;
    font-family: 'Segoe UI', sans-serif;
    font-size: 13px;
}
QComboBox {
    background-color: #2a2d30;
    border: 1px solid #3a3d40;
    border-radius: 6px;
    padding: 5px 10px;
    color: #e0e0e0;
    min-height: 28px;
}
QComboBox::drop-down { border: none; width: 20px; }
QComboBox QAbstractItemView {
    background-color: #2a2d30;
    selection-background-color: #3a3d40;
    color: #e0e0e0;
}
QPushButton {
    background-color: #2a2d30;
    border: 1px solid #3a3d40;
    border-radius: 6px;
    padding: 6px 14px;
    color: #e0e0e0;
    min-height: 28px;
}
QPushButton:hover { background-color: #33363a; border-color: #55585c; }
QPushButton:pressed { background-color: #1e2124; }
QPushButton:disabled { color: #555; border-color: #2a2d30; }
QPushButton#btn_connect {
    background-color: #1a3d2b;
    border-color: #2a6e4a;
    color: #4cda85;
}
QPushButton#btn_connect:hover { background-color: #1f4d34; }
QPushButton#btn_connect:disabled { background-color: #1a2a22; color: #2a6e4a; }
QPushButton#btn_disconnect {
    background-color: #3d1a1a;
    border-color: #6e2a2a;
    color: #da4c4c;
}
QPushButton#btn_disconnect:hover { background-color: #4d1f1f; }
QPushButton#btn_send {
    background-color: #1a2a3d;
    border-color: #2a4a6e;
    color: #4ca8da;
}
QPushButton#btn_send:hover { background-color: #1f3348; }
QPushButton#btn_send:disabled { background-color: #1a2230; color: #2a4a6e; }
QLineEdit {
    background-color: #2a2d30;
    border: 1px solid #3a3d40;
    border-radius: 6px;
    padding: 5px 10px;
    color: #e0e0e0;
    min-height: 28px;
}
QLineEdit:focus { border-color: #2a4a6e; }
QLineEdit:disabled { color: #555; }
QTextEdit {
    background-color: #111315;
    border: 1px solid #2a2d30;
    border-radius: 6px;
    color: #b0f0b0;
    font-family: 'Consolas', monospace;
    font-size: 12px;
    padding: 6px;
}
QLabel#section_label {
    color: #666;
    font-size: 11px;
    letter-spacing: 1px;
}
QFrame#separator {
    background-color: #2a2d30;
    max-height: 1px;
}
"""


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Monitor")
        self.setMinimumSize(600, 520)
        self.worker = SerialWorker()
        self.worker.data_received.connect(self.on_data_received)
        self.worker.connection_error.connect(self.on_error)
        self._build_ui()
        self.setStyleSheet(STYLE)
        self._set_connected(False)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        # --- Conexión ---
        root.addWidget(self._label("CONEXIÓN SERIE"))
        conn_row = QHBoxLayout()
        conn_row.setSpacing(8)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(100)
        self._refresh_ports()

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(BAUDRATES)
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setFixedWidth(100)

        self.btn_refresh = QPushButton("↺")
        self.btn_refresh.setFixedWidth(36)
        self.btn_refresh.setToolTip("Refrescar puertos")
        self.btn_refresh.clicked.connect(self._refresh_ports)

        self.btn_connect = QPushButton("Conectar")
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.clicked.connect(self._connect)

        self.btn_disconnect = QPushButton("Desconectar")
        self.btn_disconnect.setObjectName("btn_disconnect")
        self.btn_disconnect.clicked.connect(self._disconnect)

        conn_row.addWidget(self.port_combo, 1)
        conn_row.addWidget(self.baud_combo)
        conn_row.addWidget(self.btn_refresh)
        conn_row.addWidget(self.btn_connect)
        conn_row.addWidget(self.btn_disconnect)
        root.addLayout(conn_row)

        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignLeft)
        root.addWidget(self.status_label)

        root.addWidget(self._separator())

        # --- Enviar ---
        root.addWidget(self._label("ENVIAR DATOS"))
        send_row = QHBoxLayout()
        send_row.setSpacing(8)

        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("Escribí el dato a enviar...")
        self.input_field.returnPressed.connect(self._send_input)

        self.btn_send = QPushButton("Enviar")
        self.btn_send.setObjectName("btn_send")
        self.btn_send.setFixedWidth(80)
        self.btn_send.clicked.connect(self._send_input)

        send_row.addWidget(self.input_field, 1)
        send_row.addWidget(self.btn_send)
        root.addLayout(send_row)

        root.addWidget(self._separator())

        # --- Log ---
        log_header = QHBoxLayout()
        log_header.addWidget(self._label("LOG"))
        log_header.addStretch()
        btn_clear = QPushButton("Limpiar")
        btn_clear.setFixedWidth(70)
        root.addLayout(log_header)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        btn_clear.clicked.connect(self.log.clear)
        log_header.addWidget(btn_clear)
        root.addWidget(self.log, 1)

    def _label(self, text):
        lbl = QLabel(text)
        lbl.setObjectName("section_label")
        return lbl

    def _separator(self):
        sep = QFrame()
        sep.setObjectName("separator")
        sep.setFrameShape(QFrame.HLine)
        return sep

    def _refresh_ports(self):
        self.port_combo.clear()
        ports = list_ports()
        if ports:
            self.port_combo.addItems(ports)
        else:
            self.port_combo.addItem("Sin puertos")

    def _connect(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        ok = self.worker.connect(port, baud)
        if ok:
            self._set_connected(True)
            self._log_system(f"Conectado en {port} @ {baud} baud")

    def _disconnect(self):
        self.worker.disconnect()
        self._set_connected(False)
        self._log_system("Desconectado")

    def _send_input(self):
        text = self.input_field.text()
        if text:
            self._send(text)
            self.input_field.clear()

    def _send(self, value: str):    
        self.worker.send(value)
        self._log_tx(value)

    def _set_connected(self, connected: bool):
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.btn_refresh.setEnabled(not connected)
        self.btn_send.setEnabled(connected)
        self.input_field.setEnabled(connected)
        if connected:
            self.status_label.setText("● Conectado")
            self.status_label.setStyleSheet("color: #4cda85; font-size: 12px;")
        else:
            self.status_label.setText("○ Desconectado")
            self.status_label.setStyleSheet("color: #666; font-size: 12px;")

    def on_data_received(self, data: str):
        self._log_rx(data)

    def on_error(self, msg: str):
        self._log_system(f"Error: {msg}")
        self._set_connected(False)

    def _log_rx(self, text):
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log.append(f'<span style="color:#555">[{ts}]</span> <span style="color:#4cda85">RX</span> {text}')

    def _log_tx(self, text):
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log.append(f'<span style="color:#555">[{ts}]</span> <span style="color:#4ca8da">TX</span> {text}')

    def _log_system(self, text):
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log.append(f'<span style="color:#555">[{ts}] {text}</span>')

    def closeEvent(self, event):
        self.worker.disconnect()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
