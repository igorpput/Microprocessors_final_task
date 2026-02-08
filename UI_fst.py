import sys
import serial
import serial.tools.list_ports
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QSlider, QGroupBox, QTextEdit, QSpinBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer

BAUD_RATE = 115200
SYNC_INTERVAL = 300

class SerialWorker(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, serial_port):
        super().__init__()
        self.ser = serial_port
        self.running = True

    def run(self):
        while self.running and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.data_received.emit(line)
            except:
                break
            time.sleep(0.01)

    def stop(self):
        self.running = False

class LightControlApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STM32 Light Controller (PRO) 💡")
        self.setGeometry(100, 100, 500, 650)
        
        self.ser = None
        self.worker = None
        self.last_sent_time = 0

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QVBoxLayout(central_widget)

        self.create_connection_group()
        self.create_lux_group()
        self.create_dist_group()
        self.create_console_group()

        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        self.refresh_ports()

        self.sync_timer = QTimer()
        self.sync_timer.timeout.connect(self.send_sync_request)

    def create_connection_group(self):
        group = QGroupBox("Połączenie UART")
        layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self.connect_btn = QPushButton("Połącz")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")

        layout.addWidget(QLabel("Port:"))
        layout.addWidget(self.port_combo, 1)
        layout.addWidget(self.connect_btn)

        group.setLayout(layout)
        self.layout.addWidget(group)

    def create_lux_group(self):
        group = QGroupBox("Zadana Jasność (Cel)")
        layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        label = QLabel("Wartość (Lux):")
        label.setStyleSheet("font-size: 14px;")
        
        self.lux_spin = QSpinBox()
        self.lux_spin.setRange(0, 2000)
        self.lux_spin.setValue(100)
        self.lux_spin.setSingleStep(1)
        self.lux_spin.setStyleSheet("font-size: 18px; font-weight: bold; color: #2196F3;")
        self.lux_spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.lux_spin.editingFinished.connect(self.on_lux_spin_finish)
        self.lux_spin.valueChanged.connect(self.on_lux_spin_change)

        top_layout.addWidget(label)
        top_layout.addWidget(self.lux_spin)

        self.lux_slider = QSlider(Qt.Orientation.Horizontal)
        self.lux_slider.setRange(0, 2000)
        self.lux_slider.setValue(100)
        self.lux_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.lux_slider.setTickInterval(200)
        
        self.lux_slider.valueChanged.connect(self.on_lux_change)
        self.lux_slider.sliderReleased.connect(self.on_lux_release)

        layout.addLayout(top_layout)
        layout.addWidget(self.lux_slider)
        group.setLayout(layout)
        self.layout.addWidget(group)

    def create_dist_group(self):
        group = QGroupBox("Zakłócenie (Dioda 2)")
        layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        label = QLabel("Moc (%):")
        label.setStyleSheet("font-size: 14px;")

        self.dist_spin = QSpinBox()
        self.dist_spin.setRange(0, 100)
        self.dist_spin.setValue(0)
        self.dist_spin.setStyleSheet("font-size: 18px; font-weight: bold; color: #FF9800;")
        self.dist_spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.dist_spin.editingFinished.connect(self.on_dist_spin_finish)
        self.dist_spin.valueChanged.connect(self.on_dist_spin_change)

        top_layout.addWidget(label)
        top_layout.addWidget(self.dist_spin)

        self.dist_slider = QSlider(Qt.Orientation.Horizontal)
        self.dist_slider.setRange(0, 100)
        self.dist_slider.setValue(0)
        
        self.dist_slider.valueChanged.connect(self.on_dist_change)
        self.dist_slider.sliderReleased.connect(self.on_dist_release)

        layout.addLayout(top_layout)
        layout.addWidget(self.dist_slider)
        group.setLayout(layout)
        self.layout.addWidget(group)

    def create_console_group(self):
        group = QGroupBox("Logi Systemowe")
        layout = QVBoxLayout()
        
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas; font-size: 12px;")
        
        layout.addWidget(self.console)
        group.setLayout(layout)
        self.layout.addWidget(group)


    def refresh_ports(self):
        current_selection = self.port_combo.currentText()
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
        index = self.port_combo.findText(current_selection)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.sync_timer.stop()
            self.worker.stop()
            self.worker.wait()
            self.ser.close()
            self.connect_btn.setText("Połącz")
            self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            self.log("🔌 Rozłączono.")
            self.port_combo.setEnabled(True)
        else:
            port = self.port_combo.currentText()
            if not port:
                self.log("⚠️ Brak portu!")
                return
            try:
                self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                self.connect_btn.setText("Rozłącz")
                self.connect_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold;")
                self.port_combo.setEnabled(False)
                self.log(f"✅ Połączono z {port}!")
                
                self.worker = SerialWorker(self.ser)
                self.worker.data_received.connect(self.handle_serial_data)
                self.worker.start()

                self.sync_timer.start(SYNC_INTERVAL)

            except Exception as e:
                self.log(f"❌ Błąd: {e}")

    def send_sync_request(self):
        self.send_command("GET INFO", force=True, silent=True)

    def calculate_checksum(self, cmd_str):
        checksum = 0
        for char in cmd_str:
            checksum ^= ord(char)
        return checksum

    def send_command(self, cmd_body, force=False, silent=False):
        if not self.ser or not self.ser.is_open:
            return

        if not force:
            if time.time() - self.last_sent_time < 0.05:
                return

        csum = self.calculate_checksum(cmd_body)
        full_msg = f"{cmd_body};{csum}\n"
        
        try:
            self.ser.write(full_msg.encode('utf-8'))
            if not silent:
                self.log(f"⬆️ {full_msg.strip()}", color="#2196F3")
            self.last_sent_time = time.time()
        except Exception as e:
            self.log(f"❌ Błąd wysyłania: {e}")

    def handle_serial_data(self, data):
        if ";" in data:
            try:
                content, checksum_str = data.split(";")
                if content.startswith("INFO"):
                    parts = content.split()
                    if len(parts) >= 3:
                        lux_val = int(parts[1])
                        dist_val = int(parts[2])
                        self.update_gui_from_device(lux_val, dist_val)
                else:
                    self.log(f"⬇️ {data}", color="#4CAF50")
            except ValueError:
                pass
        elif data.startswith("INFO"):
            parts = data.split()
            if len(parts) >= 3:
                lux_val = int(parts[1])
                dist_val = int(parts[2])
                self.update_gui_from_device(lux_val, dist_val)
        else:
             self.log(f"⬇️ {data}", color="#4CAF50")

    def update_gui_from_device(self, lux, dist):
        """Aktualizacja GUI - z blokadą nadpisywania tego, co użytkownik edytuje"""
        
        slider_busy = self.lux_slider.isSliderDown() or self.dist_slider.isSliderDown()

        spinbox_busy = self.lux_spin.hasFocus() or self.dist_spin.hasFocus()

        if slider_busy or spinbox_busy:
            return

        self.lux_slider.blockSignals(True)
        self.lux_spin.blockSignals(True)
        self.dist_slider.blockSignals(True)
        self.dist_spin.blockSignals(True)

        self.lux_slider.setValue(lux)
        self.lux_spin.setValue(lux)
        self.dist_slider.setValue(dist)
        self.dist_spin.setValue(dist)

        self.lux_slider.blockSignals(False)
        self.lux_spin.blockSignals(False)
        self.dist_slider.blockSignals(False)
        self.dist_spin.blockSignals(False)



    def on_lux_change(self):
        val = self.lux_slider.value()
        self.lux_spin.blockSignals(True)
        self.lux_spin.setValue(val)
        self.lux_spin.blockSignals(False)
        self.send_command(f"SET {val}", force=False)

    def on_lux_release(self):
        val = self.lux_slider.value()
        self.send_command(f"SET {val}", force=True)

    def on_lux_spin_finish(self):
        val = self.lux_spin.value()
        self.lux_slider.blockSignals(True)
        self.lux_slider.setValue(val)
        self.lux_slider.blockSignals(False)
        self.send_command(f"SET {val}", force=True)

    def on_lux_spin_change(self):
        val = self.lux_spin.value()
        self.lux_slider.blockSignals(True)
        self.lux_slider.setValue(val)
        self.lux_slider.blockSignals(False)
        self.send_command(f"SET {val}", force=False)

    def on_dist_change(self):
        val = self.dist_slider.value()
        self.dist_spin.blockSignals(True)
        self.dist_spin.setValue(val)
        self.dist_spin.blockSignals(False)
        self.send_command(f"PRCT {val}", force=False)

    def on_dist_release(self):
        val = self.dist_slider.value()
        self.send_command(f"PRCT {val}", force=True)

    def on_dist_spin_finish(self):
        val = self.dist_spin.value()
        self.dist_slider.blockSignals(True)
        self.dist_slider.setValue(val)
        self.dist_slider.blockSignals(False)
        self.send_command(f"PRCT {val}", force=True)

    def on_dist_spin_change(self):
        val = self.dist_spin.value()
        self.dist_slider.blockSignals(True)
        self.dist_slider.setValue(val)
        self.dist_slider.blockSignals(False)
        self.send_command(f"PRCT {val}", force=False)

    def log(self, message, color="white"):
        self.console.append(f'<span style="color:{color}">{message}</span>')
        sb = self.console.verticalScrollBar()
        sb.setValue(sb.maximum())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LightControlApp()
    window.show()
    sys.exit(app.exec())