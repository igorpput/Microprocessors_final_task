import sys
import serial
import serial.tools.list_ports
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QSlider, QGroupBox, QTextEdit, QSpinBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
import pyqtgraph as pg 

BAUD_RATE = 115200
SYNC_INTERVAL = 200 

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
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)
            except Exception as e:
                break
            time.sleep(0.005)

    def stop(self):
        self.running = False

class LightControlApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("STM32 Light Controller (History Edition) 📉")
        self.setGeometry(100, 100, 1000, 700)
        

        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #353535;
                color: #ffffff;
                font-family: Segoe UI, sans-serif;
            }
            QGroupBox {
                border: 2px solid #555;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                color: #ddd;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QComboBox, QSpinBox, QPushButton {
                background-color: #505050;
                border: 1px solid #777;
                padding: 5px;
                border-radius: 3px;
                color: white;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QSlider::groove:horizontal {
                border: 1px solid #999;
                height: 8px;
                background: #505050;
                margin: 2px 0;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #4CAF50;
                border: 1px solid #5c5c5c;
                width: 18px;
                height: 18px;
                margin: -7px 0;
                border-radius: 9px;
            }
        """)

        self.ser = None
        self.worker = None
        self.last_sent_time = 0
        self.start_time = None 


        self.time_data = []
        self.lux_set_data = []
        self.lux_act_data = []
        self.dist_data = []

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        self.main_layout = QHBoxLayout(central_widget)
        
        self.left_panel = QWidget()
        self.left_layout = QVBoxLayout(self.left_panel)
        
        self.create_connection_group()
        self.create_lux_group()
        self.create_dist_group()
        self.create_console_group()
        
        self.create_graph_group()

        self.main_layout.addWidget(self.left_panel, 1)
        self.main_layout.addWidget(self.graph_group, 3)

        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        self.refresh_ports()

        self.sync_timer = QTimer()
        self.sync_timer.timeout.connect(self.send_sync_request)

    def create_graph_group(self):
        self.graph_group = QGroupBox("Cool Graph")
        layout = QVBoxLayout()

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#222222')
        self.plot_widget.setTitle("Signals on single chart", color="#aaa", size="10pt")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'Brightness', units='Lux')
        self.plot_widget.setLabel('bottom', 'Time Elapsed', units='s')
        self.plot_widget.addLegend()

        self.curve_set = self.plot_widget.plot(pen=pg.mkPen(color='#FF5252', width=2, style=Qt.PenStyle.DashLine), name="Goal")
        self.curve_act = self.plot_widget.plot(pen=pg.mkPen(color='#448AFF', width=2), name="Actual")
        self.curve_dist = self.plot_widget.plot(pen=pg.mkPen(color='#FFD740', width=1), name="Disturbance")

        layout.addWidget(self.plot_widget)
        

        btn_clear = QPushButton("Clean Chart")
        btn_clear.clicked.connect(self.clear_graph)
        btn_clear.setStyleSheet("background-color: #606060; margin-top: 5px;")
        layout.addWidget(btn_clear)

        self.graph_group.setLayout(layout)

    def clear_graph(self):
        """Resets the graph data and clears the plot."""
        self.time_data = []
        self.lux_set_data = []
        self.lux_act_data = []
        self.dist_data = []
        self.start_time = time.time()
        self.update_curves()
        self.log("Graph empty ;o")

    def update_graph(self, lux_set, lux_act, dist_val):
        if self.start_time is None:
            self.start_time = time.time()

        current_t = time.time() - self.start_time
        

        self.time_data.append(current_t)
        self.lux_set_data.append(lux_set)
        self.lux_act_data.append(lux_act)
        self.dist_data.append(dist_val)


        self.update_curves()

    def update_curves(self):
        self.curve_set.setData(self.time_data, self.lux_set_data)
        self.curve_act.setData(self.time_data, self.lux_act_data)
        self.curve_dist.setData(self.time_data, self.dist_data)


    def create_connection_group(self):
        group = QGroupBox("Serial Port")
        layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #2E7D32; font-weight: bold;")

        layout.addWidget(QLabel("Select:"))
        layout.addWidget(self.port_combo, 1)
        layout.addWidget(self.connect_btn)

        group.setLayout(layout)
        self.left_layout.addWidget(group)

    def create_lux_group(self):
        group = QGroupBox("Wanted Light Intensity")
        layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        label = QLabel("Brightness [Lux]")
        
        self.lux_spin = QSpinBox()
        self.lux_spin.setRange(0, 250)
        self.lux_spin.setValue(100)
        self.lux_spin.setStyleSheet("color: #64B5F6; font-size: 16px; font-weight: bold;")
        self.lux_spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.lux_spin.editingFinished.connect(self.on_lux_spin_finish)
        self.lux_spin.valueChanged.connect(self.on_lux_spin_change)

        top_layout.addWidget(label)
        top_layout.addWidget(self.lux_spin)

        self.lux_slider = QSlider(Qt.Orientation.Horizontal)
        self.lux_slider.setRange(0, 250)
        self.lux_slider.setValue(100)
        self.lux_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.lux_slider.setTickInterval(200)

        
        self.lux_slider.valueChanged.connect(self.on_lux_change)
        self.lux_slider.sliderReleased.connect(self.on_lux_release)

        layout.addLayout(top_layout)
        layout.addWidget(self.lux_slider)
        group.setLayout(layout)
        self.left_layout.addWidget(group)

    def create_dist_group(self):
        group = QGroupBox("Disturbance Intensity")
        layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        label = QLabel("Power [%]")

        self.dist_spin = QSpinBox()
        self.dist_spin.setRange(0, 100)
        self.dist_spin.setValue(0)
        self.dist_spin.setStyleSheet("color: #FFD54F; font-size: 16px; font-weight: bold;")
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
        self.left_layout.addWidget(group)

    def create_console_group(self):
        group = QGroupBox("Logs")
        layout = QVBoxLayout()
        
        self.console = QTextEdit()
        self.console.setReadOnly(True)

        self.console.setStyleSheet("background-color: #111; color: #00ff00; border: 1px solid #555;")
        
        layout.addWidget(self.console)
        group.setLayout(layout)
        self.left_layout.addWidget(group)

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
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("background-color: #2E7D32; font-weight: bold;")
            self.log("Disconnected :(")
            self.port_combo.setEnabled(True)
        else:
            port = self.port_combo.currentText()
            if not port: return
            try:
                self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setStyleSheet("background-color: #C62828; font-weight: bold;")
                self.port_combo.setEnabled(False)
                self.log(f"Connected to {port} :)")
                

                self.clear_graph()
                
                self.worker = SerialWorker(self.ser)
                self.worker.data_received.connect(self.handle_serial_data)
                self.worker.start()
                self.sync_timer.start(SYNC_INTERVAL)
            except Exception as e:
                self.log(f"Oopsie: {e}")

    def send_sync_request(self):
        self.send_command("GET INFO", force=True, silent=True)

    def calculate_checksum(self, cmd_str):
        checksum = 0
        for char in cmd_str:
            checksum ^= ord(char)
        return checksum

    def send_command(self, cmd_body, force=False, silent=False):
        if not self.ser or not self.ser.is_open: return
        if not force:
            if time.time() - self.last_sent_time < 0.05: return

        csum = self.calculate_checksum(cmd_body)
        full_msg = f"{cmd_body};{csum}\n"
        
        try:
            self.ser.write(full_msg.encode('utf-8'))
            if not silent:
                self.log(f"⬆️ {full_msg.strip()}", color="#90CAF9")
            self.last_sent_time = time.time()
        except:
            pass

    def handle_serial_data(self, data):
        if ";" in data:
            try:
                content, _ = data.split(";")
                if content.startswith("INFO"):
                    parts = content.split()
                    
                    lux_set = 0
                    lux_act = 0
                    dist_val = 0

                    if len(parts) >= 4: 
                        lux_set = int(parts[1])
                        lux_act = int(parts[2]) 
                        dist_val = int(parts[3])
                    elif len(parts) >= 3: 
                        lux_set = int(parts[1])
                        dist_val = int(parts[2])
                        lux_act = lux_set 

                    self.update_gui_from_device(lux_set, dist_val)
                    self.update_graph(lux_set, lux_act, dist_val)

                else:
                    self.log(f"⬇️ {data}", color="#A5D6A7")
            except ValueError:
                pass
        else:
             self.log(f"⬇️ {data}", color="#A5D6A7")

    def update_gui_from_device(self, lux, dist):
        slider_busy = self.lux_slider.isSliderDown() or self.dist_slider.isSliderDown()
        spinbox_busy = self.lux_spin.hasFocus() or self.dist_spin.hasFocus()

        if slider_busy or spinbox_busy: return

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
        self.send_command(f"SET {self.lux_slider.value()}")
        self.lux_spin.setValue(self.lux_slider.value())

    def on_lux_release(self):
        self.send_command(f"SET {self.lux_slider.value()}", force=True)

    def on_lux_spin_finish(self):
        self.send_command(f"SET {self.lux_spin.value()}", force=True)
        self.lux_slider.setValue(self.lux_spin.value())

    def on_lux_spin_change(self):
        self.send_command(f"SET {self.lux_spin.value()}")
        self.lux_slider.setValue(self.lux_spin.value())

    def on_dist_change(self):
        self.send_command(f"PRCT {self.dist_slider.value()}")
        self.dist_spin.setValue(self.dist_slider.value())

    def on_dist_release(self):
        self.send_command(f"PRCT {self.dist_slider.value()}", force=True)

    def on_dist_spin_finish(self):
        self.send_command(f"PRCT {self.dist_spin.value()}", force=True)
        self.dist_slider.setValue(self.dist_spin.value())

    def on_dist_spin_change(self):
        self.send_command(f"PRCT {self.dist_spin.value()}")
        self.dist_slider.setValue(self.dist_spin.value())

    def log(self, message, color="white"):
        self.console.append(f'<span style="color:{color}">{message}</span>')
        sb = self.console.verticalScrollBar()
        sb.setValue(sb.maximum())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LightControlApp()
    window.show()
    sys.exit(app.exec())