from __future__ import annotations

import struct
import time
from datetime import datetime
from pathlib import Path

import serial
from config import load_r_calibration, resolve_arduino_port


def parse_float(raw: bytes) -> float:
	text = raw.decode("utf-8").strip()
	return float(text)


def send_calibration_parameters(arduino: serial.Serial) -> None:
	e_1 = -0.5
	e_2 = 0.5
	e_3 = 0.0
	e_4 = 0.0
	e_5 = 0.0
	t_1 = 5000.0
	t_2 = 5000.0
	t_3 = 0.0
	t_4 = 0.0
	t_5 = 0.0
	repetitions = 1.0

	packets = [
		b"\x12\x17" + struct.pack("f", e_1),
		b"\x12\x18" + struct.pack("f", e_2),
		b"\x12\x19" + struct.pack("f", e_3),
		b"\x12\x20" + struct.pack("f", e_4),
		b"\x12\x21" + struct.pack("f", e_5),
		b"\x12\x22" + struct.pack("f", t_1),
		b"\x12\x23" + struct.pack("f", t_2),
		b"\x12\x24" + struct.pack("f", t_3),
		b"\x12\x25" + struct.pack("f", t_4),
		b"\x12\x26" + struct.pack("f", t_5),
		b"\x12\x27" + struct.pack("f", repetitions),
	]

	for packet in packets:
		arduino.write(packet)
		time.sleep(2)


def validate_connection(arduino: serial.Serial) -> None:
	rand_float = 11.01
	sent_bytes = b"\x44\x66" + struct.pack("f", rand_float)

	arduino.write(sent_bytes)
	time.sleep(2)

	serial_out_1 = arduino.readline().rstrip(b"\r\n")
	serial_out_2 = arduino.readline().rstrip(b"\r\n")

	if serial_out_1 != sent_bytes:
		raise RuntimeError("Falha na validação serial: eco do payload não confere.")

	received_float = parse_float(serial_out_2)
	if received_float != round(rand_float, 2):
		raise RuntimeError("Falha na validação serial: float de retorno não confere.")


def wait_for_starter(arduino: serial.Serial, timeout_seconds: float = 30.0) -> None:
	deadline = time.time() + timeout_seconds
	while time.time() < deadline:
		starter = arduino.readline().rstrip(b"\r\n")
		if starter == b"10101010":
			return
		time.sleep(0.001)
	raise TimeoutError("Timeout esperando pacote inicial 10101010 do Arduino.")


def collect_calibration_data(arduino: serial.Serial) -> list[list[float]]:
	rows: list[list[float]] = []
	while True:
		data = arduino.readline().rstrip(b"\r\n")
		if not data:
			continue
		if data == b"999999":
			break

		decoded = data.decode("utf-8")
		values = [float(x) for x in decoded.split("\t")]
		if len(values) != 5:
			raise RuntimeError(f"Linha inesperada recebida: {decoded!r}")
		rows.append(values)
	return rows


def write_calibration_file(output_dir: Path, rows: list[list[float]]) -> Path:
	output_dir.mkdir(parents=True, exist_ok=True)
	output_name = datetime.today().strftime("%Y_%m_%d_Cal.txt")
	output_path = output_dir / output_name

	with output_path.open("w", encoding="utf-8", newline="\n") as f:
		for row in rows:
			row_out = [row[0], row[1], 0.001 * row[2], row[3], row[4]]
			f.write("\t".join(str(value) for value in row_out) + "\n")

	return output_path


def run_calibration(port: str, output_dir: Path) -> Path:
	with serial.Serial(port, baudrate=115200, timeout=0.1) as arduino:
		print(f"[INFO] Conectado em {port} @ 115200 bps")
		time.sleep(2)

		print("[INFO] Validando comunicação serial...")
		validate_connection(arduino)

		print("[INFO] Enviando parâmetros de calibração...")
		send_calibration_parameters(arduino)

		print("[INFO] Iniciando calibração no Arduino...")
		arduino.write(b"\x14\x00\x00\x00\x00\x00")
		time.sleep(3)

		print("[INFO] Aguardando sinal de início (10101010)...")
		wait_for_starter(arduino)

		print("[INFO] Coletando dados de calibração...")
		rows = collect_calibration_data(arduino)

	output_path = write_calibration_file(output_dir, rows)
	return output_path


def print_startup_warning(r_calibration: str) -> None:
	banner = "=" * 72
	print(banner)
	print("ATENCAO: CONECTE O RESISTOR DE CALIBRACAO ANTES DE INICIAR")
	print(f"(WE---R{r_calibration}---RE/CE)")
	print(banner)

def main() -> None:
	base_dir = Path(__file__).resolve().parent
	r_calibration = load_r_calibration(base_dir, default="1000")
	print_startup_warning(r_calibration)

	port = resolve_arduino_port()
	output_dir = base_dir / "CALIBRATION"

	output_path = run_calibration(
		port=port,
		output_dir=output_dir,
	)
	print(f"[OK] Calibração concluída. Arquivo salvo em: {output_path}")


if __name__ == "__main__":
	main()
