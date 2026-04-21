from __future__ import annotations

import re
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import serial

from config import resolve_arduino_port


@dataclass(frozen=True)

class CVParameters:
	# @TODO: valor do R14, pensar em trocar para 10k para alcançar a escala da ana
	read_resistor_ohm: float = 120.0

	# Começa a varredura nesse potencial.
	e_initial: float  		= -1.0
	# Primeiro vértice (primeiro ponto de inversão da rampa).
	e_vertex_1: float 		= 1.0
	# Segundo vértice (segundo ponto de inversão).
	e_vertex_2: float 		= -1.0
	# Potencial onde o experimento deve terminar depois dos ciclos.
	e_final: float    		= -0.99
	# Número de ciclos entre os vértices. No firmware ele é tratado como inteiro.
	cycles: float			= 2.0
	# Velocidade de varredura em mV/s.
	scanrate_mvs: float 	= 50.0
	# Tempo antes de iniciar a varredura, mantendo o potencial em e_initial.
	conditioning_s: float	= 5.0

@dataclass(frozen=True)
class CalibrationOffsets:
	zero_idx_e: int
	zero_idx_i: int
	calibration_file: Path


def parse_float(raw: bytes) -> float:
	text = raw.decode("utf-8").strip()
	return float(text)


def validate_connection(arduino: serial.Serial) -> None:
	rand_float = 11.01
	sent_bytes = b"\x44\x66" + struct.pack("f", rand_float)

	arduino.write(sent_bytes)
	time.sleep(2)

	serial_out_1 = arduino.readline().rstrip(b"\r\n")
	serial_out_2 = arduino.readline().rstrip(b"\r\n")

	if serial_out_1 != sent_bytes:
		raise RuntimeError("Falha na validacao serial: eco do payload nao confere.")

	received_float = parse_float(serial_out_2)
	if received_float != round(rand_float, 2):
		raise RuntimeError("Falha na validacao serial: float de retorno nao confere.")


def wait_for_starter(arduino: serial.Serial, timeout_seconds: float = 30.0) -> None:
	deadline = time.time() + timeout_seconds
	while time.time() < deadline:
		starter = arduino.readline().rstrip(b"\r\n")
		if starter == b"10101010":
			return
		time.sleep(0.001)
	raise TimeoutError("Timeout esperando pacote inicial 10101010 do Arduino.")


def send_cv_parameters(arduino: serial.Serial, params: CVParameters) -> None:
	packets = [
		b"\x11\x10" + struct.pack("f", params.e_initial),
		b"\x11\x11" + struct.pack("f", params.e_vertex_1),
		b"\x11\x12" + struct.pack("f", params.e_vertex_2),
		b"\x11\x13" + struct.pack("f", params.e_final),
		b"\x11\x14" + struct.pack("f", params.cycles),
		b"\x11\x15" + struct.pack("f", params.scanrate_mvs),
		b"\x11\x16" + struct.pack("f", params.conditioning_s),
	]

	for packet in packets:
		arduino.write(packet)
		arduino.flush()
		time.sleep(1)


def collect_cv_data(arduino: serial.Serial) -> list[list[float]]:
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

	if not rows:
		raise RuntimeError("Nenhum ponto de CV foi recebido do dispositivo.")

	return rows


def parse_calibration_file(calibration_file: Path) -> list[list[float]]:
	rows: list[list[float]] = []
	for raw_line in calibration_file.read_text(encoding="utf-8").splitlines():
		line = raw_line.strip()
		if not line:
			continue
		parts = line.split("\t")
		if len(parts) < 5:
			continue
		try:
			rows.append([float(value) for value in parts[:5]])
		except ValueError:
			continue
	return rows


def load_calibration_offsets(base_dir: Path) -> CalibrationOffsets:
	calibration_dir = base_dir / "CALIBRATION"
	if not calibration_dir.exists():
		raise RuntimeError("Pasta CALIBRATION nao encontrada. Execute a calibracao antes da CV.")

	calibration_files = sorted(calibration_dir.glob("*_Cal.txt"), key=lambda p: p.stat().st_mtime)
	if not calibration_files:
		raise RuntimeError("Nenhum arquivo de calibracao encontrado em CALIBRATION.")

	calibration_file = calibration_files[-1]
	rows = parse_calibration_file(calibration_file)
	if not rows:
		raise RuntimeError(f"Arquivo de calibracao invalido: {calibration_file}")

	neg_rows = [row for row in rows if int(row[1]) == 1]
	pos_rows = [row for row in rows if int(row[1]) == 2]
	if not neg_rows or not pos_rows:
		raise RuntimeError("Calibracao sem linhas de referencia para indices negativos/positivos.")

	neg_idx_e = sum(row[3] for row in neg_rows) / len(neg_rows)
	pos_idx_e = sum(row[3] for row in pos_rows) / len(pos_rows)
	neg_idx_i = sum(row[4] for row in neg_rows) / len(neg_rows)
	pos_idx_i = sum(row[4] for row in pos_rows) / len(pos_rows)

	zero_idx_e = int(0.5 * (neg_idx_e + pos_idx_e))
	zero_idx_i = int(0.5 * (neg_idx_i + pos_idx_i))

	return CalibrationOffsets(
		zero_idx_e=zero_idx_e,
		zero_idx_i=zero_idx_i,
		calibration_file=calibration_file,
	)


def convert_row(row: list[float], offsets: CalibrationOffsets, params: CVParameters) -> list[float]:
	ramp_index = row[0]
	time_ms = 0.001 * row[1]
	e_we_vs_re = -0.000249 * (row[2] - offsets.zero_idx_e)
	i_ma = -0.12452 * (row[3] - offsets.zero_idx_i) / params.read_resistor_ohm
	cycle_number = row[4]
	return [ramp_index, time_ms, e_we_vs_re, i_ma, cycle_number]


def sanitize_output_name(raw_name: str) -> str:
	name = raw_name.strip()
	if not name:
		raise ValueError("Nome do arquivo nao pode ser vazio.")

	name = re.sub(r"[\\/:*?\"<>|]", "_", name)
	name = name.replace(" ", "_")
	if not name:
		raise ValueError("Nome do arquivo ficou invalido apos sanitizacao.")
	return name


def ask_output_path(output_dir: Path) -> Path:
	output_dir.mkdir(parents=True, exist_ok=True)
	while True:
		try:
			raw_name = input("Nome do arquivo de saida (sem extensao): ")
		except (KeyboardInterrupt, EOFError):
			print("\n[INFO] Operacao cancelada pelo usuario.")
			raise SystemExit(0)

		try:
			safe_name = sanitize_output_name(raw_name)
		except ValueError as exc:
			print(f"[ERRO] {exc}")
			continue

		return output_dir / f"{safe_name}.txt"


def write_cv_output(
	output_path: Path,
	stop_state: str,
	params: CVParameters,
	offsets: CalibrationOffsets,
	raw_rows: list[list[float]],
	converted_rows: list[list[float]],
) -> None:
	with output_path.open("w", encoding="utf-8", newline="\n") as f:
		f.write(f"Stop_State\t{stop_state}\n")
		f.write(f"Calibration_File\t{offsets.calibration_file.name}\n")
		f.write(f"Zero_IDX_E\t{offsets.zero_idx_e}\n")
		f.write(f"Zero_IDX_I\t{offsets.zero_idx_i}\n")
		f.write(f"E_in_vs_RE_in_V\t{params.e_initial}\n")
		f.write(f"E_v1_vs_RE_in_V\t{params.e_vertex_1}\n")
		f.write(f"E_v2_vs_RE_in_V\t{params.e_vertex_2}\n")
		f.write(f"E_fi_vs_RE_in_V\t{params.e_final}\n")
		f.write(f"n_cycles\t{params.cycles}\n")
		f.write(f"Scanr_in_mV_per_s\t{params.scanrate_mvs}\n")
		f.write(f"Cond_t_in_s\t{params.conditioning_s}\n")
		f.write(f"Rread_in_Ohm\t{params.read_resistor_ohm}\n")
		f.write("======================================================\n\n")
		f.write("Ramp-Index\ttime in ms\tE_WE_vs_RE in V\tI in mA\tCyc.No.\n\n")
		for row in converted_rows:
			f.write("\t".join(str(value) for value in row) + "\n")

	# Optional raw backup beside processed output for troubleshooting.
	raw_path = output_path.with_name(output_path.stem + "_raw.txt")
	with raw_path.open("w", encoding="utf-8", newline="\n") as f_raw:
		f_raw.write("Ramp-Index\tRawTime\tRawE\tRawI\tCycle\n")
		for row in raw_rows:
			f_raw.write("\t".join(str(value) for value in row) + "\n")


def generate_plots(
	output_path: Path,
	converted_rows: list[list[float]]
) -> Path | None:

	# isso é o que devo buscar
	# https://upload.wikimedia.org/wikipedia/commons/4/4d/Cyclic_voltammetry%2C_the_case_of_one-electron_transfer_to_a_free-diffusing_molecule.svg

	t_s = [row[1] / 1000.0 for row in converted_rows]
	e_v = [row[2] for row in converted_rows]
	i_ma = [row[3] for row in converted_rows]

	fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))

	axes[0].plot(e_v, i_ma, linewidth=0.8)
	axes[0].set_xlabel("E vs RE (V)")
	axes[0].set_ylabel("I (mA)")
	axes[0].grid(True, linestyle="--", linewidth=0.6, alpha=0.7)

	axes[1].plot(t_s, i_ma, linewidth=0.8)
	axes[1].set_xlabel("t (s)")
	axes[1].set_ylabel("I (mA)")
	axes[1].grid(True, linestyle="--", linewidth=0.6, alpha=0.7)

	axes[2].plot(t_s, e_v, linewidth=0.8)
	axes[2].set_xlabel("t (s)")
	axes[2].set_ylabel("E vs RE (V)")
	axes[2].grid(True, linestyle="--", linewidth=0.6, alpha=0.7)

	fig.tight_layout()
	plot_path: Path | None = None
	plot_path = output_path.with_suffix(".png")
	fig.savefig(plot_path, dpi=150)

	plt.close(fig)
	return plot_path


def run_cv(port: str, output_path: Path, offsets: CalibrationOffsets, params: CVParameters) -> tuple[Path, Path]:
	stop_state = "Success"
	raw_rows: list[list[float]] = []
	with serial.Serial(port, baudrate=115200, timeout=0.1) as arduino:
		print(f"[INFO] Conectado em {port} @ 115200 bps")
		time.sleep(2)

		print("[INFO] Validando comunicacao serial...")
		validate_connection(arduino)

		print("[INFO] Enviando parametros da CV...")
		send_cv_parameters(arduino, params)

		print("[INFO] Iniciando execucao da CV...")
		arduino.write(b"\x33\x01\x00\x00\x00\x00")
		time.sleep(3)

		print("[INFO] Aguardando sinal de inicio (10101010)...")
		wait_for_starter(arduino)

		print("[INFO] Coletando dados de CV...")
		try:
			raw_rows = collect_cv_data(arduino)
		except Exception:
			stop_state = "Interrupt_or_fail"
			raise

	converted_rows = [convert_row(row, offsets, params) for row in raw_rows]
	write_cv_output(output_path, stop_state, params, offsets, raw_rows, converted_rows)
	plot_path = generate_plots(output_path, converted_rows)
	return output_path, plot_path


def main() -> None:
	base_dir = Path(__file__).resolve().parent
	params = CVParameters()

	offsets = load_calibration_offsets(base_dir)
	print(f"[INFO] Arquivo de calibracao em uso: {offsets.calibration_file}")
	print(f"[INFO] Zero_IDX_E={offsets.zero_idx_e} | Zero_IDX_I={offsets.zero_idx_i}")

	port = resolve_arduino_port()
	output_dir = base_dir / "OUTPUT"
	output_path = ask_output_path(output_dir)

	txt_path, plot_path = run_cv(port=port, output_path=output_path, offsets=offsets, params=params)
	print(f"[OK] CV concluida. Dados salvos em: {txt_path}")
	print(f"[OK] Graficos salvos em: {plot_path}")


if __name__ == "__main__":
	main()
