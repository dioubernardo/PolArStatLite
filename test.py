from __future__ import annotations

from pathlib import Path

from config import load_r_calibration
from cv import CVParameters, convert_row, generate_plots, load_calibration_offsets


def parse_raw_cv_file(raw_file: Path) -> list[list[float]]:
	rows: list[list[float]] = []
	for raw_line in raw_file.read_text(encoding="utf-8").splitlines():
		line = raw_line.strip()
		if not line or line.startswith("Ramp-Index"):
			continue

		parts = line.split("\t")
		if len(parts) != 5:
			continue

		try:
			rows.append([float(value) for value in parts])
		except ValueError:
			continue

	if not rows:
		raise RuntimeError(f"Nenhum dado valido encontrado em {raw_file}")

	return rows


def main() -> None:
	base_dir = Path(__file__).resolve().parent
	output_dir = base_dir / "OUTPUT"
	raw_data_file = output_dir / "teste_raw.txt"
	processed_data_file = output_dir / "teste.txt"

	if not raw_data_file.exists():
		raise RuntimeError(f"Arquivo nao encontrado: {raw_data_file}")
	if not processed_data_file.exists():
		raise RuntimeError(f"Arquivo nao encontrado: {processed_data_file}")

	try:
		read_resistor_ohm = float(load_r_calibration(base_dir))
	except ValueError as exc:
		raise RuntimeError("R_CALIBRATION invalido no config.txt.") from exc

	offsets = load_calibration_offsets(base_dir)
	params = CVParameters(read_resistor_ohm=read_resistor_ohm)
	raw_rows = parse_raw_cv_file(raw_data_file)
	converted_rows = [convert_row(row, offsets, params) for row in raw_rows]

	print(f"[INFO] Arquivo de calibracao usado: {offsets.calibration_file}")
	print(f"[INFO] Arquivo de dados usado: {processed_data_file}")
	print(f"[INFO] Pontos convertidos: {len(converted_rows)}")
	print("[INFO] Abrindo graficos em tela...")

	generate_plots(processed_data_file, converted_rows, save_plot=False, show_plot=True)


if __name__ == "__main__":
	main()
