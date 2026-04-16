from __future__ import annotations

from pathlib import Path

from serial.tools import list_ports


def _list_active_ports() -> list[tuple[str, str]]:
	ports: list[tuple[str, str]] = []
	for port in list_ports.comports():
		description = (port.description or "").strip()
		ports.append((port.device, description))
	return ports


def resolve_arduino_port() -> str:
	ports = _list_active_ports()
	if not ports:
		raise RuntimeError("Nenhuma porta serial ativa foi encontrada.")

	print("[INFO] Portas seriais disponíveis:")
	for idx, (device, description) in enumerate(ports, start=1):
		if description and description.upper() != "N/A":
			print(f"  {idx}. {device} - {description}")
		else:
			print(f"  {idx}. {device}")

	while True:
		try:
			choice = input("Escolha o número da porta a usar: ").strip()
		except (KeyboardInterrupt, EOFError):
			print("\n[INFO] Seleção de porta cancelada pelo usuário.")
			raise SystemExit(0)
		if not choice.isdigit():
			print("[ERRO] Digite apenas o número da opção.")
			continue

		selected = int(choice)
		if 1 <= selected <= len(ports):
			return ports[selected - 1][0]

		print("[ERRO] Opção fora da lista de portas disponíveis.")


def load_r_calibration(base_dir: Path, *, default: str | None = None) -> str:
	config_path = base_dir / "config.txt"
	if not config_path.exists():
		if default is not None:
			return default
		raise RuntimeError("Arquivo config.txt nao encontrado para ler R_CALIBRATION.")

	for raw_line in config_path.read_text(encoding="utf-8").splitlines():
		line = raw_line.strip()
		if not line or line.startswith("#") or "=" not in line:
			continue
		key, value = line.split("=", 1)
		if key.strip() == "R_CALIBRATION":
			parsed = value.strip().strip('"').strip("'")
			if parsed:
				return parsed
			break

	if default is not None:
		return default

	raise RuntimeError("R_CALIBRATION nao definido no config.txt.")
