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
