import sys
import time
from typing import Dict

import serial
from config import resolve_arduino_port

FRAME_SIZE = 8
READINGS_PER_STEP = 10
ADS_MIN_V = -3.3
ADS_MAX_V = 3.3
ADS_MAX_CODE = 26400
SWEEP_STEP = 50
SWEEP_POINTS = (4095 // SWEEP_STEP) + 2


def mcp_code_to_volt(code: int) -> float:
    # MCP output configured from -3.3 V to 3.3 V over 12-bit code.
    return -3.3 + 6.6 * (code / 4095.0)


def ads_code_to_volt(code: int) -> float:
    # Calibrated ADS mapping: 0 bit = -3.3 V and 26400 bit = +3.3 V.
    clamped_code = max(0, min(ADS_MAX_CODE, code))
    return ADS_MIN_V + (ADS_MAX_V - ADS_MIN_V) * (clamped_code / ADS_MAX_CODE)


def expected_ads_volt_from_mcp_code(code: int) -> float:
    # ADS3 probes MCP output directly in the same bipolar range.
    return mcp_code_to_volt(code)


def build_command_frame(command: int) -> bytes:
    """Builds the same 8-byte command frame style used by the original firmware."""
    frame = bytearray(FRAME_SIZE)
    frame[0] = command & 0xFF
    return bytes(frame)


def parse_result_line(line: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for part in line.split():
        if "=" in part:
            key, value = part.split("=", 1)
            result[key.strip()] = value.strip()
    return result


def safe_int(value: str, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def read_meaningful_line(ser: serial.Serial, timeout_s: float = 2.0) -> str:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            return line
    return ""


def print_table(title: str, rows: list[dict[str, float | int]]) -> None:
    print(f"\n{title}")
    print("-" * 73)
    print(
        f"{'MCP':>4} | {'MCP_V':>7} | {'ADS3':>6} | {'ADS3_V':>7} | {'ERR3_V':>7} | {'ADS1':>6} | {'ADS1_V':>7} | {'ERR1_V':>7}"
    )
    print("-" * 73)

    for row in rows:
        print(
            f"{int(row['mcp_code']):>4} | "
            f"{row['mcp_v']:>7.4f} | "
            f"{int(row['ads3_code']):>6} | "
            f"{row['ads3_v']:>7.4f} | "
            f"{row['err3_v']:>7.4f} | "
            f"{int(row['ads1_code']):>6} | "
            f"{row['ads1_v']:>7.4f} | "
            f"{row['err1_v']:>7.4f}"
        )

    print("-" * 73)


def run_step(ser: serial.Serial, command: int, title: str, expected_v: float) -> None:
    input(f"Pressione Enter para iniciar {title} (0x{command:02X})...")

    rows: list[dict[str, float | int]] = []

    ser.reset_input_buffer()
    ser.write(build_command_frame(command))
    ser.flush()

    for read_idx in range(1, READINGS_PER_STEP + 1):
        line = read_meaningful_line(ser, timeout_s=3.0)
        if not line:
            print(f"[AVISO] Leitura {read_idx}: sem resposta do Arduino dentro do timeout.")
            continue

        print(line)

        parsed = parse_result_line(line)
        if not parsed:
            print(f"[AVISO] Leitura {read_idx}: resposta invalida: {line}")
            continue

        mcp_code = safe_int(parsed.get("MCP", "0"), default=0)
        ads1_code = safe_int(parsed.get("ADS1", "0"), default=0)
        ads3_code = safe_int(parsed.get("ADS3", "0"), default=0)

        mcp_v = mcp_code_to_volt(mcp_code)
        ads1_v = ads_code_to_volt(ads1_code)
        ads3_v = ads_code_to_volt(ads3_code)

        expected_ads3_v = expected_ads_volt_from_mcp_code(mcp_code)
        err3_v = abs(ads3_v - expected_ads3_v)
        err1_v = abs(ads1_v - expected_v)

        rows.append(
            {
                "idx": read_idx,
                "mcp_code": mcp_code,
                "mcp_v": mcp_v,
                "ads1_code": ads1_code,
                "ads1_v": ads1_v,
                "err1_v": err1_v,
                "ads3_code": ads3_code,
                "ads3_v": ads3_v,
                "err3_v": err3_v,
            }
        )

    if not rows:
        print("Nenhuma leitura valida para este teste.")
        return

    print_table(f"Tabela de leituras - {title}", rows)

    expected_ads1_v = expected_v
    expected_ads3_v = expected_ads_volt_from_mcp_code(int(rows[0]["mcp_code"]))
    mean_err1_v = sum(float(row["err1_v"]) for row in rows) / len(rows)
    mean_err3_v = sum(float(row["err3_v"]) for row in rows) / len(rows)

    print("Resumo:")
    print(f"  Leituras validas: {len(rows)}/{READINGS_PER_STEP}")
    print(f"  ADS3 esperado: {expected_ads3_v:.4f} V")
    print(f"  Erro medio ADS3: {mean_err3_v:.4f} V")
    print(f"  ADS1 esperado: {expected_ads1_v:.4f} V")
    print(f"  Erro medio ADS1: {mean_err1_v:.4f} V")


def run_sweep_step(ser: serial.Serial, command: int, title: str, ads1_start_expected_v: float, ads1_end_expected_v: float) -> None:
    input(f"Pressione Enter para iniciar {title} (0x{command:02X})...")

    rows: list[dict[str, float | int]] = []

    ser.reset_input_buffer()
    ser.write(build_command_frame(command))
    ser.flush()

    for read_idx in range(1, SWEEP_POINTS + 1):
        line = read_meaningful_line(ser, timeout_s=1.5)
        if not line:
            print(f"[AVISO] Sweep leitura {read_idx}: sem resposta do Arduino dentro do timeout.")
            continue

        print(line)

        parsed = parse_result_line(line)
        if not parsed:
            continue

        mcp_code = safe_int(parsed.get("MCP", "0"), default=0)
        ads1_code = safe_int(parsed.get("ADS1", "0"), default=0)
        ads3_code = safe_int(parsed.get("ADS3", "0"), default=0)

        mcp_v = mcp_code_to_volt(mcp_code)
        ads1_v = ads_code_to_volt(ads1_code)
        ads3_v = ads_code_to_volt(ads3_code)

        expected_ads3_v = expected_ads_volt_from_mcp_code(mcp_code)
        err3_v = abs(ads3_v - expected_ads3_v)

        expected_ads1_v = ads1_start_expected_v + (ads1_end_expected_v - ads1_start_expected_v) * (mcp_code / 4095.0)
        err1_v = abs(ads1_v - expected_ads1_v)

        rows.append(
            {
                "idx": read_idx,
                "mcp_code": mcp_code,
                "mcp_v": mcp_v,
                "ads1_code": ads1_code,
                "ads1_v": ads1_v,
                "err1_v": err1_v,
                "ads3_code": ads3_code,
                "ads3_v": ads3_v,
                "err3_v": err3_v,
            }
        )

    if not rows:
        print("Nenhuma leitura valida no sweep.")
        return

    print_table(f"Tabela de leituras - {title}", rows)

    mean_err1_v = sum(float(row["err1_v"]) for row in rows) / len(rows)
    mean_err3_v = sum(float(row["err3_v"]) for row in rows) / len(rows)

    print("Resumo sweep:")
    print(f"  Leituras validas: {len(rows)}")
    print(f"  Erro medio ADS3: {mean_err3_v:.6f} V")
    print(f"  Erro medio ADS1: {mean_err1_v:.6f} V")


def main() -> int:
    try:
        port = resolve_arduino_port()
    except RuntimeError as exc:
        print(f"[ERRO] {exc}")
        return 1

    try:
        with serial.Serial(port=port, baudrate=115200, timeout=0.1) as ser:
            # Arduino reset after opening serial is common; wait and drain startup text.
            time.sleep(2.0)
            ser.reset_input_buffer()

            print("Conexao serial aberta.")
            print("Teste em 3 etapas: inicio, meio e fim da escala do MCP.")

            # os Valores esperados vem da Simulação com um resistor de 1K
            run_step(ser, 0x30, "comando 0x30 (inicio da escala)", 1.2540)
            run_step(ser, 0x31, "comando 0x31 (meio da escala)", 1.6499)
            run_step(ser, 0x32, "comando 0x32 (fim da escala)", 2.0459)
            run_sweep_step(ser, 0x33, "comando 0x33 (sweep MCP)", 1.2540, 2.0459)

            print("\nTeste concluido.")
            return 0

    except KeyboardInterrupt:
        print("\n[INFO] Teste interrompido pelo usuario.")
        return 0
    except serial.SerialException as exc:
        print(f"Erro de serial: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
