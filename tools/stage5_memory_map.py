#!/usr/bin/env python3
"""Report link-time FLASH/RAM allocations from a GNU ld Cortex-M4 map.

No device-memory or CPU runtime measurement is implied. Do not include this
report in hardware safety/performance claims.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

_REGION = re.compile(r"^([A-Za-z][\w*.-]*)\s+(0x[0-9A-Fa-f]+)\s+(0x[0-9A-Fa-f]+)\s+([rwx!]+)\s*$")
_SECTION = re.compile(
    r"^(\.[\w.]+)\s+(0x[0-9A-Fa-f]+)\s+(0x[0-9A-Fa-f]+)"
    r"(?:\s+load address\s+(0x[0-9A-Fa-f]+))?(?:\s|$)"
)
_NON_ALLOCATED = (".debug", ".zdebug", ".stab", ".gnu.lto_")
_NON_ALLOCATED_EXACT = {
    ".comment", ".ARM.attributes", ".note.GNU-stack",
    ".symtab", ".strtab", ".shstrtab",
}


def parse_map(path: Path) -> dict:
    """Count output sections once, and RAM .data initialization in FLASH."""
    regions: dict[str, dict] = {}
    sections: list[dict] = []
    memory_table = False
    linker_map = False

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if stripped == "Memory Configuration":
            memory_table, linker_map = True, False
            continue
        if stripped == "Linker script and memory map":
            memory_table, linker_map = False, True
            continue
        if memory_table:
            match = _REGION.fullmatch(stripped)
            if match and match.group(1) != "*default*":
                name, origin, length, attrs = match.groups()
                regions[name] = {
                    "origin": int(origin, 16),
                    "capacity_bytes": int(length, 16),
                    "attributes": attrs,
                }
        elif linker_map:
            # Input-section lines start with whitespace. Output-section
            # headings start in column 1, so there is no double count.
            match = _SECTION.match(line)
            if match:
                name, address, size, load = match.groups()
                if name.startswith(_NON_ALLOCATED) or name in _NON_ALLOCATED_EXACT:
                    continue
                sections.append({
                    "name": name,
                    "address": int(address, 16),
                    "size_bytes": int(size, 16),
                    "load_address": int(load, 16) if load else None,
                })

    if "FLASH" not in regions or "RAM" not in regions:
        raise ValueError("Map must contain FLASH and RAM memory regions")
    if not sections:
        raise ValueError("No GNU ld output sections were parsed")

    def region_at(address: int, size: int) -> str | None:
        for name, info in regions.items():
            start = info["origin"]
            if start <= address and address + size <= start + info["capacity_bytes"]:
                return name
        return None

    for section in sections:
        size = section["size_bytes"]
        section["vma_region"] = region_at(section["address"], size)
        load = section["load_address"]
        section["lma_region"] = region_at(load, size) if load is not None else None

    unknown = [s["name"] for s in sections
               if s["size_bytes"] and s["vma_region"] is None]
    if unknown:
        raise ValueError(f"Nonempty output sections outside declared memory: {unknown}")
    unknown_load = [s["name"] for s in sections if s["size_bytes"]
                    and s["load_address"] is not None and s["lma_region"] is None]
    if unknown_load:
        raise ValueError(f"Load images outside declared memory: {unknown_load}")

    # This project has no OVERLAY. Reject output overlaps rather than silently
    # adding the same memory twice.
    for region_name in ("FLASH", "RAM"):
        allocated = sorted(
            (s for s in sections if s["vma_region"] == region_name and s["size_bytes"]),
            key=lambda s: s["address"],
        )
        for previous, current in zip(allocated, allocated[1:]):
            if previous["address"] + previous["size_bytes"] > current["address"]:
                raise ValueError(
                    f"Overlapping {region_name} output sections: "
                    f"{previous['name']} and {current['name']}"
                )

    flash_vma = sum(s["size_bytes"] for s in sections if s["vma_region"] == "FLASH")
    ram_vma = sum(s["size_bytes"] for s in sections if s["vma_region"] == "RAM")
    # GNU ld can print an LMA for .bss even though ELF NOBITS has no
    # FLASH load image. Our linker.ld initializes RAM through .data only.
    unexpected_ram = [s["name"] for s in sections if s["size_bytes"]
                      and s["vma_region"] == "RAM"
                      and s["name"] not in (".data", ".bss")]
    if unexpected_ram:
        raise ValueError(
            f"Unrecognized RAM output section, review ELF flags: {unexpected_ram}"
        )
    if any(s["size_bytes"] and s["name"] == ".data"
           and s["lma_region"] != "FLASH" for s in sections):
        raise ValueError("Initialized RAM .data needs a FLASH load image")
    data_images = sum(
        s["size_bytes"] for s in sections
        if s["name"] == ".data" and s["vma_region"] == "RAM"
    )
    flash_bytes = flash_vma + data_images
    if (flash_bytes > regions["FLASH"]["capacity_bytes"]
            or ram_vma > regions["RAM"]["capacity_bytes"]):
        raise ValueError("Section allocation exceeds memory region capacity")

    return {
        "source_map": str(path),
        "regions": regions,
        "flash_allocated_bytes": flash_bytes,
        "flash_vma_bytes": flash_vma,
        "ram_initial_values_in_flash_bytes": data_images,
        "ram_allocated_bytes": ram_vma,
        "sections": sections,
        "measurement_scope": (
            "Link-time allocated output sections; excludes runtime heap/stack "
            "peaks, WCET and task CPU occupancy"
        ),
    }


def compare_maps(current: dict, before: dict) -> dict:
    """Compare only equal memory layouts; negative delta means reduction."""
    for region in ("FLASH", "RAM"):
        if current["regions"][region] != before["regions"][region]:
            raise ValueError("Cannot compare maps built against different region definitions")
    return {
        "flash": current["flash_allocated_bytes"] - before["flash_allocated_bytes"],
        "ram": current["ram_allocated_bytes"] - before["ram_allocated_bytes"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_file", type=Path)
    parser.add_argument("--before", type=Path, help="Prior map built for same memory layout")
    parser.add_argument("--output", type=Path, help="Write JSON evidence instead of stdout")
    args = parser.parse_args()
    current = parse_map(args.map_file)
    report = {"current": current}
    if args.before:
        earlier = parse_map(args.before)
        report["before"] = earlier
        report["delta_bytes"] = compare_maps(current, earlier)
    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    else:
        print(output, end="")


if __name__ == "__main__":
    main()
