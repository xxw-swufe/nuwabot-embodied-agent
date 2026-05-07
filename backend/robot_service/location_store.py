from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


DEFAULT_LOCATIONS_PATH = Path(__file__).resolve().parents[2] / "maps" / "locations.json"
DEFAULT_WORLD_MAP_PATH = Path(__file__).resolve().parents[2] / "maps" / "world_map.json"


@dataclass(frozen=True)
class Location:
    name: str
    label: str
    x: float
    y: float

    def to_dict(self) -> dict:
        return asdict(self)


class LocationStore:
    def __init__(
        self,
        path: Path | str = DEFAULT_LOCATIONS_PATH,
        world_map_path: Path | str = DEFAULT_WORLD_MAP_PATH,
    ) -> None:
        self.path = Path(path)
        self.world_map_path = Path(world_map_path)

    def list_locations(self) -> list[dict]:
        return [location.to_dict() for location in self._load().values()]

    def get(self, name: str) -> Location:
        normalized = self._normalize_name(name)
        locations = self._load()
        if normalized in locations:
            return locations[normalized]

        for location in locations.values():
            if normalized == self._normalize_name(location.label):
                return location

        available = ", ".join(sorted(locations))
        raise KeyError(f"unknown location '{name}', available: {available}")

    def _load(self) -> dict[str, Location]:
        raw_locations = self._load_raw_locations()

        locations: dict[str, Location] = {}
        for name, data in raw_locations.items():
            normalized = self._normalize_name(name)
            locations[normalized] = Location(
                name=normalized,
                label=str(data.get("label", name)),
                x=float(data["x"]),
                y=float(data["y"]),
            )
        return locations

    def _load_raw_locations(self) -> dict:
        if self.world_map_path.exists():
            with self.world_map_path.open("r", encoding="utf-8") as f:
                world_map = json.load(f)
            if "locations" in world_map:
                return world_map["locations"]

        with self.path.open("r", encoding="utf-8") as f:
            return json.load(f)

    @staticmethod
    def _normalize_name(name: str) -> str:
        return str(name).strip().lower()
