export interface Calibration {
  longitudinalMmPerX10Deg: number;
  lateralMmPerX10Deg: number;
  yawScale: number;
}

export interface MapDocument {
  version: 1;
  name: string;
  widthMm: number;
  heightMm: number;
  gridMm: number;
  robotLengthMm: number;
  robotWidthMm: number;
  safetyMarginMm: number;
  calibration: Calibration;
  obstacles: Uint8Array;
}

interface StoredMap extends Omit<MapDocument, "obstacles"> {
  obstacles: number[];
}

export function gridSize(map: Pick<MapDocument, "widthMm" | "heightMm" | "gridMm">): { columns: number; rows: number } {
  return {
    columns: Math.ceil(map.widthMm / map.gridMm),
    rows: Math.ceil(map.heightMm / map.gridMm)
  };
}

export function createDefaultMap(): MapDocument {
  const map: MapDocument = {
    version: 1,
    name: "未命名地图",
    widthMm: 5000,
    heightMm: 3000,
    gridMm: 50,
    robotLengthMm: 420,
    robotWidthMm: 360,
    safetyMarginMm: 80,
    calibration: {
      longitudinalMmPerX10Deg: 0.06723,
      lateralMmPerX10Deg: 0.06723,
      yawScale: 1
    },
    obstacles: new Uint8Array()
  };
  const { columns, rows } = gridSize(map);
  map.obstacles = new Uint8Array(columns * rows);
  return map;
}

export function resizeMap(map: MapDocument, widthMm: number, heightMm: number, gridMm: number): MapDocument {
  if (widthMm <= 0 || heightMm <= 0 || widthMm > 10000 || heightMm > 10000 || gridMm < 10) {
    throw new Error("地图范围必须在 10m 以内，网格不得小于 10mm");
  }
  const next = { ...map, widthMm, heightMm, gridMm, obstacles: new Uint8Array() };
  const { columns, rows } = gridSize(next);
  next.obstacles = new Uint8Array(columns * rows);
  return next;
}

export function serializeMap(map: MapDocument): string {
  const stored: StoredMap = { ...map, obstacles: Array.from(map.obstacles) };
  return JSON.stringify(stored, null, 2);
}

export function deserializeMap(json: string): MapDocument {
  const raw = JSON.parse(json) as Partial<StoredMap>;
  if (raw.version !== 1 || typeof raw.widthMm !== "number" || typeof raw.heightMm !== "number" ||
      typeof raw.gridMm !== "number" || !Array.isArray(raw.obstacles) || !raw.calibration) {
    throw new Error("地图 JSON 格式无效");
  }
  const base = createDefaultMap();
  const map: MapDocument = {
    ...base,
    ...raw,
    version: 1,
    calibration: { ...base.calibration, ...raw.calibration },
    obstacles: Uint8Array.from(raw.obstacles.map((value) => value ? 1 : 0))
  };
  const { columns, rows } = gridSize(map);
  if (map.widthMm > 10000 || map.heightMm > 10000 || map.obstacles.length !== columns * rows) {
    throw new Error("地图尺寸与障碍网格不匹配");
  }
  return map;
}

export function saveLocal(map: MapDocument): void {
  localStorage.setItem("gongxun-map-console", serializeMap(map));
}

export function loadLocal(): MapDocument {
  const saved = localStorage.getItem("gongxun-map-console");
  return saved ? deserializeMap(saved) : createDefaultMap();
}
