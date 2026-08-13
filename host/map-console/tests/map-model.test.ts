import { describe, expect, it } from "vitest";
import { createDefaultMap, deserializeMap, resizeMap, serializeMap } from "../src/map-model";

describe("map model", () => {
  it("round-trips settings and obstacle cells", () => {
    const map = resizeMap(createDefaultMap(), 3200, 3000, 50);
    map.obstacles[7] = 1;
    const restored = deserializeMap(serializeMap(map));
    expect(restored.widthMm).toBe(3200);
    expect(restored.obstacles[7]).toBe(1);
  });
});
