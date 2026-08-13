import { describe, expect, it } from "vitest";
import { inflateGrid, planRoute } from "../src/planner";

describe("map planning", () => {
  it("inflates occupied cells by the requested radius", () => {
    const grid = new Uint8Array(25);
    grid[2 + 2 * 5] = 1;
    const inflated = inflateGrid(grid, 5, 5, 1);
    expect([...inflated].filter(Boolean)).toHaveLength(5);
  });

  it("routes around a corner without crossing diagonally through it", () => {
    const grid = new Uint8Array(25);
    grid[1] = 1;
    grid[5] = 1;
    expect(() => planRoute(grid, 5, 5, { x: 0, y: 0 }, { x: 4, y: 4 })).toThrow();
  });

  it("simplifies an open diagonal to its final waypoint", () => {
    const route = planRoute(new Uint8Array(100), 10, 10,
      { x: 0, y: 0 }, { x: 9, y: 9 });
    expect(route).toEqual([{ x: 9, y: 9 }]);
  });
});
