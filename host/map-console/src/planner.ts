import PF from "pathfinding";

export interface GridPoint { x: number; y: number }

export function inflateGrid(source: Uint8Array, width: number, height: number, radiusCells: number): Uint8Array {
  const inflated = new Uint8Array(source.length);
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      if (!source[x + y * width]) continue;
      for (let dy = -radiusCells; dy <= radiusCells; dy += 1) {
        for (let dx = -radiusCells; dx <= radiusCells; dx += 1) {
          const px = x + dx;
          const py = y + dy;
          if (px >= 0 && py >= 0 && px < width && py < height && dx * dx + dy * dy <= radiusCells * radiusCells) {
            inflated[px + py * width] = 1;
          }
        }
      }
    }
  }
  return inflated;
}

function clearLine(grid: Uint8Array, width: number, height: number, start: GridPoint, end: GridPoint): boolean {
  let x = start.x;
  let y = start.y;
  const dx = Math.abs(end.x - start.x);
  const dy = Math.abs(end.y - start.y);
  const sx = start.x < end.x ? 1 : -1;
  const sy = start.y < end.y ? 1 : -1;
  let error = dx - dy;
  while (true) {
    if (x < 0 || y < 0 || x >= width || y >= height || grid[x + y * width]) return false;
    if (x === end.x && y === end.y) return true;
    const twice = error * 2;
    const oldX = x;
    const oldY = y;
    if (twice > -dy) { error -= dy; x += sx; }
    if (twice < dx) { error += dx; y += sy; }
    if (x !== oldX && y !== oldY &&
        (grid[x + oldY * width] || grid[oldX + y * width])) return false;
  }
}

export function planRoute(obstacles: Uint8Array, width: number, height: number,
                          start: GridPoint, goal: GridPoint): GridPoint[] {
  if (obstacles.length !== width * height || obstacles[start.x + start.y * width] || obstacles[goal.x + goal.y * width]) {
    throw new Error("起点或目标位于禁区");
  }
  const matrix: number[][] = [];
  for (let y = 0; y < height; y += 1) {
    matrix.push(Array.from(obstacles.slice(y * width, (y + 1) * width)));
  }
  const finder = new PF.AStarFinder({
    diagonalMovement: PF.DiagonalMovement.OnlyWhenNoObstacles
  });
  const raw = finder.findPath(start.x, start.y, goal.x, goal.y, new PF.Grid(matrix));
  if (raw.length === 0) throw new Error("没有可通行路线");
  const points = raw.map(([x, y]) => ({ x, y }));
  const simplified: GridPoint[] = [];
  let anchor = 0;
  while (anchor < points.length - 1) {
    let candidate = points.length - 1;
    while (candidate > anchor + 1 && !clearLine(obstacles, width, height, points[anchor], points[candidate])) {
      candidate -= 1;
    }
    simplified.push(points[candidate]);
    anchor = candidate;
  }
  if (simplified.length > 128) throw new Error("路线拐点超过 128 个");
  return simplified;
}
