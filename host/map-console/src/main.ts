import { createIcons, Cable, Download, Eraser, Focus, LocateFixed, MapPin, Pause, Pencil, Play, Redo2, RotateCcw, Save, Settings2, Square, Trash2, Undo2, Upload } from "lucide";
import "./style.css";
import { MapDocument, createDefaultMap, deserializeMap, gridSize, loadLocal, resizeMap, saveLocal, serializeMap } from "./map-model";
import { GridPoint, inflateGrid, planRoute } from "./planner";
import { MessageType, PoseReport, encodeCalibration, encodePose } from "./protocol";
import { RoutePointMm, SerialLink } from "./serial-link";

type Tool = "rect" | "brush" | "erase" | "target" | "anchor";
type PointMm = { xMm: number; yMm: number };

const iconMap = { Cable, Download, Eraser, Focus, LocateFixed, MapPin, Pause, Pencil, Play, Redo2, RotateCcw, Save, Settings2, Square, Trash2, Undo2, Upload };
const app = document.querySelector<HTMLDivElement>("#app")!;

app.innerHTML = `
  <header class="topbar">
    <div class="brand"><span class="brand-mark">H7</span><div><strong>地图导航台</strong><small>STM32H750 · X42S</small></div></div>
    <div class="connection-strip"><span id="data-dot" class="dot offline"></span><span id="connection-label">未连接</span><button id="connect" class="primary"><i data-lucide="cable"></i>连接串口</button></div>
  </header>
  <main class="workspace">
    <aside class="panel left-panel">
      <section><h2>地图参数</h2>
        <label>名称<input id="map-name" type="text"></label>
        <div class="fields"><label>宽度 mm<input id="map-width" type="number" min="100" max="10000"></label><label>高度 mm<input id="map-height" type="number" min="100" max="10000"></label></div>
        <div class="fields"><label>网格 mm<input id="grid-size" type="number" min="10" max="500" step="10"></label><label>安全边距 mm<input id="margin" type="number" min="0" max="1000"></label></div>
        <div class="fields"><label>车长 mm<input id="robot-length" type="number" min="50"></label><label>车宽 mm<input id="robot-width" type="number" min="50"></label></div>
        <button id="apply-map"><i data-lucide="save"></i>应用尺寸</button>
      </section>
      <section><h2>编辑工具</h2>
        <div class="tool-grid">
          <button class="tool active" data-tool="rect" title="矩形禁区"><i data-lucide="square"></i><span>矩形</span></button>
          <button class="tool" data-tool="brush" title="网格画笔"><i data-lucide="pencil"></i><span>画笔</span></button>
          <button class="tool" data-tool="erase" title="擦除禁区"><i data-lucide="eraser"></i><span>擦除</span></button>
          <button class="tool" data-tool="anchor" title="设置实际起点"><i data-lucide="locate-fixed"></i><span>起点</span></button>
          <button class="tool" data-tool="target" title="选择导航目标"><i data-lucide="map-pin"></i><span>目标</span></button>
        </div>
        <div class="icon-row"><button id="undo" title="撤销"><i data-lucide="undo-2"></i></button><button id="redo" title="重做"><i data-lucide="redo-2"></i></button><button id="clear" title="清空禁区"><i data-lucide="trash-2"></i></button></div>
      </section>
      <section><h2>地图文件</h2><div class="button-row"><button id="export"><i data-lucide="download"></i>导出</button><button id="import"><i data-lucide="upload"></i>导入</button><input id="file-input" hidden type="file" accept="application/json"></div><button id="calibrate"><i data-lucide="settings-2"></i>标定参数</button></section>
    </aside>
    <section class="map-stage">
      <div class="map-toolbar"><div><strong id="map-title">未命名地图</strong><span id="map-meta"></span></div><div class="legend"><span><b class="swatch obstacle"></b>禁区</span><span><b class="swatch inflated"></b>安全区</span><span><b class="swatch route"></b>路线</span></div></div>
      <div class="canvas-wrap"><canvas id="map-canvas"></canvas><div id="anchor-warning" class="anchor-warning">起点未确认 · 当前显示相对坐标</div><div id="cursor-coord" class="cursor-coord">X 0 · Y 0</div></div>
      <div id="route-confirm" class="route-confirm hidden"><span id="route-summary"></span><button id="cancel-route">取消</button><button id="send-route" class="primary"><i data-lucide="play"></i>发送并启动</button></div>
    </section>
    <aside class="panel right-panel">
      <section><h2>实时姿态</h2><div class="pose-grid"><div><span>X</span><strong id="pose-x">0.0</strong><small>mm</small></div><div><span>Y</span><strong id="pose-y">0.0</strong><small>mm</small></div><div><span>航向</span><strong id="pose-yaw">0.000</strong><small>°</small></div></div><div id="pose-state" class="state-banner warning">UNANCHORED · 起点未确认</div></section>
      <section><h2>导航状态</h2><dl class="status-list"><div><dt>状态</dt><dd id="nav-state">空闲</dd></div><div><dt>当前路段</dt><dd id="waypoint">0 / 0</dd></div><div><dt>数据延迟</dt><dd id="data-age">--</dd></div></dl><div class="command-grid"><button id="pause"><i data-lucide="pause"></i>暂停</button><button id="resume"><i data-lucide="play"></i>继续</button><button id="stop" class="danger"><i data-lucide="square"></i>停止</button></div></section>
      <section><h2>设备健康</h2><div class="health"><div><span>CAN / 电机</span><b id="motor-health" class="bad">0 / 4</b></div><div><span>HWT101</span><b id="imu-health" class="bad">无数据</b></div><div><span>串口链路</span><b id="link-health" class="bad">断开</b></div><div><span>故障标志</span><b id="fault-flags">0x0000</b></div></div></section>
      <section><h2>轨迹</h2><button id="clear-trail"><i data-lucide="rotate-ccw"></i>清空显示轨迹</button><p class="muted"><span id="trail-count">0</span> 个采样点</p></section>
    </aside>
  </main>
  <dialog id="anchor-dialog"><form method="dialog"><h2>确认地图起点</h2><p>位置来自地图点击点，输入小车当前朝向。</p><label>朝向 °<input id="anchor-yaw" type="number" min="-180" max="180" step="0.001" value="0"></label><div class="dialog-actions"><button value="cancel">取消</button><button id="anchor-submit" value="default" class="primary">发送起点</button></div></form></dialog>
  <dialog id="calibration-dialog"><form method="dialog"><h2>底盘标定</h2><p>低速完成动作后，填写目标值和实测值。系数按目标/实测修正。</p><div class="cal-row"><b>纵向</b><label>目标 mm<input id="cal-long-target" type="number" value="1000"></label><label>实测 mm<input id="cal-long-measured" type="number" value="1000"></label></div><div class="cal-row"><b>横向</b><label>目标 mm<input id="cal-lat-target" type="number" value="1000"></label><label>实测 mm<input id="cal-lat-measured" type="number" value="1000"></label></div><div class="cal-row"><b>旋转</b><label>目标 °<input id="cal-yaw-target" type="number" value="360"></label><label>实测 °<input id="cal-yaw-measured" type="number" value="360"></label></div><div class="dialog-actions"><button value="cancel">取消</button><button id="cal-submit" value="default" class="primary">计算并下发</button></div></form></dialog>
  <div id="toast" class="toast hidden"></div>
`;

createIcons({ icons: iconMap });

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const canvas = $("map-canvas") as HTMLCanvasElement;
const ctx = canvas.getContext("2d")!;
const serial = new SerialLink();
let map: MapDocument;
try { map = loadLocal(); } catch { map = createDefaultMap(); }
let tool: Tool = "rect";
let pose: PoseReport | undefined;
let trail: PointMm[] = [];
let routeGrid: GridPoint[] = [];
let routeMm: RoutePointMm[] = [];
let target: PointMm | undefined;
let anchorCandidate: PointMm | undefined;
let pointerStart: GridPoint | undefined;
let pointerDown = false;
let lastDataAt = 0;
let lastPoseAt = 0;
let undoStack: Uint8Array[] = [];
let redoStack: Uint8Array[] = [];
let renderMetrics = { left: 0, top: 0, width: 1, height: 1, scale: 1 };

function toast(message: string, error = false): void {
  const node = $("toast");
  node.textContent = message;
  node.className = `toast ${error ? "error" : ""}`;
  window.setTimeout(() => node.classList.add("hidden"), 2600);
}

function syncInputs(): void {
  $("map-name").setAttribute("value", map.name);
  ($("map-name") as HTMLInputElement).value = map.name;
  ($("map-width") as HTMLInputElement).value = String(map.widthMm);
  ($("map-height") as HTMLInputElement).value = String(map.heightMm);
  ($("grid-size") as HTMLInputElement).value = String(map.gridMm);
  ($("margin") as HTMLInputElement).value = String(map.safetyMarginMm);
  ($("robot-length") as HTMLInputElement).value = String(map.robotLengthMm);
  ($("robot-width") as HTMLInputElement).value = String(map.robotWidthMm);
  $("map-title").textContent = map.name;
  $("map-meta").textContent = `${map.widthMm / 1000} × ${map.heightMm / 1000} m · ${map.gridMm} mm 网格`;
}

function snapshot(): void {
  undoStack.push(map.obstacles.slice());
  if (undoStack.length > 50) undoStack.shift();
  redoStack = [];
}

function persist(): void { saveLocal(map); syncInputs(); render(); }

function gridFromEvent(event: PointerEvent): GridPoint {
  const rect = canvas.getBoundingClientRect();
  const x = Math.floor(((event.clientX - rect.left) * canvas.width / rect.width - renderMetrics.left) / renderMetrics.scale / map.gridMm);
  const y = Math.floor((map.heightMm - ((event.clientY - rect.top) * canvas.height / rect.height - renderMetrics.top) / renderMetrics.scale) / map.gridMm);
  const { columns, rows } = gridSize(map);
  return { x: Math.max(0, Math.min(columns - 1, x)), y: Math.max(0, Math.min(rows - 1, y)) };
}

function mmFromEvent(event: PointerEvent): PointMm {
  const rect = canvas.getBoundingClientRect();
  const px = (event.clientX - rect.left) * canvas.width / rect.width;
  const py = (event.clientY - rect.top) * canvas.height / rect.height;
  return {
    xMm: Math.max(0, Math.min(map.widthMm, (px - renderMetrics.left) / renderMetrics.scale)),
    yMm: Math.max(0, Math.min(map.heightMm, map.heightMm - (py - renderMetrics.top) / renderMetrics.scale))
  };
}

function mmFromGrid(point: GridPoint): PointMm {
  return { xMm: Math.min(map.widthMm, (point.x + 0.5) * map.gridMm), yMm: Math.min(map.heightMm, (point.y + 0.5) * map.gridMm) };
}

function canvasPoint(point: PointMm): [number, number] {
  return [renderMetrics.left + point.xMm * renderMetrics.scale,
    renderMetrics.top + (map.heightMm - point.yMm) * renderMetrics.scale];
}

function paintCell(point: GridPoint, value: number): void {
  const { columns } = gridSize(map);
  map.obstacles[point.x + point.y * columns] = value;
}

function applyRectangle(a: GridPoint, b: GridPoint): void {
  for (let y = Math.min(a.y, b.y); y <= Math.max(a.y, b.y); y += 1)
    for (let x = Math.min(a.x, b.x); x <= Math.max(a.x, b.x); x += 1) paintCell({ x, y }, 1);
}

function inflatedObstacles(): Uint8Array {
  const { columns, rows } = gridSize(map);
  const radiusMm = Math.hypot(map.robotLengthMm / 2, map.robotWidthMm / 2) + map.safetyMarginMm;
  return inflateGrid(map.obstacles, columns, rows, Math.ceil(radiusMm / map.gridMm));
}

function drawGridCells(data: Uint8Array, color: string): void {
  const { columns, rows } = gridSize(map);
  ctx.fillStyle = color;
  for (let y = 0; y < rows; y += 1) for (let x = 0; x < columns; x += 1) {
    if (!data[x + y * columns]) continue;
    const [px, py] = canvasPoint({ xMm: x * map.gridMm, yMm: (y + 1) * map.gridMm });
    ctx.fillRect(px, py, map.gridMm * renderMetrics.scale + 0.5, map.gridMm * renderMetrics.scale + 0.5);
  }
}

function render(): void {
  const bounds = canvas.parentElement!.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(bounds.width * dpr));
  canvas.height = Math.max(1, Math.round(bounds.height * dpr));
  const pad = 42 * dpr;
  const scale = Math.min((canvas.width - pad * 2) / map.widthMm, (canvas.height - pad * 2) / map.heightMm);
  renderMetrics = { scale, width: map.widthMm * scale, height: map.heightMm * scale,
    left: (canvas.width - map.widthMm * scale) / 2, top: (canvas.height - map.heightMm * scale) / 2 };
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(renderMetrics.left, renderMetrics.top, renderMetrics.width, renderMetrics.height);
  drawGridCells(inflatedObstacles(), "rgba(245, 158, 11, .13)");
  drawGridCells(map.obstacles, "rgba(220, 38, 38, .66)");
  ctx.strokeStyle = "rgba(148, 163, 184, .22)";
  ctx.lineWidth = Math.max(1, dpr);
  const { columns, rows } = gridSize(map);
  if (columns <= 200 && rows <= 200) {
    for (let x = 0; x <= columns; x += 1) { const px = renderMetrics.left + x * map.gridMm * scale; ctx.beginPath(); ctx.moveTo(px, renderMetrics.top); ctx.lineTo(px, renderMetrics.top + renderMetrics.height); ctx.stroke(); }
    for (let y = 0; y <= rows; y += 1) { const py = renderMetrics.top + y * map.gridMm * scale; ctx.beginPath(); ctx.moveTo(renderMetrics.left, py); ctx.lineTo(renderMetrics.left + renderMetrics.width, py); ctx.stroke(); }
  }
  if (trail.length > 1) drawPolyline(trail, "#0284c7", 2 * dpr);
  if (routeMm.length) drawPolyline(pose ? [{ xMm: pose.xMm, yMm: pose.yMm }, ...routeMm] : routeMm, "#16a34a", 3 * dpr, [8 * dpr, 5 * dpr]);
  if (target) drawMarker(target, "#16a34a", 8 * dpr);
  if (pose) drawRobot(pose, dpr);
  ctx.strokeStyle = "#64748b"; ctx.lineWidth = 1.5 * dpr; ctx.strokeRect(renderMetrics.left, renderMetrics.top, renderMetrics.width, renderMetrics.height);
}

function drawPolyline(points: PointMm[], color: string, width: number, dash: number[] = []): void {
  ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.beginPath();
  points.forEach((point, index) => { const [x, y] = canvasPoint(point); if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
  ctx.stroke(); ctx.restore();
}

function drawMarker(point: PointMm, color: string, radius: number): void {
  const [x, y] = canvasPoint(point); ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2); ctx.fill();
}

function drawRobot(report: PoseReport, dpr: number): void {
  const [x, y] = canvasPoint({ xMm: report.xMm, yMm: report.yMm });
  ctx.save(); ctx.translate(x, y); ctx.rotate(-report.yawDeg * Math.PI / 180);
  ctx.fillStyle = "#0f172a"; ctx.strokeStyle = "#fff"; ctx.lineWidth = 2 * dpr;
  const length = Math.max(18 * dpr, map.robotLengthMm * renderMetrics.scale);
  const width = Math.max(14 * dpr, map.robotWidthMm * renderMetrics.scale);
  ctx.fillRect(-length / 2, -width / 2, length, width); ctx.strokeRect(-length / 2, -width / 2, length, width);
  ctx.fillStyle = "#38bdf8"; ctx.beginPath(); ctx.moveTo(length / 2 + 7 * dpr, 0); ctx.lineTo(length / 2 - 5 * dpr, -6 * dpr); ctx.lineTo(length / 2 - 5 * dpr, 6 * dpr); ctx.fill(); ctx.restore();
}

function planTo(point: GridPoint, exactTarget: PointMm): void {
  if (!pose) { toast("尚未收到小车位置", true); return; }
  const { columns, rows } = gridSize(map);
  const start = { x: Math.max(0, Math.min(columns - 1, Math.floor(pose.xMm / map.gridMm))), y: Math.max(0, Math.min(rows - 1, Math.floor(pose.yMm / map.gridMm))) };
  try {
    routeGrid = planRoute(inflatedObstacles(), columns, rows, start, point);
    routeMm = routeGrid.map(mmFromGrid);
    const gridCenter = routeMm[routeMm.length - 1];
    if (Math.abs(gridCenter.xMm - exactTarget.xMm) > 0.1 ||
        Math.abs(gridCenter.yMm - exactTarget.yMm) > 0.1) {
      routeMm.push(exactTarget);
    }
    if (routeMm.length > 128) throw new Error("路线拐点超过 128 个");
    target = exactTarget;
    $("route-summary").textContent = `${routeMm.length} 个拐点 · 目标 (${target.xMm.toFixed(0)}, ${target.yMm.toFixed(0)}) mm`;
    $("route-confirm").classList.remove("hidden"); render();
  } catch (error) { toast((error as Error).message, true); }
}

canvas.addEventListener("pointerdown", (event) => {
  const navState = pose?.navigationState ?? 0;
  if ([2, 3, 5].includes(navState) && ["rect", "brush", "erase"].includes(tool)) { toast("导航期间地图已锁定", true); return; }
  pointerDown = true; pointerStart = gridFromEvent(event); canvas.setPointerCapture(event.pointerId);
  if (tool === "brush" || tool === "erase") { snapshot(); paintCell(pointerStart, tool === "brush" ? 1 : 0); persist(); }
});

canvas.addEventListener("pointermove", (event) => {
  const point = gridFromEvent(event); const mm = mmFromGrid(point); $("cursor-coord").textContent = `X ${mm.xMm.toFixed(0)} · Y ${mm.yMm.toFixed(0)}`;
  if (pointerDown && (tool === "brush" || tool === "erase")) { paintCell(point, tool === "brush" ? 1 : 0); persist(); }
});

canvas.addEventListener("pointerup", (event) => {
  if (!pointerStart) return; const point = gridFromEvent(event); pointerDown = false;
  if (tool === "rect") { snapshot(); applyRectangle(pointerStart, point); persist(); }
  if (tool === "target") planTo(point, mmFromEvent(event));
  if (tool === "anchor") { anchorCandidate = mmFromEvent(event); ($("anchor-dialog") as HTMLDialogElement).showModal(); }
  pointerStart = undefined;
});

document.querySelectorAll<HTMLButtonElement>(".tool").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".tool").forEach((item) => item.classList.remove("active")); button.classList.add("active"); tool = button.dataset.tool as Tool;
}));

$("apply-map").addEventListener("click", () => {
  try {
    const width = Number(($("map-width") as HTMLInputElement).value); const height = Number(($("map-height") as HTMLInputElement).value); const grid = Number(($("grid-size") as HTMLInputElement).value);
    const sizeChanged = width !== map.widthMm || height !== map.heightMm || grid !== map.gridMm;
    if (sizeChanged && map.obstacles.some(Boolean) && !confirm("修改地图尺寸会清空禁区，继续吗？")) return;
    if (sizeChanged) map = resizeMap(map, width, height, grid);
    map.name = ($("map-name") as HTMLInputElement).value.trim() || "未命名地图";
    map.safetyMarginMm = Number(($("margin") as HTMLInputElement).value);
    map.robotLengthMm = Number(($("robot-length") as HTMLInputElement).value);
    map.robotWidthMm = Number(($("robot-width") as HTMLInputElement).value);
    routeMm = []; target = undefined; persist(); toast("地图参数已应用");
  } catch (error) { toast((error as Error).message, true); }
});

$("undo").addEventListener("click", () => { const previous = undoStack.pop(); if (!previous) return; redoStack.push(map.obstacles.slice()); map.obstacles = previous; persist(); });
$("redo").addEventListener("click", () => { const next = redoStack.pop(); if (!next) return; undoStack.push(map.obstacles.slice()); map.obstacles = next; persist(); });
$("clear").addEventListener("click", () => { if (!map.obstacles.some(Boolean)) return; snapshot(); map.obstacles.fill(0); persist(); });
$("clear-trail").addEventListener("click", () => { trail = []; $("trail-count").textContent = "0"; render(); });
$("cancel-route").addEventListener("click", () => { routeMm = []; target = undefined; $("route-confirm").classList.add("hidden"); render(); });

$("export").addEventListener("click", () => { const blob = new Blob([serializeMap(map)], { type: "application/json" }); const url = URL.createObjectURL(blob); const link = document.createElement("a"); link.href = url; link.download = `${map.name}.json`; link.click(); URL.revokeObjectURL(url); });
$("import").addEventListener("click", () => ($("file-input") as HTMLInputElement).click());
$("file-input").addEventListener("change", async () => { const file = ($("file-input") as HTMLInputElement).files?.[0]; if (!file) return; try { map = deserializeMap(await file.text()); routeMm = []; target = undefined; persist(); toast("地图已导入"); } catch (error) { toast((error as Error).message, true); } });

$("connect").addEventListener("click", async () => { try { if (serial.connected) await serial.disconnect(); else await serial.connect(); } catch (error) { toast((error as Error).message, true); } });
$("send-route").addEventListener("click", async () => { try { if (!serial.connected) throw new Error("请先连接串口"); if (pose?.poseState !== 1) throw new Error("请先确认地图起点"); await serial.uploadRoute(routeMm); await serial.command(MessageType.NAV_START); $("route-confirm").classList.add("hidden"); toast("路线已提交并启动"); } catch (error) { toast((error as Error).message, true); } });
$("pause").addEventListener("click", () => void serial.command(MessageType.NAV_PAUSE).catch((error) => toast(error.message, true)));
$("resume").addEventListener("click", () => void serial.command(MessageType.NAV_RESUME).catch((error) => toast(error.message, true)));
$("stop").addEventListener("click", () => void serial.command(MessageType.NAV_STOP, new Uint8Array(), 0).catch((error) => toast(error.message, true)));

$("anchor-submit").addEventListener("click", (event) => { event.preventDefault(); if (!anchorCandidate) return; const yaw = Number(($("anchor-yaw") as HTMLInputElement).value); void serial.command(MessageType.SET_POSE, new Uint8Array(encodePose(anchorCandidate.xMm, anchorCandidate.yMm, yaw))).then(() => { ($("anchor-dialog") as HTMLDialogElement).close(); toast("地图起点已发送"); }).catch((error) => toast(error.message, true)); });
$("calibrate").addEventListener("click", () => ($("calibration-dialog") as HTMLDialogElement).showModal());
$("cal-submit").addEventListener("click", (event) => { event.preventDefault(); const lt = Number(($("cal-long-target") as HTMLInputElement).value), lm = Number(($("cal-long-measured") as HTMLInputElement).value), at = Number(($("cal-lat-target") as HTMLInputElement).value), am = Number(($("cal-lat-measured") as HTMLInputElement).value), yt = Number(($("cal-yaw-target") as HTMLInputElement).value), ym = Number(($("cal-yaw-measured") as HTMLInputElement).value); if ([lm, am, ym].some((v) => !Number.isFinite(v) || v === 0)) { toast("实测值不能为 0", true); return; } map.calibration.longitudinalMmPerX10Deg *= lt / lm; map.calibration.lateralMmPerX10Deg *= at / am; map.calibration.yawScale *= yt / ym; const calibration = new Uint8Array(encodeCalibration(map.calibration.longitudinalMmPerX10Deg, map.calibration.lateralMmPerX10Deg, map.calibration.yawScale)); void serial.command(MessageType.SET_CALIBRATION, calibration).then(() => { saveLocal(map); ($("calibration-dialog") as HTMLDialogElement).close(); toast("标定参数已保存并下发"); }).catch((error) => toast(error.message, true)); });

const navNames = ["空闲", "路线就绪", "对向", "行驶", "已到达", "已暂停", "故障", "急停"];
serial.addEventListener("connection", () => { $("connection-label").textContent = serial.connected ? "串口已连接" : "未连接"; $("connect").innerHTML = serial.connected ? '<i data-lucide="cable"></i>断开串口' : '<i data-lucide="cable"></i>连接串口'; createIcons({ icons: iconMap }); });
serial.addEventListener("data", (event) => { lastDataAt = (event as CustomEvent<number>).detail; });
serial.addEventListener("error", (event) => toast(String((event as CustomEvent).detail), true));
serial.addEventListener("pose", (event) => {
  lastPoseAt = performance.now();
  pose = (event as CustomEvent<PoseReport>).detail;
  trail.push({ xMm: pose.xMm, yMm: pose.yMm }); if (trail.length > 5000) trail.shift();
  $("pose-x").textContent = pose.xMm.toFixed(1); $("pose-y").textContent = pose.yMm.toFixed(1); $("pose-yaw").textContent = pose.yawDeg.toFixed(3);
  $("pose-state").textContent = pose.poseState ? "ANCHORED · 地图坐标已确认" : "UNANCHORED · 起点未确认"; $("pose-state").className = `state-banner ${pose.poseState ? "ok" : "warning"}`; $("anchor-warning").classList.toggle("hidden", pose.poseState === 1);
  $("nav-state").textContent = navNames[pose.navigationState] ?? `未知 ${pose.navigationState}`; $("waypoint").textContent = `${pose.currentWaypoint} / ${routeMm.length}`;
  const motors = [0, 1, 2, 3].filter((bit) => pose!.motorValidMask & (1 << bit)).length; $("motor-health").textContent = `${motors} / 4`; $("motor-health").className = motors === 4 ? "good" : "bad";
  $("imu-health").textContent = pose.imuValid ? "正常" : "超时"; $("imu-health").className = pose.imuValid ? "good" : "bad"; $("fault-flags").textContent = `0x${pose.faultFlags.toString(16).padStart(4, "0")}`; $("trail-count").textContent = String(trail.length); render();
});

window.setInterval(() => {
  const linkAge = lastDataAt ? performance.now() - lastDataAt : Infinity;
  const poseAge = lastPoseAt ? performance.now() - lastPoseAt : Infinity;
  $("data-age").textContent = Number.isFinite(poseAge) ? `${Math.round(poseAge)} ms` : "--";
  const dot = $("data-dot"); const label = $("link-health"); dot.className = `dot ${linkAge > 500 ? "offline" : poseAge > 300 ? "delayed" : "online"}`; label.textContent = linkAge > 500 ? "连接中断" : poseAge > 300 ? "数据延迟" : "正常"; label.className = linkAge > 500 || poseAge > 300 ? "bad" : "good";
}, 100);
window.addEventListener("resize", render);
syncInputs(); render();
