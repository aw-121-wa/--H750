import {
  FrameParser,
  MessageType,
  PoseReport,
  ProtocolFrame,
  crc16,
  decodePoseReport,
  encodeFrame
} from "./protocol";

export interface RoutePointMm { xMm: number; yMm: number }

type PendingRequest = {
  resolve: () => void;
  reject: (error: Error) => void;
  timer: number;
};

export class SerialLink extends EventTarget {
  private port?: SerialPort;
  private reader?: ReadableStreamDefaultReader<Uint8Array>;
  private writer?: WritableStreamDefaultWriter<Uint8Array>;
  private sequence = 1;
  private heartbeatTimer = 0;
  private pending = new Map<number, PendingRequest>();
  private parser = new FrameParser((frame) => this.onFrame(frame));

  get connected(): boolean { return this.port !== undefined; }

  async connect(): Promise<void> {
    if (!("serial" in navigator)) throw new Error("当前浏览器不支持 Web Serial");
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: "none" });
    this.writer = this.port.writable?.getWriter();
    if (!this.writer || !this.port.readable) throw new Error("串口读写流不可用");
    this.heartbeatTimer = window.setInterval(() => void this.send(MessageType.HEARTBEAT), 100);
    void this.readLoop();
    await this.command(MessageType.GET_STATUS);
    this.dispatchEvent(new Event("connection"));
  }

  async disconnect(): Promise<void> {
    window.clearInterval(this.heartbeatTimer);
    await this.reader?.cancel().catch(() => undefined);
    this.reader?.releaseLock();
    this.reader = undefined;
    this.writer?.releaseLock();
    this.writer = undefined;
    await this.port?.close().catch(() => undefined);
    this.port = undefined;
    for (const item of this.pending.values()) {
      window.clearTimeout(item.timer);
      item.reject(new Error("串口已断开"));
    }
    this.pending.clear();
    this.dispatchEvent(new Event("connection"));
  }

  private async readLoop(): Promise<void> {
    if (!this.port?.readable) return;
    this.reader = this.port.readable.getReader();
    try {
      while (true) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (value) this.parser.consume(value);
      }
    } catch (error) {
      this.dispatchEvent(new CustomEvent("error", { detail: error }));
    }
  }

  private onFrame(frame: ProtocolFrame): void {
    this.dispatchEvent(new CustomEvent("data", { detail: performance.now() }));
    if (frame.type === MessageType.POSE_REPORT) {
      try {
        const pose: PoseReport = decodePoseReport(frame.payload);
        this.dispatchEvent(new CustomEvent("pose", { detail: pose }));
      } catch (error) {
        this.dispatchEvent(new CustomEvent("error", { detail: error }));
      }
      return;
    }
    if (frame.type === MessageType.ACK || frame.type === MessageType.NACK) {
      const request = this.pending.get(frame.sequence);
      if (!request) return;
      window.clearTimeout(request.timer);
      this.pending.delete(frame.sequence);
      if (frame.type === MessageType.ACK) request.resolve();
      else request.reject(new Error(`设备拒绝命令 0x${(frame.payload[0] ?? 0).toString(16)}`));
    }
  }

  private async send(type: MessageType, payload = new Uint8Array(), sequence?: number): Promise<number> {
    if (!this.writer) throw new Error("串口未连接");
    const seq = sequence ?? this.sequence++ & 0xffff;
    await this.writer.write(encodeFrame(type, seq, payload));
    return seq;
  }

  async command(type: MessageType, payload = new Uint8Array(), retries = 2): Promise<void> {
    const sequence = this.sequence++ & 0xffff;
    for (let attempt = 0; attempt <= retries; attempt += 1) {
      try {
        await new Promise<void>(async (resolve, reject) => {
          const timer = window.setTimeout(() => {
            this.pending.delete(sequence);
            reject(new Error("设备响应超时"));
          }, 350);
          this.pending.set(sequence, { resolve, reject, timer });
          try { await this.send(type, payload, sequence); }
          catch (error) { window.clearTimeout(timer); this.pending.delete(sequence); reject(error); }
        });
        return;
      } catch (error) {
        if (attempt === retries) throw error;
      }
    }
  }

  async uploadRoute(points: RoutePointMm[]): Promise<void> {
    if (points.length === 0 || points.length > 128) throw new Error("路线点数量必须为 1 到 128");
    const routeBytes = new Uint8Array(points.length * 8);
    const routeView = new DataView(routeBytes.buffer);
    points.forEach((point, index) => {
      routeView.setInt32(index * 8, Math.round(point.xMm * 10), true);
      routeView.setInt32(index * 8 + 4, Math.round(point.yMm * 10), true);
    });
    const begin = new Uint8Array(4);
    const beginView = new DataView(begin.buffer);
    beginView.setUint16(0, points.length, true);
    beginView.setUint16(2, crc16(routeBytes), true);
    await this.command(MessageType.ROUTE_BEGIN, begin);
    for (let start = 0; start < points.length; start += 30) {
      const count = Math.min(30, points.length - start);
      const payload = new Uint8Array(3 + count * 8);
      const view = new DataView(payload.buffer);
      view.setUint16(0, start, true);
      payload[2] = count;
      payload.set(routeBytes.slice(start * 8, (start + count) * 8), 3);
      await this.command(MessageType.ROUTE_CHUNK, payload);
    }
    await this.command(MessageType.ROUTE_COMMIT);
  }
}
