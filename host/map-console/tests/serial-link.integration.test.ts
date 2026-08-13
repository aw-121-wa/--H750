import { afterEach, describe, expect, it, vi } from "vitest";

import { MessageType, ProtocolFrame, FrameParser, encodeFrame } from "../src/protocol";
import { SerialLink } from "../src/serial-link";

class MockH750Port {
  readonly receivedTypes: MessageType[] = [];
  private controller?: ReadableStreamDefaultController<Uint8Array>;
  private readonly parser = new FrameParser((frame) => this.onHostFrame(frame));

  readonly readable = new ReadableStream<Uint8Array>({
    start: (controller) => { this.controller = controller; }
  });

  readonly writable = new WritableStream<Uint8Array>({
    write: (chunk) => { this.parser.consume(chunk); }
  });

  async open(): Promise<void> {
    queueMicrotask(() => this.sendPose());
  }

  async close(): Promise<void> {}

  private onHostFrame(frame: ProtocolFrame): void {
    this.receivedTypes.push(frame.type);
    const payload = new Uint8Array([frame.type, 0]);
    this.controller?.enqueue(encodeFrame(MessageType.ACK, frame.sequence, payload));
  }

  private sendPose(): void {
    const payload = new Uint8Array(30);
    const view = new DataView(payload.buffer);
    view.setInt32(0, 12345, true);
    view.setInt32(4, 6789, true);
    view.setInt32(8, 90000, true);
    view.setUint32(18, 1000, true);
    payload[27] = 1;
    payload[28] = 0x0f;
    payload[29] = 1;
    this.controller?.enqueue(encodeFrame(MessageType.POSE_REPORT, 77, payload));
  }

}

describe("SerialLink with simulated H750", () => {
  afterEach(() => vi.useRealTimers());

  it("exchanges status, telemetry, heartbeat and an atomic route", async () => {
    vi.useFakeTimers();
    const port = new MockH750Port();
    Object.defineProperty(globalThis, "navigator", {
      configurable: true,
      value: { serial: { requestPort: async () => port } }
    });
    Object.defineProperty(globalThis, "window", {
      configurable: true,
      value: globalThis
    });
    const link = new SerialLink();
    const poses: Array<{ xMm: number; yMm: number; yawDeg: number }> = [];
    link.addEventListener("pose", (event) => {
      poses.push((event as CustomEvent).detail);
    });

    const connected = link.connect();
    await vi.runAllTicks();
    await connected;
    await link.uploadRoute([
      { xMm: 100, yMm: 200 },
      { xMm: 300, yMm: 400 }
    ]);
    await link.command(MessageType.NAV_START);
    await vi.advanceTimersByTimeAsync(250);

    expect(poses).toMatchObject([{ xMm: 1234.5, yMm: 678.9, yawDeg: 90 }]);
    expect(port.receivedTypes).toContain(MessageType.GET_STATUS);
    expect(port.receivedTypes).toContain(MessageType.ROUTE_BEGIN);
    expect(port.receivedTypes).toContain(MessageType.ROUTE_CHUNK);
    expect(port.receivedTypes).toContain(MessageType.ROUTE_COMMIT);
    expect(port.receivedTypes).toContain(MessageType.NAV_START);
    expect(port.receivedTypes.filter((type) => type === MessageType.HEARTBEAT).length)
      .toBeGreaterThanOrEqual(2);

    await link.disconnect();
  });
});
