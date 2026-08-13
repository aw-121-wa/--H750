import { spawn, type ChildProcessWithoutNullStreams } from "node:child_process";

import { afterEach, describe, expect, it, vi } from "vitest";

import { MessageType } from "../src/protocol";
import { SerialLink } from "../src/serial-link";

class SimulatorPort {
  private child?: ChildProcessWithoutNullStreams;
  private controller?: ReadableStreamDefaultController<Uint8Array>;
  readonly diagnostics: string[] = [];

  readonly readable = new ReadableStream<Uint8Array>({
    start: (controller) => { this.controller = controller; }
  });

  readonly writable = new WritableStream<Uint8Array>({
    write: async (chunk) => {
      if (!this.child) throw new Error("H750 simulator is not running");
      for (let offset = 0; offset < chunk.length; offset += 3) {
        const part = chunk.slice(offset, Math.min(offset + 3, chunk.length));
        await new Promise<void>((resolve, reject) => {
          this.child!.stdin.write(part, (error) => error ? reject(error) : resolve());
        });
      }
    }
  });

  async open(): Promise<void> {
    const executable = process.env.H750_SIMULATOR_PATH;
    if (!executable) throw new Error("H750_SIMULATOR_PATH is not set");
    this.child = spawn(executable, [], { stdio: ["pipe", "pipe", "pipe"] });
    this.child.stdout.on("data", (data: Buffer) => {
      this.controller?.enqueue(new Uint8Array(data));
    });
    this.child.stderr.setEncoding("utf8");
    this.child.stderr.on("data", (data: string) => {
      this.diagnostics.push(...data.split(/\r?\n/).filter(Boolean));
    });
  }

  async close(): Promise<void> {
    if (!this.child) return;
    this.child.stdin.end();
    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.child?.kill();
        reject(new Error(`simulator did not exit: ${this.diagnostics.join(" | ")}`));
      }, 1000);
      this.child!.once("exit", (code) => {
        clearTimeout(timer);
        code === 0 ? resolve() : reject(new Error(
          `simulator exited with ${code}: ${this.diagnostics.join(" | ")}`
        ));
      });
    });
  }

  async waitForDiagnostic(text: string, timeoutMs = 1500): Promise<void> {
    const deadline = Date.now() + timeoutMs;
    while (!this.diagnostics.some((line) => line.includes(text))) {
      if (Date.now() >= deadline) {
        throw new Error(`missing ${text}: ${this.diagnostics.join(" | ")}`);
      }
      await new Promise((resolve) => setTimeout(resolve, 10));
    }
  }
}

const describeSoftwareLink = process.env.H750_SIMULATOR_PATH ? describe : describe.skip;

describeSoftwareLink("browser to H750 to ZDT software link", () => {
  afterEach(() => vi.restoreAllMocks());

  it("streams pose, commits a route and emits verified X-firmware CAN frames", async () => {
    const port = new SimulatorPort();
    Object.defineProperty(globalThis, "navigator", {
      configurable: true,
      value: { serial: { requestPort: async () => port } }
    });
    Object.defineProperty(globalThis, "window", {
      configurable: true,
      value: globalThis
    });
    const link = new SerialLink();
    const poses: unknown[] = [];
    link.addEventListener("pose", (event) => {
      poses.push((event as CustomEvent).detail);
    });

    await link.connect();
    await new Promise((resolve) => setTimeout(resolve, 240));
    expect(poses.length).toBeGreaterThanOrEqual(2);

    await link.uploadRoute([{ xMm: 1000, yMm: 0 }]);
    await link.command(MessageType.NAV_START);
    await port.waitForDiagnostic("CAN_COMMANDS_OK");

    clearInterval((link as unknown as { heartbeatTimer: number }).heartbeatTimer);
    await port.waitForDiagnostic("HEARTBEAT_STOP_OK", 1200);
    await link.disconnect();

    expect(port.diagnostics).toContain("MOTOR_REPLIES_OK");
    expect(port.diagnostics).toContain("CAN_COMMANDS_OK");
    expect(port.diagnostics).toContain("HEARTBEAT_STOP_OK");
  }, 5000);
});
