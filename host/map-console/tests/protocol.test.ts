import { describe, expect, it } from "vitest";
import { FrameParser, MessageType, crc16, encodeFrame } from "../src/protocol";

describe("host protocol", () => {
  it("encodes and parses a fragmented frame", () => {
    const received: { type: MessageType; sequence: number; payload: Uint8Array }[] = [];
    const parser = new FrameParser((frame) => received.push(frame));
    const encoded = encodeFrame(MessageType.SET_POSE, 0x1234, new Uint8Array([1, 2, 3]));

    parser.consume(encoded.slice(0, 4));
    parser.consume(encoded.slice(4));

    expect(received).toHaveLength(1);
    expect(received[0].sequence).toBe(0x1234);
    expect([...received[0].payload]).toEqual([1, 2, 3]);
  });

  it("matches CRC16-CCITT check vector", () => {
    expect(crc16(new TextEncoder().encode("123456789"))).toBe(0x29b1);
  });

});
