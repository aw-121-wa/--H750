export const PROTOCOL_VERSION = 1;
const HEADER_A = 0xaa;
const HEADER_B = 0x55;
const MAX_PAYLOAD = 256;

export enum MessageType {
  HEARTBEAT = 0x01,
  GET_STATUS = 0x02,
  SET_POSE = 0x03,
  SET_CALIBRATION = 0x04,
  ROUTE_BEGIN = 0x10,
  ROUTE_CHUNK = 0x11,
  ROUTE_COMMIT = 0x12,
  NAV_START = 0x20,
  NAV_PAUSE = 0x21,
  NAV_RESUME = 0x22,
  NAV_STOP = 0x23,
  ACK = 0x80,
  NACK = 0x81,
  POSE_REPORT = 0x90,
  STATUS_REPORT = 0x91,
  FAULT_REPORT = 0x92
}

export interface ProtocolFrame {
  type: MessageType;
  sequence: number;
  payload: Uint8Array;
}

export interface PoseReport {
  xMm: number;
  yMm: number;
  yawDeg: number;
  vxMmS: number;
  vyMmS: number;
  yawRateDegS: number;
  uptimeMs: number;
  currentWaypoint: number;
  faultFlags: number;
  navigationState: number;
  poseState: number;
  motorValidMask: number;
  imuValid: boolean;
}

export function crc16(data: Uint8Array): number {
  let crc = 0xffff;
  for (const byte of data) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc & 0x8000) !== 0 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

export function encodeFrame(type: MessageType, sequence: number, payload = new Uint8Array()): Uint8Array {
  if (payload.length > MAX_PAYLOAD) throw new Error("负载超过 256 字节");
  const frame = new Uint8Array(payload.length + 10);
  const view = new DataView(frame.buffer);
  frame[0] = HEADER_A;
  frame[1] = HEADER_B;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = type;
  view.setUint16(4, sequence, true);
  view.setUint16(6, payload.length, true);
  frame.set(payload, 8);
  view.setUint16(8 + payload.length, crc16(frame.slice(2, 8 + payload.length)), true);
  return frame;
}

export class FrameParser {
  private buffer: number[] = [];

  constructor(private readonly onFrame: (frame: ProtocolFrame) => void) {}

  consume(chunk: Uint8Array): void {
    this.buffer.push(...chunk);
    while (this.buffer.length >= 2) {
      if (this.buffer[0] !== HEADER_A || this.buffer[1] !== HEADER_B) {
        this.buffer.shift();
        continue;
      }
      if (this.buffer.length < 8) return;
      const length = this.buffer[6] | (this.buffer[7] << 8);
      if (length > MAX_PAYLOAD) {
        this.buffer.splice(0, 2);
        continue;
      }
      const frameLength = length + 10;
      if (this.buffer.length < frameLength) return;
      const raw = Uint8Array.from(this.buffer.slice(0, frameLength));
      const view = new DataView(raw.buffer);
      const receivedCrc = view.getUint16(8 + length, true);
      if (raw[2] !== PROTOCOL_VERSION || crc16(raw.slice(2, 8 + length)) !== receivedCrc) {
        this.buffer.shift();
        continue;
      }
      this.onFrame({
        type: raw[3] as MessageType,
        sequence: view.getUint16(4, true),
        payload: raw.slice(8, 8 + length)
      });
      this.buffer.splice(0, frameLength);
    }
  }
}

export function decodePoseReport(payload: Uint8Array): PoseReport {
  if (payload.length !== 30) throw new Error("POSE_REPORT 长度错误");
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  return {
    xMm: view.getInt32(0, true) / 10,
    yMm: view.getInt32(4, true) / 10,
    yawDeg: view.getInt32(8, true) / 1000,
    vxMmS: view.getInt16(12, true) / 10,
    vyMmS: view.getInt16(14, true) / 10,
    yawRateDegS: view.getInt16(16, true) / 100,
    uptimeMs: view.getUint32(18, true),
    currentWaypoint: view.getUint16(22, true),
    faultFlags: view.getUint16(24, true),
    navigationState: payload[26],
    poseState: payload[27],
    motorValidMask: payload[28],
    imuValid: payload[29] !== 0
  };
}

export function encodePose(xMm: number, yMm: number, yawDeg: number): Uint8Array {
  const payload = new Uint8Array(12);
  const view = new DataView(payload.buffer);
  view.setInt32(0, Math.round(xMm * 10), true);
  view.setInt32(4, Math.round(yMm * 10), true);
  view.setInt32(8, Math.round(yawDeg * 1000), true);
  return payload;
}

export function encodeCalibration(longitudinal: number, lateral: number, yawScale: number): Uint8Array {
  const payload = new Uint8Array(12);
  const view = new DataView(payload.buffer);
  view.setFloat32(0, longitudinal, true);
  view.setFloat32(4, lateral, true);
  view.setFloat32(8, yawScale, true);
  return payload;
}
