import { IKernelFunctionThis } from 'gpu.js';

/**
 * Rebuilds one signed 16-bit elevation from its low and high byte, each read as byte / 255.
 * Values from 32768 up are the negative half: water (-1) is stored as 255, 255.
 */
export function decodeTerrainElevation(lowChannel: number, highChannel: number): number {
  const low = Math.round(lowChannel * 255);
  const high = Math.round(highChannel * 255);
  const value = low + high * 256;
  return value >= 32768 ? value - 65536 : value;
}

/** Self-test kernel: reads one packed elevation exactly the way the terrain kernels index it. */
export function readPackedTerrainElevation(this: IKernelFunctionThis, map: number[][]): number {
  return decodeTerrainElevation(map[this.thread.y][this.thread.x * 2], map[this.thread.y][this.thread.x * 2 + 1]);
}
