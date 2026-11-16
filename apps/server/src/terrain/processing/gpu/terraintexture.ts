import { CPUKernel, GPU, IKernelRunShortcut, Texture } from 'gpu.js';
import { InvalidElevation, UnknownElevation, WaterElevation } from '../generic/constants';
import { decodeTerrainElevation, readPackedTerrainElevation } from './terrainsampling';

export type TerrainElevationData = Texture | Float32Array[];

// GPU.js exposes these fields at runtime, but omits them from its Texture typings.
interface PackedTexture extends Texture {
  texture: WebGLTexture;
  context: WebGLRenderingContext;
  size: number[];
  dimensions: number[];
  output: number[];
}

/**
 * Keeps a terrain map on the GPU at 2 bytes per elevation instead of a 4-byte float.
 *
 * Elevations are whole numbers, so each one is stored as the two bytes of an Int16
 * (two elevations per RGBA8 texel). GPU.js still hands kernels every byte as its own
 * number between 0 and 1, so they rebuild the value with decodeTerrainElevation.
 * Example for 1904 m:
 *   stored bytes  112, 7          (1904 = 112 + 7 * 256)
 *   kernel reads  0.439, 0.027    (112 / 255, 7 / 255)
 *   decoded       112 + 7 * 256 = 1904
 *
 * GPU.js has no supported way to store data like this. This class takes the texture
 * of a tiny kernel, replaces its storage through WebGL and overwrites its size fields.
 * Because that relies on GPU.js internals, gpu.js is pinned to an exact version in
 * package.json and MapHandler runs verifyPackedTerrain at startup. There is no
 * fallback on purpose: if that check fails, terrain stays off until this is fixed.
 */
export class TerrainTexture {
  private owner: IKernelRunShortcut = null;

  private texture: PackedTexture = null;

  private storageSize: number[] = [0, 0];

  constructor(private readonly gpu: GPU) {}

  /** RGBA8 storage requested from GL, including padding; excludes driver overhead. */
  public get allocatedBytes(): number {
    return this.storageSize[0] * this.storageSize[1] * 4;
  }

  public upload(data: Int16Array, width: number, height: number): TerrainElevationData {
    if (
      !Number.isInteger(width) ||
      !Number.isInteger(height) ||
      width < 1 ||
      height < 1 ||
      data.length !== width * height
    ) {
      throw new Error('Invalid terrain texture dimensions');
    }
    if (this.gpu.Kernel === CPUKernel) {
      return Array.from({ length: height }, (_, y) => Float32Array.from(data.subarray(y * width, (y + 1) * width)));
    }

    if (!this.owner) {
      // Let GPU.js own a packed 2D Texture for sampling and destruction.
      // Run this tiny, argument-free kernel only once; later uploads write its
      // texture directly. The default drawing buffer stays small.
      this.owner = this.gpu.createKernel(
        function () {
          return 0;
        },
        {
          output: [1, 1],
          precision: 'single',
          optimizeFloatMemory: true,
          pipeline: true,
          immutable: false,
        },
      );
      this.texture = this.owner() as PackedTexture;
      if (!(this.texture instanceof Texture)) {
        this.reset();
        throw new Error('Terrain requires a GPU float texture');
      }
    }

    const texture = this.texture;
    const gl = texture.context;
    const pixelCount = Math.ceil(data.length / 2);
    const textureWidth = Math.ceil(Math.sqrt(pixelCount));
    const textureHeight = Math.ceil(pixelCount / textureWidth);
    if (Math.max(textureWidth, textureHeight) > gl.getParameter(gl.MAX_TEXTURE_SIZE)) {
      throw new Error('Terrain texture exceeds the GPU texture size limit');
    }

    const previousBinding = gl.getParameter(gl.TEXTURE_BINDING_2D);
    const previousFlip = gl.getParameter(gl.UNPACK_FLIP_Y_WEBGL);
    try {
      gl.bindTexture(gl.TEXTURE_2D, texture.texture);
      gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
      if (this.storageSize[0] !== textureWidth || this.storageSize[1] !== textureHeight) {
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, textureWidth, textureHeight, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
      }

      // Two signed elevations per RGBA8 texel, low byte first. On the little-endian
      // hosts this runs on, the samples already have that byte layout: upload
      // views of them in ~1 MiB steps without copying.
      const rowBytes = textureWidth * 4;
      const chunkRows = Math.max(1, Math.floor((1024 * 1024) / rowBytes));
      const uploadRows = (row: number, rows: number, pixels: Uint8Array) =>
        gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, row, textureWidth, rows, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
      const bytes = new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
      const fullRows = Math.floor(bytes.length / rowBytes);
      for (let row = 0; row < fullRows; row += chunkRows) {
        const rows = Math.min(chunkRows, fullRows - row);
        uploadRows(row, rows, bytes.subarray(row * rowBytes, (row + rows) * rowBytes));
      }
      // Only a partial last row needs zero padding.
      if (fullRows < textureHeight) {
        const lastRow = new Uint8Array(rowBytes);
        lastRow.set(bytes.subarray(fullRows * rowBytes));
        uploadRows(fullRows, 1, lastRow);
      }
      this.storageSize = [textureWidth, textureHeight];
      texture.size = [textureWidth, textureHeight];
      // GPU.js exposes normalized byte channels as scalar samples. Only the
      // terrain sampling helper decodes these back into signed elevations.
      texture.dimensions = [width * 2, height, 1];
      texture.output = [width * 2, height];
      return texture;
    } finally {
      gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, previousFlip);
      gl.bindTexture(gl.TEXTURE_2D, previousBinding);
    }
  }

  public reset(): void {
    if (this.owner) this.owner.destroy();
    this.owner = null;
    this.texture = null;
    this.storageSize = [0, 0];
  }
}

/**
 * Proves the packed layout on this GPU and GPU.js build: uploads known elevations
 * and reads them back through a kernel that indexes and decodes them the way the
 * terrain kernels do. Throws on the first difference.
 */
export function verifyPackedTerrain(gpu: GPU): void {
  const texture = new TerrainTexture(gpu);
  let reader = gpu
    .createKernel(readPackedTerrainElevation, {
      dynamicArguments: true,
      dynamicOutput: true,
      precision: 'single',
      optimizeFloatMemory: true,
    })
    .setFunctions([decodeTerrainElevation]);
  try {
    // The second upload re-specifies an existing texture, as every map rebuild does.
    // Odd sample counts leave a half-filled texel and a partial last texture row.
    for (const [width, height] of [
      [37, 21],
      [251, 127],
    ]) {
      const expected = new Int16Array(width * height);
      // 7919 is coprime to 65536: every sample differs, so misplaced reads show up.
      for (let i = 0; i < expected.length; i++) expected[i] = ((i * 7919) % 65536) - 32768;
      expected.set([InvalidElevation, UnknownElevation, WaterElevation, 0, -32768]);

      reader = reader.setOutput([width, height]);
      const rows = reader(texture.upload(expected, width, height)) as Float32Array[];
      for (let i = 0; i < expected.length; i++) {
        const actual = rows[Math.floor(i / width)][i % width];
        if (actual !== expected[i]) {
          throw new Error(`sample ${i} of ${width}x${height} read back as ${actual} instead of ${expected[i]}`);
        }
      }
    }
  } finally {
    reader.destroy();
    texture.reset();
  }
}
