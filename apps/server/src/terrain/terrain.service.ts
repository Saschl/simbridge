import { Injectable, Logger, OnApplicationShutdown } from '@nestjs/common';
import { ChildProcess, spawn } from 'child_process';
import * as fs from 'fs';
import * as path from 'path';
import { getExecutablePath } from 'apps/server/src/utilities/pathUtil';
import { NavigationDisplayThresholdsDto } from './dto/navigationdisplaythresholds.dto';
import { DisplaySide } from './types';
import { ElevationSamplePathDto } from './dto/elevationsamplepath.dto';
import { TawsAircraftStatusDataDto } from 'apps/server/src/terrain/dto/tawsaircraftstatusdata.dto';

const RestartBackoffMs = [1000, 2000, 5000, 30000];
const HealthyUptimeResetMs = 60_000;
const ShutdownKillTimeoutMs = 2000;

/**
 * Supervises the Rust terrain sidecar (terrain-service.exe) and proxies the
 * terrain HTTP API to it. The sidecar owns SimConnect, the tile cache and all
 * rendering; its local API mirrors the public one, so the controller stays
 * unchanged.
 */
@Injectable()
export class TerrainService implements OnApplicationShutdown {
  private readonly logger = new Logger(TerrainService.name);

  private child: ChildProcess = null;

  private baseUrl: string = null;

  private shuttingDown = false;

  private restartAttempt = 0;

  private startedAt = 0;

  constructor() {
    this.spawnSidecar();
  }

  private locateSidecarExecutable(): string | undefined {
    const candidates = [
      process.env.SIMBRIDGE_TERRAIN_EXE,
      path.join(getExecutablePath(), 'terrain-service.exe'),
      path.join(process.cwd(), 'terrain-rust', 'target', 'release', 'terrain-service.exe'),
      //path.join(process.cwd(), 'terrain-rust', 'target', 'debug', 'terrain-service.exe'),
    ].filter((candidate) => candidate !== undefined);

    return candidates.find((candidate) => fs.existsSync(candidate));
  }

  private spawnSidecar(): void {
    const executable = this.locateSidecarExecutable();
    if (executable === undefined) {
      this.logger.warn('Terrain sidecar (terrain-service.exe) not found - terrain rendering unavailable');
      return;
    }

    const terrainDb = path.join(getExecutablePath(), 'terrain', 'terrain.map');
    this.startedAt = Date.now();
    this.child = spawn(executable, ['--terrain-db', terrainDb], {
      stdio: ['pipe', 'pipe', 'pipe'],
      windowsHide: true,
    });
    this.logger.log(`Started terrain sidecar: ${executable} (pid ${this.child.pid})`);

    let stdoutBuffer = '';
    this.child.stdout.on('data', (chunk: Buffer) => {
      stdoutBuffer += chunk.toString();
      let newlineIndex = stdoutBuffer.indexOf('\n');
      while (newlineIndex !== -1) {
        const line = stdoutBuffer.slice(0, newlineIndex).trim();
        stdoutBuffer = stdoutBuffer.slice(newlineIndex + 1);

        const ready = line.match(/^SIMBRIDGE-TERRAIN READY port=(\d+) pid=\d+$/);
        if (ready) {
          this.baseUrl = `http://127.0.0.1:${ready[1]}`;
          this.logger.log(`Terrain sidecar ready on ${this.baseUrl}`);
        } else if (line.length > 0) {
          this.logger.log(`[terrain] ${line}`);
        }
        newlineIndex = stdoutBuffer.indexOf('\n');
      }
    });
    this.child.stderr.on('data', (chunk: Buffer) => {
      chunk
        .toString()
        .split('\n')
        .map((line) => line.trim())
        .filter((line) => line.length > 0)
        .forEach((line) => this.logger.log(`[terrain] ${line}`));
    });

    this.child.on('exit', (code) => {
      this.baseUrl = null;
      this.child = null;
      if (this.shuttingDown) return;

      if (Date.now() - this.startedAt > HealthyUptimeResetMs) this.restartAttempt = 0;
      const backoff = RestartBackoffMs[Math.min(this.restartAttempt, RestartBackoffMs.length - 1)];
      this.restartAttempt += 1;
      this.logger.warn(`Terrain sidecar exited with code ${code} - restarting in ${backoff} ms`);
      setTimeout(() => {
        if (!this.shuttingDown) this.spawnSidecar();
      }, backoff);
    });
  }

  async onApplicationShutdown(_signal?: string) {
    this.logger.log(`Destroying ${TerrainService.name}`);
    this.shuttingDown = true;
    if (this.child === null) return;

    const child = this.child;
    try {
      if (this.baseUrl !== null) {
        await fetch(`${this.baseUrl}/shutdown`, { method: 'POST' });
      } else {
        child.stdin.end();
      }
    } catch (_) {
      // fall through to the hard kill
    }

    setTimeout(() => {
      if (child.exitCode === null) child.kill();
    }, ShutdownKillTimeoutMs);
  }

  private async proxyGet<T>(route: string): Promise<T | undefined> {
    if (this.baseUrl === null) return undefined;
    try {
      const response = await fetch(`${this.baseUrl}${route}`);
      if (!response.ok) return undefined;
      const body = await response.text();
      if (body.length === 0) return undefined;
      return JSON.parse(body) as T;
    } catch (_) {
      return undefined;
    }
  }

  private async proxyPost(route: string, body: unknown): Promise<void> {
    if (this.baseUrl === null) return;
    try {
      const response = await fetch(`${this.baseUrl}${route}`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(body),
      });
      if (!response.ok) {
        this.logger.warn(`Terrain sidecar POST ${route} rejected (${response.status}): ${await response.text()}`);
      }
    } catch (error) {
      this.logger.warn(`Terrain sidecar POST ${route} failed: ${error}`);
    }
  }

  public async frameData(
    display: DisplaySide,
  ): Promise<{ timestamp: number; frames: Uint8ClampedArray[]; thresholds: NavigationDisplayThresholdsDto }> {
    if (this.baseUrl === null) return undefined;

    const [timestamp, thresholds, frames] = await Promise.all([
      this.proxyGet<number>(`/api/v1/terrain/renderingTimestamp?display=${display}`),
      this.proxyGet<NavigationDisplayThresholdsDto>(`/api/v1/terrain/renderingThresholds?display=${display}`),
      this.proxyGet<string[]>(`/api/v1/terrain/renderingFrames?display=${display}`),
    ]);
    if (timestamp === undefined) return undefined;

    return {
      timestamp,
      thresholds,
      frames: (frames ?? []).map((frame) => new Uint8ClampedArray(Buffer.from(frame, 'base64'))),
    };
  }

  public updateAircraftStatusData(aircraftStatusData: TawsAircraftStatusDataDto): void {
    //console.log('Updating aircraft status data:', aircraftStatusData);
    this.proxyPost('/api/v1/terrain/aircraftStatusData', aircraftStatusData);
  }

  public updateFlightPath(path: ElevationSamplePathDto): void {
    this.proxyPost('/api/v1/terrain/verticalDisplayPath', path);
  }
}
