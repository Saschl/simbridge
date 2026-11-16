import { spawn } from 'child_process';
import { createInterface } from 'readline';
import { Logger } from './logger';

interface TerrainMemoryUsage {
  coarseBytes: number;
  detailBytes: number;
  cpuCacheBytes: number;
}

interface ProcessMemorySample {
  time: number;
  dedicated: number;
  shared: number;
}

const mib = (bytes: number) => (bytes / (1024 * 1024)).toFixed(1);

// WMI class/property names are independent of the Windows display language.
// One persistent, hidden helper avoids launching PowerShell on every sample.
export function gpuMemoryCounterScript(pid: number): string {
  if (!Number.isSafeInteger(pid) || pid <= 0) throw new Error('Invalid GPU counter process ID');
  return `
$ErrorActionPreference = 'Stop'
while (Get-Process -Id ${pid} -ErrorAction SilentlyContinue) {
  $rows = @(Get-CimInstance -ClassName Win32_PerfFormattedData_GPUPerformanceCounters_GPUProcessMemory -Filter "Name LIKE 'pid_${pid}_%'" |
    Where-Object { $_.Name.StartsWith('pid_${pid}_') })
  $dedicated = 0.0
  $shared = 0.0
  foreach ($row in $rows) {
    $dedicated += [double]$row.DedicatedUsage
    $shared += [double]$row.SharedUsage
  }
  [Console]::Out.WriteLine((@{ available = ($rows.Count -gt 0); dedicated = $dedicated; shared = $shared } | ConvertTo-Json -Compress))
  Start-Sleep -Seconds 2
}
`;
}

/** Diagnostic only: logical terrain storage and separately labelled OS process counters. */
export function startGpuMemoryLogging(logger: Logger, readTerrain: () => TerrainMemoryUsage): () => void {
  if (process.env.SIMBRIDGE_GPU_MEMORY_DEBUG !== '1') return () => undefined;

  let samples: ProcessMemorySample[] = [];
  let dedicatedPeak = 0;
  let sharedPeak = 0;
  let terrainPeak = 0;
  let status = process.platform === 'win32' ? 'waiting for Windows counters' : 'Windows counters unavailable';
  let stopped = false;
  let failed = false;
  let errorText = '';

  const child =
    process.platform === 'win32'
      ? spawn('powershell.exe', ['-NoProfile', '-NonInteractive', '-Command', gpuMemoryCounterScript(process.pid)], {
          windowsHide: true,
          stdio: ['ignore', 'pipe', 'pipe'],
        })
      : null;
  const lines = child ? createInterface({ input: child.stdout }) : null;
  const fail = (reason: string) => {
    if (stopped || failed) return;
    failed = true;
    samples = [];
    status = 'Windows counters unavailable';
    logger.warn(`[Terrain memory] Windows GPU counters unavailable: ${reason}. Terrain allocation logging continues.`);
  };
  lines?.on('line', (line) => {
    if (stopped || failed) return;
    try {
      const sample = JSON.parse(line);
      if (sample.available === false) {
        samples = [];
        status = 'no Windows GPU-memory instance for this PID';
        return;
      }
      if (
        sample.available !== true ||
        !Number.isFinite(sample.dedicated) ||
        !Number.isFinite(sample.shared) ||
        sample.dedicated < 0 ||
        sample.shared < 0
      )
        throw new Error('Invalid counter sample');
      const now = Date.now();
      samples = samples.filter((entry) => now - entry.time <= 30000);
      samples.push({ time: now, dedicated: sample.dedicated, shared: sample.shared });
      status = 'Windows counters stale';
      dedicatedPeak = Math.max(dedicatedPeak, sample.dedicated);
      sharedPeak = Math.max(sharedPeak, sample.shared);
    } catch (error) {
      fail(String(error));
      child.kill();
    }
  });
  child?.stderr.on('data', (data) => {
    errorText = (errorText + data.toString()).slice(-2000);
  });
  child?.on('error', (error) => fail(error.message));
  child?.on('close', (code) => fail(errorText.trim() || `counter helper exited (${code})`));

  const report = () => {
    const terrain = readTerrain();
    const total = terrain.coarseBytes + terrain.detailBytes;
    terrainPeak = Math.max(terrainPeak, total);
    const prefix =
      `[Terrain memory] PID=${process.pid}; terrain textures (estimated allocation)=${mib(total)} MiB ` +
      `(coarse=${mib(terrain.coarseBytes)}, detail=${mib(terrain.detailBytes)}, sampled peak=${mib(terrainPeak)}); ` +
      `CPU terrain cache=${mib(terrain.cpuCacheBytes)} MiB; `;
    samples = samples.filter((entry) => Date.now() - entry.time <= 30000);
    const current = samples[samples.length - 1];
    if (!current || Date.now() - current.time > 10000) {
      logger.info(prefix + (current ? 'Windows counters stale' : status));
      return;
    }
    const dedicatedAverage = samples.reduce((sum, sample) => sum + sample.dedicated, 0) / samples.length;
    const sharedAverage = samples.reduce((sum, sample) => sum + sample.shared, 0) / samples.length;
    logger.info(
      prefix +
        `Windows process GPU memory: dedicated=${mib(current.dedicated)} MiB ` +
        `(avg30s=${mib(dedicatedAverage)}, sampled peak=${mib(dedicatedPeak)}), shared=${mib(current.shared)} MiB ` +
        `(avg30s=${mib(sharedAverage)}, sampled peak=${mib(sharedPeak)}); samples30s=${samples.length}`,
    );
  };
  logger.info(
    '[Terrain memory] Debug logging enabled: reports every 10s; OS sampling approximately every 2s. ' +
      'Allocation estimates exclude display textures and driver overhead; Windows counters cover the whole process across GPUs.',
  );
  report();
  const timer = setInterval(report, 10000);
  timer.unref();
  const stop = () => {
    if (stopped) return;
    stopped = true;
    clearInterval(timer);
    lines?.close();
    child?.kill();
    process.removeListener('exit', stop);
  };
  process.once('exit', stop);
  return stop;
}
