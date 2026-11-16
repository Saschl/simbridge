import { EventEmitter } from 'events';
import { PassThrough } from 'stream';
import { spawn } from 'child_process';
import { gpuMemoryCounterScript, startGpuMemoryLogging } from './gpumemory';

jest.mock('child_process', () => ({ spawn: jest.fn() }));

describe('optional GPU memory logging', () => {
  const originalSetting = process.env.SIMBRIDGE_GPU_MEMORY_DEBUG;
  let child: EventEmitter & { stdout: PassThrough; stderr: PassThrough; kill: jest.Mock };
  let logger: { info: jest.Mock; warn: jest.Mock; error: jest.Mock };
  let stop: () => void;

  beforeEach(() => {
    jest.useFakeTimers();
    jest.setSystemTime(0);
    process.env.SIMBRIDGE_GPU_MEMORY_DEBUG = '1';
    child = Object.assign(new EventEmitter(), {
      stdout: new PassThrough(),
      stderr: new PassThrough(),
      kill: jest.fn(),
    });
    (spawn as unknown as jest.Mock).mockReturnValue(child);
    logger = { info: jest.fn(), warn: jest.fn(), error: jest.fn() };
  });
  afterEach(() => {
    stop?.();
    stop = undefined;
    child.stdout.destroy();
    child.stderr.destroy();
    jest.useRealTimers();
    jest.clearAllMocks();
    if (originalSetting === undefined) delete process.env.SIMBRIDGE_GPU_MEMORY_DEBUG;
    else process.env.SIMBRIDGE_GPU_MEMORY_DEBUG = originalSetting;
  });

  const readTerrain = () => ({
    coarseBytes: 40 * 1024 ** 2,
    detailBytes: 10 * 1024 ** 2,
    cpuCacheBytes: 160 * 1024 ** 2,
  });
  const sample = (dedicated: number, shared: number) =>
    JSON.stringify({ available: true, dedicated: dedicated * 1024 ** 2, shared: shared * 1024 ** 2 }) + '\n';

  it('does nothing unless explicitly enabled', () => {
    delete process.env.SIMBRIDGE_GPU_MEMORY_DEBUG;
    stop = startGpuMemoryLogging(logger, readTerrain);
    expect(spawn).not.toHaveBeenCalled();
    expect(logger.info).not.toHaveBeenCalled();
    expect(jest.getTimerCount()).toBe(0);
  });

  (process.platform === 'win32' ? it : it.skip)(
    'logs smoothed counters separately from allocation estimates and cleans up',
    () => {
      stop = startGpuMemoryLogging(logger, readTerrain);
      expect(spawn).toHaveBeenCalledWith(
        'powershell.exe',
        expect.any(Array),
        expect.objectContaining({ windowsHide: true }),
      );
      child.stdout.write(sample(100, 10));
      jest.advanceTimersByTime(5000);
      child.stdout.write(sample(200, 30));
      jest.advanceTimersByTime(5000);
      const report = logger.info.mock.calls[logger.info.mock.calls.length - 1][0];
      expect(report).toContain('estimated allocation)=50.0 MiB');
      expect(report).toContain('CPU terrain cache=160.0 MiB');
      expect(report).toContain('dedicated=200.0 MiB (avg30s=150.0, sampled peak=200.0)');
      expect(report).toContain('shared=30.0 MiB (avg30s=20.0, sampled peak=30.0)');
      jest.advanceTimersByTime(30000);
      child.stdout.write(sample(50, 5));
      jest.advanceTimersByTime(10000);
      expect(logger.info.mock.calls[logger.info.mock.calls.length - 1][0]).toContain('avg30s=50.0, sampled peak=200.0');
      stop();
      stop();
      expect(child.kill).toHaveBeenCalledTimes(1);
      expect(jest.getTimerCount()).toBe(0);
    },
  );

  (process.platform === 'win32' ? it : it.skip)('reports counter failures without dropping the allocation log', () => {
    stop = startGpuMemoryLogging(logger, readTerrain);
    child.emit('error', new Error('access denied'));
    child.emit('close', 1);
    jest.advanceTimersByTime(10000);
    expect(logger.warn).toHaveBeenCalledTimes(1);
    expect(logger.warn.mock.calls[0][0]).toContain('access denied');
    expect(logger.info.mock.calls[logger.info.mock.calls.length - 1][0]).toContain('Windows counters unavailable');
  });

  it('restricts the counter query to a validated process ID', () => {
    expect(() => gpuMemoryCounterScript(-1)).toThrow('Invalid GPU counter process ID');
    expect(gpuMemoryCounterScript(1234)).toContain("StartsWith('pid_1234_')");
  });
});
