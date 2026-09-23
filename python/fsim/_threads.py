"""torch's CPU threads beside the platform's.

A VecEnv steps its aircraft on worker threads, one per core. torch runs its
CPU ops on an OpenMP team as big as the machine, and the OpenMP torch ships
on Windows (Intel's) keeps an idle team spinning for 200 ms after every
parallel region. So after a policy's forward pass, torch's threads spin on
the cores the platform is stepping on. Measured with PPO
(examples/python/train_sb3.py, 8 cores): torch's default threads collect
rollouts at 84k steps/s and one thread at 110k; training on the whole
buffer is the opposite (1.6 s with 8 threads, 3.1 s with 1). One thread
while stepping, all of them while training, and a spin of 1 ms so the team
goes to sleep between the two: 17% more steps/s end to end
(docs/sdk/python.md, "torch's threads").
"""
import contextlib
import ctypes
import sys


def intel_openmp():
    """The Intel OpenMP runtime if the process has loaded it (torch does, on
    Windows), else None. Nothing is loaded here."""
    if sys.platform != "win32":
        return None
    kernel32 = ctypes.WinDLL("kernel32")
    kernel32.GetModuleHandleW.restype = ctypes.c_void_p
    kernel32.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
    handle = kernel32.GetModuleHandleW("libiomp5md.dll")
    return ctypes.CDLL("libiomp5md.dll", handle=handle) if handle else None


def short_openmp_spin(ms=1):
    """Idle OpenMP threads sleep after ``ms`` instead of 200 ms, for the
    parallel regions the calling thread starts. The KMP_BLOCKTIME environment
    variable does the same for every thread, if it is set before torch is
    imported. Returns whether there was an Intel OpenMP to set it in."""
    omp = intel_openmp()
    if omp is None:
        return False
    omp.kmp_set_blocktime(ctypes.c_int(ms))
    return True


@contextlib.contextmanager
def torch_threads(threads=1):
    """torch's CPU threads inside the block, restored after it. The OpenMP
    spin is shortened to 1 ms first (and stays so: training measured no
    slower), so that threads busy before the block do not spin on into it.

        for update in range(updates):
            with fsim.torch_threads(1):   # collecting: the platform needs the cores
                for t in range(n_steps):
                    obs, reward, terminated, truncated = env.step(policy(obs))
            train(buffer)                 # training: torch's own thread count
    """
    import torch

    short_openmp_spin()
    before = torch.get_num_threads()
    torch.set_num_threads(threads)
    try:
        yield
    finally:
        torch.set_num_threads(before)
