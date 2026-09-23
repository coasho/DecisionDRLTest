"""Arrays from whatever the caller has: numpy, sequences, and tensors on any
device. Only reached when the native call has refused its argument as it
is - a C-contiguous array of the right dtype never comes through here."""
import numpy as np


def as_array(values, dtype):
    """``values`` as a C-contiguous numpy array of ``dtype``.

    A torch tensor - on the GPU, carrying a gradient, in half precision - is
    brought to the CPU first. The platform simulates on the CPU, so that copy
    is made whatever the API looks like; making it here only spares the
    caller the ``.detach().cpu()``. torch is recognised by its methods, not
    imported: it stays an optional dependency.
    """
    if hasattr(values, "detach") and hasattr(values, "cpu") and hasattr(values, "numpy"):
        t = values.detach()
        t = t.double() if dtype == np.float64 else t.float()  # also half, bfloat16 (numpy has none)
        values = t.cpu().numpy()
    return np.ascontiguousarray(values, dtype=dtype)
