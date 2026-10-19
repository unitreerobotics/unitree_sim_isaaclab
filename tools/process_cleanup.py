"""Stop only multiprocessing children owned by the current simulator process."""

import multiprocessing


def cleanup_child_processes(timeout=2.0):
    """Reap owned children without searching or signalling unrelated processes."""
    children = multiprocessing.active_children()
    for child in children:
        child.join(timeout=timeout)
    for child in children:
        if child.is_alive():
            child.terminate()
    for child in children:
        child.join(timeout=timeout)
        if child.is_alive():
            child.kill()
            child.join(timeout=timeout)
