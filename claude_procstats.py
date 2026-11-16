#!/usr/bin/env python3
"""Samples how much CPU, memory and GPU a Claude Code process and everything it started
are using, for claude_statusline.sh to show.

Usage: claude_procstats.py <Claude Code's pid> <output file>

Writes one line to the output file:
    <when sampled> <CPU %> <CPUs> <memory> <total memory> <GPU %> <GPU memory>
with the time in seconds since the epoch, memory in KiB (GPU memory in MiB), CPU % as in
`top` (so up to 100 times the number of CPUs), and the GPU fields `-` if none of the
processes has a GPU open. CPU use is averaged since the last sample, from each process's
CPU time saved next to the output (as <output file>.state), so the first sample's is `-`.
Does nothing if the last sample was under MIN_GAP seconds ago, or another is underway.

The processes counted are Claude Code and its descendants, plus any orphaned (e.g. a job
run as `nohup <job> &` from a shell that then exited) that still have its pid as
CLAUDE_PID in their environment, as Claude Code sets for the commands it runs, and their
descendants. CPU use includes that of processes which came and went between samples (e.g.
each compiler `make` runs), and memory is the proportional set size, which splits shared
pages between the processes sharing them, so e.g. forked workers aren't counted many times
over. GPU use comes from `nvidia-smi pmon`, which is only run if one of the processes has
an NVIDIA device open.
"""

# (json, re, pathlib and subprocess are left out, or only imported when needed, as each
# takes longer to import than the sampling does)
import fcntl
import os
import sys
import time

MIN_GAP = 1.0
TICKS = os.sysconf('SC_CLK_TCK')


def read_processes():
    """Reads the parent, start time, CPU time and name of every process.

    Returns:
        A dict of pid to (parent pid, start time, CPU time, name), with times in clock
        ticks (the start since boot), and CPU time including that of any children the
        process has waited for (so of those that have exited).
    """
    procs = {}
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/stat', 'rb') as f:
                stat = f.read()
        except OSError:
            continue
        # (the name is in parentheses, and may itself contain spaces or parentheses)
        end = stat.rindex(b')')
        # (fields[0] is field 3 in proc_pid_stat(5): the state)
        fields = stat[end + 2:].split()
        procs[int(name)] = (int(fields[1]), int(fields[19]),
            sum(int(x) for x in fields[11:15]), stat[stat.index(b'(') + 1:end])
    return procs


def select(procs, root, skip):
    """Picks out the processes to count.

    Args:
        procs: as read_processes() returns.
        root: Claude Code's pid.
        skip: pids to leave out, along with their descendants.

    Returns:
        The set of pids to count.
    """
    children = {}
    for pid, (ppid, *_) in procs.items():
        children.setdefault(ppid, []).append(pid)

    chosen = set()

    def add_tree(pid):
        stack = [pid]
        while stack:
            pid = stack.pop()
            if pid not in skip and pid not in chosen:
                chosen.add(pid)
                stack.extend(children.get(pid, ()))

    add_tree(root)
    # (orphans are adopted by init, or by the nearest "subreaper", e.g. a user's systemd)
    marker = b'\0CLAUDE_PID=%d\0' % root
    for reaper in [1] + [pid for pid, p in procs.items() if p[3] == b'systemd']:
        for pid in children.get(reaper, ()):
            if pid in chosen or pid in skip:
                continue
            try:
                with open(f'/proc/{pid}/environ', 'rb') as f:
                    env = f.read()
            except OSError:
                continue
            if marker in b'\0' + env:
                add_tree(pid)
    return chosen


def pss(pid):
    """Returns the proportional set size of a process, in KiB (0 if unreadable)."""
    try:
        with open(f'/proc/{pid}/smaps_rollup', 'rb') as f:
            for line in f:
                if line.startswith(b'Pss:'):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


def has_gpu_open(pid):
    """Returns whether a process has any NVIDIA device file open."""
    fd_dir = f'/proc/{pid}/fd'
    try:
        fds = os.listdir(fd_dir)
    except OSError:
        return False
    for fd in fds:
        try:
            if os.readlink(f'{fd_dir}/{fd}').startswith('/dev/nvidia'):
                return True
        except OSError:
            pass
    return False


def gpu_use(pids):
    """Sums the GPU use of some processes, across all GPUs.

    Returns:
        (percent of SMs used, MiB of GPU memory used), or None if nvidia-smi failed.
    """
    import subprocess
    try:
        out = subprocess.run(['nvidia-smi', 'pmon', '-c', '1', '-s', 'um'],
            capture_output=True, text=True, timeout=10, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    lines = out.splitlines()
    if not lines or not lines[0].startswith('#'):
        return None
    # (the first line names the columns, which vary by driver version)
    columns = lines[0][1:].split()
    try:
        pid_i, sm_i, fb_i = (columns.index(c) for c in ('pid', 'sm', 'fb'))
    except ValueError:
        return None
    sm = fb = 0
    for line in lines:
        parts = line.split()
        if line.startswith('#') or len(parts) <= max(pid_i, sm_i, fb_i):
            continue
        if parts[pid_i].isdigit() and int(parts[pid_i]) in pids:
            sm += int(parts[sm_i]) if parts[sm_i].isdigit() else 0
            fb += int(parts[fb_i]) if parts[fb_i].isdigit() else 0
    return sm, fb


def cpu_percent(cur, prev, now):
    """Computes the CPU use since the last sample.

    Args:
        cur: dict of each counted process's key ("<pid>:<start time>") to [its parent's
            key (None if its parent isn't counted), its CPU time].
        prev: the last sample's state: {'uptime': when (seconds since boot), 'procs': as
            cur, then}.
        now: seconds since boot.

    Returns:
        The percent of a CPU used on average, as in `top`.
    """
    old = prev['procs']
    since = prev['uptime']
    ticks = 0
    for key, (_, t) in cur.items():
        if key in old:
            ticks += t - old[key][1]
        # (all of a process's CPU time is new, if it started since)
        elif int(key.split(':')[1]) >= since * TICKS:
            ticks += t
    # A process that's exited moves all its CPU time (incl. its children's) into that of
    # whichever of its ancestors waited for it, so what's been counted of it already is
    # taken back out. Without such an ancestor still counted (e.g. it was an orphan), its
    # last bit of CPU time is just missed.
    for key, (parent, t) in old.items():
        if key in cur:
            continue
        while parent is not None and parent not in cur:
            parent = old.get(parent, [None])[0]
        if parent is not None:
            ticks -= t
    return max(0, ticks) / TICKS / (now - since) * 100


def read_state(path):
    """Reads the last sample's state, as write_state() saved it.

    Returns:
        (Claude Code's key, as for the processes in cpu_percent(); cpu_percent()'s prev), or
        None if there's no state.
    """
    try:
        with open(path) as f:
            root, uptime = f.readline().split()
            procs = {}
            for line in f:
                key, parent, t = line.split()
                procs[key] = [None if parent == '-' else parent, int(t)]
    except (OSError, ValueError):
        return None
    return root, {'uptime': float(uptime), 'procs': procs}


def write_state(path, root, uptime, procs):
    """Saves a sample's state, for the next one's read_state(), as a line with
    Claude Code's key and the uptime, then one per process with its key, its parent's (or
    `-`) and its CPU time.
    """
    lines = [f'{root} {uptime}\n']
    lines += [f'{k} {p or "-"} {t}\n' for k, (p, t) in procs.items()]
    replace(path, ''.join(lines))


def replace(path, text):
    """Writes a file, atomically replacing any already there."""
    with open(path + '.tmp', 'w') as f:
        f.write(text)
    os.replace(path + '.tmp', path)


def main():
    root = int(sys.argv[1])
    out = sys.argv[2]
    state_path = out + '.state'

    lock = open(out + '.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        return

    # (files for Claude Code processes that have since exited)
    out_dir = os.path.dirname(out) or '.'
    for name in os.listdir(out_dir):
        pid = name.removeprefix('procs-').split('.')[0]
        if name.startswith('procs-') and pid.isdigit() \
                and not os.path.exists(f'/proc/{pid}'):
            try:
                os.unlink(os.path.join(out_dir, name))
            except OSError:
                pass

    with open('/proc/uptime') as f:
        now = float(f.read().split()[0])
    procs = read_processes()
    if root not in procs:
        return
    root_key = f'{root}:{procs[root][1]}'
    prev = None
    state = read_state(state_path)
    if state and state[0] == root_key:
        prev = state[1]
        if now - prev['uptime'] < MIN_GAP:
            return

    chosen = select(procs, root, {os.getpid()})
    keys = {pid: f'{pid}:{procs[pid][1]}' for pid in chosen}
    cur = {keys[pid]: [keys.get(procs[pid][0]), procs[pid][2]] for pid in chosen}
    cpu = f'{cpu_percent(cur, prev, now):.0f}' if prev else '-'

    mem = sum(pss(pid) for pid in chosen)
    mem_total = 0
    with open('/proc/meminfo') as f:
        for line in f:
            if line.startswith('MemTotal:'):
                mem_total = int(line.split()[1])
                break

    gpu = None
    if any(has_gpu_open(pid) for pid in chosen):
        gpu = gpu_use(chosen)
    gpu_pct, gpu_mem = gpu if gpu else ('-', '-')

    write_state(state_path, root_key, now, cur)
    replace(out, f'{int(time.time())} {cpu} {os.cpu_count()} {mem} {mem_total} '
        f'{gpu_pct} {gpu_mem}\n')


if __name__ == '__main__':
    main()
