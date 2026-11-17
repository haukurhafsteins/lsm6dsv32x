"""Compare actual old/new SPI transactions and exact derived decode values."""
import subprocess
import sys
import difflib

def trace(binary: str) -> list[str]:
    output = subprocess.check_output([binary], text=True)
    values = [line for line in output.splitlines() if line.startswith(('VALUE ', 'TRACE '))]
    assert len(values) > 100, 'Missing baseline coverage'
    return values

old, new = map(trace, sys.argv[1:])
if old != new:
    print('\n'.join(difflib.unified_diff(old, new, fromfile='5990d5b', tofile='current')))
    sys.exit(1)
print(f'Healthy 80X parity: {len(old)} exact transactions/derived values')
