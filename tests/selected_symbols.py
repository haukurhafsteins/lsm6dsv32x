"""A product test executable must contain exactly the selected register layer."""
import subprocess
import sys
for part, binary in zip(('80', '32'), sys.argv[1:]):
    symbols = subprocess.check_output(['nm', '--defined-only', binary], text=True)
    other = '32' if part == '80' else '80'
    assert f'lsm6dsv{part}x_' in symbols, f'Missing {part}X implementation'
    assert f'lsm6dsv{other}x_' not in symbols, f'Unexpected {other}X implementation'
print('Exactly one selected ST layer in each part executable')
