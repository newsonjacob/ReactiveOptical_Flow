import re

def test_loop_continues_processing_after_dodge():
    with open('main.py', 'r') as f:
        lines = f.readlines()
    for i, line in enumerate(lines):
        if 'navigator.settling' in line:
            snippet = ''.join(lines[i:i+8])
            assert 'continue' not in snippet
            break
    else:
        raise AssertionError('settling logic not found')
