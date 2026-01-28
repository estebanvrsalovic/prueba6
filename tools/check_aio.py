#!/usr/bin/env python3
import re
import sys
from pathlib import Path

cred_file = Path(__file__).resolve().parents[1] / 'include' / 'credentials.h'
if not cred_file.exists():
    print('credentials.h not found at', cred_file)
    sys.exit(2)
text = cred_file.read_text()
user_m = re.search(r'#\s*define\s+AIO_USERNAME\s+"([^"]+)"', text)
key_m = re.search(r'#\s*define\s+AIO_KEY\s+"([^"]+)"', text)
if not user_m or not key_m:
    print('AIO_USERNAME or AIO_KEY not found in', cred_file)
    sys.exit(2)
username = user_m.group(1)
aio_key = key_m.group(1)

import requests
url = f'https://io.adafruit.com/api/v2/{username}/feeds'
headers = {'X-AIO-Key': aio_key}
print('Pinging Adafruit IO REST API for user', username)
try:
    r = requests.get(url, headers=headers, timeout=10)
    print('HTTP', r.status_code)
    if r.status_code == 200:
        feeds = r.json()
        print('Success — found', len(feeds), 'feeds (showing up to 5):')
        for f in feeds[:5]:
            name = f.get('key') or f.get('name') or str(f)
            print('-', name)
        sys.exit(0)
    else:
        print('Error response:')
        print(r.text[:1000])
        sys.exit(3)
except Exception as e:
    print('Request failed:', e)
    sys.exit(4)
