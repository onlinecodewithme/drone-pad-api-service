# BLE Scanner

Standalone CLI tool to scan for and list nearby BLE devices (address, RSSI, name).

## Usage

```bash
# Use the repo's existing venv, which already has bleak installed
source ../venv/bin/activate

# Single 5s scan (default)
python scan.py

# Custom duration
python scan.py -d 10

# Continuous scan, refreshing every 2s until Ctrl+C
python scan.py -l -d 2
```

## Standalone install

```bash
pip install -r requirements.txt
```
