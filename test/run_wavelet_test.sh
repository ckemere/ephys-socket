#!/usr/bin/env bash
# Build + run the standalone WAVELET parser test and diff the plugin's C++
# decode + multirate-surface reassembly against the net.py-style Python
# reference decode of the SAME synthetic per-octave packets. Exit non-zero on
# any mismatch.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
cd "$here"

echo "== building C++ wavelet harness =="
g++ -std=c++17 -O2 -Wall -Wextra wavelet_parse_test.cpp -o wavelet_parse_test

echo "== C++ (plugin wavelet decode + reassembly) =="
./wavelet_parse_test | tee cpp_wav_out.txt

echo "== Python (net.py reference wavelet decode) =="
python3 ref_decode_wavelet.py | tee py_wav_out.txt

echo "== diff =="
if diff -u py_wav_out.txt cpp_wav_out.txt; then
    echo "PASS: wavelet decode + reassembly matches net.py reference byte-for-byte"
else
    echo "FAIL: mismatch between plugin wavelet decode and net.py reference"
    exit 1
fi
