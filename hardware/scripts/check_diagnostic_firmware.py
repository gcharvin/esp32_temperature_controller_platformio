"""Compile the standalone diagnostic project, record artifacts; never upload.

Standard Python. Does not erase, read, reset or flash any connected ESP32.
"""
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parent


def main():
    folder=REPO/'firmware/diagnostic'
    pio=Path.home()/'.platformio/penv/Scripts/platformio.exe'
    env=dict(os.environ,PYTHONUTF8='1',PYTHONIOENCODING='utf-8')
    run=subprocess.run([str(pio),'run','--project-dir',str(folder),'-e','esp32-bench'],capture_output=True,text=True,encoding='utf-8',errors='replace',env=env)
    if run.returncode:print(run.stdout[-4500:]);print(run.stderr[-2000:]);raise RuntimeError('Diagnostic compile failed')
    code=(folder/'main.cpp').read_text(encoding='utf-8')
    assert 'digitalWrite(bench::enablePin, HIGH)' not in code
    assert 'digitalWrite(bench::enablePin, LOW)' in code
    assert 'ledcAttach(bench::pwmPin, 500, 8)' in code
    policy=(folder/'BenchPolicy.h').read_text(encoding='utf-8')
    module=ROOT/'module_selection.json'
    devkit=json.loads(module.read_text(encoding='utf-8'))['devkit']
    for key,net in [('pwmPin','PWM0'),('enablePin','ENABLE'),('faultPin','FAULT_N'),
                    ('encoderA','ENC_A'),('encoderB','ENC_B'),('encoderButton','ENC_SW')]:
        number=int(re.search(r'\b'+key+r'\s*=\s*(\d+)',policy).group(1))
        pin=next(pin for pin,n in devkit['used_pin_nets'].items() if n==net)
        assert devkit['pin_labels'][pin]=='GPIO'+str(number),(key,net,number)
    release=folder/'release';release.mkdir(exist_ok=True)
    build=folder/'.pio/build/esp32-bench'
    for name in ('firmware.bin','bootloader.bin','partitions.bin'):shutil.copy2(build/name,release/name)
    files=[folder/n for n in ('platformio.ini','BenchPolicy.h','main.cpp','README.md')]+[module]
    outputs=[release/n for n in ('firmware.bin','bootloader.bin','partitions.bin')]
    report=dict(status='PASS / PlatformIO compilation only, not a hardware test',environment='esp32-bench',
        project='firmware/diagnostic',platform='espressif32@7.1.3',hardware_enable_permanently_low=True,
        logic_probe_frequency_Hz=500,logic_probe_max_percent=20,logic_probe_duration_ms=2000,
        flashed=False,hardware_tests_performed=False,source_sha256={str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        binaries_sha256={str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest() for p in outputs},
        compile_summary=[line.strip() for line in run.stdout.splitlines() if any(t in line for t in ('RAM:','Flash:','SUCCESS'))])
    (ROOT/'review/diagnostic_firmware_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
