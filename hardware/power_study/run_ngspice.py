"""Execute a SPICE deck using KiCad's installed ngspice shared library.

Python 64-bit on Windows. No KiCad GUI or extra simulator installation needed.
"""
import argparse
import ctypes as c
import os
from pathlib import Path
import re
import json
import time
import hashlib


def spice_number(value):
    """Read a SPICE scalar, including the case-insensitive engineering suffix."""
    match = re.fullmatch(r'([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?)(meg|[tgkmunpf]?)', value.lower())
    if not match:
        raise ValueError('Unsupported SPICE scalar: ' + value)
    return float(match[1]) * {'': 1, 't': 1e12, 'g': 1e9, 'meg': 1e6,
                             'k': 1e3, 'm': 1e-3, 'u': 1e-6, 'n': 1e-9,
                             'p': 1e-12, 'f': 1e-15}[match[2]]


def run(deck, logfile, vectors=None, compatibility='psa'):
    """Run one explicit transient deck; only publish data after reaching TSTOP.

    A simulator can return status zero even after a transient abort. Check the
    actual exported time column as well as the log. Keep failure metadata so a
    previous successful waveform cannot be mistaken for the current attempt.
    """
    deck = Path(deck).resolve()
    logfile = Path(logfile).resolve()
    vectors = vectors or ['v(vout)', 'v(vin)', 'v(pg)', 'i(L1)']
    source = deck.read_text(encoding='utf-8')
    if re.search(r'^\s*\.control\b', source, re.I | re.M):
        raise ValueError('Use an explicit .tran deck without .control for verified execution')
    transient = re.search(r'^\s*\.tran\s+\S+\s+(\S+)', source, re.I | re.M)
    if not transient:
        raise ValueError('One explicit .tran directive is required')
    expected_stop = spice_number(transient[1])
    datafile = deck.with_suffix('.dat')
    partial = deck.with_name(deck.stem + '.partial.dat')
    reportfile = deck.with_suffix('.run.json')
    for stale in (datafile, partial):
        stale.unlink(missing_ok=True)
    inputs = {str(deck): hashlib.sha256(deck.read_bytes()).hexdigest()}
    def trace_includes(path):
        for quoted, bare in re.findall(r'^\s*\.include\s+(?:"([^"]+)"|(\S+))',
                                       path.read_text(encoding='utf-8', errors='replace'), re.I | re.M):
            included = (path.parent / (quoted or bare)).resolve()
            if str(included) in inputs:
                continue
            inputs[str(included)] = hashlib.sha256(included.read_bytes()).hexdigest()
            trace_includes(included)
    trace_includes(deck)
    report = dict(deck=str(deck), status='running', expected_stop_s=expected_stop,
                  vectors=vectors, engine='KiCad 10 ngspice shared library',
                  compatibility=compatibility, input_sha256=inputs)
    reportfile.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    started = time.monotonic()
    bindir=Path('C:/Program Files/KiCad/10.0/bin')
    directory=os.add_dll_directory(str(bindir))
    lib=c.CDLL(str(bindir/'ngspice.dll'))
    messages=[]
    charfn=c.CFUNCTYPE(c.c_int,c.c_char_p,c.c_int,c.c_void_p)
    exitfn=c.CFUNCTYPE(c.c_int,c.c_int,c.c_bool,c.c_bool,c.c_int,c.c_void_p)
    @charfn
    def output(message,ident,user):
        text=message.decode('utf-8',errors='replace');messages.append(text)
        if any(key in text.lower() for key in ('error','warning','no. of data','fatal')):print(text,flush=True)
        return 0
    @exitfn
    def controlled_exit(status,immediate,quit_,ident,user):
        messages.append('Controlled exit '+str(status));return 0
    lib.ngSpice_Init.argtypes=[c.c_void_p]*7
    lib.ngSpice_Command.argtypes=[c.c_char_p];lib.ngSpice_Command.restype=c.c_int
    lib.ngSpice_Init(output,None,controlled_exit,None,None,None,None)
    short=c.create_unicode_buffer(1024)
    for cm in (bindir.parent/'lib/ngspice').glob('*.cm'):
        c.windll.kernel32.GetShortPathNameW(str(cm),short,1024)
        lib.ngSpice_Command(('codemodel '+short.value.replace(chr(92),'/')).encode())
    lib.ngSpice_Command(('set ngbehavior='+compatibility).encode())
    old=Path.cwd();os.chdir(deck.parent)
    try:
        result=lib.ngSpice_Command(('source '+deck.name).encode())
        lib.ngSpice_Command(('save '+' '.join(vectors)).encode())
        result=lib.ngSpice_Command(b'run')
        lib.ngSpice_Command(b'set wr_singlescale')
        lib.ngSpice_Command(b'set wr_vecnames')
        lib.ngSpice_Command(b'set numdgt=15')
        lib.ngSpice_Command(('wrdata '+partial.name+' '+' '.join(vectors)).encode())
        errors = [v for v in messages if re.search(r'^(?:stderr|stdout)\s+(?:error\b|fatal\b)', v, re.I)
                  or any(k in v.lower() for k in
                         ('simulation interrupted', 'timestep too small', 'simulation(s) aborted'))]
        report.update(command_status=result, errors=errors)
        if partial.exists():
            last = None
            rows = 0
            with partial.open() as stream:
                next(stream, None)  # column header
                for line in stream:
                    if line.strip():
                        last = line.split()
                        rows += 1
            if last:
                report.update(last_time_s=float(last[0]), rows=rows)
        if errors or result != 0:
            raise RuntimeError('ngspice aborted; see '+str(logfile))
        if not any('No. of Data Rows' in v for v in messages):
            raise RuntimeError('No completed transient reported')
        if abs(report.get('last_time_s', -1)-expected_stop) > max(1e-12, expected_stop*1e-6):
            raise RuntimeError('Transient did not reach requested stop time')
        partial.replace(datafile)
        report.update(status='complete', datafile=str(datafile))
        return report
    except BaseException as exc:
        report.update(status='failed', reason=str(exc))
        raise
    finally:
        os.chdir(old)
        report['elapsed_s'] = round(time.monotonic()-started, 3)
        logfile.write_text('\n'.join(messages)+'\n', encoding='utf-8')
        reportfile.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, indent=2), flush=True)
        # Callbacks and the DLL-directory handle remain alive until completion.
        directory.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('deck',type=Path)
    parser.add_argument('--vectors', nargs='+', help='SPICE expressions to save/export')
    parser.add_argument('--compatibility', choices=['ps', 'psa'], default='psa',
                        help='psa matches KiCad GUI PSpice compatibility mode')
    args=parser.parse_args()
    deck=args.deck.resolve();run(deck,deck.with_suffix('.log'),args.vectors,args.compatibility)
