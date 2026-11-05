"""Execute a SPICE deck using KiCad's installed ngspice shared library.

Python 64-bit on Windows. No KiCad GUI or extra simulator installation needed.
"""
import argparse
import ctypes as c
import os
from pathlib import Path
import re

def run(deck,logfile):
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
    lib.ngSpice_Command(b'set ngbehavior=ps')
    old=Path.cwd();os.chdir(deck.parent)
    try:
        result=lib.ngSpice_Command(('source '+deck.name).encode())
        if '.control' not in deck.read_text(encoding='utf-8').lower():
            lib.ngSpice_Command(b'save v(vout) v(vin) v(pg) i(L1)')
            result=lib.ngSpice_Command(b'run')
            lib.ngSpice_Command(b'set wr_singlescale')
            lib.ngSpice_Command(b'set wr_vecnames')
            lib.ngSpice_Command(('wrdata '+deck.stem+'.dat v(vout) v(vin) v(pg) i(L1)').encode())
        logfile.write_text('\n'.join(messages)+'\n',encoding='utf-8')
        print('ngspice command status:',result,'log:',logfile,flush=True)
        assert not any('error' in v.lower() or 'simulation interrupted' in v.lower() or 'timestep too small' in v.lower() for v in messages),'See simulation log'
        assert any('No. of Data Rows' in v for v in messages),'No transient analysis completed'
    finally:os.chdir(old)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('deck',type=Path);args=parser.parse_args()
    deck=args.deck.resolve();run(deck,deck.with_suffix('.log'))
