# /// script
# requires-python = ">=3.11"
# dependencies = ["pypdf>=5,<7"]
# ///
"""Export native copper plots, then improve silk contrast for a white PDF page."""
from pathlib import Path
import subprocess
from pypdf import PdfReader,PdfWriter
from pypdf.generic import ContentStream,FloatObject

ROOT=Path(__file__).resolve().parents[1]
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
for name in ('pcb_a_display','pcb_b_temperature'):
    out=ROOT/'review/routing'/(name+'_routed.pdf')
    subprocess.run([CLI,'pcb','export','pdf',str(ROOT/name/(name+'.kicad_pcb')),
                    '--layers','F.Cu,B.Cu','--common-layers','F.SilkS,Edge.Cuts',
                    '--mode-multipage','--output',str(out),'--scale','2'],check=True)
    reader=PdfReader(out);writer=PdfWriter(clone_from=reader)
    for page in writer.pages:
        content=ContentStream(page.get_contents(),writer)
        for args,op in content.operations:
            if op in (b'rg',b'RG') and len(args)==3 and float(args[0])>.9 and float(args[1])>.9 and float(args[2])<.7:
                args[:]=[FloatObject(.08),FloatObject(.10),FloatObject(.12)]
        page.replace_contents(content)
    with out.open('wb') as stream:writer.write(stream)
    assert len(PdfReader(out).pages)==2
    subprocess.run(['pdftoppm','-f','1','-scale-to','1600','-singlefile','-png',str(out),str(out.with_suffix(''))+'_front'],check=True)
