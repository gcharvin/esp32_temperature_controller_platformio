"""Route J10 only; retain original power, sensor, control and diagnostic copper."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from route_diagnostics import plan

ROOT=Path(__file__).resolve().parents[1]
KPY='C:/Program Files/KiCad/10.0/bin/python.exe'


def main():
    folder=Path(tempfile.mkdtemp(prefix='lab-uart-routing-'));print(folder,flush=True)
    layout=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
    for spec in layout['boards']:
        name=spec['name'];source=ROOT/name/(name+'.kicad_pcb');base=folder/(name+'.kicad_pcb')
        shutil.copy2(source,base)
        copper=folder/(name+'.json');routes=folder/(name+'_uart_routes.json')
        router=ROOT/'scripts/route_diagnostics.py'
        subprocess.run([KPY,str(router),'--extract',str(base),str(copper)],check=True,capture_output=True)
        keepouts=spec['clip_bands_mm']+spec['side_support_bands_mm']+spec['clip_side_contact_patches_mm']
        if name=='pcb_a_display':keepouts+=[[0,17,12.5,46]]
        added=plan(json.loads(copper.read_text()),[dict(led='J10',resistor='J10')],*spec['board_mm'][:2],keepouts,skip_unconnected=True)
        routes.write_text(json.dumps(added,indent=2)+'\n',encoding='utf-8')
        project=source.with_suffix('.kicad_pro');original=project.read_bytes()
        try:subprocess.run([KPY,str(router),'--apply',str(base),str(source),str(routes)],check=True,capture_output=True)
        finally:project.write_bytes(original)
    print('Only J10 branches routed. Native DRC/parity and original copper preservation checks remain mandatory.',flush=True)


if __name__=='__main__':main()
