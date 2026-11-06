"""Route only the new copper probe branches using the existing conservative router.

Run with primary Python/Pillow. Native edits run with KiCad Python. Original
tracks/vias are retained; final KiCad DRC and schematic parity are mandatory.
"""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from route_diagnostics import plan

ROOT=Path(__file__).resolve().parents[1]
KPY='C:/Program Files/KiCad/10.0/bin/python.exe'


def main():
    folder=Path(tempfile.mkdtemp(prefix='lab-probe-routing-'));print(folder,flush=True)
    layout=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
    probes=json.loads((ROOT/'test_points.json').read_text(encoding='utf-8'))
    for spec in layout['boards']:
        name=spec['name'];source=ROOT/name/(name+'.kicad_pcb');base=folder/(name+'.kicad_pcb');shutil.copy2(source,base)
        copper=folder/(name+'.json');routes=folder/(name+'_routes.json')
        router=ROOT/'scripts/route_diagnostics.py'
        subprocess.run([KPY,str(router),'--extract',str(base),str(copper)],check=True)
        keepouts=spec['clip_bands_mm']+spec['side_support_bands_mm']+spec['clip_side_contact_patches_mm']
        if name=='pcb_a_display':keepouts=keepouts+[[0,17,12.5,46]]
        # The generic branch planner identifies added pads by these two fields.
        added=[dict(led=r['ref'],resistor=r['ref']) for r in probes['boards'][name]]
        result=plan(json.loads(copper.read_text()),added,*spec['board_mm'][:2],keepouts)
        routes.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        # LoadBoard/SaveBoard on the temporary board has no adjacent project.
        # Preserve the destination project: otherwise default net classes from
        # that temporary board can overwrite the existing buck design rules.
        project=source.with_suffix('.kicad_pro');project_bytes=project.read_bytes()
        try:
            subprocess.run([KPY,str(router),'--apply',str(base),str(source),str(routes)],check=True)
        finally:
            project.write_bytes(project_bytes)
    subprocess.run([KPY,str(ROOT/'scripts/add_test_points.py'),'--refresh-placement'],check=True)


if __name__=='__main__':main()
