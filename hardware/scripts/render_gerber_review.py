"""Render the exported manufacturing layers with an independent Gerber parser.

Run with KiCad Python (Pillow) and uv. PyGerber 2.4.3 is isolated by uv;
no source board is uploaded. Bottom views are kept unmirrored to show the
shared CAM coordinate system, explicitly labelled on every image.
"""
import hashlib
import json
from pathlib import Path
import subprocess
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'manufacturing/review_A04_B05'
UV=Path.home()/'.local/bin/uv.exe'


def render():
    folder=OUT/'gerber_review';folder.mkdir(exist_ok=True)
    result=dict(status='Rendered exports; visual review recorded separately',renderer='PyGerber2.4.3',dpmm=16,
                source_sha256={},output_sha256={},bottom_views_mirrored=False)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',22)
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        gerbers=OUT/code/'gerbers_DRAFT_NOT_FOR_ORDER'
        for side,layer,silk in [('front','F_Cu.gtl','F_Silkscreen.gto'),('back','B_Cu.gbl','B_Silkscreen.gbo')]:
            sources=[gerbers/(name+'-'+suffix) for suffix in (layer,silk,'Edge_Cuts.gm1')]
            path=folder/(code+'_'+side+'.png')
            subprocess.run([str(UV),'tool','run','--from','pygerber==2.4.3','pygerber','render','project',
                            '-d','16','-o',str(path),*map(str,sources)],check=True,capture_output=True,text=True)
            with Image.open(path) as raw:
                image=Image.new('RGB',(raw.width+40,raw.height+100),'#10212b')
                image.paste(raw.convert('RGB'),(20,80))
            draw=ImageDraw.Draw(image)
            draw.text((20,12),f'PCB {code} / {side.upper()} - EXPORTED GERBER - NOT FOR ORDER',font=font,fill='white')
            draw.text((20,44),'CAM top coordinates; '+('bottom NOT mirrored.' if side=='back' else 'component side.')+' Copper + silk + outline; no drill cutouts.',font=font,fill='#b7d7df')
            image.save(path)
            for source in sources:result['source_sha256'][source.relative_to(ROOT).as_posix()]=hashlib.sha256(source.read_bytes()).hexdigest()
            result['output_sha256'][path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    result['source_sha256'][Path(__file__).relative_to(ROOT).as_posix()]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (folder/'render_manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('Rendered four source-bound Gerber review images.')
    return result


if __name__=='__main__':render()
