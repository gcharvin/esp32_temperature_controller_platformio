"""Check copper-only back pads, native nets and preservation of original routing.

KiCad Python/Pillow. Produces a front-coordinate CSV and physically mirrored
back-view maps. No test pads may enter a purchasing or placement BOM.
"""
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p
from PIL import Image,ImageDraw,ImageFont
from route_diagnostics import copper_signature

ROOT=Path(__file__).resolve().parents[1]


def main():
    config=ROOT/'test_points.json';data=json.loads(config.read_text(encoding='utf-8'))
    sources=[Path(__file__),config,ROOT/'lab_hardware.pretty/ProbePad_D1.5mm_NoPaste.kicad_mod']
    rows=[];counts={};preserved={}
    for name,points in data['boards'].items():
        code='A' if name=='pcb_a_display' else 'B';file=ROOT/name/(name+'.kicad_pcb')
        net=ROOT/'review'/(name+'.net.xml');sources.extend([file,file.with_suffix('.kicad_sch'),net])
        board=p.LoadBoard(str(file));fps={f.GetReference():f for f in board.GetFootprints()}
        project=file.with_suffix('.kicad_pro');sources.append(project)
        if code=='B':
            rules=json.loads(project.read_text(encoding='utf-8'))['net_settings']
            buck=next(c for c in rules['classes'] if c['name']=='Buck5V')
            assert buck['clearance']==0.25 and buck['track_width']==1.0
            assert {'netclass':'Buck5V','pattern':'/+5V_BUCK'} in rules['netclass_patterns']
        pins={(n.get('ref'),n.get('pin')):g.get('name') for g in ET.parse(net).findall('./nets/net') for n in g.findall('node')}
        assert {r for r in fps if r.startswith('TP')}=={r['ref'] for r in points}
        for row in points:
            fp=fps[row['ref']];pads=list(fp.Pads());assert len(pads)==1
            pad=pads[0];assert pad.GetNumber()=='1' and pad.GetNetname()==pins[row['ref'],'1']=='/'+row['net']
            assert pad.IsOnLayer(p.B_Cu) and pad.IsOnLayer(p.B_Mask)
            assert not any(pad.IsOnLayer(layer) for layer in (p.F_Cu,p.F_Paste,p.B_Paste))
            assert pad.GetDrillSize().x==0 and pad.GetSize().x==pad.GetSize().y==p.FromMM(1.5)
            assert fp.GetFieldText('Assembly')=='NONE_COPPER' and not fp.GetFieldText('MPN')
            assert [round(p.ToMM(pad.GetPosition().x)-60,5),round(p.ToMM(pad.GetPosition().y)-55,5)]==row['at_mm']
            rows.append(dict(Board=code,TestPoint=row['ref'],Net=row['net'],Side='BACK / B.Cu',
                             Front_coordinate_X_mm=row['at_mm'][0],Front_coordinate_Y_mm=row['at_mm'][1],
                             Nominal_pad_diameter_mm=1.5,Assembly='NONE_COPPER_DO_NOT_POPULATE'))
        baseline=data['original_copper'][name];current=copper_signature(board)
        original={key:current[key] for key in baseline['UUIDs']}
        assert len(original)==baseline['count']
        assert hashlib.sha256(json.dumps(original,sort_keys=True).encode()).hexdigest()==baseline['signature_sha256']
        preserved[code]=baseline['count'];counts[code]=len(points)
        width,height=(110,75) if code=='A' else (90,65)
        scale=9;im=Image.new('RGB',(int(width*scale)+120,int(height*scale)+155),'#f4f7f9');draw=ImageDraw.Draw(im)
        font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',16);title=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',23)
        draw.text((30,18),'PCB '+code+' - BACK VIEW / VUE DE DOS',font=title,fill='#153745')
        draw.text((30,51),'Board flipped about its vertical axis; pads are copper, not fitted parts.',font=font,fill='#48616e')
        draw.rectangle((60,85,60+width*scale,85+height*scale),fill='#dfede5',outline='#234f35',width=2)
        for row in points:
            x,y=row['at_mm'];x=60+(width-x)*scale;y=85+y*scale
            draw.ellipse((x-7,y-7,x+7,y+7),fill='#d6ad52',outline='#805f1c')
            draw.text((x,y+12),row['ref'],font=font,fill='#123a49',anchor='mt')
        draw.text((30,im.height-38),'Use the CSV for nets; its coordinates are the native FRONT view, not this mirrored view.',font=font,fill='#48616e')
        im.save(ROOT/'review'/('test_points_'+code+'_back.png'))
    assert counts=={'A':10,'B':12}
    out=ROOT/'review/test_point_map.csv'
    with out.open('w',encoding='utf-8-sig',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    report=dict(status='PASS / native copper pads and original routes preserved',test_points=counts,
        preserved_original_tracks_and_vias=preserved,purchased_testpoint_components=0,paste_apertures_on_testpads=0,
        measured_on_hardware=False,source_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources})
    (ROOT/'review/routing/test_points_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
