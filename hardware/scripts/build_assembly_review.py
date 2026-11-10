"""Source-bound assembly drawings, not an assembler-approved placement preview.

Run with KiCad Python and --pdf-python PATH_TO_PYTHON_WITH_REPORTLAB.
The second interpreter only renders geometry extracted from native boards.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'manufacturing/output'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract():
    import pcbnew as p
    sources = [Path(__file__), ROOT/'manufacturing/land_pattern_review.json']
    data = dict(boards=[], source_sha256={})
    for code, name in [('A', 'pcb_a_display'), ('B', 'pcb_b_temperature')]:
        path = ROOT/name/(name+'.kicad_pcb')
        sources.append(path)
        board = p.LoadBoard(str(path))
        edges = [g for g in board.GetDrawings() if g.GetLayer() == p.Edge_Cuts]
        points = [pt for g in edges for pt in (g.GetStart(), g.GetEnd())]
        x0 = min(pt.x for pt in points); y0 = min(pt.y for pt in points)
        def xy(pt):
            return [round(p.ToMM(pt.x-x0), 6), round(p.ToMM(pt.y-y0), 6)]
        width = p.ToMM(max(pt.x for pt in points)-x0)
        height = p.ToMM(max(pt.y for pt in points)-y0)
        item = dict(code=code, size_mm=[width, height], native_origin_mm=[p.ToMM(x0), p.ToMM(y0)], parts=[])
        for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
            row = dict(ref=fp.GetReference(), value=fp.GetValue(), footprint=fp.GetFPID().GetUniStringLibId(),
                       at=xy(fp.GetPosition()), label=xy(fp.Reference().GetPosition()),
                       angle=fp.GetOrientationDegrees(), fab=[], pads=[])
            for g in fp.GraphicalItems():
                if not isinstance(g, p.PCB_SHAPE) or g.GetLayer() != p.F_Fab:
                    continue
                kind = g.GetShape()
                if kind == p.SHAPE_T_SEGMENT:
                    row['fab'].append(dict(kind='line', points=[xy(g.GetStart()), xy(g.GetEnd())]))
                elif kind == p.SHAPE_T_RECT:
                    row['fab'].append(dict(kind='rect', points=[xy(g.GetStart()), xy(g.GetEnd())]))
                elif kind == p.SHAPE_T_CIRCLE:
                    row['fab'].append(dict(kind='circle', center=xy(g.GetCenter()), radius=p.ToMM(g.GetRadius())))
                elif kind == p.SHAPE_T_POLY:
                    poly = g.GetPolyShape()
                    for j in range(poly.OutlineCount()):
                        outline = poly.COutline(j)
                        row['fab'].append(dict(kind='poly', points=[xy(outline.CPoint(i)) for i in range(outline.PointCount())]))
                elif kind == p.SHAPE_T_ARC:
                    center = xy(g.GetCenter()); start, mid, end = [xy(pt) for pt in (g.GetStart(), g.GetArcMid(), g.GetEnd())]
                    a, m, b = [math.atan2(pt[1]-center[1], pt[0]-center[0]) for pt in (start, mid, end)]
                    sweep = (b-a) % (2*math.pi)
                    if (m-a) % (2*math.pi) > sweep:
                        sweep -= 2*math.pi
                    radius = math.dist(center, start)
                    row['fab'].append(dict(kind='path', points=[[center[0]+radius*math.cos(a+sweep*i/32),
                        center[1]+radius*math.sin(a+sweep*i/32)] for i in range(33)]))
                else:
                    raise ValueError('Unrendered fabrication shape '+str(kind))
            for pad in fp.Pads():
                if not pad.GetNumber():
                    continue
                row['pads'].append(dict(number=pad.GetNumber(), net=pad.GetNetname().removeprefix('/'),
                    at=xy(pad.GetPosition()), size=[p.ToMM(pad.GetSize().x), p.ToMM(pad.GetSize().y)],
                    angle=pad.GetOrientationDegrees(), drill=p.ToMM(pad.GetDrillSize().x),
                    circle=pad.GetShape() == p.PAD_SHAPE_CIRCLE))
            item['parts'].append(row)
        data['boards'].append(item)
    data['source_sha256'] = {path.relative_to(ROOT).as_posix(): sha(path) for path in sources}
    OUT.mkdir(exist_ok=True, parents=True)
    (OUT/'assembly_geometry.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    # Every native pin remains traceable, including duplicate LM2596 tab/pin3.
    with (OUT/'assembly_pad_map.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        fields = ['Board','Reference','Pin','Net','Local_X_mm','Local_Y_mm','Native_X_mm','Native_Y_mm']
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for b in data['boards']:
            for f in b['parts']:
                for pad in f['pads']:
                    x, y = pad['at']; ox, oy = b['native_origin_mm']
                    writer.writerow(dict(Board=b['code'], Reference=f['ref'], Pin=pad['number'], Net=pad['net'],
                        Local_X_mm=x, Local_Y_mm=y, Native_X_mm=round(x+ox,6), Native_Y_mm=round(y+oy,6)))
    with (OUT/'assembly_THT_placement.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['Board','Assembly_item','Native_reference','Origin','Local_X_mm',
            'Local_Y_mm','Native_X_mm','Native_Y_mm','Native_rotation_deg','Pads','First_pad_X_mm','First_pad_Y_mm','Status'])
        writer.writeheader(); count=0
        for b in data['boards']:
            for f in b['parts']:
                if not any(pad['drill'] for pad in f['pads']): continue
                groups=([('U1_SOCKET_BOTTOM',range(1,16)),('U1_SOCKET_TOP',range(16,31))]
                        if b['code']=='A' and f['ref']=='U1' else [(f['ref'],None)])
                for label,numbers in groups:
                    pads=([pad for pad in f['pads'] if int(pad['number']) in numbers] if numbers else f['pads'])
                    # Socket centroid is distinct from the virtual DevKit origin.
                    at=([sum(pad['at'][axis] for pad in pads)/len(pads) for axis in (0,1)] if numbers else f['at'])
                    first=min(pads,key=lambda pad:int(pad['number'])); ox,oy=b['native_origin_mm']
                    writer.writerow(dict(Board=b['code'],Assembly_item=label,Native_reference=f['ref'],
                        Origin='socket pad-centre centroid' if numbers else 'native footprint origin',
                        Local_X_mm=round(at[0],6),Local_Y_mm=round(at[1],6),Native_X_mm=round(at[0]+ox,6),
                        Native_Y_mm=round(at[1]+oy,6),Native_rotation_deg=f['angle'],
                        Pads=' '.join(pad['number'] for pad in pads),First_pad_X_mm=first['at'][0],First_pad_Y_mm=first['at'][1],
                        Status='Top view; native angle, not approved assembly rotation; sockets require pad-row alignment'))
                    count+=1
        assert count==21, 'Unexpected physical THT assembly scope'
    return data


def render():
    from reportlab.pdfgen import canvas
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.colors import HexColor, white
    from reportlab.lib.units import mm
    data = json.loads((OUT/'assembly_geometry.json').read_text(encoding='utf-8'))
    for relative, digest in data['source_sha256'].items():
        assert sha(ROOT/relative) == digest, 'Stale drawing source '+relative
    folder = OUT/'pdf'; folder.mkdir(exist_ok=True)
    path = folder/'assembly_review_A04_B05.pdf'
    c = canvas.Canvas(str(path), pagesize=landscape(A4), invariant=1)
    c.setTitle('A v0.4 / B v0.5 - Assembly review, NOT FOR ORDER')
    W, H = landscape(A4)
    def text(x, y, line, size=10, color='#18313e'):
        c.setFillColor(HexColor(color)); c.setFont('Helvetica', size); c.drawString(x, y, line)
    def header(title, page):
        text(30, H-35, title, 19)
        text(30, H-54, 'ASSEMBLY REVIEW - NOT FOR ORDER | 2026-11-10 | Top view, not mirrored', 9, '#a14d16')
        text(30, 20, 'A0.4.3 / B0.5.3 | Native KiCad geometry | PCB corner coordinates in mm; not a printable 1:1 template.', 8)
        text(W-55, 20, str(page), 8)
    for index, b in enumerate(data['boards'], 1):
        header('PCB '+b['code']+' - component side and pin 1', index)
        width, height = b['size_mm']
        scale = min(610/width, 415/height)
        left, top = 42, H-90
        def pt(point):
            return left+point[0]*scale, top-point[1]*scale
        c.setStrokeColor(HexColor('#546c78')); c.setLineWidth(.8)
        c.setFillColor(HexColor('#f3f7f8')); c.rect(left, top-height*scale, width*scale, height*scale, fill=1)
        text(left, top+13, f'{width:g} x {height:g} mm | origin (0,0) = upper-left Edge.Cuts corner', 9)
        for f in b['parts']:
            if f['ref'].startswith('TP'):continue  # Back-side copper, separate bench map.
            c.setStrokeColor(HexColor('#687d89')); c.setLineWidth(.5)
            for shape in f['fab']:
                kind = shape['kind']
                if kind == 'circle':
                    x, y = pt(shape['center']); c.circle(x, y, shape['radius']*scale)
                elif kind == 'rect':
                    (x1, y1), (x2, y2) = [pt(v) for v in shape['points']]
                    c.rect(min(x1,x2), min(y1,y2), abs(x2-x1), abs(y2-y1))
                else:
                    points = [pt(v) for v in shape['points']]
                    pth = c.beginPath(); pth.moveTo(*points[0])
                    for v in points[1:]: pth.lineTo(*v)
                    if kind == 'poly': pth.close()
                    c.drawPath(pth)
            for pad in f['pads']:
                x, y = pt(pad['at']); pw, ph = [v*scale for v in pad['size']]
                c.saveState(); c.translate(x,y); c.rotate(pad['angle'])
                c.setFillColor(HexColor('#e5b867')); c.setStrokeColor(HexColor('#ae8943')); c.setLineWidth(.3)
                if pad['circle']: c.circle(0,0,pw/2,fill=1)
                else: c.rect(-pw/2,-ph/2,pw,ph,fill=1)
                if pad['drill']:
                    c.setFillColor(white); c.circle(0,0,pad['drill']*scale/2,fill=1)
                c.restoreState()
                if pad['number'] == '1':
                    c.setFillColor(HexColor('#a62f24')); c.circle(x,y,1.25,stroke=0,fill=1)
            # Reference anchors come from native silkscreen positions.
            x,y = pt(f['label']); size=6.5
            c.setFont('Helvetica-Bold',size); tw=c.stringWidth(f['ref'],'Helvetica-Bold',size)
            c.setFillColor(white);c.rect(x-tw/2-1,y-1,tw+2,size+1,fill=1,stroke=0)
            c.setFillColor(HexColor('#123e53'));c.drawCentredString(x,y,f['ref'])
        x=left+width*scale+24
        text(x, H-105, 'READING THE DRAWING', 9)
        for j,line in enumerate(['Grey: native F.Fab outlines.', 'Gold: copper lands.', 'Red dot: native pad 1.', 'Holes: white centres.', 'All components on top.', 'TP pads on BACK: separate map.', 'Not a stencil/rotation approval.']):
            text(x, H-126-j*16, line, 8)
        if b['code']=='A':
            notes=['U1: fit TWO 1x15 female sockets; ESP32 is removable.',
                   'Row centres 25.40 mm; contact pitch 2.54 mm. Antenna left, USB right.',
                   'Pads 1..15: bottom row; pads 16..30: top row. Do not mirror.',
                   'JP1 shunt supplied loose; follow USB power service instructions.']
        else:
            notes=['J2 = 24 V input; J3 = heater; J4 = passive NTC probe (15k).',
                   'C8/C10: pad 1 positive. D9: pad 1 cathode, BUCK_SW.',
                   'U1 tab/pad 3 = PGND. NT1 = copper only; do not populate.',
                   'R13 uses Vishay-specific lands; see page 3.']
        baseline=top-height*scale-22
        for j,line in enumerate(notes): text(left,baseline-j*13,line,9)
        c.showPage()
    header('Assembly details - selected orientations and R13 lands', 3)
    text(35,H-86,'R13 / WSL1206R0500FEA - manufacturer recommended pattern, 0.006..0.200 ohm range',11)
    scale=28; cx,cy=170,H-153
    for sign in (-1,1):
        c.setFillColor(HexColor('#e5b867'));c.setStrokeColor(HexColor('#ae8943'))
        c.rect(cx+(sign*1.725-.825)*scale,cy-.965*scale,1.65*scale,1.93*scale,fill=1)
    text(300,cy+15,'Lands: 1.65 x 1.93 mm; centre distance: 3.45 mm',10)
    text(300,cy-3,'Inner gap: 1.80 mm; total copper span: 5.10 mm',10)
    text(300,cy-21,'Vishay datasheet 30100, page 2; not a generic 1206 replacement rule.',9)
    text(35,H-214,'Native pad functions to check in the assembler placement preview',11)
    boards={b['code']:{f['ref']:f for f in b['parts']} for b in data['boards']}
    def pin_text(ref):
        pins={pad['number']:pad['net'] for pad in boards['B'][ref]['pads']}
        return ', '.join(number+'='+pins[number] for number in sorted(pins,key=int))
    rows=[('A U2 / B U2','AP2112: 1=VIN, 2=GND, 3=EN, 4=NC, 5=3V3'),
          ('A D1,D2 / B D5..D8','LED pad 1 = cathode; pad 2 = anode'),
          ('B U1','LM2596 fixed5V: 1=VIN, 2=SW, 3/tab=PGND, 4=FB(5V), 5=PGND'),
          ('B U3',pin_text('U3')),
          ('B U5',pin_text('U5')),
          ('B Q1 / Q2','Q1 G-D-S = 1-2-3; Q2 G-D-S = 1-2-3 (tab=2)'),
          ('B D3 / D4 / D9','Pad1 cathode: VIN_PROTECTED / VIN_PROTECTED / BUCK_SW respectively'),
          ('B C8 / C10','Positive pin1: VIN_PROTECTED / +5V_BUCK; C10 pin2 returns via R13'),
          ('A/B J1; J6; J7; J10','Pin1 GND. J10:2=TX_A,3=RX_A; pins2/3 NC on temperature B.'),
          ('B J2 / J3 / J4','Pin1: VIN_RAW / VIN_PROTECTED / 3V3_B. J4 pin2=NTC_RAW.'),
          ('A U1 sockets','Bottom pads1..15 EN->VIN; top pads16..30 3V3->GPIO23; pitch2.54'),
          ('B NT1','No purchased or fitted part; PGND/GND copper junction')]
    y=H-240
    for ref,line in rows:
        text(35,y,ref,9);text(192,y,line,9);y-=19
    text(35,y-8,'Full pin/net/coordinate map: assembly_pad_map.csv. Native origin differs from board-corner origin.',9)
    text(35,y-24,'Read these functional notes with the schematics; assembly approval does not validate power or thermal behavior.',9)
    c.save()
    result=dict(status='DRAFT / human and assembler review required',pages=3,
        source_sha256=data['source_sha256'],output_sha256={p.relative_to(ROOT).as_posix():sha(p)
        for p in (path,OUT/'assembly_geometry.json',OUT/'assembly_pad_map.csv',OUT/'assembly_THT_placement.csv')},
        limits=['Native top view; pad1 dots indicate pin numbering, not universal polarity.',
                'Rotation convention must be checked in assembler preview; no automatic correction.',
                'The PDF is not a 1:1 print, fabrication layer or exact supplier 3D model.'])
    (OUT/'assembly_review_manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(path)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pdf-python',type=Path)
    parser.add_argument('--render',action='store_true')
    args=parser.parse_args()
    if args.render:
        render()
    else:
        if not args.pdf_python: parser.error('--pdf-python is required for the renderer')
        extract()
        subprocess.run([str(args.pdf_python),str(Path(__file__)), '--render'],check=True)
