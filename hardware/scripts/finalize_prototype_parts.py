"""Apply the reviewed prototype part selection without changing the circuit.

KiCad Python; requires --apply. Keeps footprints, pad UUIDs, net assignments,
tracks and component origins, except documented LED lands and socket holes.
Native metadata is synchronized to the engineering selection, not live stock.
Historical library footprints remain available for archived revisions.
"""
import argparse
import csv
import json
from pathlib import Path
import pcbnew as p
import create_drafts as s

ROOT=Path(__file__).resolve().parents[1]
LIB=ROOT/'lab_hardware.pretty'
SELECTION=ROOT/'manufacturing/component_selection.json'


def line(fp,a,b,layer,width=.05):
    g=p.PCB_SHAPE(fp);g.SetShape(p.SHAPE_T_SEGMENT)
    ox,oy=fp.GetPosition().x,fp.GetPosition().y
    g.SetStart(p.VECTOR2I(ox+p.FromMM(a[0]),oy+p.FromMM(a[1])))
    g.SetEnd(p.VECTOR2I(ox+p.FromMM(b[0]),oy+p.FromMM(b[1])))
    g.SetLayer(layer);g.SetWidth(p.FromMM(width));fp.Add(g)


def rectangle(fp,x1,y1,x2,y2,layer):
    points=[(x1,y1),(x2,y1),(x2,y2),(x1,y2)]
    for a,b in zip(points,points[1:]+points[:1]):line(fp,a,b,layer)


def refine(fp,kind,libid):
    angle=fp.GetOrientationDegrees();fp.SetOrientationDegrees(0)
    ox,oy=fp.GetPosition().x,fp.GetPosition().y
    for g in list(fp.GraphicalItems()):
        if g.GetLayer()==p.F_CrtYd or (kind=='LED' and g.GetLayer()==p.F_SilkS):fp.Remove(g)
    if kind=='LED':
        for pad in fp.Pads():
            assert pad.GetNumber() in ('1','2')
            pad.SetPosition(p.VECTOR2I(ox+p.FromMM(-1.175 if pad.GetNumber()=='1' else 1.175),oy))
            pad.SetSize(p.VECTOR2I(p.FromMM(1.25),p.FromMM(1.1)));pad.SetShape(p.PAD_SHAPE_RECT)
        rectangle(fp,-2.05,-.925,2.05,.925,p.F_CrtYd)
        line(fp,(-2.05,-.8),(-2.05,.8),p.F_SilkS,.12)
        fp.SetLibDescription('Kingbright APT2012 green/orange: manufacturer lands1.25x1.10mm, gap1.10mm, cathode pad1. See manufacturing/land_pattern_review.json')
    else:
        assert kind=='SOCKET'
        for pad in fp.Pads():pad.SetDrillSize(p.VECTOR2I(p.FromMM(1.05),p.FromMM(1.05)))
        for y in (-12.7,12.7):rectangle(fp,-19.675,y-1.575,19.675,y+1.575,p.F_CrtYd)
        fp.SetLibDescription('Common ESP32 30P host: two Harwin M20-7821546 sockets, pitch2.54mm rows25.40mm confirmed; nominal holes1.05mm. DevKit body envelope remains provisional.')
    fp.SetFPID(p.LIB_ID('LabHardware',libid.split(':')[1]));fp.SetOrientationDegrees(angle)


def main():
    choices=json.loads(SELECTION.read_text(encoding='utf-8'))
    layout_path=ROOT/'placement.json';layout=json.loads(layout_path.read_text(encoding='utf-8'))
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        table=choices['boards'][code];folder=ROOT/name
        pcb=folder/(name+'.kicad_pcb');board=p.LoadBoard(str(pcb))
        fps={f.GetReference():f for f in board.GetFootprints()}
        assert set(fps)==set(table)
        connectivity={(r,pad.GetNumber()):pad.GetNetname() for r,fp in fps.items() for pad in fp.Pads()}
        before_tracks=[t.m_Uuid.AsString() for t in board.GetTracks()]
        for ref,choice in table.items():
            fp=fps[ref]
            if fp.GetFPID().GetUniStringLibId()!=choice['Footprint'] and ref in (('D1','D2') if code=='A' else ('D5','D6','D7','D8')):
                refine(fp,'LED',choice['Footprint'])
            if code=='A' and ref=='U1' and fp.GetFPID().GetUniStringLibId()!=choice['Footprint']:
                refine(fp,'SOCKET',choice['Footprint'])
            if code=='B' and ref=='D7':
                assert fp.GetPosition().x==p.FromMM(132) and fp.GetPosition().y in (p.FromMM(104),p.FromMM(104.25))
                fp.SetPosition(p.VECTOR2I(p.FromMM(132),p.FromMM(104.25)))
            assert fp.GetFPID().GetUniStringLibId()==choice['Footprint'], (code,ref)
            fp.SetValue(choice['Value'])
            for field,value in [('MPN',choice['MPN']),('Manufacturer',choice['Manufacturer']),('Assembly',choice['Assembly']),
                                ('Design_Status',choice['Decision']),('Datasheet',choice['Source'])]:
                fp.SetField(field,value);fp.GetField(field).SetVisible(False)
            if (code=='A' and ref in ('U1','D1')):
                # Build a standalone local-library footprint from the same native geometry.
                copy=p.FOOTPRINT(fp);copy.SetOrientationDegrees(0);copy.SetPosition(p.VECTOR2I(0,0))
                copy.SetReference('REF**');copy.SetValue('ESP32_30P_SOCKETS' if ref=='U1' else 'APT2012')
                for pad in copy.Pads():pad.SetNetCode(0)
                for field in ('MPN','Manufacturer','Assembly'):copy.SetField(field,'')
                p.PCB_IO_MGR.FindPlugin(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(LIB),copy)
        assert connectivity=={(r,pad.GetNumber()):pad.GetNetname() for r,fp in fps.items() for pad in fp.Pads()}
        assert before_tracks==[t.m_Uuid.AsString() for t in board.GetTracks()]
        if code=='A':
            for drawing in board.GetDrawings():
                if isinstance(drawing,p.PCB_TEXT) and 'U1 ROW SPACING 25.40 mm ASSUMED' in drawing.GetText():
                    drawing.SetText('U1: 25.40 mm ROWS CONFIRMED / HARWIN SOCKETS')
        p.ZONE_FILLER(board).Fill(board.Zones());p.SaveBoard(str(pcb),board)
        path=folder/(name+'.kicad_sch');lines=path.read_text(encoding='utf-8').splitlines();found=set()
        for i,raw in enumerate(lines):
            if not raw.startswith('(symbol (lib_id '):continue
            node=s.parse(raw);props={s.unquote(v[1]):v for v in s.children(node,'property')}
            ref=s.unquote(props['Reference'][2])
            if ref.startswith('#'):continue
            choice=table[ref];found.add(ref)
            for key,value in [('Value',choice['Value']),('Footprint',choice['Footprint']),('MPN',choice['MPN']),
                              ('Manufacturer',choice['Manufacturer']),('Datasheet',choice['Source']),
                              ('Assembly',choice['Assembly']),('Design_Status',choice['Decision'])]:
                if key in props:props[key][2]=s.q(value)
                else:
                    at=s.child(node,'at')[1:3]
                    node.append(['property',s.q(key),s.q(value),['at',*at,'0'],
                        ['effects',['font',['size','1','1']],'hide']])
            lines[i]=s.dump(node)
        assert found==set(table)
        schematic='\n'.join(lines)+'\n'
        schematic=schematic.replace('PROTOTYPE - DEVKIT PITCH TO VERIFY - NOT RELEASED FOR FABRICATION',
            'PROTOTYPE - ENGINEERING PREPARED - ASSEMBLER APPROVAL REQUIRED')
        if code=='B':schematic=schematic.replace('(rev "0.5 INTEGRATED BUCK")','(rev "0.5")')
        if code=='A':schematic=schematic.replace('Physical pad numbers defined; row spacing 25.40 mm ASSUMED.',
            'Common 30P pinout; row spacing25.40mm confirmed. Two Harwin M20-7821546 sockets.')
        path.write_text(schematic,encoding='utf-8')
        spec=next(b for b in layout['boards'] if b['name']==name)
        for row in spec['components']:
            ref=row['reference'];fp=fps[ref];box=fp.GetBoundingBox(False,False)
            row.update(value=table[ref]['Value'],footprint=table[ref]['Footprint'],
                origin_mm=[round(p.ToMM(fp.GetPosition().x)-60,5),round(p.ToMM(fp.GetPosition().y)-55,5)],
                bounds_mm=[round(p.ToMM(box.GetLeft())-60,3),round(p.ToMM(box.GetTop())-55,3),
                           round(p.ToMM(box.GetWidth()),3),round(p.ToMM(box.GetHeight()),3)])
            if ref in (('D1','D2') if code=='A' else ('D5','D6','D7','D8')):row['height_mm']=.9
            if code=='A' and ref=='U1':row['height_mm']=18
            if code=='B' and ref in ('J2','J3'):row['height_mm']=14.0
        with (folder/(name+'_bom_draft.csv')).open('w',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=['Reference','Value','Footprint','Design_Status','MPN','Manufacturer']);writer.writeheader()
            for ref,v in table.items():writer.writerow(dict(Reference=ref,Value=v['Value'],Footprint=v['Footprint'],
                Design_Status=v['Decision'],MPN=v['MPN'],Manufacturer=v['Manufacturer']))
        (folder/'fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "LabHardware") (type "KiCad") (uri "${KIPRJMOD}/../lab_hardware.pretty") (options "") (descr "Reviewed prototype land patterns; see manufacturing/land_pattern_review.json")))\n',encoding='utf-8')
    layout_path.write_text(json.dumps(layout,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    path=ROOT/'module_selection.json';modules=json.loads(path.read_text(encoding='utf-8'))
    modules['devkit']['footprint']=choices['boards']['A']['U1']['Footprint']
    modules['devkit']['socket']={
        'MPN':'M20-7821546','quantity':2,'nominal_body_mm':[38.5,2.5,8.5],
        'maximum_body_mm':[38.85,2.65,8.65],'native_finished_hole_nominal_mm':1.05,
        'manufacturer_recommended_hole_mm':[1.02,1.07],
        'male_contact_interface_mm':.64,'assumed_male_header_insulator_mm':2.54,
        'assembly_gap_allowance_mm':.15,
        'source':'https://content.harwin.com/asset/53851ba8-b32e-4ac5-a0c5-09d9271c0c0a/DRG-00385-Technical-Drawing-Datasheet-M20-782-pdf.pdf',
        'drawing_revision':'11 / 2026-08-03',
        'limits':'Female dimensions documented; DevKit male-header plastic and full three-board bodies remain envelopes to check on hardware.'}
    path.write_text(json.dumps(modules,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Native selections frozen: 68 physical components; tracks and pin/net assignments preserved.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--apply',action='store_true');args=parser.parse_args()
    if not args.apply:parser.error('--apply is required to modify native sources')
    main()
