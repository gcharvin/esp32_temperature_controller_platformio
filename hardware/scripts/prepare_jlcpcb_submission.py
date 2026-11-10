"""Create/check a local JLCPCB RFQ package, including physical THT placements.

Run with the documented primary Python (Pillow) after preparation of the
source-bound A04/B05 fabrication handoff. No network, upload or ordering.
The CSVs are machine manufacturing exports, not a new engineering BOM.
"""
import argparse
import csv
import hashlib
import io
import json
import math
import re
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
M=ROOT/'manufacturing'
OUT=M/'jlcpcb'
ARCHIVE=M/'JLCPCB_A04_B05_RFQ_DFM_ONLY.zip'
BOM_FIELDS=['Comment','Designator','Footprint','Manufacturer','Manufacturer Part','LCSC Part #']
CPL_FIELDS=['Designator','Mid X','Mid Y','Layer','Rotation']
SOURCE_HASHES={}
OUTPUT_FILES=set()


def sha(data):return hashlib.sha256(data).hexdigest()
def source(path):
    data=path.read_bytes();SOURCE_HASHES[path.relative_to(ROOT).as_posix()]=sha(data);return data
def read_json(path):return json.loads(source(path))
def read_csv(path):return list(csv.DictReader(io.StringIO(source(path).decode('utf-8-sig'))))
def write(path,data):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data);OUTPUT_FILES.add(path)
def json_bytes(data):return (json.dumps(data,indent=2,ensure_ascii=False)+'\n').encode('utf-8')
def write_csv(path,rows,fields):
    stream=io.StringIO(newline='');writer=csv.DictWriter(stream,fieldnames=fields)
    writer.writeheader();writer.writerows(rows);write(path,stream.getvalue().encode('utf-8-sig'))
def zip_bytes(files):
    stream=io.BytesIO()
    with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as z:
        for name,data in sorted(files.items()):
            entry=zipfile.ZipInfo(name,(2026,11,6,0,0,0));entry.compress_type=zipfile.ZIP_DEFLATED
            z.writestr(entry,data)
    return stream.getvalue()


def check_legacy():
    # Refuse stale upstream manufacturing evidence without regenerating CAD.
    for filename,source_key,file_key in [
        ('RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip','sources_sha256','output_sha256'),
        ('FABRICATION_HANDOFF_A04_B05.zip','workspace_sources_sha256','archive_files_sha256')]:
        blob=source(M/filename)
        with zipfile.ZipFile(io.BytesIO(blob)) as z:
            assert z.testzip() is None
            manifest=json.loads(z.read('manifest.json' if filename.startswith('RFQ') else 'handoff_manifest.json'))
            for relative,digest in manifest[source_key].items():
                assert sha(source(ROOT/relative))==digest,'Stale upstream source '+relative
            hashes=dict(manifest[file_key])
            if filename.startswith('RFQ'):hashes.update(manifest['attachments_sha256'])
            assert set(z.namelist())==set(hashes)|{'manifest.json' if filename.startswith('RFQ') else 'handoff_manifest.json'}
            for name,digest in hashes.items():assert sha(z.read(name))==digest,'Upstream archive corruption '+name
    status=read_json(M/'preparation_status.json')
    assert status['local_preparation_complete'] and not status['fabrication_release'] and not status['order_placed']


def check_probe_mask(folder,points):
    """Check the actual exported mask openings, not just footprint attributes."""
    mask=next(folder.glob('*-B_Mask.gbs'));text=source(mask).decode('utf-8')
    ref=None;aperture=None;diameters={};actual={}
    for line in text.splitlines():
        if m:=re.fullmatch(r'%ADD(\d+)C,([\d.]+)\*%',line):diameters[m[1]]=float(m[2])
        elif m:=re.fullmatch(r'D(\d+)\*',line):aperture=m[1]
        elif m:=re.fullmatch(r'%TO.C,([^*]+)\*%',line):ref=m[1]
        elif line=='%TD*%':ref=None
        elif ref and ref.startswith('TP') and (m:=re.fullmatch(r'X(-?\d+)Y(-?\d+)D03\*',line)):
            assert aperture in diameters and diameters[aperture]>=1.5
            actual[ref]=[int(m[1])/1e6,int(m[2])/1e6]
    expected={r['ref']:[r['at_mm'][0]+60,-(r['at_mm'][1]+55)] for r in points}
    assert actual==expected,'Missing/misplaced back solder-mask opening'
    for suffix in ('F_Paste.gtp','B_Paste.gbp'):
        paste=source(next(folder.glob('*-'+suffix))).decode('utf-8')
        assert not re.search(r'%TO.C,TP\d+\*%',paste),'Test pad must not receive solder paste'


def body_center(part):
    """THT centroid from actual native F.Fab, not a pin1 footprint origin.

    Only the reviewed classes are accepted. Socket rows are handled separately.
    Fab extremities include the body bevel/terminal details, not silkscreen.
    """
    fab=part['fab'];footprint=part['footprint']
    if 'CP_Radial_' in footprint:
        circles=[s for s in fab if s['kind']=='circle'];assert len(circles)==1
        return circles[0]['center'],'F.Fab radial body circle'
    if 'JST_XH_' in footprint or 'TO-220-3_Vertical' in footprint:
        rects=[s for s in fab if s['kind']=='rect'];assert len(rects)==1
        a,b=rects[0]['points'];return [(a[i]+b[i])/2 for i in (0,1)],'F.Fab body rectangle'
    assert 'PinHeader_1x02' in footprint or 'Phoenix_MKDS' in footprint,footprint
    points=[point for s in fab for point in s.get('points',[])]
    return [(min(p[i] for p in points)+max(p[i] for p in points))/2 for i in (0,1)],'F.Fab body boundary extrema'


def placement_preview(board,mapping,path):
    from PIL import Image,ImageDraw,ImageFont
    scale=9;left=65;top=110;width,height=board['size_mm']
    image=Image.new('RGB',(int(width*scale)+130,int(height*scale)+225),'#f3f6f8');d=ImageDraw.Draw(image)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',17)
    title=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',23)
    def xy(p):return (left+p[0]*scale,top+p[1]*scale)
    d.text((30,18),'JLCPCB RFQ - '+board['code']+' - placement reference only',font=title,fill='#123343')
    d.text((30,53),'THT body centres: red; SMT native origins: blue. Factory alignment NOT approved.',font=font,fill='#354b58')
    d.rectangle((*xy((0,0)),*xy((width,height))),fill='#e0eee4',outline='#234b36',width=2)
    for part in board['parts']:
        if part['ref'].startswith('TP') or (part['ref']=='U1' and board['code']=='A'):continue
        for shape in part['fab']:
            points=shape.get('points',[])
            if shape['kind']=='circle':
                x,y=xy(shape['center']);r=shape['radius']*scale
                d.ellipse((x-r,y-r,x+r,y+r),outline='#657c70',width=1)
            elif shape['kind']=='rect':
                a,b=map(xy,points);d.rectangle((min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1])),outline='#657c70')
            elif points:d.line([xy(p) for p in points],fill='#657c70',width=1)
        first=min(part['pads'],key=lambda p:int(p['number']))
        x,y=xy(first['at']);d.ellipse((x-2,y-2,x+2,y+2),fill='#223b30')
    for row in mapping:
        x,y=xy((row['Local_X_mm'],row['Local_Y_mm']));color='#b33030' if row['Type']=='THT' else '#2b64a0'
        d.line((x-4,y,x+4,y),fill=color,width=2);d.line((x,y-4,x,y+4),fill=color,width=2)
        if row['Type']=='THT':d.text((x+6,y+4),row['Upload_reference'],font=font,fill=color)
        if row['Native_reference']=='U1' and board['code']=='A':
            pads=[int(p) for p in row['Native_pads'].split()]
            native=next(p for p in board['parts'] if p['ref']=='U1')
            pp=[p['at'] for p in native['pads'] if int(p['number']) in pads]
            d.line([xy(pp[0]),xy(pp[-1])],fill=color,width=3)
            x1,y1=xy((row['Native_first_pad_X_mm']-board['native_origin_mm'][0],
                       row['Native_first_pad_Y_mm']-board['native_origin_mm'][1]))
            d.ellipse((x1-3,y1-3,x1+3,y1+3),fill='#223b30')
    y=image.height-85
    d.text((30,y),'Top view; native pin1 markers in black; no coordinate mirroring of the board.',font=font,fill='#354b58')
    note=('A: J8 = U1 pads1..15; J9 = U1 pads16..30. No ESP32 module fitted.' if board['code']=='A'
          else 'B: U1 is the LM2596 and is fitted. NT1 is copper only, no purchased part.')
    d.text((30,y+25),note,font=font,fill='#354b58')
    stream=io.BytesIO();image.save(stream,format='PNG');write(path,stream.getvalue())


def build():
    source(Path(__file__))
    for name in ('README.md','request_support_EN.txt','official_sources.json'):source(OUT/name)
    check_legacy()
    geometry=read_json(M/'output/assembly_geometry.json')
    for relative,digest in geometry['source_sha256'].items():assert sha(source(ROOT/relative))==digest
    geometry_boards={b['code']:b for b in geometry['boards']}
    probes=read_json(ROOT/'test_points.json')['boards']
    mappings=[];all_boms=[];counts={}
    for code,prefix,expected_smt,expected_tht in [('A','A04',13,11),('B','B05',38,10)]:
        folder=M/'review_A04_B05'/code;board=geometry_boards[code];ox,oy=board['native_origin_mm']
        native={p['ref']:p for p in board['parts']}
        bom=read_csv(folder/'BOM_SMT_RFQ_NOT_FOR_ORDER.csv')
        old_cpl=read_csv(folder/'CPL_SMT_RFQ_ROTATIONS_UNVERIFIED.csv')
        assert len(bom)==len(old_cpl)==expected_smt
        assert {r['Designator'] for r in bom}=={r['Designator'] for r in old_cpl}
        cpl=[];mapping=[]
        def place(ref,native_ref,position,rotation,method,kind,pads):
            nx,ny=position;part=native[native_ref]
            first=min(pads,key=lambda p:int(p['number']))
            rotation=rotation%360
            cpl.append(dict(Designator=ref,**{'Mid X':f'{nx:.6f}','Mid Y':f'{-ny:.6f}'},Layer='Top',Rotation=f'{rotation:.6f}'))
            mapping.append(dict(Board=code,Upload_reference=ref,Native_reference=native_ref,Type=kind,
                Native_pads=' '.join(p['number'] for p in sorted(pads,key=lambda p:int(p['number']))),
                Local_X_mm=round(nx-ox,6),Local_Y_mm=round(ny-oy,6),
                Native_X_mm=round(nx,6),Native_Y_mm=round(ny,6),
                CPL_X_mm=round(nx,6),CPL_Y_mm=round(-ny,6),CPL_rotation_deg=rotation,
                Native_origin_X_mm=round(part['at'][0]+ox,6),Native_origin_Y_mm=round(part['at'][1]+oy,6),
                Native_first_pad=first['number'],Native_first_pad_X_mm=round(first['at'][0]+ox,6),
                Native_first_pad_Y_mm=round(first['at'][1]+oy,6),Position_method=method))
        for row in old_cpl:
            ref=row['Designator'];part=native[ref]
            assert math.dist([float(row['Mid X']),-float(row['Mid Y'])],[part['at'][0]+ox,part['at'][1]+oy])<1e-5
            place(ref,ref,[float(row['Mid X']),-float(row['Mid Y'])],float(row['Rotation']),
                  'Unchanged checked SMT CPL origin','SMT',part['pads'])
        tht=read_csv(folder/'THT_assembly_quote.csv')
        for row in tht:
            ref=row['Reference'];part=native[ref];center,method=body_center(part)
            bom.append(dict(Comment=row['Value'],Designator=ref,Footprint=row['Footprint'],Manufacturer=row['Manufacturer'],
                            **{'Manufacturer Part':row['MPN'],'LCSC Part #':row['Supplier_part_number']}))
            place(ref,ref,[center[0]+ox,center[1]+oy],part['angle'],method,'THT',part['pads'])
        if code=='A':
            sockets=read_csv(folder/'U1_socket_assembly_quote.csv')
            assert len(sockets)==2 and 'J8' not in native and 'J9' not in native
            for alias,start,end in [('J8',1,15),('J9',16,30)]:
                pads=sorted([p for p in native['U1']['pads'] if start<=int(p['number'])<=end],key=lambda p:int(p['number']))
                assert len(pads)==15 and all(abs(math.dist(a['at'],b['at'])-2.54)<1e-6 for a,b in zip(pads,pads[1:]))
                center=[sum(p['at'][i] for p in pads)/15 for i in (0,1)]
                a,b=pads[0]['at'],pads[-1]['at'];angle=math.degrees(math.atan2(-(b[1]-a[1]),b[0]-a[0]))
                assert sockets[0]['MPN']==sockets[1]['MPN']=='M20-7821546'
                bom.append(dict(Comment='Female socket 1x15 2.54mm 8.5mm high',Designator=alias,
                    Footprint='Harwin_M20-7821546_1x15_P2.54mm',Manufacturer='Harwin',
                    **{'Manufacturer Part':'M20-7821546','LCSC Part #':''}))
                place(alias,'U1',[center[0]+ox,center[1]+oy],angle,'Separate socket row pad centroid','THT',pads)
            assert math.isclose(abs(mapping[-1]['Local_Y_mm']-mapping[-2]['Local_Y_mm']),25.4,abs_tol=1e-6)
        assert len(bom)==len(cpl)==expected_smt+expected_tht
        refs={r['Designator'] for r in bom};assert refs=={r['Designator'] for r in cpl} and len(refs)==len(bom)
        expected=set(native)-{ref for ref in native if ref.startswith('TP')}-({'U1'} if code=='A' else {'NT1'})|({'J8','J9'} if code=='A' else set())
        assert refs==expected
        assert all(0<=r['Local_X_mm']<=board['size_mm'][0] and 0<=r['Local_Y_mm']<=board['size_mm'][1] for r in mapping)
        upload=OUT/('upload_'+code)
        write_csv(upload/(prefix+'_BOM_ALL_ASSEMBLED.csv'),bom,BOM_FIELDS)
        write_csv(upload/(prefix+'_CPL_ALL_ASSEMBLED.csv'),cpl,CPL_FIELDS)
        gerbers=folder/'gerbers_DRAFT_NOT_FOR_ORDER'
        check_probe_mask(gerbers,probes['pcb_a_display' if code=='A' else 'pcb_b_temperature'])
        paths=[p for p in sorted(gerbers.iterdir()) if p.suffix in ('.gtl','.gbl','.gts','.gbs','.gto','.gbo','.gtp','.gbp','.gm1','.drl')]
        assert len(paths)==11 and sum(p.suffix=='.drl' for p in paths)==2
        write(upload/(prefix+'_GERBER_DFM_ONLY.zip'),zip_bytes({p.name:source(p) for p in paths}))
        mappings.extend(mapping);all_boms.extend((code,r) for r in bom)
        counts[code]=dict(quantity=5,SMT_per_board=expected_smt,THT_per_board=expected_tht,
                          physical_placements_per_board=len(bom),placements_in_lot=5*len(bom))
        placement_preview(board,mapping,OUT/'support'/('placement_'+code+'_REFERENCE.png'))
    write_csv(OUT/'support/upload_to_native_mapping.csv',mappings,list(mappings[0]))
    missing={}
    for code,row in all_boms:
        if not row['LCSC Part #']:
            key=row['Manufacturer Part'];entry=missing.setdefault(key,dict(Manufacturer=row['Manufacturer'],MPN=key,
                References=[],Net_quantity_for_lot=0,Assembly='FIT',Status='EXACT_SOURCING_REQUIRED_NOT_DNP'))
            entry['References'].append(code+'/'+row['Designator']);entry['Net_quantity_for_lot']+=5
    loose=read_csv(M/'review_A04_B05/loose_accessories.csv');assert len(loose)==1 and loose[0]['MPN']=='M7566-05'
    missing['M7566-05']=dict(Manufacturer='Harwin',MPN='M7566-05',References=['A/JP1_SHUNT'],Net_quantity_for_lot=5,
                           Assembly='SUPPLY_LOOSE_NOT_ON_CPL',Status='EXACT_SOURCING_REQUIRED')
    assert set(missing)=={'M20-7821546','M20-9990246','M7566-05','EEUFR1V331UB','WSL1206R0560FEA'}
    for entry in missing.values():entry['References']=' '.join(entry['References'])
    write_csv(OUT/'support/exact_sourcing_REQUIRED.csv',list(missing.values()),list(next(iter(missing.values()))))
    copies={
        'output/pdf/assembly_review_A04_B05.pdf':'assembly_review_A04_B05.pdf',
        'output/assembly_pad_map.csv':'assembly_pad_map.csv',
        'output/assembly_THT_placement.csv':'assembly_THT_placement_NATIVE.csv',
        'component_selection.json':'component_selection.json','fabrication_specification.json':'fabrication_specification.json',
        'system_bom_review.csv':'customer_purchase_list.csv',
        'customer_supply_policy.json':'customer_supply_policy.json',
        'land_pattern_review.json':'land_pattern_review.json','process_requirements.json':'process_requirements.json',
        'factory_acceptance_test.md':'factory_acceptance_test.md','factory_response.csv':'factory_response_REQUIRED.csv',
        'panel_B_proposal.svg':'panel_B_PROPOSAL_ONLY.svg'}
    for relative,target in copies.items():write(OUT/'support'/target,source(M/relative))
    # Customer harness guidance is separate from the assembler's BOM/CPL.
    customer_harnesses=source(M/'wiring_and_harnesses.md').decode('utf-8')
    customer_harnesses=customer_harnesses.replace('(README.md#achats-à-notre-charge-et-liens)',
        '(../README.md)').replace('(system_bom_review.csv)','(customer_purchase_list.csv)')
    write(OUT/'support/customer_harnesses.md',customer_harnesses.encode('utf-8'))
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        write(OUT/'support'/('schematic_'+code+'.pdf'),source(ROOT/'review'/(name+'.pdf')))
        write(OUT/'support'/('bare_board_test_'+code+'.d356'),source(M/'review_A04_B05'/code/'bare_board_test.d356'))
        write(OUT/'support'/('test_points_'+code+'_back.png'),source(ROOT/'review'/('test_points_'+code+'_back.png')))
    for path,target in [(ROOT/'TEST_POINTS.md','TEST_POINTS.md'),(ROOT/'UART_EXTENSION.md','UART_EXTENSION.md'),(ROOT/'test_points.json','test_points.json'),
                        (ROOT/'review/test_point_map.csv','test_point_map.csv'),
                        (ROOT/'../firmware/diagnostic/README.md','diagnostic_firmware_README.md'),
                        (ROOT/'review/routing/electronics_bench_plan.md','electronics_bench_plan.md'),
                        (ROOT/'manufacturing/power_and_usb_review.md','power_and_usb_review.md')]:
        blob=source(path)
        if target=='TEST_POINTS.md':
            text=blob.decode('utf-8').replace('(review/test_points_','(test_points_').replace('(review/test_point_map.csv)','(test_point_map.csv)')
            text=text.replace('(../firmware/diagnostic/README.md)','(diagnostic_firmware_README.md)').replace('(review/routing/electronics_bench_plan.md)','(electronics_bench_plan.md)')
            blob=text.encode('utf-8')
        write(OUT/'support'/target,blob)
    validation=dict(status='PASS_LOCAL_PACKAGE / RFQ_DFM_ONLY',boards=counts,total_placements_for_5A_5B=360,
        additional_loose_shunts=5,SMT_and_THT_included=True,source_files_modified=False,
        excluded=dict(A_U1='Customer removable ESP32; replaced by two physical socket upload aliases J8/J9',
                      B_NT1='Copper only; not a purchased component'),
        missing_exact_catalogue_MPNs=sorted(missing),JLCPCB_import_tested=False,
        factory_placement_approved=False,pre_sales_DFM_obtained=False,stock_confirmed=False,
        uploaded=False,support_message_sent=False,order_authorized=False,order_placed=False,
        B_panel_generated=False,B_Standard_panel_review_required=True)
    validation['copper_testpads_excluded_from_purchasing_and_placement']=22
    validation['actual_back_mask_testpad_openings_verified']=22
    write(OUT/'validation.json',json_bytes(validation))
    for path in OUTPUT_FILES:
        if path.suffix=='.csv':
            rows=list(csv.DictReader(io.StringIO(path.read_bytes().decode('utf-8-sig'))));assert rows
        if path.suffix=='.zip':
            with zipfile.ZipFile(path) as z:assert z.testzip() is None
    for name in ('README.md','request_support_EN.txt','official_sources.json'):OUTPUT_FILES.add(OUT/name)
    manifest=dict(status=validation['status'],date='2026-11-10',workspace_sources_sha256=SOURCE_HASHES,
        files_sha256={p.relative_to(OUT).as_posix():sha(p.read_bytes()) for p in sorted(OUTPUT_FILES)},
        counts=counts,order_authorized=False,order_placed=False)
    (OUT/'package_manifest.json').write_bytes(json_bytes(manifest))
    files={p.relative_to(OUT).as_posix():p.read_bytes() for p in OUTPUT_FILES}
    files['package_manifest.json']=(OUT/'package_manifest.json').read_bytes()
    ARCHIVE.write_bytes(zip_bytes(files))
    ARCHIVE.with_suffix('.zip.sha256').write_text(sha(ARCHIVE.read_bytes())+'  '+ARCHIVE.name+'\n',encoding='ascii')
    check()


def check():
    with zipfile.ZipFile(ARCHIVE) as z:
        assert z.testzip() is None
        manifest=json.loads(z.read('package_manifest.json'))
        assert len(z.namelist())==len(set(z.namelist()))==len(manifest['files_sha256'])+1
        assert set(z.namelist())==set(manifest['files_sha256'])|{'package_manifest.json'}
        for relative,digest in manifest['workspace_sources_sha256'].items():assert sha((ROOT/relative).read_bytes())==digest,'Stale source '+relative
        for name,digest in manifest['files_sha256'].items():
            assert sha(z.read(name))==sha((OUT/name).read_bytes())==digest,name
        assert z.read('package_manifest.json')==(OUT/'package_manifest.json').read_bytes()
        for code,prefix in [('A','A04'),('B','B05')]:
            def rows(kind):return list(csv.DictReader(io.StringIO(z.read(f'upload_{code}/{prefix}_{kind}_ALL_ASSEMBLED.csv').decode('utf-8-sig'))))
            bom,cpl=rows('BOM'),rows('CPL');assert set(bom[0])==set(BOM_FIELDS) and set(cpl[0])==set(CPL_FIELDS)
            assert len(bom)==len(cpl)==manifest['counts'][code]['physical_placements_per_board']
            assert len({r['Designator'] for r in bom})==len(bom)
            assert {r['Designator'] for r in bom}=={r['Designator'] for r in cpl}
            assert all(r['Layer']=='Top' and 0<=float(r['Rotation'])<360 for r in cpl)
            with zipfile.ZipFile(io.BytesIO(z.read(f'upload_{code}/{prefix}_GERBER_DFM_ONLY.zip'))) as g:
                assert g.testzip() is None and len(g.namelist())==11
        checksum=ARCHIVE.with_suffix('.zip.sha256').read_text(encoding='ascii').split()[0]
        assert sha(ARCHIVE.read_bytes())==checksum
        assert not manifest['order_authorized'] and not manifest['order_placed']
    print(json.dumps(dict(status='PASS',archive=str(ARCHIVE),members=len(manifest['files_sha256'])+1,
                          placements=360,loose_shunts=5,upload_or_order_performed=False),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true');args=parser.parse_args()
    check() if args.check else build()
