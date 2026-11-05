"""Build a local RFQ/review package, never a fabrication release.

Run with KiCad's Python. Read native boards and fresh exported netlists; verify
pad connectivity and source hashes. MPNs below are procurement candidates,
not approved substitutions or live JLC/LCSC stock assertions. U1 on A is a
customer-supplied DevKit: quote TWO soldered sockets, not one soldered module.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import zipfile

import pcbnew

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'manufacturing' / 'review_A04_B05'
CLI = Path('C:/Program Files/KiCad/10.0/bin/kicad-cli.exe')
JST = 'https://www.jst-mfg.com/product/pdf/eng/eXH.pdf'
RC = 'https://www.yageogroup.com/component-documentation/download/specsheet/'
DEVKIT = json.loads((ROOT/'module_selection.json').read_text(encoding='utf-8'))['devkit']
SUPPLIER = json.loads((ROOT/'manufacturing/supplier_review.json').read_text(encoding='utf-8'))
SUPPLIER_BY_MPN = {r['requested_mpn']:r for r in SUPPLIER['records'] if r['identity_verified']}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def part(mpn, manufacturer, specs, source, evidence='existing design choice'):
    return dict(MPN=mpn, Manufacturer=manufacturer, Requirements=specs,
                Source=source, Evidence=evidence)


CAP1 = part('C1608X7R1C105K080AC', 'TDK', '1uF 16V X7R 10% 0603; effective capacitance review required',
            'https://product.tdk.com/en/search/capacitor/ceramic/mlcc/info?part_no=C1608X7R1C105K080AC', 'manufacturer specification checked')
CAP100 = part('C0603C104K4RACTU', 'KEMET / Yageo Group', '100nF 16V X7R 10% 0603',
              'https://yageogroup.com/download/specsheet/C0603C104K4RACTU', 'manufacturer specification checked')
GREEN = part('APT2012LZGCK', 'Kingbright', 'green low-current LED 0805; cathode pad 1',
             'https://www.kingbrightusa.com/images/catalog/spec/apt2012lzgck.pdf')
LDO = part('AP2112K-3.3TRG1', 'Diodes Incorporated', '3.3V SOT25 / SOT-23-5, pin5 output',
           'https://www.diodes.com/assets/Datasheets/AP2112.pdf', 'manufacturer ordering table checked')


def resistor(value, size='0603'):
    codes = {'10k':'10K', '4.7k':'4K7', '1.5k':'1K5', '470R':'470R',
             '47k':'47K', '22R':'22R', '100k':'100K', '8.45k':'8K45',
             '1k':'1K', '15k':'15K', '2.2k':'2K2'}
    mpn = f'RC{size}FR-07{codes[value]}L'
    return part(mpn, 'YAGEO', f'{value} 1% {size} >= '+('0.125W' if size=='0805' else '0.1W'),
                RC+mpn, 'manufacturer ordering-code candidate; exact spec/supply confirmation required')


def selection(board, ref):
    if ref.startswith('J') and ref != 'JP1':
        count = ({'J1':2,'J2':4,'J3':4,'J4':4,'J5':4,'J6':3,'J7':6} if board=='A'
                 else {'J1':2,'J4':2,'J6':3,'J7':6}).get(ref)
        if count:
            return part(f'B{count}B-XH-A(LF)(SN)', 'JST', f'{count}P 2.50mm vertical THT, no locating boss', JST)
        return part('1715721', 'Phoenix Contact', 'MKDS 1,5/2-5,08 THT terminal; heater current to qualify',
                    'https://www.phoenixcontact.com/en-gb/products/printed-circuit-board-terminal-mkds-15-2-508-1715721')
    if ref == 'JP1':
        return part('M20-9990246', 'Harwin', '2P 2.54mm THT male header H8.64mm; separate M7566-05 shunt',
                    'https://www.harwin.com/products/M20-9990246', 'manufacturer specification checked; mechanical approval pending')
    if board == 'A':
        if ref in ('C1','C2'):
            return CAP1
        if ref == 'C3':
            return part('C1608X5R1A106K080AC','TDK','10uF 10V X5R 10% 0603; verify DC bias',
                        'https://product.tdk.com/en/search/capacitor/ceramic/mlcc/info?part_no=C1608X5R1A106K080AC', 'manufacturer specification checked')
        if ref in ('D1','D2'):
            return GREEN
        if ref.startswith('R'):
            return resistor({'R1':'10k','R2':'10k','R3':'10k','R4':'4.7k','R5':'4.7k','R6':'1.5k','R7':'470R'}[ref])
        if ref == 'U2':
            return LDO
        if ref == 'U1':
            return part('diymore / ASIN B0DGLCWR76','diymore',
                        'customer-supplied removable 30P DevKit; row spacing '+str(DEVKIT['row_spacing_mm'])+' mm; other fit checks OPEN',
                        'https://www.amazon.fr/dp/B0DGLCWR76',DEVKIT['row_spacing_status'])
    else:
        if ref in ('C1','C2','C7'):
            return CAP1  # 16 V rating also covers the existing 10 V minimum for C2.
        if ref in ('C3','C4','C5','C6'):
            return CAP100
        if ref in ('D5','D6','D7'):
            return GREEN
        if ref.startswith('R') and ref != 'R13':
            return resistor({'R1':'47k','R2':'47k','R3':'22R','R4':'100k','R5':'8.45k',
                             'R6':'1k','R7':'10k','R8':'100k','R9':'15k','R10':'1.5k',
                             'R11':'470R','R12':'2.2k'}[ref], '0805' if ref=='R9' else '0603')
        special = {
            'C8':part('EEUFR1H101','Panasonic','100uF 50V D8 P3.5 H11.5mm; polarity', 'https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFR1H101'),
            'C9':part('C0805C104K5RACTU','KEMET / Yageo Group','100nF 50V X7R 10% 0805 Hmax0.88mm','https://yageogroup.com/download/specsheet/C0805C104K5RACTU','manufacturer specification checked; new purchasing candidate'),
            'C10':part('EEUFR1V331UB','Panasonic','330uF 35V D10 P5 H12.5mm; polarity; retain specified ESR/damping','https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFR1V331UB'),
            'D2':part('SMBJ24CA','Littelfuse','bidirectional TVS SMB; upstream transients NOT qualified','https://www.littelfuse.com/products/tvs-diodes/surface-mount/smbj'),
            'D3':part('BZT52C12-7-F','Diodes Incorporated','12V zener SOD123; gate-source polarity; exact ordering suffix to confirm','https://www.diodes.com/part/view/BZT52C12','ordering candidate; exact spec/supply confirmation required'),
            'D4':part('STPS5L60U','STMicroelectronics','60V 5A SMB Schottky; cathode pad1','https://www.st.com/resource/en/datasheet/stps5l60.pdf'),
            'D8':part('APT2012LSECK/J4-PRV','Kingbright','orange low-current LED 0805; cathode pad1','https://www.kingbrightusa.com/images/catalog/spec/apt2012lseck-j4-prv.pdf'),
            'D9':part('SS36-E3/57T','Vishay','60V 3A SMC Schottky; cathode pad1','https://www.vishay.com/docs/88751/ss32.pdf'),
            'F1':part('045106.3MRL','Littelfuse','6.3A 125VDC NANO2 451 fast fuse; actual fault clearing to qualify','https://www.littelfuse.com/assetdocs/fuse-451-and-453-datasheet?assetguid=533cd5cc-956c-4243-867f-6ab5a62f6ba1'),
            'F2':part('0451001.MRL','Littelfuse','1A 125VDC NANO2 451 fast fuse; startup coordination to qualify','https://www.littelfuse.com/assetdocs/fuse-451-and-453-datasheet?assetguid=533cd5cc-956c-4243-867f-6ab5a62f6ba1'),
            'L1':part('SRP1770TA-101M','Bourns','100uH 20% DCRmax118mOhm; H7mm; Isat1=6.5A at25C, Irms=5.3A','https://www.bourns.com/docs/Product-Datasheets/srp1770ta.pdf','manufacturer specification checked'),
            'NT1':part('','','copper net tie; no physical component; never populate','', 'virtual copper'),
            'Q1':part('IRLZ44NPBF','Infineon','TO220 vertical G-D-S; no heatsink included; heater thermal test required','https://www.infineon.com/part/IRLZ44N'),
            'Q2':part('DMP6023LE-13','Diodes Incorporated','SOT223 G=1 D=2/tab S=3 P-channel 60V','https://www.diodes.com/datasheet/download/DMP6023LE.pdf'),
            'R13':part('WSL1206R0560FEA','Vishay Dale','56mOhm 1% 0.25W 1206 in C10 return, not load return','https://www.vishay.com/docs/30100/wsl.pdf','manufacturer family / ordering code checked; new purchasing candidate'),
            'U1':part('LM2596S-5.0/NOPB','Texas Instruments','fixed5V TO263-5; pin3/tab PGND; do not substitute ADJ or TO220','https://www.ti.com/lit/ds/symlink/lm2596.pdf'),
            'U2':LDO,
            'U3':part('SN74LVC1G08DBVR','Texas Instruments','AND SOT23-5 DBV; 3.3V','https://www.ti.com/lit/ds/symlink/sn74lvc1g08.pdf'),
            'U4':part('ADS1115IDGSR','Texas Instruments','VSSOP10 DG S / KiCad TSSOP10 3x3 P0.5; ADDR=VDD ->0x49','https://www.ti.com/lit/ds/symlink/ads1115.pdf'),
            'U5':part('UCC27517DBVR','Texas Instruments','gate driver SOT23-5 DBV; 5V; verify IN- pin4 PGND','https://www.ti.com/lit/ds/symlink/ucc27517.pdf'),
        }
        if ref in special:
            return special[ref]
    raise ValueError(f'No purchasing decision for {board}/{ref}')


def write_csv(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer=csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run(*args):
    result=subprocess.run([str(CLI), *map(str,args)], capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(result.stdout+result.stderr)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    manifest=dict(status='DRAFT / RFQ review only; NOT RELEASED FOR FABRICATION',
                  sources_sha256={}, boards={}, automatic_checks=[],
                  limits=['Public supplier catalogue identities checked; live order stock and all-in prices remain unconfirmed.',
                          'MPNs are external review candidates, not yet approved native schematic fields.',
                          'No placement rotations approved by assembly house; THT requires explicit quotation.',
                          'USB/external simultaneous operation remains unqualified.'])
    for source in (Path(__file__), ROOT/'interface_ab.json',ROOT/'module_selection.json',
                   ROOT/'manufacturing/power_and_usb_review.md', ROOT/'manufacturing/README.md',
                   ROOT/'manufacturing/release_checklist.json',ROOT/'manufacturing/system_bom_review.csv',
                   ROOT/'manufacturing/supplier_candidates.json',ROOT/'manufacturing/supplier_review.json',
                   ROOT/'manufacturing/process_requirements.json',ROOT/'manufacturing/manufacturing_fit_audit.json',
                   ROOT/'manufacturing/assembly_RFQ_draft.md',ROOT/'manufacturing/usb_service_test.md',
                   ROOT/'manufacturing/panel_B_proposal.svg'):
        manifest['sources_sha256'][source.relative_to(ROOT).as_posix()]=sha(source)
    native_hashes={p:sha(p) for name in ('pcb_a_display','pcb_b_temperature')
                   for suffix in ('.kicad_sch','.kicad_pcb','.kicad_pro')
                   for p in [ROOT/name/(name+suffix)]}
    contract=json.loads((ROOT/'interface_ab.json').read_text(encoding='utf-8'))
    if contract['power_voltage_V']!=5 or contract['logic_voltage_V']!=3.3:
        raise ValueError('Review targets the 5 V / 3.3 V A/B contract')
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        folder=OUT/code
        folder.mkdir(exist_ok=True)
        sch=ROOT/name/(name+'.kicad_sch')
        pcb=ROOT/name/(name+'.kicad_pcb')
        net=folder/'netlist.xml'
        run('sch','export','netlist','--format','kicadxml','--output',net,sch)
        run('sch','erc','--output',folder/'erc.rpt','--exit-code-violations',sch)
        run('pcb','drc','--output',folder/'drc.json','--format','json','--exit-code-violations',pcb)
        drc=json.loads((folder/'drc.json').read_text(encoding='utf-8'))
        if any(drc.get(key) for key in ('violations','unconnected_items','schematic_parity')):
            raise ValueError('DRC/parity issue on '+code)
        root=ET.parse(net).getroot()
        components={c.attrib['ref']:c for c in root.findall('./components/comp')}
        board=pcbnew.LoadBoard(str(pcb))
        footprints={f.GetReference():f for f in board.GetFootprints()}
        if set(components)!=set(footprints):
            raise ValueError('Netlist / board component mismatch')
        pins={(n.attrib['ref'],n.attrib['pin']):g.attrib['name'].removeprefix('/')
              for g in root.findall('./nets/net') for n in g.findall('node')}
        rows=[]
        for ref,comp in components.items():
            fp=footprints[ref]
            if comp.findtext('footprint')!=fp.GetFPID().GetUniStringLibId() or comp.findtext('value')!=fp.GetValue():
                raise ValueError('Value / footprint mismatch '+ref)
            for pad in fp.Pads():
                if pad.GetNumber() and pins.get((ref,pad.GetNumber()))!=pad.GetNetname().removeprefix('/'):
                    raise ValueError('Pad/net mismatch '+ref+'.'+pad.GetNumber())
            p=selection(code,ref)
            supplier=SUPPLIER_BY_MPN.get(p['MPN'],{})
            assembly=('NONE_COPPER' if ref=='NT1' else 'CUSTOMER_PLUG_IN' if code=='A' and ref=='U1'
                      else 'FACTORY_THT_QUOTE' if any(pad.GetDrillSize().x for pad in fp.Pads()) else 'FACTORY_SMT_QUOTE')
            rows.append(dict(Board=code,Reference=ref,Quantity=1,Value=comp.findtext('value'),
                             Footprint=comp.findtext('footprint'),Assembly=assembly,**p,
                             Approval='OPEN / NOT RELEASED',Supplier_part_number=supplier.get('supplier_id',''),
                             Availability=supplier.get('stock_status','UNMATCHED / quotation required'),Price='NOT QUOTED',
                             Catalogue_identity='EXACT MPN AND MANUFACTURER VERIFIED' if supplier else 'NOT VERIFIED',
                             Catalogue_source=supplier.get('source_url',''),
                             Supplier_assembly_mode=supplier.get('assembly_mode','')))
        for port in contract['connectors']:
            for pin in port['pins']:
                if code=='B' and not pin.get('connected_on_b',True):
                    continue
                if pins[(port['reference'],str(pin['pin']))]!=pin['net']:
                    raise ValueError('A/B electrical contract mismatch')
        write_csv(folder/'procurement_review.csv',rows)
        # SMT-only BOM and CPL have EXACTLY the same references. Net ties are
        # sometimes marked SMD in KiCad; exclude them explicitly after export.
        smt=[r for r in rows if r['Assembly']=='FACTORY_SMT_QUOTE']
        write_csv(folder/'BOM_SMT_RFQ_NOT_FOR_ORDER.csv',[
            {'Comment':r['Value'],'Designator':r['Reference'],'Footprint':r['Footprint'],
             'Manufacturer':r['Manufacturer'],'Manufacturer Part':r['MPN'],'LCSC Part #':r['Supplier_part_number']} for r in smt])
        raw=folder/'positions_native.csv'
        run('pcb','export','pos','--format','csv','--units','mm','--side','both',
            '--smd-only','--exclude-fp-th','--exclude-dnp','--output',raw,pcb)
        with raw.open(encoding='utf-8-sig',newline='') as stream:
            native=list(csv.DictReader(stream))
        allowed={r['Reference'] for r in smt}
        positions=[{'Designator':r['Ref'],'Mid X':r['PosX'],'Mid Y':r['PosY'],
                    'Layer':{'top':'Top','bottom':'Bottom'}[r['Side']], 'Rotation':r['Rot']}
                   for r in native if r['Ref'] in allowed]
        if {r['Designator'] for r in positions}!=allowed or len(positions)!=len(allowed):
            raise ValueError('SMT BOM / CPL mismatch')
        for row in positions:
            f=footprints[row['Designator']]
            if abs(float(row['Mid X'])-pcbnew.ToMM(f.GetPosition().x))>1e-5 or abs(float(row['Mid Y'])+pcbnew.ToMM(f.GetPosition().y))>1e-5:
                raise ValueError('CPL coordinate convention mismatch')
        write_csv(folder/'CPL_SMT_RFQ_ROTATIONS_UNVERIFIED.csv',positions)
        tht=[r for r in rows if r['Assembly']=='FACTORY_THT_QUOTE']
        write_csv(folder/'THT_assembly_quote.csv',tht)
        gerbers=folder/'gerbers_DRAFT_NOT_FOR_ORDER'
        gerbers.mkdir(exist_ok=True)
        run('pcb','export','gerbers','--layers','F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts',
            '--subtract-soldermask','--check-zones','--output',str(gerbers)+'/',pcb)
        run('pcb','export','drill','--format','excellon','--drill-origin','absolute','--excellon-units','mm',
            '--excellon-separate-th','--generate-map','--map-format','pdf','--output',str(gerbers)+'/',pcb)
        # Extra physical items are not falsely assigned board centroid U1.
        if code=='A':
            pads={p.GetNumber():p.GetPosition() for p in footprints['U1'].Pads()}
            centers=[tuple(sum(getattr(pads[str(i)],axis) for i in group)/15 for axis in ('x','y'))
                     for group in (range(1,16),range(16,31))]
            native_spacing=pcbnew.ToMM(math.dist(*centers))
            if abs(native_spacing-DEVKIT['row_spacing_mm'])>1e-5:
                raise ValueError('DevKit footprint row spacing differs from recorded measurement')
            manifest['devkit_row_spacing']=dict(native_mm=round(native_spacing,5),
                recorded_mm=DEVKIT['row_spacing_mm'],status=DEVKIT['row_spacing_status'],
                evidence=DEVKIT.get('row_spacing_measurement'),other_fit_checks='OPEN')
            write_csv(folder/'U1_socket_assembly_quote.csv',[
                dict(Assembly_item='U1_SOCKET_BOTTOM',Quantity=1,MPN_candidate='M20-7821546',Manufacturer='Harwin',
                     Pads='U1.1..15',Pitch_mm=2.54,Body_height_mm=8.5,Row_spacing_mm=25.4,
                     Status='Row spacing25.40mm confirmed by user2026-11-05; OPEN: drill, header insertion and USB height',
                     Source='https://www.harwin.com/products/M20-7821546'),
                dict(Assembly_item='U1_SOCKET_TOP',Quantity=1,MPN_candidate='M20-7821546',Manufacturer='Harwin',
                     Pads='U1.16..30',Pitch_mm=2.54,Body_height_mm=8.5,Row_spacing_mm=25.4,
                     Status='Row spacing25.40mm confirmed by user2026-11-05; OPEN: drill, header insertion and USB height',
                     Source='https://www.harwin.com/products/M20-7821546')])
        for source in (sch,pcb,ROOT/name/(name+'.kicad_pro')):
            manifest['sources_sha256'][source.relative_to(ROOT).as_posix()]=native_hashes[source]
        # Edge_Cuts bounding boxes include drawing stroke width. Fabrication
        # dimensions are measured on the centerline, not that enlarged box.
        edges=[d for d in board.GetDrawings() if d.GetLayer()==pcbnew.Edge_Cuts]
        if len(edges)!=4 or any(d.GetShape()!=pcbnew.SHAPE_T_SEGMENT for d in edges):
            raise ValueError('Dimension extraction expects the current four straight outline segments')
        points=[point for edge in edges for point in (edge.GetStart(),edge.GetEnd())]
        dimensions=[round(pcbnew.ToMM(max(getattr(p,axis) for p in points)-min(getattr(p,axis) for p in points)),3)
                    for axis in ('x','y')]
        if dimensions!=([110,75] if code=='A' else [90,65]):
            raise ValueError('Unexpected board outline dimensions')
        manifest['boards'][code]=dict(native_references=len(rows), SMT=len(smt), THT_native=len(tht),
            extra_sockets=2 if code=='A' else 0, copper_only=1 if code=='B' else 0,
            plug_in_modules=1 if code=='A' else 0,
            dimensions_mm=dimensions,
            thickness_mm=pcbnew.ToMM(board.GetDesignSettings().GetBoardThickness()), copper_um=35 if code=='A' else 70,
            placement_convention='KiCad native absolute origin, mm, native rotation; assembly preview approval required',
            erc='0 errors / 0 warnings; see erc.rpt',drc='0 violations / 0 unconnected / 0 parity',
            SMT_BOM_CPL_reference_match=True)
    for source,digest in native_hashes.items():
        if sha(source)!=digest:
            raise ValueError('Native input changed during export: '+str(source))
    for relative,digest in manifest['sources_sha256'].items():
        if sha(ROOT/relative)!=digest:
            raise ValueError('Review input changed during export: '+relative)
    manifest['automatic_checks']=['fresh ERC/DRC for both boards', 'native board / netlist ref, value, footprint and pad/net parity',
                                    'all 70 native references accounted for', 'SMT BOM and CPL exact reference matching',
                                    'NT1 excluded from purchasing/placement; DevKit separated from two physical sockets']
    manifest['supplier_review']=dict(exact_catalogue_MPNs=SUPPLIER['verified_exact_MPNs'],
        live_stock_confirmed=SUPPLIER['live_stock_confirmed_MPNs'],
        unverified_rows=[r['requested_mpn'] for r in SUPPLIER['records'] if not r['identity_verified']],
        unmatched_MPNs=list(SUPPLIER['unresolved_exact_MPNs']),
        engineering_and_order_approval=False)
    manifest['output_sha256']={p.relative_to(OUT).as_posix():sha(p) for p in sorted(OUT.rglob('*'))
                               if p.is_file() and p.name!='manifest.json' and p.suffix!='.zip'}
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    archive=OUT.parent/'RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted(OUT.rglob('*')):
            if p.is_file():
                bundle.write(p,p.relative_to(OUT))
        for name in ('README.md','power_and_usb_review.md','release_checklist.json','system_bom_review.csv',
                     'supplier_candidates.json','supplier_review.json','supplier_review.csv',
                     'process_requirements.json','manufacturing_fit_audit.json','assembly_RFQ_draft.md','usb_service_test.md','panel_B_proposal.svg'):
            bundle.write(OUT.parent/name,name)
    print(json.dumps(dict(status=manifest['status'],boards=manifest['boards'],archive=str(archive)),indent=2))


if __name__=='__main__':
    main()
