"""Build a local RFQ/review package, never a fabrication release.

Run with KiCad's Python. Read native boards and fresh exported netlists; verify
pad connectivity and source hashes. Frozen prototype MPNs are engineering
selections, not live JLC/LCSC stock assertions or factory approval. U1 on A is a
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
import shutil

import pcbnew
from export_bare_board_test import export as export_test_netlist
from verify_fabrication_outputs import verify as verify_fabrication
from render_gerber_review import render as render_gerbers

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'manufacturing' / 'review_A04_B05'
CLI = Path('C:/Program Files/KiCad/10.0/bin/kicad-cli.exe')
DEVKIT = json.loads((ROOT/'module_selection.json').read_text(encoding='utf-8'))['devkit']
SUPPLIER = json.loads((ROOT/'manufacturing/supplier_review.json').read_text(encoding='utf-8'))
SUPPLIER_BY_MPN = {r['requested_mpn']:r for r in SUPPLIER['records'] if r['identity_verified']}
SELECTION = json.loads((ROOT/'manufacturing/component_selection.json').read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selection(board, ref):
    row=SELECTION['boards'][board][ref]
    return {key:row[key] for key in ('MPN','Manufacturer','Requirements','Source')} | {'Evidence':row['Decision']}


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
    # Never attach an old drawing to newly exported Gerbers or procurement data.
    drawings=ROOT/'manufacturing/output/assembly_review_manifest.json'
    drawing_review=json.loads(drawings.read_text(encoding='utf-8'))
    for section in ('source_sha256','output_sha256'):
        for relative,digest in drawing_review[section].items():
            if sha(ROOT/relative)!=digest:
                raise ValueError('Assembly drawing is stale: '+relative)
    lands=ROOT/'review/routing/manufacturer_land_patterns.json'
    for relative,digest in json.loads(lands.read_text(encoding='utf-8'))['source_sha256'].items():
        if sha(ROOT/relative)!=digest:
            raise ValueError('Manufacturer land-pattern check is stale: '+relative)
    parts=ROOT/'review/routing/component_selection_validation.json'
    for relative,digest in json.loads(parts.read_text(encoding='utf-8'))['source_sha256'].items():
        if sha(ROOT/relative)!=digest:raise ValueError('Part-selection check is stale: '+relative)
    manifest=dict(status='DRAFT / RFQ review only; NOT RELEASED FOR FABRICATION',
                  sources_sha256={}, boards={}, automatic_checks=[],
                  limits=['Public supplier catalogue identities checked; live order stock and all-in prices remain unconfirmed.',
                          'Exact prototype MPNs are frozen in native schematics/PCBs; sourcing, price and assembly-house approval remain external.',
                          'No placement rotations approved by assembly house; THT requires explicit quotation.',
                          'USB/external simultaneous operation remains unqualified.'])
    for source in (Path(__file__), ROOT/'scripts/verify_manufacturing_review.py', ROOT/'interface_ab.json',ROOT/'module_selection.json',
                   ROOT/'manufacturing/power_and_usb_review.md', ROOT/'manufacturing/README.md',
                   ROOT/'manufacturing/release_checklist.json',ROOT/'manufacturing/system_bom_review.csv',
                   ROOT/'manufacturing/customer_supply_policy.json',
                   ROOT/'manufacturing/supplier_candidates.json',ROOT/'manufacturing/supplier_review.json',
                   ROOT/'manufacturing/process_requirements.json',ROOT/'manufacturing/manufacturing_fit_audit.json',
                   ROOT/'manufacturing/assembly_RFQ_draft.md',ROOT/'manufacturing/usb_service_test.md',
                   ROOT/'manufacturing/panel_B_proposal.svg',ROOT/'devkit_compatibility.md',
                   ROOT/'manufacturing/land_pattern_review.json', ROOT/'manufacturing/component_selection.json', lands, drawings,
                   ROOT/'scripts/build_assembly_review.py', ROOT/'scripts/check_manufacturer_land_patterns.py',
                   ROOT/'lab_hardware.pretty/Vishay_WSL1206_6m_to_200m.kicad_mod',
                   ROOT/'lab_hardware.pretty/ESP32_30P_Harwin_M20_7821546_Rows25.40mm.kicad_mod',
                   ROOT/'lab_hardware.pretty/LED_Kingbright_APT2012_Manufacturer.kicad_mod',
                   ROOT/'scripts/export_bare_board_test.py',ROOT/'scripts/verify_fabrication_outputs.py',
                   ROOT/'scripts/render_gerber_review.py',ROOT/'scripts/check_component_selection.py',parts,
                   ROOT/'manufacturing/fabrication_specification.json',ROOT/'manufacturing/engineering_review.md',
                   ROOT/'manufacturing/wiring_and_harnesses.md',ROOT/'manufacturing/factory_acceptance_test.md',
                   ROOT/'manufacturing/harness_pin_map.csv',ROOT/'manufacturing/harness_validation.json',
                   ROOT/'scripts/build_harness_map.py',ROOT/'scripts/prepare_fabrication.py',
                   ROOT/'manufacturing/START_HERE.md',ROOT/'manufacturing/factory_response.csv',
                   ROOT/'pcb_a_display/fp-lib-table',ROOT/'pcb_b_temperature/fp-lib-table',
                   ROOT/'test_points.json',ROOT/'TEST_POINTS.md',ROOT/'UART_EXTENSION.md',ROOT/'scripts/check_uart_extension.py',ROOT/'review/routing/uart_extension_validation.json',ROOT/'scripts/check_test_points.py',
                   ROOT/'review/routing/test_points_validation.json',ROOT/'review/diagnostic_firmware_validation.json'):
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
            fields={f.get('name'):f.text or '' for f in comp.findall('./fields/field')}
            for key in ('MPN','Manufacturer'):
                if fields.get(key,'')!=p[key] or fp.GetFieldText(key)!=p[key]:raise ValueError('Frozen native part identity mismatch '+code+'/'+ref+'/'+key)
            supplier=SUPPLIER_BY_MPN.get(p['MPN'],{})
            assembly=('NONE_COPPER' if ref=='NT1' or ref.startswith('TP') else 'CUSTOMER_PLUG_IN' if code=='A' and ref=='U1'
                      else 'FACTORY_THT_QUOTE' if any(pad.GetDrillSize().x for pad in fp.Pads()) else 'FACTORY_SMT_QUOTE')
            rows.append(dict(Board=code,Reference=ref,Quantity=1,Value=comp.findtext('value'),
                             Footprint=comp.findtext('footprint'),Assembly=assembly,**p,
                             Approval='ENGINEERING SELECTED / ASSEMBLER APPROVAL BEFORE BUILD',Supplier_part_number=supplier.get('supplier_id',''),
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
        export_test_netlist(code,name,folder)
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
                dict(Assembly_item='U1_SOCKET_BOTTOM',Quantity=1,MPN='M20-7821546',Manufacturer='Harwin',
                     Pads='U1.1..15',Pitch_mm=2.54,Body_height_mm=8.5,Row_spacing_mm=25.4,
                     Status='Selected: holes1.05mm, pitch2.54mm, rows25.4mm. Harwin body8.5mm high; OPEN: check actual insertion/body variants at integration',
                     Source='https://www.harwin.com/products/M20-7821546'),
                dict(Assembly_item='U1_SOCKET_TOP',Quantity=1,MPN='M20-7821546',Manufacturer='Harwin',
                     Pads='U1.16..30',Pitch_mm=2.54,Body_height_mm=8.5,Row_spacing_mm=25.4,
                     Status='Selected: holes1.05mm, pitch2.54mm, rows25.4mm. Harwin body8.5mm high; OPEN: check actual insertion/body variants at integration',
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
    fabrication=verify_fabrication()
    render_gerbers()
    write_csv(OUT/'loose_accessories.csv',[dict(Board='A',Reference='JP1_SHUNT',Quantity=1,MPN='M7566-05',Manufacturer='Harwin',Assembly='SUPPLY_LOOSE_NOT_ON_CPL')])
    manifest['fabrication_output_checks']=fabrication['boards']
    manifest['automatic_checks']=['fresh ERC/DRC for both boards', 'native board / netlist ref, value, footprint and pad/net parity',
                                    'all native references including copper-only test pads accounted for', 'SMT BOM and CPL exact reference matching',
                                    'NT1 excluded from purchasing/placement; DevKit separated from two physical sockets',
                                    'actual Excellon holes and Gerber outlines/paste coordinates compared with native geometry',
                                    'IPC-D-356 test netlists exported; only B NT1 ground equivalence normalized',
                                    'actual Gerbers independently parsed and rendered by PyGerber2.4.3']
    manifest['supplier_review']=dict(exact_catalogue_MPNs=SUPPLIER['verified_exact_MPNs'],
        live_stock_confirmed=SUPPLIER['live_stock_confirmed_MPNs'],
        unverified_rows=[r['requested_mpn'] for r in SUPPLIER['records'] if not r['identity_verified']],
        unmatched_MPNs=list(SUPPLIER['unresolved_exact_MPNs']),
        engineering_and_order_approval=False)
    assembly_folder=OUT/'assembly_review'
    assembly_folder.mkdir(exist_ok=True)
    for relative in drawing_review['output_sha256']:
        shutil.copy2(ROOT/relative, assembly_folder/Path(relative).name)
    shutil.copy2(drawings,assembly_folder/drawings.name)
    shutil.copy2(lands,assembly_folder/lands.name)
    manifest['assembly_drawings']=dict(pages=drawing_review['pages'],
        status=drawing_review['status'],physical_THT_items=21,
        coordinate_map='assembly_review/assembly_pad_map.csv',
        THT_placement='assembly_review/assembly_THT_placement.csv')
    manifest['output_sha256']={p.relative_to(OUT).as_posix():sha(p) for p in sorted(OUT.rglob('*'))
                               if p.is_file() and p.name!='manifest.json' and p.suffix!='.zip'}
    attachments=[OUT.parent/name for name in (
        'README.md','power_and_usb_review.md','release_checklist.json','system_bom_review.csv','customer_supply_policy.json',
        'supplier_candidates.json','supplier_review.json','supplier_review.csv','process_requirements.json',
        'manufacturing_fit_audit.json','assembly_RFQ_draft.md','usb_service_test.md','panel_B_proposal.svg',
        'land_pattern_review.json','component_selection.json','fabrication_specification.json','engineering_review.md',
        'wiring_and_harnesses.md','harness_pin_map.csv','harness_validation.json','factory_acceptance_test.md',
        'fabrication_output_validation.json','START_HERE.md','factory_response.csv')]+[ROOT/'devkit_compatibility.md',ROOT/'TEST_POINTS.md',ROOT/'UART_EXTENSION.md',ROOT/'test_points.json',
        ROOT/'review/test_point_map.csv',ROOT/'review/test_points_A_back.png',ROOT/'review/test_points_B_back.png']
    manifest['attachments_sha256']={p.name:sha(p) for p in attachments}
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    archive=OUT.parent/'RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted(OUT.rglob('*')):
            if p.is_file():
                bundle.write(p,p.relative_to(OUT))
        for p in attachments:
            bundle.write(p,p.name)
    print(json.dumps(dict(status=manifest['status'],boards=manifest['boards'],archive=str(archive)),indent=2))


if __name__=='__main__':
    main()
