"""Finish local preparation after real visual review; keep factory/order holds.

Standard Python. This packages source-bound evidence and editable native CAD;
it never invents an assembler response or grants purchase authorization.
"""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'manufacturing'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
    checks=read(FOLDER/'preparation_checks.json')
    assert len(checks['stages'])==13 and all(s['exit_code']==0 for s in checks['stages'])
    for relative,digest in checks['evidence_sha256'].items():assert sha(ROOT/relative)==digest,'Stale evidence '+relative
    visual=read(FOLDER/'visual_review.json');assert visual['status']=='PASS'
    required={'manufacturing/output/pdf/assembly_review_A04_B05.pdf',
              *('manufacturing/review_A04_B05/gerber_review/'+code+'_'+side+'.png' for code in 'AB' for side in ('front','back')),
              'mechanical/exports/preview_service.png','mechanical/exports/preview_b_power.png',
              'mechanical/exports/preview_light_sensor.png','mechanical/exports/preview_assembled.png'}
    assert required <= set(visual['reviewed_files_sha256'])
    for relative,digest in visual['reviewed_files_sha256'].items():assert sha(ROOT/relative)==digest,'Stale visual review '+relative
    rfq=FOLDER/'RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip';integrity=read(FOLDER/'rfq_integrity_validation.json')
    assert integrity['archive_sha256']==sha(rfq)
    routing=read(ROOT/'review/routing/routing_validation.json')
    assert all(not row[k] for row in routing['boards'] for k in ('DRC_violations','unconnected_items','schematic_parity_issues'))
    parameters=read(ROOT/'mechanical/enclosure_parameters.json');tag=parameters['export_tag']
    mechanical=read(ROOT/('mechanical/exports/source_manifest_'+tag+'.json'))
    assert mechanical['static_collisions']==mechanical['assembly_failures']==0
    for relative,digest in mechanical['source_sha256'].items():assert sha(ROOT/relative)==digest,'Stale mechanical source '+relative
    parts=read(ROOT/'review/routing/component_selection_validation.json');assert parts['physical_native_components']==68
    release=read(FOLDER/'release_checklist.json');assert not release['fabrication_release'] and not release['order_placed']
    files={};sources={}
    def add(path,target):
        path=path.resolve();assert path.is_relative_to(ROOT.resolve())
        assert target not in files,'Duplicate archive member '+target
        files[target]=path.read_bytes();sources[path.relative_to(ROOT).as_posix()]=sha(path)
    with zipfile.ZipFile(rfq) as bundle:
        assert bundle.testzip() is None
        for name in bundle.namelist():files['rfq_manifest.json' if name=='manifest.json' else name]=bundle.read(name)
    for name in ('README.md','interface_ab.json','module_selection.json','placement.json','devkit_compatibility.md','display_backlight_study.md','lab_draft.kicad_sym'):
        add(ROOT/name,'native/hardware/'+name)
    for board in ('pcb_a_display','pcb_b_temperature'):
        for name in (board+'.kicad_pro',board+'.kicad_sch',board+'.kicad_pcb',board+'_bom_draft.csv','fp-lib-table','sym-lib-table','README.md'):
            add(ROOT/board/name,'native/hardware/'+board+'/'+name)
    for path in sorted((ROOT/'lab_hardware.pretty').glob('*.kicad_mod')):add(path,'native/hardware/lab_hardware.pretty/'+path.name)
    for name in ('build_enclosure.py','light_sensor_mount.py','package_enclosure.py','enclosure_parameters.json','README.md'):
        add(ROOT/'mechanical'/name,'native/hardware/mechanical/'+name)
    # Native manufacturing choices remain alongside CAD in the editable copy.
    for name in ('component_selection.json','land_pattern_review.json','fabrication_specification.json','power_and_usb_review.md'):
        add(FOLDER/name,'native/hardware/manufacturing/'+name)
    for name in ('enclosure_'+tag+'_integrated_buck.step','enclosure_'+tag+'_printed_parts.step','source_manifest_'+tag+'.json',
                 'validation.json','assembly_sequence_validation.json','preview_service.png','preview_b_power.png',
                 'preview_light_sensor.png','preview_light_sensor_front.png','preview_assembled.png'):
        add(ROOT/'mechanical/exports'/name,'mechanical/'+name)
    for path in sorted((ROOT/('mechanical/exports/parts_'+tag)).iterdir()):
        if path.suffix in ('.step','.stl'):add(path,'mechanical/parts_'+tag+'/'+path.name)
    for relative in checks['evidence_sha256']:
        add(ROOT/relative,'evidence/'+relative)
    for path in (FOLDER/'visual_review.json',FOLDER/'preparation_checks.json'):
        add(path,'evidence/manufacturing/'+path.name)
    for name in ('pcb_a_display_erc.rpt','pcb_b_temperature_erc.rpt','pcb_a_display.pdf','pcb_b_temperature.pdf'):
        add(ROOT/'review'/name,'evidence/review/'+name)
    for name in ('pcb_a_display_drc.json','pcb_b_temperature_drc.json','electronics_bench_plan.md'):
        add(ROOT/'review/routing'/name,'evidence/review/routing/'+name)
    status=dict(state='LOCAL_TECHNICAL_PREPARATION_COMPLETE / FACTORY_HANDOFF_PENDING',
        local_preparation_complete=True,date_utc=datetime.now(timezone.utc).isoformat(),
        automatic_stages=13,final_visual_review=True,physical_native_components=68,SMT_components=51,
        physical_THT_items=19,extra_loose_shunts=1,mechanical_service_stages=mechanical['assembly_service_stages'],
        source_geometry_and_MPNs_frozen=True,simulations_rerun_in_this_revision=False,
        mechanical_revision=parameters['revision'],BH1750_breakout_and_optical_transmission_qualified=False,
        hardware_functional_tests_performed=False,fabrication_release=False,order_authorized=False,order_placed=False,
        external_gates='factory_response.csv: exact sourcing, DFM/panel/holes, thermal/MSL process, placement preview, tests/quote and purchase authorization',
        scope='Prototype PCB preparation complete locally; no actual manufacturer acceptance or operational heater qualification inferred.')
    (FOLDER/'preparation_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf-8')
    add(FOLDER/'preparation_status.json','preparation_status.json')
    files['READ_FIRST.txt']=(
        'A04/B05 - COMPLETE LOCAL PROTOTYPE PREPARATION\n'
        'Quote5A+5B assembled boards; this is not an authorized order.\n'
        'A/ and B/: BOM/CPL/THT, Gerbers/drills, IPC-D-356 tests and ERC/DRC.\n'
        'assembly_review/: three-page PDF, pad map and physical THT placements.\n'
        'gerber_review/: independent renders of the actual exported layers.\n'
        'native/hardware/: editable KiCad projects, local footprints and mechanical source.\n'
        f'mechanical/: Fusion-compatible {tag} STEP and {mechanical["printed_parts"]} print parts.\n'
        'evidence/: source-bound checks and actual final visual review.\n'
        'Read fabrication_specification.json, assembly_RFQ_draft.md and factory_response.csv.\n'
        'Keep exact MPNs; obtain factory DFM/process/sourcing/placement-preview and quote approval before build.\n'
        'No factory response, purchase or physical heater/USB/printed-fit test is claimed.\n'
        'The complete regeneration workflow is in the originating Git repository.\n').encode('utf-8')
    manifest=dict(status=status['state'],original_RFQ_sha256=sha(rfq),
        original_RFQ_manifest='rfq_manifest.json',workspace_sources_sha256=sources,
        archive_files_sha256={name:hashlib.sha256(data).hexdigest() for name,data in sorted(files.items())})
    (FOLDER/'handoff_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    files['handoff_manifest.json']=(FOLDER/'handoff_manifest.json').read_bytes()
    archive=FOLDER/'FABRICATION_HANDOFF_A04_B05.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as bundle:
        for name,data in sorted(files.items()):bundle.writestr(name,data)
    (FOLDER/(archive.name+'.sha256')).write_text(sha(archive)+'  '+archive.name+'\n',encoding='ascii')
    print(json.dumps(dict(archive=str(archive),members=len(files),status=status['state']),indent=2))


if __name__=='__main__':main()
